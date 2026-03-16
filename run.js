#!/usr/bin/env node
import { readFileSync, existsSync, statSync } from "fs";
import { FUSION_FILTERS } from "./registry.js";

const args = process.argv.slice(2);
let recordingDir = null;
let filterName = "madgwick";
let useMag = true;

for (let i = 0; i < args.length; i++) {
  if (args[i] === "--filter" && args[i + 1]) filterName = args[++i];
  else if (args[i] === "--no-mag") useMag = false;
  else if (!args[i].startsWith("--")) recordingDir = args[i];
}

const names = Object.keys(FUSION_FILTERS).join(", ");

if (!recordingDir) {
  process.stderr.write(
    `Usage: node run.js <recording-dir> [--filter name] [--no-mag]\n\nFilters: ${names}\n`,
  );
  process.exit(1);
}
if (!existsSync(recordingDir) || !statSync(recordingDir).isDirectory()) {
  process.stderr.write(`Error: "${recordingDir}" is not a directory\n`);
  process.exit(1);
}

const filterDef = FUSION_FILTERS[filterName];
if (!filterDef) {
  process.stderr.write(
    `Error: unknown filter "${filterName}". Options: ${names}\n`,
  );
  process.exit(1);
}

const params = {};
for (const p of filterDef.params) params[p.key] = p.defaultValue;
const filter = filterDef.createFilter(params);
const category = filterDef.categoryKey;

function parseSensorCSV(filePath) {
  const lines = readFileSync(filePath, "utf-8").trim().split("\n");
  const header = lines[0].split(",").map((h) => h.trim());
  const idxT = header.indexOf("seconds_elapsed");
  const idxX = header.indexOf("x");
  const idxY = header.indexOf("y");
  const idxZ = header.indexOf("z");
  return lines.slice(1).map((line) => {
    const c = line.split(",");
    return { seconds_elapsed: +c[idxT], x: +c[idxX], y: +c[idxY], z: +c[idxZ] };
  });
}

function loadSensor(name) {
  const p = recordingDir + "/" + name + ".csv";
  return existsSync(p) ? parseSensorCSV(p) : null;
}

function findNearest(arr, idx, t) {
  while (
    idx < arr.length - 1 &&
    Math.abs(arr[idx + 1].seconds_elapsed - t) <
      Math.abs(arr[idx].seconds_elapsed - t)
  )
    idx++;
  return idx;
}

const write = (line) => process.stdout.write(line + "\n");

if (category === "orientation") {
  const accel =
    loadSensor("AccelerometerUncalibrated") || loadSensor("Accelerometer");
  const gyro = loadSensor("Gyroscope");
  const mag = useMag ? loadSensor("Magnetometer") : null;

  if (!accel || accel.length === 0) { process.stderr.write("Error: no accelerometer data found\n"); process.exit(1); }
  if (!gyro || gyro.length === 0) { process.stderr.write("Error: no gyroscope data found\n"); process.exit(1); }

  const merged = [];
  let gi = 0, mi = 0;
  for (const a of accel) {
    const t = a.seconds_elapsed;
    gi = findNearest(gyro, gi, t);
    if (Math.abs(gyro[gi].seconds_elapsed - t) > 0.05) continue;

    let mx, my, mz;
    if (mag) {
      mi = findNearest(mag, mi, t);
      if (Math.abs(mag[mi].seconds_elapsed - t) > 0.05) continue;
      mx = mag[mi].x; my = mag[mi].y; mz = mag[mi].z;
    }

    merged.push({ t, a, gx: gyro[gi].x, gy: gyro[gi].y, gz: gyro[gi].z, mx, my, mz });
  }

  if (merged.length === 0) {
    process.stderr.write("Error: no samples survived time merge\n");
    process.exit(1);
  }

  filter.init(merged[0].a.x, merged[0].a.y, merged[0].a.z);
  write("time,qw,qx,qy,qz,roll,pitch,yaw,gravityX,gravityY,gravityZ,userAccelX,userAccelY,userAccelZ");

  for (let i = 0; i < merged.length; i++) {
    const r = merged[i];
    const dt = i === 0 ? 0.01 : r.t - merged[i - 1].t;
    const safeDt = dt > 0 && dt < 1 ? dt : 0.01;
    const res = filter.update(
      r.a.x, r.a.y, r.a.z, r.gx, r.gy, r.gz, safeDt, r.mx, r.my, r.mz,
    );
    write(
      [r.t, res.qw, res.qx, res.qy, res.qz, res.roll, res.pitch, res.yaw,
        res.gravityX, res.gravityY, res.gravityZ,
        res.userAccelX, res.userAccelY, res.userAccelZ].join(","),
    );
  }

  const dur = merged[merged.length - 1].t - merged[0].t;
  process.stderr.write(
    `Filter: ${filterName} | Samples: ${merged.length} | Duration: ${dur.toFixed(1)}s | Magnetometer: ${mag ? "on" : "off"}\n`,
  );
} else if (category === "stepCounter") {
  const accel =
    loadSensor("AccelerometerUncalibrated") || loadSensor("Accelerometer");
  if (!accel || accel.length === 0) { process.stderr.write("Error: no accelerometer data found\n"); process.exit(1); }

  filter.init();
  write("time,steps");

  for (let i = 0; i < accel.length; i++) {
    const s = accel[i];
    const dt = i === 0 ? 0.01 : s.seconds_elapsed - accel[i - 1].seconds_elapsed;
    const safeDt = dt > 0 && dt < 1 ? dt : 0.01;
    const res = filter.update(s.x, s.y, s.z, safeDt);
    write(`${s.seconds_elapsed},${res.steps}`);
  }

  const dur = accel[accel.length - 1].seconds_elapsed - accel[0].seconds_elapsed;
  process.stderr.write(
    `Filter: ${filterName} | Samples: ${accel.length} | Duration: ${dur.toFixed(1)}s\n`,
  );
} else if (category === "compass") {
  const mag = loadSensor("Magnetometer");
  const grav = loadSensor("Gravity");
  if (!mag || mag.length === 0) { process.stderr.write("Error: no magnetometer data found\n"); process.exit(1); }

  filter.init();
  write("time,magneticBearing");

  let gi = 0;
  for (let i = 0; i < mag.length; i++) {
    const m = mag[i];
    const dt = i === 0 ? 0.01 : m.seconds_elapsed - mag[i - 1].seconds_elapsed;
    const safeDt = dt > 0 && dt < 1 ? dt : 0.01;

    let gx, gy, gz;
    if (grav) {
      gi = findNearest(grav, gi, m.seconds_elapsed);
      if (Math.abs(grav[gi].seconds_elapsed - m.seconds_elapsed) <= 0.05) {
        gx = grav[gi].x; gy = grav[gi].y; gz = grav[gi].z;
      }
    }

    const res = filter.update(m.x, m.y, m.z, gx, gy, gz, safeDt);
    write(`${m.seconds_elapsed},${res.magneticBearing}`);
  }

  const dur = mag[mag.length - 1].seconds_elapsed - mag[0].seconds_elapsed;
  process.stderr.write(
    `Filter: ${filterName} | Samples: ${mag.length} | Duration: ${dur.toFixed(1)}s | Gravity: ${grav ? "on" : "off"}\n`,
  );
}
