// Expected gravity magnitude (m/s²) and tolerance for init.
// Accepts 7.8–11.8 m/s² to handle noisy readings at startup.
const GRAVITY_EXPECTED = 9.8;
const GRAVITY_TOLERANCE = 2.0;

export function quaternionNormalize(q) {
  const [w, x, y, z] = q;
  const norm = Math.sqrt(w * w + x * x + y * y + z * z);
  if (norm === 0) return [1, 0, 0, 0];
  return [w / norm, x / norm, y / norm, z / norm];
}

export function quaternionMultiply(q1, q2) {
  const [w1, x1, y1, z1] = q1;
  const [w2, x2, y2, z2] = q2;
  return [
    w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
    w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
    w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
  ];
}

export function quaternionFromGyro(gx, gy, gz, dt) {
  const norm = Math.sqrt(gx * gx + gy * gy + gz * gz);
  const angle = norm * dt;
  if (norm < 1e-12) return [1, 0, 0, 0];
  const halfAngle = angle / 2;
  const s = Math.sin(halfAngle) / norm;
  const q = [Math.cos(halfAngle), gx * s, gy * s, gz * s];
  return quaternionNormalize(q);
}

export function quaternionToEuler(q) {
  const [w, x, y, z] = q;
  const sinr_cosp = 2 * (w * x + y * z);
  const cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);
  const sinp = 2 * (w * y - z * x);
  let pitch;
  if (Math.abs(sinp) >= 1) {
    pitch = Math.sign(sinp) * (Math.PI / 2);
  } else {
    pitch = Math.asin(sinp);
  }
  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);
  return { roll, pitch, yaw };
}

export function quaternionSlerp(q1, q2, t) {
  let dot = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
  if (dot < 0) {
    q2 = [-q2[0], -q2[1], -q2[2], -q2[3]];
    dot = -dot;
  }
  // When quaternions are nearly aligned, SLERP's sin(theta) denominator
  // approaches zero. Fall back to normalised linear interpolation (NLERP).
  if (dot > 0.9995) {
    const result = [
      q1[0] + t * (q2[0] - q1[0]),
      q1[1] + t * (q2[1] - q1[1]),
      q1[2] + t * (q2[2] - q1[2]),
      q1[3] + t * (q2[3] - q1[3]),
    ];
    return quaternionNormalize(result);
  }
  const theta0 = Math.acos(Math.abs(dot));
  const sinTheta0 = Math.sin(theta0);
  const theta = theta0 * t;
  const sinTheta = Math.sin(theta);
  const s0 = Math.cos(theta) - dot * sinTheta / sinTheta0;
  const s1 = sinTheta / sinTheta0;
  return [
    s0 * q1[0] + s1 * q2[0],
    s0 * q1[1] + s1 * q2[1],
    s0 * q1[2] + s1 * q2[2],
    s0 * q1[3] + s1 * q2[3],
  ];
}

export function quaternionRotateVector(q, v) {
  const [w, x, y, z] = q;
  const [vx, vy, vz] = v;
  // v' = v + 2 * cross(qxyz, cross(qxyz, v) + w * v)
  const cx1 = y * vz - z * vy;
  const cy1 = z * vx - x * vz;
  const cz1 = x * vy - y * vx;
  const tx = cx1 + w * vx;
  const ty = cy1 + w * vy;
  const tz = cz1 + w * vz;
  const cx2 = y * tz - z * ty;
  const cy2 = z * tx - x * tz;
  const cz2 = x * ty - y * tx;
  return [vx + 2 * cx2, vy + 2 * cy2, vz + 2 * cz2];
}

export function calculateGravityAppleConvention(q, g = 9.807) {
  const qConj = [q[0], -q[1], -q[2], -q[3]];
  return quaternionRotateVector(qConj, [0, 0, g]);
}

// Shared AHRS init: compute quaternion from gravity reading.
// Returns [w,x,y,z] such that calculateGravityAppleConvention(q) ≈ accel.
export function initQuaternionFromGravity(accelX, accelY, accelZ) {
  const norm = Math.sqrt(
    accelX * accelX + accelY * accelY + accelZ * accelZ
  );
  if (Math.abs(norm - GRAVITY_EXPECTED) < GRAVITY_TOLERANCE && norm > 0) {
    const gx = accelX / norm;
    const gy = accelY / norm;
    const gz = accelZ / norm;

    // Rotation from [0,0,1] to gravity: axis = [0,0,1] × [gx,gy,gz]
    const vx = -gy;
    const vy = gx;
    const vNorm = Math.sqrt(vx * vx + vy * vy);
    const c = gz;

    if (vNorm < 1e-6) {
      return c > 0 ? [1, 0, 0, 0] : [0, 1, 0, 0];
    } else {
      const angle = Math.acos(Math.max(-1, Math.min(1, c)));
      const halfAngle = angle / 2;
      const s = Math.sin(halfAngle) / vNorm;
      // Conjugate: calculateGravityAppleConvention rotates by q⁻¹,
      // so we store the inverse of the [0,0,1]→gravity rotation
      return quaternionNormalize([
        Math.cos(halfAngle),
        -vx * s,
        -vy * s,
        0,
      ]);
    }
  }
  return [1, 0, 0, 0];
}

// Shared NaN guard: compute gravity + euler from quaternion q and return
// a standard AHRS result object with zero user acceleration.
export function ahrsNaNGuard(q) {
  const gravity = calculateGravityAppleConvention(q);
  const euler = quaternionToEuler(q);
  return {
    gravityX: gravity[0], gravityY: gravity[1], gravityZ: gravity[2],
    userAccelX: 0, userAccelY: 0, userAccelZ: 0,
    qw: q[0], qx: q[1], qy: q[2], qz: q[3],
    roll: euler.roll, pitch: euler.pitch, yaw: euler.yaw,
  };
}
