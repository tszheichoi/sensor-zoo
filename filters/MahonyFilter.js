import {
  quaternionNormalize,
  quaternionRotateVector,
  calculateGravityAppleConvention,
  quaternionToEuler,
  initQuaternionFromGravity,
  ahrsNaNGuard,
} from "../utils/quaternion.js";

// Magnetometer magnitude bounds for outlier rejection (µT).
// Android calibrated magnetometers can report 100–150 µT; we
// normalise before use so magnitude only gates acceptance.
const MAG_MIN_NORM = 10;
const MAG_MAX_NORM = 200;

export class MahonyFilter {
  constructor(kP, kI, kPMag) {
    this.kP = kP;
    this.kI = kI;
    this.kPMag = kPMag != null ? kPMag : kP;
    this.q = [1, 0, 0, 0];
    this.initialized = false;
    this.integralError = [0, 0, 0];
  }

  init(accelX, accelY, accelZ) {
    this.q = initQuaternionFromGravity(accelX, accelY, accelZ);
    this.initialized = true;
  }

  update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX, magY, magZ) {
    // Guard against NaN input - skip update and return last known state
    if (isNaN(accelX) || isNaN(accelY) || isNaN(accelZ) ||
        isNaN(gyroX) || isNaN(gyroY) || isNaN(gyroZ) || isNaN(dt)) {
      return ahrsNaNGuard(this.q);
    }

    if (!this.initialized) {
      this.init(accelX, accelY, accelZ);
    }

    const q = this.q;

    // 1. Normalize accelerometer to get measured gravity direction
    const accelNorm = Math.sqrt(
      accelX * accelX + accelY * accelY + accelZ * accelZ
    );
    let anx, any, anz;
    if (accelNorm > 0) {
      anx = accelX / accelNorm;
      any = accelY / accelNorm;
      anz = accelZ / accelNorm;
    } else {
      anx = any = anz = 0;
    }

    // 2. Estimated gravity from current quaternion (body frame)
    const qConj = [q[0], -q[1], -q[2], -q[3]];
    const estGravity = quaternionRotateVector(qConj, [0, 0, 1]);
    const egx = estGravity[0];
    const egy = estGravity[1];
    const egz = estGravity[2];

    // 3. Gravity error = cross product of measured vs estimated gravity
    const eax = any * egz - anz * egy;
    const eay = anz * egx - anx * egz;
    const eaz = anx * egy - any * egx;

    // 4. Optional magnetometer: compute heading error (separate gain)
    let emx = 0, emy = 0, emz = 0;
    if (
      magX != null &&
      magY != null &&
      magZ != null &&
      isFinite(magX) && isFinite(magY) && isFinite(magZ)
    ) {
      const magNorm = Math.sqrt(magX * magX + magY * magY + magZ * magZ);
      if (magNorm >= MAG_MIN_NORM && magNorm <= MAG_MAX_NORM) {
        const mnx = magX / magNorm;
        const mny = magY / magNorm;
        const mnz = magZ / magNorm;

        // Rotate mag into earth frame
        const h = quaternionRotateVector(q, [mnx, mny, mnz]);
        // Reference direction: project onto horizontal plane
        const bx = Math.sqrt(h[0] * h[0] + h[1] * h[1]);
        const bz = h[2];

        // Rotate reference back to body frame
        const w = quaternionRotateVector(qConj, [bx, 0, bz]);

        // Magnetometer error (cross product of measured vs expected)
        const rawEmx = mny * w[2] - mnz * w[1];
        const rawEmy = mnz * w[0] - mnx * w[2];
        const rawEmz = mnx * w[1] - mny * w[0];

        // Project onto estimated gravity axis to isolate yaw correction.
        // Without this, large heading errors leak into roll/pitch and
        // destabilize the tilt estimate.
        const dot = rawEmx * egx + rawEmy * egy + rawEmz * egz;
        emx = dot * egx;
        emy = dot * egy;
        emz = dot * egz;
      }
    }

    // 5. Integral term: accumulate gravity error only (mag excluded to
    //    prevent noisy heading from corrupting the gyro bias estimate)
    this.integralError[0] += this.kI * eax * dt;
    this.integralError[1] += this.kI * eay * dt;
    this.integralError[2] += this.kI * eaz * dt;

    // 6. Corrected gyro = raw gyro + kP * gravity error + kPMag * mag error + bias
    const corrGyroX = gyroX + this.kP * eax + this.kPMag * emx + this.integralError[0];
    const corrGyroY = gyroY + this.kP * eay + this.kPMag * emy + this.integralError[1];
    const corrGyroZ = gyroZ + this.kP * eaz + this.kPMag * emz + this.integralError[2];

    // 7. Quaternion derivative from corrected gyro, integrate, normalize
    const qDot = [
      0.5 * (-q[1] * corrGyroX - q[2] * corrGyroY - q[3] * corrGyroZ),
      0.5 * (q[0] * corrGyroX + q[2] * corrGyroZ - q[3] * corrGyroY),
      0.5 * (q[0] * corrGyroY - q[1] * corrGyroZ + q[3] * corrGyroX),
      0.5 * (q[0] * corrGyroZ + q[1] * corrGyroY - q[2] * corrGyroX),
    ];

    this.q = quaternionNormalize([
      q[0] + qDot[0] * dt,
      q[1] + qDot[1] * dt,
      q[2] + qDot[2] * dt,
      q[3] + qDot[3] * dt,
    ]);

    // 8. Compute gravity, user accel, Euler from resulting quaternion
    const gravity = calculateGravityAppleConvention(this.q);
    const euler = quaternionToEuler(this.q);

    return {
      gravityX: gravity[0],
      gravityY: gravity[1],
      gravityZ: gravity[2],
      userAccelX: accelX - gravity[0],
      userAccelY: accelY - gravity[1],
      userAccelZ: accelZ - gravity[2],
      qw: this.q[0],
      qx: this.q[1],
      qy: this.q[2],
      qz: this.q[3],
      roll: euler.roll,
      pitch: euler.pitch,
      yaw: euler.yaw,
    };
  }
}
