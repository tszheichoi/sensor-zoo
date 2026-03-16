import {
  quaternionNormalize,
  quaternionRotateVector,
  calculateGravityAppleConvention,
  quaternionToEuler,
  initQuaternionFromGravity,
  ahrsNaNGuard,
} from "../utils/quaternion.js";

// Magnetometer magnitude bounds for outlier rejection (µT).
// Earth's field is ~25–65 µT, but Android calibrated magnetometers
// often report 100–150 µT due to incomplete hard-iron compensation.
// Since we normalise before use, magnitude only gates acceptance.
const MAG_MIN_NORM = 10;
const MAG_MAX_NORM = 200;

export class MadgwickFilter {
  constructor(beta, betaMag) {
    this.beta = beta;
    this.betaMag = betaMag;
    this.q = [1, 0, 0, 0];
    this.initialized = false;
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

    // Normalize accelerometer for gradient computation
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

    // Gradient descent - accelerometer objective function
    const f0 = 2 * (q[1] * q[3] - q[0] * q[2]) - anx;
    const f1 = 2 * (q[0] * q[1] + q[2] * q[3]) - any;
    const f2 = 2 * (0.5 - q[1] * q[1] - q[2] * q[2]) - anz;

    // Jacobian transposed * f (gradient)
    let g0 = -2 * q[2] * f0 + 2 * q[1] * f1;
    let g1 = 2 * q[3] * f0 + 2 * q[0] * f1 - 4 * q[1] * f2;
    let g2 = -2 * q[0] * f0 + 2 * q[3] * f1 - 4 * q[2] * f2;
    let g3 = 2 * q[1] * f0 + 2 * q[2] * f1;

    // Magnetometer gradient (optional)
    if (
      magX != null &&
      magY != null &&
      magZ != null &&
      this.betaMag > 0 &&
      isFinite(magX) && isFinite(magY) && isFinite(magZ)
    ) {
      const magStrength = Math.sqrt(magX * magX + magY * magY + magZ * magZ);
      if (magStrength >= MAG_MIN_NORM && magStrength <= MAG_MAX_NORM) {
        const mnx = magX / magStrength;
        const mny = magY / magStrength;
        const mnz = magZ / magStrength;

        // Rotate mag to reference frame to get horizontal component
        const h = quaternionRotateVector(q, [mnx, mny, mnz]);
        const bx = Math.sqrt(h[0] * h[0] + h[1] * h[1]);
        const bz = h[2];

        // Magnetic field objective function
        const fm0 =
          2 * bx * (0.5 - q[2] * q[2] - q[3] * q[3]) +
          2 * bz * (q[1] * q[3] - q[0] * q[2]) -
          mnx;
        const fm1 =
          2 * bx * (q[1] * q[2] - q[0] * q[3]) +
          2 * bz * (q[0] * q[1] + q[2] * q[3]) -
          mny;
        const fm2 =
          2 * bx * (q[0] * q[2] + q[1] * q[3]) +
          2 * bz * (0.5 - q[1] * q[1] - q[2] * q[2]) -
          mnz;

        // Magnetic Jacobian^T * f_mag
        const mg0 =
          (-2 * bz * q[2]) * fm0 +
          (-2 * bx * q[3] + 2 * bz * q[1]) * fm1 +
          (2 * bx * q[2]) * fm2;
        const mg1 =
          (2 * bz * q[3]) * fm0 +
          (2 * bx * q[2] + 2 * bz * q[0]) * fm1 +
          (2 * bx * q[3] - 4 * bz * q[1]) * fm2;
        const mg2 =
          (-4 * bx * q[2] - 2 * bz * q[0]) * fm0 +
          (2 * bx * q[1] + 2 * bz * q[3]) * fm1 +
          (2 * bx * q[0] - 4 * bz * q[2]) * fm2;
        const mg3 =
          (-4 * bx * q[3] + 2 * bz * q[1]) * fm0 +
          (-2 * bx * q[0] + 2 * bz * q[2]) * fm1 +
          (2 * bx * q[1]) * fm2;

        g0 += this.betaMag * mg0;
        g1 += this.betaMag * mg1;
        g2 += this.betaMag * mg2;
        g3 += this.betaMag * mg3;
      }
    }

    // Normalize gradient (matching original Madgwick paper & x-io C reference).
    // This makes beta equivalent to estimated mean gyro error in rad/s.
    const gradNorm = Math.sqrt(g0 * g0 + g1 * g1 + g2 * g2 + g3 * g3);
    if (gradNorm > 1e-12) {
      g0 /= gradNorm;
      g1 /= gradNorm;
      g2 /= gradNorm;
      g3 /= gradNorm;
    } else {
      g0 = g1 = g2 = g3 = 0;
    }

    // Quaternion derivative from gyroscope
    // Note: unlike complementary filter, Madgwick does NOT negate gyroZ
    // (matches Python reference which uses gyro directly in quaternion derivative)
    const qDotGyro = [
      0.5 * (-q[1] * gyroX - q[2] * gyroY - q[3] * gyroZ),
      0.5 * (q[0] * gyroX + q[2] * gyroZ - q[3] * gyroY),
      0.5 * (q[0] * gyroY - q[1] * gyroZ + q[3] * gyroX),
      0.5 * (q[0] * gyroZ + q[1] * gyroY - q[2] * gyroX),
    ];

    // Integrate: q = q + (qDotGyro - beta * gradient) * dt
    this.q = quaternionNormalize([
      q[0] + (qDotGyro[0] - this.beta * g0) * dt,
      q[1] + (qDotGyro[1] - this.beta * g1) * dt,
      q[2] + (qDotGyro[2] - this.beta * g2) * dt,
      q[3] + (qDotGyro[3] - this.beta * g3) * dt,
    ]);

    // Calculate gravity and user acceleration
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
