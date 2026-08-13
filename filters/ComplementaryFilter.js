import {
  quaternionNormalize,
  quaternionMultiply,
  quaternionFromGyro,
  quaternionToEuler,
  quaternionRotateVector,
  calculateGravityAppleConvention,
  initQuaternionFromGravity,
  ahrsNaNGuard,
} from "../utils/quaternion.js";

export class ComplementaryFilter {
  constructor(alpha) {
    this.alpha = alpha;
    this.q = [1, 0, 0, 0];
    this.initialized = false;
  }

  init(accelX, accelY, accelZ) {
    this.q = initQuaternionFromGravity(accelX, accelY, accelZ);
    this.initialized = true;
  }

  update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt) {
    // Guard against NaN input - skip update and return last known state
    if (isNaN(accelX) || isNaN(accelY) || isNaN(accelZ) ||
        isNaN(gyroX) || isNaN(gyroY) || isNaN(gyroZ) || isNaN(dt)) {
      return ahrsNaNGuard(this.q);
    }

    if (!this.initialized) {
      this.init(accelX, accelY, accelZ);
    }

    // Gyroscope integration (prediction step)
    if (dt > 0) {
      const qGyro = quaternionFromGyro(gyroX, gyroY, gyroZ, dt);
      let q = quaternionNormalize(quaternionMultiply(this.q, qGyro));

      // Tilt-only accelerometer correction (swing–twist): rotate the
      // predicted gravity direction toward the measured one by a fraction
      // (1 - alpha) of the angle between them, about the axis perpendicular
      // to both. This corrects roll/pitch drift while leaving the twist
      // about gravity (yaw) untouched — a full-attitude SLERP toward an
      // accel-derived quaternion would drag yaw toward zero, since gravity
      // carries no yaw information.
      const accelMag = Math.sqrt(
        accelX * accelX + accelY * accelY + accelZ * accelZ
      );
      if (accelMag > 0) {
        const anx = accelX / accelMag;
        const any = accelY / accelMag;
        const anz = accelZ / accelMag;

        // Predicted gravity direction in the body frame
        const qConj = [q[0], -q[1], -q[2], -q[3]];
        const gPred = quaternionRotateVector(qConj, [0, 0, 1]);

        // Rotation axis (gPred × aMeasured) and angle between the two
        const cx = gPred[1] * anz - gPred[2] * any;
        const cy = gPred[2] * anx - gPred[0] * anz;
        const cz = gPred[0] * any - gPred[1] * anx;
        const cNorm = Math.sqrt(cx * cx + cy * cy + cz * cz);
        const dot = gPred[0] * anx + gPred[1] * any + gPred[2] * anz;

        // Skip when aligned (nothing to correct) or anti-parallel (no
        // unique axis; gyro integration will move it off the singularity).
        if (cNorm > 1e-8) {
          const angle = Math.atan2(cNorm, dot);
          const phi = (1 - this.alpha) * angle;
          const half = phi / 2;
          // δq applied as q ⊗ δq moves the predicted gravity toward the
          // measurement: R(δq⁻¹)·gPred, with δq⁻¹ = rot(axis, phi).
          const s = Math.sin(half) / cNorm;
          const dq = [Math.cos(half), -cx * s, -cy * s, -cz * s];
          q = quaternionNormalize(quaternionMultiply(q, dq));
        }
      }
      this.q = q;
    }

    this.q = quaternionNormalize(this.q);

    // Calculate gravity in device frame using Apple CoreMotion convention
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
