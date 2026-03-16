import {
  quaternionNormalize,
  quaternionMultiply,
  quaternionFromGyro,
  quaternionToEuler,
  quaternionSlerp,
  calculateGravityAppleConvention,
  initQuaternionFromGravity,
  ahrsNaNGuard,
} from "../utils/quaternion.js";

// Axis-angle approach: finds quaternion q such that
// calculateGravityAppleConvention(q) ≈ [ax, ay, az].
// Accepts any nonzero accel magnitude (unlike initQuaternionFromGravity
// which guards for plausible gravity norms). Used per-frame as SLERP target.
function quaternionFromAccel(ax, ay, az) {
  const norm = Math.sqrt(ax * ax + ay * ay + az * az);
  if (norm === 0) return [1, 0, 0, 0];
  const dx = ax / norm;
  const dy = ay / norm;
  const dz = az / norm;

  // qConj must rotate [0,0,1] to [dx,dy,dz]
  // Cross product: [0,0,1] × [dx,dy,dz] = [-dy, dx, 0]
  const vx = -dy;
  const vy = dx;
  const vNorm = Math.sqrt(vx * vx + vy * vy);
  const c = dz; // dot product with [0,0,1]

  let cw, cx, cy;
  if (vNorm < 1e-6) {
    if (c > 0) {
      return [1, 0, 0, 0]; // parallel — identity
    } else {
      return [0, -1, 0, 0]; // anti-parallel — 180° around x (conjugated)
    }
  } else {
    const angle = Math.acos(Math.max(-1, Math.min(1, c)));
    const halfAngle = angle / 2;
    const s = Math.sin(halfAngle) / vNorm;
    cw = Math.cos(halfAngle);
    cx = vx * s;
    cy = vy * s;
  }
  // q = conjugate(qConj) since we need the inverse rotation
  return quaternionNormalize([cw, -cx, -cy, 0]);
}

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
      let qPredicted = quaternionMultiply(this.q, qGyro);
      qPredicted = quaternionNormalize(qPredicted);

      // Accelerometer-derived quaternion (correction step)
      const accelMag = Math.sqrt(
        accelX * accelX + accelY * accelY + accelZ * accelZ
      );
      if (accelMag > 0) {
        const qAccel = quaternionFromAccel(accelX, accelY, accelZ);
        // SLERP: alpha=high -> trust gyro more -> mostly use qPredicted
        this.q = quaternionSlerp(qAccel, qPredicted, this.alpha);
      } else {
        this.q = qPredicted;
      }
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
