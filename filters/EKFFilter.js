import {
  quaternionNormalize,
  quaternionMultiply,
  quaternionFromGyro,
  quaternionRotateVector,
  calculateGravityAppleConvention,
  quaternionToEuler,
  initQuaternionFromGravity,
  ahrsNaNGuard,
} from "../utils/quaternion.js";

// ── Matrix helpers (flat Float64Array, row-major) ──────────────────────

// 6x6 * 6x6 → 6x6
function mat6x6Multiply(A, B) {
  const C = new Float64Array(36);
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) {
      let sum = 0;
      for (let k = 0; k < 6; k++) {
        sum += A[i * 6 + k] * B[k * 6 + j];
      }
      C[i * 6 + j] = sum;
    }
  }
  return C;
}

// A * B^T where A, B are 6x6
function mat6x6MultiplyTranspose(A, B) {
  const C = new Float64Array(36);
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 6; j++) {
      let sum = 0;
      for (let k = 0; k < 6; k++) {
        sum += A[i * 6 + k] * B[j * 6 + k];
      }
      C[i * 6 + j] = sum;
    }
  }
  return C;
}

// 6x6 + 6x6 in place (A += B)
function mat6x6Add(A, B) {
  for (let i = 0; i < 36; i++) A[i] += B[i];
}

// transpose 3x6 → 6x3
function transpose3x6(H) {
  const HT = new Float64Array(18);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 6; j++) {
      HT[j * 3 + i] = H[i * 6 + j];
    }
  }
  return HT;
}

// 6x6 * 6x3 → 6x3
function mat6x6Times6x3(A, B) {
  const C = new Float64Array(18);
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 3; j++) {
      let sum = 0;
      for (let k = 0; k < 6; k++) {
        sum += A[i * 6 + k] * B[k * 3 + j];
      }
      C[i * 3 + j] = sum;
    }
  }
  return C;
}

// 3x6 * 6x3 → 3x3
function mat3x6Times6x3(A, B) {
  const C = new Float64Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      let sum = 0;
      for (let k = 0; k < 6; k++) {
        sum += A[i * 6 + k] * B[k * 3 + j];
      }
      C[i * 3 + j] = sum;
    }
  }
  return C;
}

// 6x3 * 3x3 → 6x3
function mat6x3Times3x3(A, B) {
  const C = new Float64Array(18);
  for (let i = 0; i < 6; i++) {
    for (let j = 0; j < 3; j++) {
      let sum = 0;
      for (let k = 0; k < 3; k++) {
        sum += A[i * 3 + k] * B[k * 3 + j];
      }
      C[i * 3 + j] = sum;
    }
  }
  return C;
}

// Analytic 3x3 inverse
function mat3x3Inverse(M) {
  const a = M[0], b = M[1], c = M[2];
  const d = M[3], e = M[4], f = M[5];
  const g = M[6], h = M[7], i = M[8];
  const det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (Math.abs(det) < 1e-20) return null;
  const invDet = 1 / det;
  const inv = new Float64Array(9);
  inv[0] = (e * i - f * h) * invDet;
  inv[1] = (c * h - b * i) * invDet;
  inv[2] = (b * f - c * e) * invDet;
  inv[3] = (f * g - d * i) * invDet;
  inv[4] = (a * i - c * g) * invDet;
  inv[5] = (c * d - a * f) * invDet;
  inv[6] = (d * h - e * g) * invDet;
  inv[7] = (b * g - a * h) * invDet;
  inv[8] = (a * e - b * d) * invDet;
  return inv;
}

// ── Constants ──────────────────────────────────────────────────────────
// Gyro bias random walk noise — controls how fast bias estimates drift.
const BIAS_PROCESS_NOISE = 1e-5;
// If normalised accel magnitude deviates more than 15% from 1g,
// inflate measurement noise to reduce accel correction during motion.
const ACCEL_ADAPTIVE_THRESHOLD = 0.15;
// Magnetometer magnitude bounds for outlier rejection (µT).
// Android calibrated magnetometers can report 100–150 µT; we
// normalise before use so magnitude only gates acceptance.
const MAG_MIN_NORM = 10;
const MAG_MAX_NORM = 200;

// ── EKF Filter ─────────────────────────────────────────────────────────

export class EKFFilter {
  constructor(processNoise, accelNoise, magNoise) {
    this.processNoise = processNoise;
    this.accelNoise = accelNoise;
    this.magNoise = magNoise;
    this.q = [1, 0, 0, 0];
    this.bias = [0, 0, 0];
    this.P = new Float64Array(36);
    this.initialized = false;
    // Pre-allocate scratch matrices to reduce GC pressure
    this._F = new Float64Array(36);
    this._Q = new Float64Array(36);
    this._H = new Float64Array(18);
    this._R = new Float64Array(9);
    this._Hm = new Float64Array(18);
    this._Rm = new Float64Array(9);
    this._IKH = new Float64Array(36);
    this._KR = new Float64Array(18);
    this._dx = new Float64Array(6);
    this._initCovariance();
  }

  _initCovariance() {
    this.P.fill(0);
    for (let i = 0; i < 3; i++) this.P[i * 6 + i] = 0.1;
    for (let i = 3; i < 6; i++) this.P[i * 6 + i] = 1e-4;
  }

  init(accelX, accelY, accelZ) {
    this.q = initQuaternionFromGravity(accelX, accelY, accelZ);
    this.bias = [0, 0, 0];
    this._initCovariance();
    this.initialized = true;
  }

  update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX, magY, magZ) {
    if (
      isNaN(accelX) || isNaN(accelY) || isNaN(accelZ) ||
      isNaN(gyroX) || isNaN(gyroY) || isNaN(gyroZ) || isNaN(dt)
    ) {
      return ahrsNaNGuard(this.q);
    }

    if (!this.initialized) {
      this.init(accelX, accelY, accelZ);
    }

    // ─── Predict ───────────────────────────────────────────────────────

    // 1. Subtract bias
    const wx = gyroX - this.bias[0];
    const wy = gyroY - this.bias[1];
    const wz = gyroZ - this.bias[2];

    // 2. Integrate quaternion
    this.q = quaternionNormalize(
      quaternionMultiply(this.q, quaternionFromGyro(wx, wy, wz, dt))
    );

    // 3. State transition F (6x6)
    //    F = [ I - [ω×]dt,  -I·dt ]
    //        [     0,         I    ]
    const F = this._F; F.fill(0);
    const wxdt = wx * dt, wydt = wy * dt, wzdt = wz * dt;
    // Top-left: I - skew(ω)·dt
    F[0]  = 1;      F[1]  = wzdt;   F[2]  = -wydt;
    F[6]  = -wzdt;  F[7]  = 1;      F[8]  = wxdt;
    F[12] = wydt;   F[13] = -wxdt;  F[14] = 1;
    // Top-right: -I·dt
    F[3]  = -dt;    F[10] = -dt;    F[17] = -dt;
    // Bottom-right: I
    F[21] = 1;      F[28] = 1;      F[35] = 1;

    // 4. Process noise Q (6x6 diagonal)
    const sigGyroSq = this.processNoise * this.processNoise * dt;
    const sigBiasSq = BIAS_PROCESS_NOISE * BIAS_PROCESS_NOISE * dt;
    const Q = this._Q; Q.fill(0);
    Q[0] = sigGyroSq;  Q[7] = sigGyroSq;   Q[14] = sigGyroSq;
    Q[21] = sigBiasSq;  Q[28] = sigBiasSq;  Q[35] = sigBiasSq;

    // 5. Propagate covariance: P = F·P·Fᵀ + Q
    const FP = mat6x6Multiply(F, this.P);
    this.P = mat6x6MultiplyTranspose(FP, F);
    mat6x6Add(this.P, Q);

    // ─── Accelerometer Update ──────────────────────────────────────────

    const accelNorm = Math.sqrt(
      accelX * accelX + accelY * accelY + accelZ * accelZ
    );
    if (accelNorm > 0) {
      const anx = accelX / accelNorm;
      const any = accelY / accelNorm;
      const anz = accelZ / accelNorm;

      // Expected gravity in body frame: rotate [0,0,1] by q⁻¹
      // (matches calculateGravityAppleConvention which rotates [0,0,g] by qConj)
      const qConj = [this.q[0], -this.q[1], -this.q[2], -this.q[3]];
      const gEst = quaternionRotateVector(qConj, [0, 0, 1]);

      // Innovation
      const yAccel = [anx - gEst[0], any - gEst[1], anz - gEst[2]];

      // Measurement Jacobian H (3x6): [ skew(gEst)  0₃ₓ₃ ]
      const H = this._H; H.fill(0);
      H[0]  = 0;         H[1]  = -gEst[2]; H[2]  = gEst[1];
      H[6]  = gEst[2];   H[7]  = 0;        H[8]  = -gEst[0];
      H[12] = -gEst[1];  H[13] = gEst[0];  H[14] = 0;

      // Adaptive noise: inflate R when accelerating
      let rVal = this.accelNoise * this.accelNoise;
      const normDev = Math.abs(accelNorm / 9.81 - 1.0);
      if (normDev > ACCEL_ADAPTIVE_THRESHOLD) {
        rVal *= 1.0 + 10.0 * normDev * normDev;
      }
      const R = this._R; R.fill(0);
      R[0] = rVal; R[4] = rVal; R[8] = rVal;

      this._kalmanUpdate(H, R, yAccel);
    }

    // ─── Magnetometer Update ───────────────────────────────────────────

    if (
      magX != null && magY != null && magZ != null &&
      isFinite(magX) && isFinite(magY) && isFinite(magZ)
    ) {
      const magNorm = Math.sqrt(magX * magX + magY * magY + magZ * magZ);
      if (magNorm >= MAG_MIN_NORM && magNorm <= MAG_MAX_NORM) {
        const mnx = magX / magNorm;
        const mny = magY / magNorm;
        const mnz = magZ / magNorm;

        // Rotate mag to earth frame
        const h = quaternionRotateVector(this.q, [mnx, mny, mnz]);

        // Reference direction: keep horizontal magnitude + vertical component
        const bx = Math.sqrt(h[0] * h[0] + h[1] * h[1]);
        const bz = h[2];

        // Rotate reference back to body frame
        const qConj = [this.q[0], -this.q[1], -this.q[2], -this.q[3]];
        const w = quaternionRotateVector(qConj, [bx, 0, bz]);

        // Innovation
        const yMag = [mnx - w[0], mny - w[1], mnz - w[2]];

        // H_mag (3x6): [ skew(w)  0₃ₓ₃ ]
        const Hm = this._Hm; Hm.fill(0);
        Hm[0]  = 0;      Hm[1]  = -w[2]; Hm[2]  = w[1];
        Hm[6]  = w[2];   Hm[7]  = 0;     Hm[8]  = -w[0];
        Hm[12] = -w[1];  Hm[13] = w[0];  Hm[14] = 0;

        // Magnetometer noise
        const rmVal = this.magNoise * this.magNoise;
        const Rm = this._Rm; Rm.fill(0);
        Rm[0] = rmVal; Rm[4] = rmVal; Rm[8] = rmVal;

        this._kalmanUpdate(Hm, Rm, yMag);
      }
    }

    // ─── Output ────────────────────────────────────────────────────────

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

  // Shared Kalman update for both accel and mag measurements
  // H: 3x6, R: 3x3, y: [3]
  _kalmanUpdate(H, R, y) {
    const HT = transpose3x6(H);

    // P·Hᵀ (6x3)
    const PHT = mat6x6Times6x3(this.P, HT);

    // S = H·P·Hᵀ + R (3x3)
    const S = mat3x6Times6x3(H, PHT);
    S[0] += R[0]; S[1] += R[1]; S[2] += R[2];
    S[3] += R[3]; S[4] += R[4]; S[5] += R[5];
    S[6] += R[6]; S[7] += R[7]; S[8] += R[8];

    const SInv = mat3x3Inverse(S);
    if (!SInv) return; // singular — skip update

    // K = P·Hᵀ·S⁻¹ (6x3)
    const K = mat6x3Times3x3(PHT, SInv);

    // Error-state correction: dx = K·y (6x1)
    const dx = this._dx;
    for (let i = 0; i < 6; i++) {
      dx[i] = K[i * 3] * y[0] + K[i * 3 + 1] * y[1] + K[i * 3 + 2] * y[2];
    }

    // Attitude correction: q = q ⊗ [1, δθ/2]
    this.q = quaternionNormalize(
      quaternionMultiply(this.q, [1, dx[0] / 2, dx[1] / 2, dx[2] / 2])
    );

    // Bias correction
    this.bias[0] += dx[3];
    this.bias[1] += dx[4];
    this.bias[2] += dx[5];

    // Joseph-form covariance update: P = (I−K·H)·P·(I−K·H)ᵀ + K·R·Kᵀ
    // Build I − K·H (6x6)
    const IKH = this._IKH; IKH.fill(0);
    for (let i = 0; i < 6; i++) {
      IKH[i * 6 + i] = 1; // identity
      for (let j = 0; j < 6; j++) {
        let kh = 0;
        for (let m = 0; m < 3; m++) {
          kh += K[i * 3 + m] * H[m * 6 + j];
        }
        IKH[i * 6 + j] -= kh;
      }
    }

    // (I−KH)·P·(I−KH)ᵀ
    const IKHP = mat6x6Multiply(IKH, this.P);
    const joseph = mat6x6MultiplyTranspose(IKHP, IKH);

    // K·R·Kᵀ (6x6)
    // First K·R (6x3)
    const KR = this._KR;
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 3; j++) {
        let sum = 0;
        for (let m = 0; m < 3; m++) {
          sum += K[i * 3 + m] * R[m * 3 + j];
        }
        KR[i * 3 + j] = sum;
      }
    }
    // Then (K·R)·Kᵀ (6x6)
    for (let i = 0; i < 6; i++) {
      for (let j = 0; j < 6; j++) {
        let sum = 0;
        for (let m = 0; m < 3; m++) {
          sum += KR[i * 3 + m] * K[j * 3 + m];
        }
        joseph[i * 6 + j] += sum;
      }
    }

    this.P = joseph;
  }
}
