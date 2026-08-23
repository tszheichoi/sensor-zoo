import {
  quaternionMultiply,
  quaternionNormalize,
  quaternionRotateVector,
  quaternionToEuler,
  calculateGravityAppleConvention,
  ahrsNaNGuard,
} from "../utils/quaternion.js";

/**
 * VQF: A Versatile Quaternion-based Filter for IMU Orientation Estimation.
 *
 * JavaScript port of the reference pure Python implementation (PyVQF) from
 * https://github.com/dlaidig/vqf, which implements:
 *
 *   D. Laidig and T. Seel. "VQF: Highly Accurate IMU Orientation Estimation
 *   with Bias Estimation and Magnetic Disturbance Rejection." Information
 *   Fusion 2023, 91, 187-204. doi:10.1016/j.inffus.2022.10.014
 *
 * Original: SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
 * Original: SPDX-License-Identifier: MIT
 * This port: MIT, adapted for sensor-zoo by the Sensor Logger project.
 *
 * Runs 6D (accel + gyro) and 9D (+ mag) estimation simultaneously, with gyro
 * bias estimation during rest and motion and magnetic disturbance rejection.
 *
 * Deviations from the reference:
 * - The reference takes a fixed sampling time. This port derives coefficients
 *   from the first observed dt but integrates with the actual per-sample dt,
 *   so a jittery live stream is handled. Fed a constant dt it is
 *   sample-for-sample identical to the reference.
 * - acos inputs are clamped to [-1, 1] against floating-point overshoot.
 * - update() returns the sensor-zoo output object, using the 9D estimate when
 *   magnetometer data is supplied and 6D otherwise.
 */

const EPS = Number.EPSILON;
const DEG = Math.PI / 180.0;

// Defaults from VQFParams in the reference implementation.
const DEFAULTS = {
  tauAcc: 3.0,
  tauMag: 9.0,
  motionBiasEstEnabled: true,
  restBiasEstEnabled: true,
  magDistRejectionEnabled: true,
  biasSigmaInit: 0.5,
  biasForgettingTime: 100.0,
  biasClip: 2.0,
  biasSigmaMotion: 0.1,
  biasVerticalForgettingFactor: 0.0001,
  biasSigmaRest: 0.03,
  restMinT: 1.5,
  restFilterTau: 0.5,
  restThGyr: 2.0,
  restThAcc: 0.5,
  magCurrentTau: 0.05,
  magRefTau: 20.0,
  magNormTh: 0.1,
  magDipTh: 10.0,
  magNewTime: 20.0,
  magNewFirstTime: 5.0,
  magNewMinGyr: 20.0,
  magMinUndisturbedTime: 0.5,
  magMaxRejectionTime: 60.0,
  magRejectionFactor: 2.0,
};

// ---- small vector/matrix helpers (3x3 stored row-major as length-9 arrays) --

function mat3Inverse(M) {
  const [a, b, c, d, e, f, g, h, i] = M;
  const A = e * i - f * h;
  const B = -(d * i - f * g);
  const C = d * h - e * g;
  const det = a * A + b * B + c * C;
  const s = 1.0 / det;
  return [
    A * s, -(b * i - c * h) * s, (b * f - c * e) * s,
    B * s, (a * i - c * g) * s, -(a * f - c * d) * s,
    C * s, -(a * h - b * g) * s, (a * e - b * d) * s,
  ];
}

function mat3Multiply(A, B) {
  const out = new Array(9);
  for (let r = 0; r < 3; r++) {
    for (let c = 0; c < 3; c++) {
      out[r * 3 + c] =
        A[r * 3] * B[c] + A[r * 3 + 1] * B[3 + c] + A[r * 3 + 2] * B[6 + c];
    }
  }
  return out;
}

function quatApplyDelta(q, delta) {
  const c = Math.cos(delta / 2);
  const s = Math.sin(delta / 2);
  return [
    c * q[0] - s * q[3],
    c * q[1] - s * q[2],
    c * q[2] + s * q[1],
    c * q[3] + s * q[0],
  ];
}

// Second-order Butterworth low-pass coefficients from the time constant of
// the dampened, non-oscillating part of the step response.
// Falls back to a direct passthrough when tau < Ts/2 to avoid instability.
function filterCoeffs(tau, Ts) {
  if (tau < Ts / 2) {
    return { b: [1.0, 0.0, 0.0], a: [0.0, 0.0] };
  }
  const fc = Math.SQRT2 / (2.0 * Math.PI * tau);
  const C = Math.tan(Math.PI * fc * Ts);
  const D = C * C + Math.SQRT2 * C + 1;
  const b0 = (C * C) / D;
  return {
    b: [b0, 2 * b0, b0],
    a: [(2 * (C * C - 1)) / D, (1 - Math.SQRT2 * C + C * C) / D],
  };
}

// First-order low-pass gain from the 1/e time constant.
function gainFromTau(tau, Ts) {
  if (tau < 0) return 0.0;
  if (tau === 0.0) return 1.0;
  return 1 - Math.exp(-Ts / tau);
}

// Initial second-order filter state for a given steady-state value.
function filterInitialState(x0, b, a) {
  return [x0 * (1 - b[0]), x0 * (b[2] - a[1])];
}

// Filter step for an N-vector signal with averaging-based initialization:
// for the first `tau` seconds the output is the running mean, then the filter
// state is initialized from that mean and regular IIR filtering takes over.
// State layout: Float64Array(2*N); state[i] = row 0, state[N+i] = row 1.
// During initialization, state[1] stores the sample count and row 1 the sum.
function filterVec(x, N, tau, Ts, b, a, state) {
  if (Number.isNaN(state[0])) {
    if (Number.isNaN(state[1])) {
      state[1] = 0;
      for (let i = 0; i < N; i++) state[N + i] = 0;
    }
    state[1] += 1;
    const out = new Array(N);
    for (let i = 0; i < N; i++) {
      state[N + i] += x[i];
      out[i] = state[N + i] / state[1];
    }
    if (state[1] * Ts >= tau) {
      for (let i = 0; i < N; i++) {
        const init = filterInitialState(out[i], b, a);
        state[i] = init[0];
        state[N + i] = init[1];
      }
    }
    return out;
  }
  // difference equations, a0 assumed 1
  const out = new Array(N);
  for (let i = 0; i < N; i++) {
    const y = b[0] * x[i] + state[i];
    state[i] = b[1] * x[i] - a[0] * y + state[N + i];
    state[N + i] = b[2] * x[i] - a[1] * y;
    out[i] = y;
  }
  return out;
}

export class VQFFilter {
  // tauAcc and tauMag are the two primary tuning parameters. Any other
  // reference parameter can be overridden via the third argument,
  // e.g. new VQFFilter(3, 9, { magDistRejectionEnabled: false }).
  constructor(tauAcc, tauMag, overrides) {
    this.params = { ...DEFAULTS, ...(overrides || {}) };
    if (tauAcc != null) this.params.tauAcc = tauAcc;
    if (tauMag != null) this.params.tauMag = tauMag;
    this.coeffs = null; // computed from the first observed dt
    this._dtSamples = null; // collected until the sampling rate is confirmed
    this._resetState();
  }

  _resetState() {
    this.state = {
      gyrQuat: [1, 0, 0, 0],
      accQuat: [1, 0, 0, 0],
      delta: 0.0,
      restDetected: false,
      magDistDetected: true,
      lastAccLp: [0, 0, 0],
      accLpState: new Float64Array(6).fill(NaN),
      lastAccCorrAngularRate: 0.0,
      kMagInit: 1.0,
      lastMagDisAngle: 0.0,
      lastMagCorrAngularRate: 0.0,
      bias: [0, 0, 0],
      // biasP is set when coefficients are ready
      biasP: null,
      motionBiasEstRLpState: new Float64Array(18).fill(NaN),
      motionBiasEstBiasLpState: new Float64Array(4).fill(NaN),
      restLastSquaredDeviations: [0, 0],
      restT: 0.0,
      restLastGyrLp: [0, 0, 0],
      restGyrLpState: new Float64Array(6).fill(NaN),
      restLastAccLp: [0, 0, 0],
      restAccLpState: new Float64Array(6).fill(NaN),
      magRefNorm: 0.0,
      magRefDip: 0.0,
      magUndisturbedT: 0.0,
      magRejectT: this.params.magMaxRejectionTime,
      magCandidateNorm: -1.0,
      magCandidateDip: 0.0,
      magCandidateT: 0.0,
      magNormDip: [0, 0],
      magNormDipLpState: new Float64Array(4).fill(NaN),
    };
    this.lastQ = [1, 0, 0, 0];
  }

  // Accelerometer arguments are unused: the first inclination correction is a
  // full snap, so VQF finds its own attitude. Dropping coeffs matters: a
  // second stream must re-derive its rate, not inherit the first one's.
  init() {
    this.coeffs = null;
    this._dtSamples = null;
    this._resetState();
  }

  _initialBiasP() {
    const p0 = this.coeffs.biasP0;
    return [p0, 0, 0, 0, p0, 0, 0, 0, p0];
  }

  _setup(Ts) {
    const p = this.params;
    const c = { gyrTs: Ts, accTs: Ts, magTs: Ts };

    const acc = filterCoeffs(p.tauAcc, c.accTs);
    c.accLpB = acc.b;
    c.accLpA = acc.a;

    c.kMag = gainFromTau(p.tauMag, c.magTs);

    c.biasP0 = (p.biasSigmaInit * 100.0) ** 2;
    // system noise increases the variance from 0 to (0.1 deg/s)^2 in
    // biasForgettingTime seconds
    c.biasV = (0.1 * 100.0) ** 2 * c.accTs / p.biasForgettingTime;
    const pMotion = (p.biasSigmaMotion * 100.0) ** 2;
    c.biasMotionW = pMotion ** 2 / c.biasV + pMotion;
    c.biasVerticalW = c.biasMotionW / Math.max(p.biasVerticalForgettingFactor, 1e-10);
    const pRest = (p.biasSigmaRest * 100.0) ** 2;
    c.biasRestW = pRest ** 2 / c.biasV + pRest;

    const rg = filterCoeffs(p.restFilterTau, c.gyrTs);
    c.restGyrLpB = rg.b;
    c.restGyrLpA = rg.a;
    const ra = filterCoeffs(p.restFilterTau, c.accTs);
    c.restAccLpB = ra.b;
    c.restAccLpA = ra.a;

    c.kMagRef = gainFromTau(p.magRefTau, c.magTs);
    if (p.magCurrentTau > 0) {
      const mc = filterCoeffs(p.magCurrentTau, c.magTs);
      c.magNormDipLpB = mc.b;
      c.magNormDipLpA = mc.a;
    }

    this.coeffs = c;
    this.state.biasP = this._initialBiasP();
  }

  _updateGyr(gyr, dt) {
    const s = this.state;
    const p = this.params;
    const c = this.coeffs;

    // rest detection (gyroscope part)
    if (p.restBiasEstEnabled || p.magDistRejectionEnabled) {
      const gyrLp = filterVec(gyr, 3, p.restFilterTau, c.gyrTs, c.restGyrLpB, c.restGyrLpA, s.restGyrLpState);
      const dx = gyr[0] - gyrLp[0];
      const dy = gyr[1] - gyrLp[1];
      const dz = gyr[2] - gyrLp[2];
      const squaredDeviation = dx * dx + dy * dy + dz * dz;
      const biasClip = p.biasClip * DEG;
      const th = p.restThGyr * DEG;
      if (
        squaredDeviation >= th * th ||
        Math.max(Math.abs(gyrLp[0]), Math.abs(gyrLp[1]), Math.abs(gyrLp[2])) > biasClip
      ) {
        s.restT = 0.0;
        s.restDetected = false;
      }
      s.restLastGyrLp = gyrLp;
      s.restLastSquaredDeviations[0] = squaredDeviation;
    }

    // remove estimated gyro bias
    const gx = gyr[0] - s.bias[0];
    const gy = gyr[1] - s.bias[1];
    const gz = gyr[2] - s.bias[2];

    // strapdown integration (uses the actual dt, see header)
    const gyrNorm = Math.sqrt(gx * gx + gy * gy + gz * gz);
    const angle = gyrNorm * dt;
    if (gyrNorm > EPS) {
      const cw = Math.cos(angle / 2);
      const sw = Math.sin(angle / 2) / gyrNorm;
      const stepQuat = [cw, sw * gx, sw * gy, sw * gz];
      s.gyrQuat = quaternionNormalize(quaternionMultiply(s.gyrQuat, stepQuat));
    }
  }

  _updateAcc(acc, dt) {
    const s = this.state;
    const p = this.params;
    const c = this.coeffs;

    if (acc[0] === 0.0 && acc[1] === 0.0 && acc[2] === 0.0) return;

    // rest detection (accelerometer part)
    if (p.restBiasEstEnabled) {
      const accLp = filterVec(acc, 3, p.restFilterTau, c.accTs, c.restAccLpB, c.restAccLpA, s.restAccLpState);
      const dx = acc[0] - accLp[0];
      const dy = acc[1] - accLp[1];
      const dz = acc[2] - accLp[2];
      const squaredDeviation = dx * dx + dy * dy + dz * dz;
      if (squaredDeviation >= p.restThAcc * p.restThAcc) {
        s.restT = 0.0;
        s.restDetected = false;
      } else {
        s.restT += dt;
        if (s.restT >= p.restMinT) s.restDetected = true;
      }
      s.restLastAccLp = accLp;
      s.restLastSquaredDeviations[1] = squaredDeviation;
    }

    // filter acc in the inertial frame
    let accEarth = quaternionRotateVector(s.gyrQuat, acc);
    s.lastAccLp = filterVec(accEarth, 3, p.tauAcc, c.accTs, c.accLpB, c.accLpA, s.accLpState);

    // transform to 6D earth frame and normalize
    accEarth = quaternionRotateVector(s.accQuat, s.lastAccLp);
    const n = Math.sqrt(accEarth[0] ** 2 + accEarth[1] ** 2 + accEarth[2] ** 2);
    if (n !== 0.0) {
      accEarth = [accEarth[0] / n, accEarth[1] / n, accEarth[2] / n];
    }

    // inclination correction
    const qW = Math.sqrt((accEarth[2] + 1) / 2);
    const accCorrQuat = qW > 1e-6
      ? [qW, 0.5 * accEarth[1] / qW, -0.5 * accEarth[0] / qW, 0]
      : [0, 1, 0, 0];
    s.accQuat = quaternionNormalize(quaternionMultiply(accCorrQuat, s.accQuat));

    s.lastAccCorrAngularRate = Math.acos(Math.max(-1, Math.min(1, accEarth[2]))) / c.accTs;

    // bias estimation
    if (p.motionBiasEstEnabled || p.restBiasEstEnabled) {
      const biasClip = p.biasClip * DEG;
      const bias = s.bias;

      const q = this.getQuat6D();
      let R = [
        1 - 2 * q[2] ** 2 - 2 * q[3] ** 2,
        2 * (q[2] * q[1] - q[0] * q[3]),
        2 * (q[0] * q[2] + q[3] * q[1]),
        2 * (q[0] * q[3] + q[2] * q[1]),
        1 - 2 * q[1] ** 2 - 2 * q[3] ** 2,
        2 * (q[2] * q[3] - q[1] * q[0]),
        2 * (q[3] * q[1] - q[0] * q[2]),
        2 * (q[0] * q[1] + q[3] * q[2]),
        1 - 2 * q[1] ** 2 - 2 * q[2] ** 2,
      ];

      // R*b_hat (x and y components only)
      let biasLp = [
        R[0] * bias[0] + R[1] * bias[1] + R[2] * bias[2],
        R[3] * bias[0] + R[4] * bias[1] + R[5] * bias[2],
      ];

      // low-pass filter R and R*b_hat (the LP states always advance)
      R = filterVec(R, 9, p.tauAcc, c.accTs, c.accLpB, c.accLpA, s.motionBiasEstRLpState);
      biasLp = filterVec(biasLp, 2, p.tauAcc, c.accTs, c.accLpB, c.accLpA, s.motionBiasEstBiasLpState);

      let e = null;
      let w = null;
      if (s.restDetected && p.restBiasEstEnabled) {
        e = [
          s.restLastGyrLp[0] - bias[0],
          s.restLastGyrLp[1] - bias[1],
          s.restLastGyrLp[2] - bias[2],
        ];
        R = [1, 0, 0, 0, 1, 0, 0, 0, 1];
        w = [c.biasRestW, c.biasRestW, c.biasRestW];
      } else if (p.motionBiasEstEnabled) {
        e = [
          -accEarth[1] / c.accTs + biasLp[0] - R[0] * bias[0] - R[1] * bias[1] - R[2] * bias[2],
          accEarth[0] / c.accTs + biasLp[1] - R[3] * bias[0] - R[4] * bias[1] - R[5] * bias[2],
          -R[6] * bias[0] - R[7] * bias[1] - R[8] * bias[2],
        ];
        w = [c.biasMotionW, c.biasMotionW, c.biasVerticalW];
      }

      // Kalman filter update
      // step 1: P = P + V (increase covariance even without a measurement)
      const P = s.biasP;
      if (P[0] < c.biasP0) P[0] += c.biasV;
      if (P[4] < c.biasP0) P[4] += c.biasV;
      if (P[8] < c.biasP0) P[8] += c.biasV;

      if (e !== null) {
        // clip disagreement to the bias clip range
        for (let i = 0; i < 3; i++) {
          e[i] = Math.max(-biasClip, Math.min(biasClip, e[i]));
        }
        // K = P R^T inv(W + R P R^T)
        const RP = mat3Multiply(R, P);
        const RPRT = new Array(9);
        for (let r = 0; r < 3; r++) {
          for (let cc = 0; cc < 3; cc++) {
            RPRT[r * 3 + cc] =
              RP[r * 3] * R[cc * 3] + RP[r * 3 + 1] * R[cc * 3 + 1] + RP[r * 3 + 2] * R[cc * 3 + 2];
          }
        }
        RPRT[0] += w[0];
        RPRT[4] += w[1];
        RPRT[8] += w[2];
        // P R^T computed directly: (P R^T)[r][c] = sum_k P[r][k] * R[c][k]
        const PRt = new Array(9);
        for (let r = 0; r < 3; r++) {
          for (let cc = 0; cc < 3; cc++) {
            PRt[r * 3 + cc] =
              P[r * 3] * R[cc * 3] + P[r * 3 + 1] * R[cc * 3 + 1] + P[r * 3 + 2] * R[cc * 3 + 2];
          }
        }
        const K = mat3Multiply(PRt, mat3Inverse(RPRT));
        // bias += K e
        bias[0] += K[0] * e[0] + K[1] * e[1] + K[2] * e[2];
        bias[1] += K[3] * e[0] + K[4] * e[1] + K[5] * e[2];
        bias[2] += K[6] * e[0] + K[7] * e[1] + K[8] * e[2];
        // P -= K R P
        const KRP = mat3Multiply(K, RP);
        for (let i = 0; i < 9; i++) P[i] -= KRP[i];
        // clip bias
        for (let i = 0; i < 3; i++) {
          bias[i] = Math.max(-biasClip, Math.min(biasClip, bias[i]));
        }
      }
    }
  }

  _updateMag(mag, dt) {
    const s = this.state;
    const p = this.params;
    const c = this.coeffs;

    if (mag[0] === 0.0 && mag[1] === 0.0 && mag[2] === 0.0) return;

    // bring the magnetometer measurement into the 6D earth frame
    const magEarth = quaternionRotateVector(this.getQuat6D(), mag);

    if (p.magDistRejectionEnabled) {
      const norm = Math.sqrt(magEarth[0] ** 2 + magEarth[1] ** 2 + magEarth[2] ** 2);
      let magNormDip = [norm, -Math.asin(Math.max(-1, Math.min(1, magEarth[2] / norm)))];

      if (p.magCurrentTau > 0) {
        magNormDip = filterVec(magNormDip, 2, p.magCurrentTau, c.magTs, c.magNormDipLpB, c.magNormDipLpA, s.magNormDipLpState);
      }
      s.magNormDip = magNormDip;

      // magnetic disturbance detection
      if (
        Math.abs(magNormDip[0] - s.magRefNorm) < p.magNormTh * s.magRefNorm &&
        Math.abs(magNormDip[1] - s.magRefDip) < p.magDipTh * DEG
      ) {
        s.magUndisturbedT += dt;
        if (s.magUndisturbedT >= p.magMinUndisturbedTime) {
          s.magDistDetected = false;
          s.magRefNorm += c.kMagRef * (magNormDip[0] - s.magRefNorm);
          s.magRefDip += c.kMagRef * (magNormDip[1] - s.magRefDip);
        }
      } else {
        s.magUndisturbedT = 0.0;
        s.magDistDetected = true;
      }

      // new magnetic field acceptance
      if (
        Math.abs(magNormDip[0] - s.magCandidateNorm) < p.magNormTh * s.magCandidateNorm &&
        Math.abs(magNormDip[1] - s.magCandidateDip) < p.magDipTh * DEG
      ) {
        const gyrNorm = Math.sqrt(
          s.restLastGyrLp[0] ** 2 + s.restLastGyrLp[1] ** 2 + s.restLastGyrLp[2] ** 2
        );
        if (gyrNorm >= p.magNewMinGyr * DEG) {
          s.magCandidateT += dt;
        }
        s.magCandidateNorm += c.kMagRef * (magNormDip[0] - s.magCandidateNorm);
        s.magCandidateDip += c.kMagRef * (magNormDip[1] - s.magCandidateDip);

        if (
          s.magDistDetected &&
          (s.magCandidateT >= p.magNewTime ||
            (s.magRefNorm === 0.0 && s.magCandidateT >= p.magNewFirstTime))
        ) {
          s.magRefNorm = s.magCandidateNorm;
          s.magRefDip = s.magCandidateDip;
          s.magDistDetected = false;
          s.magUndisturbedT = p.magMinUndisturbedTime;
        }
      } else {
        s.magCandidateT = 0.0;
        s.magCandidateNorm = magNormDip[0];
        s.magCandidateDip = magNormDip[1];
      }
    }

    // disagreement angle based on the current magnetometer measurement
    s.lastMagDisAngle = Math.atan2(magEarth[0], magEarth[1]) - s.delta;
    if (s.lastMagDisAngle > Math.PI) s.lastMagDisAngle -= 2 * Math.PI;
    else if (s.lastMagDisAngle < -Math.PI) s.lastMagDisAngle += 2 * Math.PI;

    let k = c.kMag;

    if (p.magDistRejectionEnabled) {
      if (s.magDistDetected) {
        if (s.magRejectT <= p.magMaxRejectionTime) {
          s.magRejectT += dt;
          k = 0;
        } else {
          k /= p.magRejectionFactor;
        }
      } else {
        s.magRejectT = Math.max(s.magRejectT - p.magRejectionFactor * dt, 0.0);
      }
    }

    // ensure fast initial convergence
    if (s.kMagInit !== 0.0) {
      if (k < s.kMagInit) k = s.kMagInit;
      s.kMagInit = s.kMagInit / (s.kMagInit + 1);
      if (s.kMagInit * p.tauMag < c.magTs) s.kMagInit = 0.0;
    }

    // first-order filter step on the heading offset
    s.delta += k * s.lastMagDisAngle;
    s.lastMagCorrAngularRate = (k * s.lastMagDisAngle) / c.magTs;

    if (s.delta > Math.PI) s.delta -= 2 * Math.PI;
    else if (s.delta < -Math.PI) s.delta += 2 * Math.PI;
  }

  getQuat6D() {
    return quaternionMultiply(this.state.accQuat, this.state.gyrQuat);
  }

  getQuat9D() {
    return quatApplyDelta(this.getQuat6D(), this.state.delta);
  }

  update(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, dt, magX, magY, magZ) {
    // Guard against NaN input: skip update and return last known state
    if (
      isNaN(accelX) || isNaN(accelY) || isNaN(accelZ) ||
      isNaN(gyroX) || isNaN(gyroY) || isNaN(gyroZ) || isNaN(dt)
    ) {
      return ahrsNaNGuard(this.lastQ);
    }

    if (dt > 0) {
      // The first dt is often a burst artifact, so seed coefficients from it
      // provisionally and rebuild from the median of 40 samples if they
      // disagree by >20%. A steady stream never triggers the heal.
      if (this.coeffs === null) {
        this._setup(Math.min(Math.max(dt, 0.001), 0.1));
        this._dtSamples = [];
      }
      if (this._dtSamples && this._dtSamples.length < 40) {
        this._dtSamples.push(dt);
        if (this._dtSamples.length === 40) {
          const sorted = [...this._dtSamples].sort((a, b) => a - b);
          const medianDt = Math.min(Math.max(sorted[20], 0.001), 0.1);
          if (Math.abs(medianDt - this.coeffs.gyrTs) / this.coeffs.gyrTs > 0.2) {
            const q6 = this.getQuat6D();
            const delta = this.state.delta;
            this._setup(medianDt);
            this._resetState();
            this.state.biasP = this._initialBiasP();
            this.state.gyrQuat = q6;
            this.state.delta = delta;
          }
          this._dtSamples = null;
        }
      }
      this._updateGyr([gyroX, gyroY, gyroZ], dt);
      this._updateAcc([accelX, accelY, accelZ], dt);
    }

    const hasMag =
      magX != null && magY != null && magZ != null &&
      isFinite(magX) && isFinite(magY) && isFinite(magZ);
    if (dt > 0 && hasMag && this.coeffs !== null) {
      this._updateMag([magX, magY, magZ], dt);
    }

    const q = hasMag ? this.getQuat9D() : this.getQuat6D();
    this.lastQ = q;

    const gravity = calculateGravityAppleConvention(q);
    const euler = quaternionToEuler(q);
    return {
      gravityX: gravity[0],
      gravityY: gravity[1],
      gravityZ: gravity[2],
      userAccelX: accelX - gravity[0],
      userAccelY: accelY - gravity[1],
      userAccelZ: accelZ - gravity[2],
      qw: q[0],
      qx: q[1],
      qy: q[2],
      qz: q[3],
      roll: euler.roll,
      pitch: euler.pitch,
      yaw: euler.yaw,
    };
  }
}
