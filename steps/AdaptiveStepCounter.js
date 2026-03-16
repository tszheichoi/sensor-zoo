// Low-pass filter coefficient for smoothing accelerometer magnitude.
// Lower values = smoother but more latency in step detection.
const SMOOTHING_ALPHA = 0.2;

// Expected sensor sampling rate in Hz. The ring buffer is sized to hold
// one full window at this rate (e.g. 2s × 100 Hz = 200 samples).
const EXPECTED_SAMPLE_RATE = 100;

export class AdaptiveStepCounter {
  constructor(windowSize, cooldown, minAmplitude) {
    this.cooldown = cooldown;
    this.minAmplitude = minAmplitude;
    this.capacity = Math.max(50, Math.round(windowSize * EXPECTED_SAMPLE_RATE));
    this.steps = 0;
    this.smoothedMag = 0;
    this.ringBuffer = [];
    this.ringIndex = 0;
    this.prevAboveThreshold = false;
    this.timeSinceLastStep = Infinity;
    this.elapsed = 0;
  }

  init() {
    this.steps = 0;
    this.smoothedMag = 0;
    this.ringBuffer = [];
    this.ringIndex = 0;
    this.prevAboveThreshold = false;
    this.timeSinceLastStep = Infinity;
    this.elapsed = 0;
  }

  update(ax, ay, az, dt) {
    const mag = Math.sqrt(ax * ax + ay * ay + az * az);

    // Low-pass filter the magnitude
    if (this.elapsed === 0) {
      this.smoothedMag = mag;
    } else {
      this.smoothedMag = SMOOTHING_ALPHA * mag + (1 - SMOOTHING_ALPHA) * this.smoothedMag;
    }

    this.elapsed += dt;
    this.timeSinceLastStep += dt;

    // Maintain a ring buffer of smoothed values for the sliding window
    if (this.ringBuffer.length < this.capacity) {
      this.ringBuffer.push(this.smoothedMag);
    } else {
      this.ringBuffer[this.ringIndex % this.capacity] = this.smoothedMag;
    }
    this.ringIndex++;

    // Need at least half a window before detecting
    if (this.ringBuffer.length < this.capacity / 2) {
      return { steps: this.steps };
    }

    // Compute min/max over the buffer
    let min = Infinity;
    let max = -Infinity;
    for (let i = 0; i < this.ringBuffer.length; i++) {
      if (this.ringBuffer[i] < min) min = this.ringBuffer[i];
      if (this.ringBuffer[i] > max) max = this.ringBuffer[i];
    }
    const amplitude = max - min;
    const threshold = (min + max) / 2;

    // Reject low-variance periods (standing still, not walking)
    if (amplitude < this.minAmplitude) {
      this.prevAboveThreshold = this.smoothedMag > threshold;
      return { steps: this.steps };
    }

    // Detect upward crossing of the adaptive threshold
    const aboveThreshold = this.smoothedMag > threshold;

    if (aboveThreshold && !this.prevAboveThreshold && this.timeSinceLastStep >= this.cooldown) {
      this.steps++;
      this.timeSinceLastStep = 0;
    }

    this.prevAboveThreshold = aboveThreshold;

    return { steps: this.steps };
  }
}
