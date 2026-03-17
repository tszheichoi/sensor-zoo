// Low-pass filter coefficient for smoothing accelerometer magnitude.
// Lower values = smoother but more latency in step detection.
const SMOOTHING_ALPHA = 0.2;

// Expected sensor sampling rate in Hz. The ring buffer is sized to hold
// one full window at this rate (e.g. 0.6s × 100 Hz = 60 samples).
const EXPECTED_SAMPLE_RATE = 100;

export class WindowedPeakStepCounter {
  constructor(windowSize, cooldown, minProminence) {
    this.windowSize = windowSize;
    this.cooldown = cooldown;
    this.minProminence = minProminence;
    this.capacity = Math.max(10, Math.round(windowSize * EXPECTED_SAMPLE_RATE));
    this.steps = 0;
    this.smoothedMag = 0;
    this.ringBuffer = [];
    this.ringIndex = 0;
    this.timeSinceLastStep = Infinity;
    this.elapsed = 0;
  }

  init() {
    this.steps = 0;
    this.smoothedMag = 0;
    this.ringBuffer = [];
    this.ringIndex = 0;
    this.timeSinceLastStep = Infinity;
    this.elapsed = 0;
  }

  update(ax, ay, az, dt) {
    const mag = Math.sqrt(ax * ax + ay * ay + az * az);

    // Low-pass filter the magnitude
    if (this.elapsed === 0) {
      this.smoothedMag = mag;
    } else {
      this.smoothedMag =
        SMOOTHING_ALPHA * mag + (1 - SMOOTHING_ALPHA) * this.smoothedMag;
    }

    this.elapsed += dt;
    this.timeSinceLastStep += dt;

    // Store { value, time } in ring buffer
    const entry = { value: this.smoothedMag, time: this.elapsed };
    if (this.ringBuffer.length < this.capacity) {
      this.ringBuffer.push(entry);
    } else {
      this.ringBuffer[this.ringIndex % this.capacity] = entry;
    }
    this.ringIndex++;

    // Need a full window before detecting peaks
    if (this.ringBuffer.length < this.capacity) {
      return { steps: this.steps };
    }

    // The center of the ring buffer is the candidate peak
    const half = Math.floor(this.capacity / 2);
    // Oldest entry is at ringIndex % capacity (since we just wrote there, it was the oldest)
    // The center sample is half entries before the newest
    const centerIdx = ((this.ringIndex - 1 - half) % this.capacity + this.capacity) % this.capacity;
    const centerValue = this.ringBuffer[centerIdx].value;

    // Check if center is a local maximum (strictly greater than all others)
    let isPeak = true;
    let minValue = Infinity;
    for (let i = 0; i < this.capacity; i++) {
      const v = this.ringBuffer[i].value;
      if (v < minValue) minValue = v;
      if (i !== centerIdx && v >= centerValue) {
        isPeak = false;
        // Don't break — still need minValue for prominence check
      }
    }

    if (!isPeak) {
      return { steps: this.steps };
    }

    // Validate prominence: peak must stand out above noise
    const prominence = centerValue - minValue;
    if (prominence < this.minProminence) {
      return { steps: this.steps };
    }

    // Validate timing: enforce cooldown from last accepted peak
    if (this.timeSinceLastStep < this.cooldown) {
      return { steps: this.steps };
    }

    this.steps++;
    this.timeSinceLastStep = 0;

    return { steps: this.steps };
  }
}
