// Minimum gravity norm to attempt tilt compensation (m/s²).
const MIN_GRAVITY_NORM = 0.01;
// Flatness = uz² (squared cosine of tilt from vertical).
// Below this, phone is nearly upright — raw XY heading is more stable.
const FLATNESS_RAW_THRESHOLD = 0.1;
// Above this, phone is fairly flat — tilt compensation is reliable.
const FLATNESS_TILT_THRESHOLD = 0.5;

export class TiltCompensatedCompass {
  constructor(smoothing) {
    this.smoothing = smoothing;
    this.smoothedHeading = null;
  }

  init() {
    this.smoothedHeading = null;
  }

  update(mx, my, mz, gx, gy, gz, dt) {
    let heading;

    if (gx != null && gy != null && gz != null) {
      const gnorm = Math.sqrt(gx * gx + gy * gy + gz * gz);
      if (gnorm < MIN_GRAVITY_NORM) {
        heading = -Math.atan2(mx, my);
      } else {
        const ux = gx / gnorm;
        const uy = gy / gnorm;
        const uz = gz / gnorm;
        // |uz| indicates how "flat" the phone is:
        //   |uz| ≈ 1 → flat (screen up/down), tilt compensation helps
        //   |uz| ≈ 0 → upright (held in hand), raw heading is better
        const flatness = uz * uz; // 0 = upright, 1 = flat

        if (flatness < FLATNESS_RAW_THRESHOLD) {
          // Phone is nearly upright — raw x/y heading is best
          heading = -Math.atan2(mx, my);
        } else {
          // Tilt-compensated heading via horizontal projection.
          // Projects magnetometer onto the plane perpendicular to gravity,
          // then measures angle relative to device forward (y-axis).
          const mdotu = mx * ux + my * uy + mz * uz;
          const hx = uz * mx - ux * mz;
          const hy = my - uy * mdotu;
          const tiltHeading = -Math.atan2(hx, hy);

          if (flatness > FLATNESS_TILT_THRESHOLD) {
            // Phone is fairly flat — tilt compensation is reliable
            heading = tiltHeading;
          } else {
            // Blend between raw and tilt-compensated
            const rawHeading = -Math.atan2(mx, my);
            let diff = tiltHeading - rawHeading;
            if (diff > Math.PI) diff -= 2 * Math.PI;
            if (diff < -Math.PI) diff += 2 * Math.PI;
            const t = (flatness - FLATNESS_RAW_THRESHOLD) / (FLATNESS_TILT_THRESHOLD - FLATNESS_RAW_THRESHOLD);
            heading = rawHeading + t * diff;
          }
        }
      }
    } else {
      heading = -Math.atan2(mx, my);
    }

    // Convert to degrees and normalize to 0-360
    let degrees = heading * (180 / Math.PI);
    degrees = ((degrees % 360) + 360) % 360;

    // EMA smoothing with circular wrap-around handling
    if (this.smoothedHeading == null) {
      this.smoothedHeading = degrees;
    } else {
      let diff = degrees - this.smoothedHeading;
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      this.smoothedHeading = ((this.smoothedHeading + this.smoothing * diff) % 360 + 360) % 360;
    }

    return { magneticBearing: this.smoothedHeading };
  }
}
