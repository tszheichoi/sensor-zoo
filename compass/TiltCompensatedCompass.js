// Minimum gravity norm to attempt tilt compensation (m/s²).
const MIN_GRAVITY_NORM = 0.01;
// Minimum east-vector norm; guards against magnetic field parallel to gravity.
const MIN_EAST_NORM = 0.01;

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
        // Normalized gravity (points away from Earth in accelerometer convention)
        const ux = gx / gnorm;
        const uy = gy / gnorm;
        const uz = gz / gnorm;

        // East = normalize(cross(mag, up)) — Android getRotationMatrix convention
        let ex = my * uz - mz * uy;
        let ey = mz * ux - mx * uz;
        let ez = mx * uy - my * ux;
        const enorm = Math.sqrt(ex * ex + ey * ey + ez * ez);

        if (enorm < MIN_EAST_NORM) {
          // Magnetic field nearly parallel to gravity — cannot determine heading
          heading = -Math.atan2(mx, my);
        } else {
          ex /= enorm;
          ey /= enorm;
          ez /= enorm;

          // North = cross(up, east)
          const nx = uy * ez - uz * ey;
          const ny = uz * ex - ux * ez;
          const nz = ux * ey - uy * ex;

          // Axis quality: how horizontal each device axis is (0 = vertical, 1 = horizontal).
          // Since ux²+uy²+uz²=1, qy+qz = 1+ux² ≥ 1, so the denominator is always safe.
          const qy = 1 - uy * uy;
          const qz = 1 - uz * uz;

          // Heading from device y-axis (top of phone) — reliable when phone is flat
          const heading_y = Math.atan2(ey, ny);
          // Heading from device -z axis (camera direction) — reliable when phone is upright
          const heading_nz = Math.atan2(-ez, -nz);

          // Circular weighted average (approximately inverse-variance weighting).
          // When one axis is near-vertical its quality approaches zero, smoothly
          // reducing its influence without any hard threshold discontinuity.
          const wy = qy / (qy + qz);
          let diff = heading_nz - heading_y;
          if (diff > Math.PI) diff -= 2 * Math.PI;
          if (diff < -Math.PI) diff += 2 * Math.PI;
          heading = heading_y + (1 - wy) * diff;
        }
      }
    } else {
      // No gravity data — flat-device fallback
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
