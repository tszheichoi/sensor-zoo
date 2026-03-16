/**
 * Standalone defaults for registry.js requirement checks.
 *
 * In the parent app (sensor-logger), these functions read React state to check
 * whether sensors are enabled and at the correct sampling rate. For standalone
 * library use, all requirements are assumed to be met — the caller is
 * responsible for providing valid sensor data at the expected rate.
 *
 * If you integrate sensor-zoo into your own app and need real requirement
 * checks, replace these with your own state accessors.
 */

/**
 * Converts a sensor speed interval (ms) to a human-readable string.
 * E.g. 10 → "100 Hz", 20 → "50 Hz", 100 → "10 Hz".
 */
export function formatSpeed(intervalMs) {
  if (intervalMs == null || intervalMs <= 0) return "Off";
  return `${Math.round(1000 / intervalMs)} Hz`;
}

/** Returns true — standardisation is assumed to be on. */
export function getStandardise() {
  return true;
}

/** Returns true — uncalibrated data is assumed to be on. */
export function getUncalibrated() {
  return true;
}

/** Returns true — all sensors are assumed to be enabled. */
export function getSensorEnabled() {
  return true;
}

/** Returns 10 (= 100 Hz) — sensors are assumed to be at the expected rate. */
export function getSensorSpeed() {
  return 10;
}
