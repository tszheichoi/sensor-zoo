/**
 * Standalone defaults for registry.js requirement checks.
 *
 * This is the ONLY file allowed to differ between the standalone sensor-zoo
 * repository and the copy vendored into the sensor-logger app. In the app,
 * these functions read React state to check whether sensors are enabled and
 * at the correct sampling rate. For standalone library use, all requirements
 * are assumed to be met — the caller is responsible for providing valid
 * sensor data at the expected rate.
 *
 * If you integrate sensor-zoo into your own app and need real requirement
 * checks, replace these with your own state accessors.
 */

/** Returns true — standardisation is assumed to be on. */
export function getStandardise() {
  return true;
}

/** Returns true — uncalibrated data is assumed to be on. */
export function getUncalibrated() {
  return true;
}

/** Returns true — uncalibrated recording is assumed to be enabled. */
export function recordsUncalibrated() {
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
