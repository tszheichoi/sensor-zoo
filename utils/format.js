/**
 * Converts a sensor speed interval (ms) to a human-readable string.
 * E.g. 0 → "Max", 10 → "100 Hz", 20 → "50 Hz", 5000 → "5s".
 */
export function formatSpeed(speed) {
  if (speed === 0) return "Max";
  if (speed < 1000) return `${(1000 / speed).toFixed(0)} Hz`;
  return `${(speed / 1000).toFixed(0)}s`;
}
