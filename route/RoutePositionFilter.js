const M_PER_DEG_LAT = (phi) =>
  111132.92 - 559.82 * Math.cos(2 * phi) + 1.175 * Math.cos(4 * phi);
const M_PER_DEG_LON = (phi) =>
  111412.84 * Math.cos(phi) - 93.5 * Math.cos(3 * phi);

function position(coords, what) {
  const [longitude, latitude] = coords ?? [];
  if (!isFinite(latitude) || !isFinite(longitude)) {
    throw new Error(`route: ${what} has no usable coordinates`);
  }
  if (latitude < -90 || latitude > 90) {
    throw new Error(
      `route: ${what} has latitude ${latitude}, outside -90..90, ` +
        "GeoJSON positions are [longitude, latitude], check the order",
    );
  }
  return { latitude, longitude };
}

function isLabelValue(value) {
  if (typeof value === "number") return isFinite(value);
  return typeof value === "string" && value.trim() !== "";
}

function parseGeoJSON(route) {
  if (!route || typeof route !== "object") {
    throw new Error("route: expected a GeoJSON object");
  }

  const features =
    route.type === "FeatureCollection"
      ? (route.features ?? [])
      : [
          route.type === "Feature"
            ? route
            : { type: "Feature", geometry: route, properties: {} },
        ];

  const lines = [];
  const points = [];
  let values = [];

  for (const f of features) {
    const geometry = f?.geometry ?? f;
    if (geometry?.type === "LineString") {
      lines.push(geometry.coordinates ?? []);
      values = f?.properties?.values ?? [];
    } else if (geometry?.type === "Point") {
      const value = (f?.properties ?? {}).value;
      if (isLabelValue(value)) {
        points.push({ ...position(geometry.coordinates, "a Point"), value });
      }
    }
  }

  if (lines.length !== 1) {
    throw new Error(
      `route: expected exactly one LineString, found ${lines.length}`,
    );
  }
  const line = lines[0].map((c, i) => position(c, `LineString vertex ${i}`));
  if (line.length < 2) {
    throw new Error("route: LineString needs at least two positions");
  }
  return { line, points, values };
}

export class RoutePositionFilter {
  constructor(route, maxOffset, maxSpeed) {
    this.maxOffset = maxOffset;
    this.maxSpeed = maxSpeed;

    const { line, points, values } = parseGeoJSON(route);
    this.geometry = line;

    this.cumulative = [];
    this.segmentLength = [];
    let total = 0;
    for (let i = 0; i < this.geometry.length; i++) {
      this.cumulative.push(total);
      if (i < this.geometry.length - 1) {
        const a = this.geometry[i];
        const b = this.geometry[i + 1];
        const phi = ((a.latitude + b.latitude) / 2) * (Math.PI / 180);
        const dy = (b.latitude - a.latitude) * M_PER_DEG_LAT(phi);
        const dx = (b.longitude - a.longitude) * M_PER_DEG_LON(phi);
        const len = Math.sqrt(dx * dx + dy * dy);
        this.segmentLength.push(len);
        total += len;
      }
    }
    this.totalLength = total;

    this.markers = [];
    for (let i = 0; i < this.geometry.length; i++) {
      const value = values[i];
      if (isLabelValue(value)) {
        this.markers.push({ distance: this.cumulative[i], value });
      }
    }

    if (this.segmentLength.length > 0) {
      for (const p of points) {
        const hit = this.#project(
          p.latitude,
          p.longitude,
          0,
          this.segmentLength.length - 1,
        );
        if (hit) this.markers.push({ distance: hit.distance, value: p.value });
      }
    }

    this.markers.sort((a, b) => a.distance - b.distance);

    this.categorical = this.markers.some((m) => typeof m.value !== "number");

    this.init();
  }

  init() {
    this.index = null; // segment index of the last accepted fix
    this.distance = null;
  }

  update(latitude, longitude, dt) {
    if (
      this.segmentLength.length === 0 ||
      latitude == null ||
      longitude == null ||
      !isFinite(latitude) ||
      !isFinite(longitude)
    ) {
      return { distance: null, marker: null, offset: null };
    }

    let from = 0;
    let to = this.segmentLength.length - 1;
    if (this.index != null) {
      const reach = this.maxSpeed * Math.max(dt || 0, 1) + this.maxOffset;
      from = this.#indexAtDistance(this.distance - reach);
      to = this.#indexAtDistance(this.distance + reach);
    }

    const hit = this.#project(latitude, longitude, from, to);

    if (hit == null || hit.offset > this.maxOffset) {
      this.index = null;
      this.distance = null;
      return { distance: null, marker: null, offset: null };
    }

    this.index = hit.index;
    this.distance = hit.distance;

    return {
      distance: hit.distance,
      marker: this.#markerAt(hit.distance),
      offset: hit.offset,
    };
  }

  #project(latitude, longitude, from, to) {
    if (latitude == null || longitude == null) return null;

    const phi = latitude * (Math.PI / 180);
    const mLat = M_PER_DEG_LAT(phi);
    const mLon = M_PER_DEG_LON(phi);

    let bestOffset = Infinity;
    let best = null;

    for (let i = from; i <= to; i++) {
      const len = this.segmentLength[i];
      if (len === 0) continue;
      const a = this.geometry[i];
      const b = this.geometry[i + 1];

      const sx = (b.longitude - a.longitude) * mLon;
      const sy = (b.latitude - a.latitude) * mLat;
      const px = (longitude - a.longitude) * mLon;
      const py = (latitude - a.latitude) * mLat;

      let t = (px * sx + py * sy) / (sx * sx + sy * sy);
      if (t < 0) t = 0;
      else if (t > 1) t = 1;

      const ex = px - t * sx;
      const ey = py - t * sy;
      const offset = Math.sqrt(ex * ex + ey * ey);

      if (offset < bestOffset) {
        bestOffset = offset;
        best = { index: i, distance: this.cumulative[i] + t * len, offset };
      }
    }
    return best;
  }

  #markerAt(distance) {
    const n = this.markers.length;
    if (n === 0 || (!this.categorical && n < 2)) return null;

    let lo = 0;
    let hi = n - 1;
    while (lo < hi - 1) {
      const mid = (lo + hi) >> 1;
      if (this.markers[mid].distance <= distance) lo = mid;
      else hi = mid;
    }
    const a = this.markers[lo];
    const b = this.markers[hi];

    if (this.categorical) {
      return Math.abs(distance - a.distance) <= Math.abs(distance - b.distance)
        ? a.value
        : b.value;
    }

    const span = b.distance - a.distance;
    if (span === 0) return a.value;
    return a.value + ((distance - a.distance) / span) * (b.value - a.value);
  }

  #indexAtDistance(d) {
    if (d <= 0) return 0;
    const last = this.segmentLength.length - 1;
    if (d >= this.totalLength) return last;
    let lo = 0;
    let hi = last;
    while (lo < hi) {
      const mid = (lo + hi + 1) >> 1;
      if (this.cumulative[mid] <= d) lo = mid;
      else hi = mid - 1;
    }
    return lo;
  }
}

export function routeMarkers(route) {
  return new RoutePositionFilter(route, 1, 1).markers;
}
