const PRECISION = 1e6;

export const MAX_POINTS = 500;

function encodeSigned(value) {
  let v = value < 0 ? ~(value << 1) : value << 1;
  let out = "";
  while (v >= 0x20) {
    out += String.fromCharCode((0x20 | (v & 0x1f)) + 63);
    v >>>= 5;
  }
  return out + String.fromCharCode(v + 63);
}

function encodePolyline(points) {
  let lastLat = 0;
  let lastLon = 0;
  let out = "";
  for (const p of points) {
    const lat = Math.round(p.latitude * PRECISION);
    const lon = Math.round(p.longitude * PRECISION);
    out += encodeSigned(lat - lastLat) + encodeSigned(lon - lastLon);
    lastLat = lat;
    lastLon = lon;
  }
  return out;
}

function decodePolyline(text) {
  const coordinates = [];
  let index = 0;
  let lat = 0;
  let lon = 0;
  while (index < text.length) {
    const pair = [];
    for (let axis = 0; axis < 2; axis++) {
      let result = 0;
      let shift = 0;
      let byte;
      do {
        if (index >= text.length) {
          throw new Error("route: the encoded geometry ends mid-coordinate");
        }
        byte = text.charCodeAt(index++) - 63;
        if (byte < 0 || shift > 30) {
          throw new Error("route: the encoded geometry is not readable");
        }
        result |= (byte & 0x1f) << shift;
        shift += 5;
      } while (byte >= 0x20);
      pair.push(result & 1 ? ~(result >> 1) : result >> 1);
    }
    lat += pair[0];
    lon += pair[1];
    coordinates.push([lon / PRECISION, lat / PRECISION]);
  }
  return coordinates;
}

export function encodeRoute(points) {
  if (!points || points.length === 0) return null;
  points.forEach((p, i) => {
    if (!Number.isFinite(p.latitude) || !Number.isFinite(p.longitude)) {
      throw new Error(`route: point ${i + 1} has no usable coordinates`);
    }
  });
  return {
    p: encodePolyline(points),
    v: points.map((p) => p.value ?? null),
  };
}

export function decodeRoutePoints(route) {
  if (route == null) return [];
  if (typeof route !== "object" || typeof route.p !== "string") {
    throw new Error("route: stored route is not an encoded route");
  }
  const values = Array.isArray(route.v) ? route.v : [];
  return decodePolyline(route.p).map(([longitude, latitude], i) => {
    const value = values[i];
    return {
      latitude,
      longitude,
      value:
        typeof value === "number" || typeof value === "string" ? value : null,
    };
  });
}

export function routeToGeoJSON(route) {
  const points = decodeRoutePoints(route);
  if (points.length === 0) return null;
  return {
    type: "FeatureCollection",
    features: [
      {
        type: "Feature",
        properties: { role: "route", values: points.map((p) => p.value) },
        geometry: {
          type: "LineString",
          coordinates: points.map((p) => [p.longitude, p.latitude]),
        },
      },
    ],
  };
}

export function isPlace(points) {
  const [first] = points;
  if (first == null) return false;
  return points.every(
    (p) => p.latitude === first.latitude && p.longitude === first.longitude,
  );
}

export function routeProblem(route) {
  if (route == null) return null;
  if (typeof route !== "object" || Array.isArray(route)) {
    return "not an encoded route";
  }
  if (typeof route.p !== "string") return "no encoded geometry";
  if (route.v != null && !Array.isArray(route.v))
    return "labels are not a list";
  let points;
  try {
    points = decodeRoutePoints(route);
  } catch (e) {
    return String(e?.message ?? e);
  }
  if (points.length < 2) return "a route needs at least two points";
  if (points.length > MAX_POINTS) {
    return `${points.length} points; a route holds at most ${MAX_POINTS}`;
  }
  if (isPlace(points)) return "every point is in the same place";
  if (route.v != null && route.v.length !== points.length) {
    return `${route.v.length} labels for ${points.length} points`;
  }
  for (const p of points) {
    if (p.latitude < -90 || p.latitude > 90) {
      return `latitude ${p.latitude} is outside -90..90`;
    }
    if (p.longitude < -180 || p.longitude > 180) {
      return `longitude ${p.longitude} is outside -180..180`;
    }
  }
  return null;
}
