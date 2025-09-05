// seg3d.js
// 3D terrain map rendering for segments

(function () {
  "use strict";

  let map3d = null;

  const SKIN_HALF_WIDTH_M = 35;
  const SKIN_STRIDE = 1;

  // Initialise Mapbox 3D map
  function init3DMap() {
    if (map3d) return;
    const el = document.getElementById("seg-map");
    if (el) el.replaceChildren();
    if (!window.MAPBOX_TOKEN) {
      console.warn("[seg3d] MAPBOX_TOKEN not set – skipping 3D terrain init");
      return;
    }
    if (typeof mapboxgl === "undefined") {
      console.error("[seg3d] mapboxgl missing");
      return;
    }
    mapboxgl.accessToken = window.MAPBOX_TOKEN;
    map3d = new mapboxgl.Map({
      container: "seg-map",
      style: "mapbox://styles/mapbox/outdoors-v12",
      center: [0, 0],
      zoom: 12,
      pitch: 60,
      bearing: -17.6,
      antialias: true,
    });
    map3d.on("load", () => {
      map3d.addSource("mapbox-dem", {
        type: "raster-dem",
        url: "mapbox://mapbox.mapbox-terrain-dem-v1",
        tileSize: 512,
        maxzoom: 14,
      });

      map3d.setTerrain({ source: "mapbox-dem", exaggeration: 2.5 });

      let labelLayerId = null;
      for (const l of map3d.getStyle().layers) {
        if (l.type === "symbol" && l.layout && l.layout["text-field"]) {
          labelLayerId = l.id;
          break;
        }
      }

      if (!map3d.getSource("opentopo")) {
        map3d.addSource("opentopo", {
          type: "raster",
          tiles: [
            "https://a.tile.opentopomap.org/{z}/{x}/{y}.png",
            "https://b.tile.opentopomap.org/{z}/{x}/{y}.png",
            "https://c.tile.opentopomap.org/{z}/{x}/{y}.png",
          ],
          tileSize: 256,
          attribution:
            "Map data © OpenStreetMap contributors, SRTM | Map style © OpenTopoMap (CC-BY-SA)",
        });
      }

      if (!map3d.getLayer("opentopo-layer")) {
        map3d.addLayer({
          id: "opentopo-layer",
          type: "raster",
          source: "opentopo",
          paint: {
            "raster-opacity": 1,
          },
        });
      }

      for (const l of map3d.getStyle().layers) {
        if (l.type === "symbol") {
          map3d.moveLayer(l.id);
        }
      }

      try {
        map3d.addLayer(
          {
            id: "3d-buildings",
            source: "composite",
            "source-layer": "building",
            type: "fill-extrusion",
            minzoom: 15,
            paint: {
              "fill-extrusion-color": "#aaa",
              "fill-extrusion-height": ["get", "height"],
              "fill-extrusion-base": ["get", "min_height"],
              "fill-extrusion-opacity": 0.6,
            },
          },
          labelLayerId || undefined,
        );
      } catch (_) {}

      map3d.addControl(
        new mapboxgl.NavigationControl({ visualizePitch: true }),
        "top-right",
      );

      map3d.addLayer({
        id: "sky",
        type: "sky",
        paint: {
          "sky-type": "atmosphere",
          "sky-atmosphere-sun": [0.0, 0.0],
          "sky-atmosphere-sun-intensity": 10,
        },
      });
      window.map3d = map3d;
    });
  }

  // Fit segment coordinates around origin
  function normalizeCoords(seg) {
    if (!seg) return [];
    let A = seg.coordinates || seg.coords || seg.points || [];
    if (!Array.isArray(A)) return [];
    const out = [];
    for (const p of A) {
      if (!p) continue;

      if (typeof p === "object" && !Array.isArray(p)) {
        const lat = p.lat ?? p.latitude;
        const lon = p.lng ?? p.lon ?? p.longitude;
        if (Number.isFinite(lat) && Number.isFinite(lon))
          out.push([+lat, +lon]);
        continue;
      }

      if (Array.isArray(p) && p.length >= 2) {
        const a = +p[0],
          b = +p[1];
        if (!Number.isFinite(a) || !Number.isFinite(b)) continue;

        const looksLatLon = Math.abs(a) <= 90 && Math.abs(b) <= 180;
        const looksLonLat = Math.abs(a) <= 180 && Math.abs(b) <= 90;
        if (looksLatLon && !looksLonLat) out.push([a, b]);
        else if (!looksLatLon && looksLonLat) out.push([b, a]);
        else {
          if (Math.abs(a) > 90 && Math.abs(b) <= 90) out.push([b, a]);
          else out.push([a, b]);
        }
      }
    }

    return out.filter(
      (p, i) => i === 0 || p[0] !== out[i - 1][0] || p[1] !== out[i - 1][1],
    );
  }

  // Great-circle distance between two points
  function haversine(a, b) {
    const R = 6371000,
      toRad = (d) => (d * Math.PI) / 180;
    const dLat = toRad(b[0] - a[0]),
      dLon = toRad(b[1] - a[1]);
    const lat1 = toRad(a[0]),
      lat2 = toRad(b[0]);
    const h =
      Math.sin(dLat / 2) ** 2 +
      Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
    return 2 * R * Math.asin(Math.sqrt(h));
  }

  // Map slope value to colour
  function slopeToColour(min, max, val) {
    const t = (val - min) / (max - min || 1);
    const hue = (1 - t) * 240;
    return `hsl(${hue},100%,50%)`;
  }

  // Convert latitude degrees to meters
  function metersPerDeg(latDeg) {
    const latRad = (latDeg * Math.PI) / 180;
    const mPerDegLat = 111320;
    const mPerDegLon = 111320 * Math.cos(latRad);
    return { mPerDegLat, mPerDegLon };
  }
  // Build quad polygon for map corridor
  function quadForSegment(c1, c2, halfWidthM) {
    const latMid = (c1[0] + c2[0]) * 0.5;
    const { mPerDegLat, mPerDegLon } = metersPerDeg(latMid);

    const vx_m = (c2[1] - c1[1]) * mPerDegLon;
    const vy_m = (c2[0] - c1[0]) * mPerDegLat;
    const L = Math.hypot(vx_m, vy_m) || 1;

    const nx = -vy_m / L,
      ny = vx_m / L;
    const dx_m = nx * halfWidthM,
      dy_m = ny * halfWidthM;

    const dLon = dx_m / mPerDegLon,
      dLat = dy_m / mPerDegLat;
    const dLonR = -dLon,
      dLatR = -dLat;

    const L1 = [c1[1] + dLon, c1[0] + dLat];
    const L2 = [c2[1] + dLon, c2[0] + dLat];
    const R2 = [c2[1] + dLonR, c2[0] + dLatR];
    const R1 = [c1[1] + dLonR, c1[0] + dLatR];
    return [L1, L2, R2, R1, L1];
  }

  // Retrieve elevation data
  async function fetchElevations(coords) {
    if (!coords || coords.length === 0) return [];
    const batchSize = 100;
    const results = new Array(coords.length).fill(0);
    for (let start = 0; start < coords.length; start += batchSize) {
      const chunk = coords.slice(start, start + batchSize);
      const locs = chunk.map((c) => `${c[0]},${c[1]}`).join("|");
      const url = `https://api.open-elevation.com/api/v1/lookup?locations=${encodeURIComponent(locs)}`;
      try {
        const r = await fetch(url);
        if (!r.ok) throw new Error(`HTTP ${r.status}`);
        const j = await r.json();
        const arr = (j.results || []).map((x) => x.elevation ?? 0);
        for (let i = 0; i < arr.length; i++) results[start + i] = arr[i];
      } catch (e) {
        console.warn("[seg3d] elevation fetch failed for chunk", e);
      }
    }
    return results;
  }

  // Render 3D segment with skin and headings
  async function render3DSegment(seg) {
    const coords = normalizeCoords(seg);
    if (coords.length < 2) {
      console.warn("[seg3d] no valid coords for segment");
      return;
    }
    init3DMap();
    if (!map3d) return;

    const elevs = await fetchElevations(coords);

    const slopes = [];
    for (let i = 0; i < coords.length - 1; i++) {
      const d = haversine(coords[i], coords[i + 1]);
      slopes.push((elevs[i + 1] - elevs[i]) / Math.max(d, 1));
    }
    const minSlope = Math.min(...slopes);
    const maxSlope = Math.max(...slopes);

    const features = [];
    const lineFeatures = [];
    const skinFeatures = [];
    for (let i = 0; i < slopes.length; i++) {
      const c1 = coords[i],
        c2 = coords[i + 1];
      const z1 = elevs[i],
        z2 = elevs[i + 1];
      const colour = slopeToColour(minSlope, maxSlope, slopes[i]);

      lineFeatures.push({
        type: "Feature",
        geometry: {
          type: "LineString",
          coordinates: [
            [c1[1], c1[0], z1],
            [c2[1], c2[0], z2],
          ],
        },
        properties: { colour },
      });

      if (i % SKIN_STRIDE === 0) {
        const ring = quadForSegment(c1, c2, SKIN_HALF_WIDTH_M);
        skinFeatures.push({
          type: "Feature",
          geometry: { type: "Polygon", coordinates: [ring] },
          properties: { colour },
        });
      }
    }

    for (const id of ["segment-skin", "segment-line"]) {
      if (map3d.getLayer(id)) map3d.removeLayer(id);
    }
    for (const id of ["segment-skin", "segment"]) {
      if (map3d.getSource(id)) map3d.removeSource(id);
    }

    map3d.addSource("segment", {
      type: "geojson",
      data: { type: "FeatureCollection", features: lineFeatures },
    });
    map3d.addLayer({
      id: "segment-line",
      type: "line",
      source: "segment",
      layout: { "line-join": "round", "line-cap": "round" },
      paint: { "line-width": 4, "line-color": "#0FFF00" },
    });

    const bounds = coords.reduce(
      (bb, c) => bb.extend([c[1], c[0]]),
      new mapboxgl.LngLatBounds(
        [coords[0][1], coords[0][0]],
        [coords[0][1], coords[0][0]],
      ),
    );
    map3d.fitBounds(bounds, { padding: 24, maxZoom: 17, duration: 400 });
    map3d.once("idle", () => {
      map3d.resize();
      if (map3d.getPitch() < 45) map3d.setPitch(60, { duration: 0 });
      if (Math.abs(map3d.getBearing()) < 1)
        map3d.setBearing(-17.6, { duration: 0 });
    });
  }

  const prevWavelet = // Entry point from server response
  window.onWaveletResponse;
  window.onWaveletResponse = function (out) {
    if (typeof prevWavelet === "function") prevWavelet(out);
    window.segmentsGlobal =
      out && Array.isArray(out.segments) ? out.segments : [];
    if (window.MAPBOX_TOKEN && window.segmentsGlobal.length)
      render3DSegment(window.segmentsGlobal[0]);
  };

  document.addEventListener("DOMContentLoaded", () => {
    const cont = document.querySelector(".segments-container");
    if (!cont) return;
    cont.addEventListener("click", (ev) => {
      const btn = ev.target.closest(".segment-btn");
      if (!btn) return;
      const idx = Number(btn.dataset.seg);
      const seg = window.segmentsGlobal && window.segmentsGlobal[idx];
      if (seg) render3DSegment(seg);
    });
  });

  window.render3DSegment = render3DSegment;
})();
