// view.js
// Utilities for map viewing and colour gradients

(function () {
  const V = window.VIEW || {};

  // Calculate great-circle distance
  function haversine(a, b) {
    const R = 6371e3,
      toRad = (d) => (d * Math.PI) / 180;
    const dφ = toRad(b[0] - a[0]),
      dλ = toRad(b[1] - a[1]);
    const φ1 = toRad(a[0]),
      φ2 = toRad(b[0]);
    const s =
      Math.sin(dφ / 2) ** 2 +
      Math.cos(φ1) * Math.cos(φ2) * Math.sin(dλ / 2) ** 2;
    return 2 * R * Math.asin(Math.sqrt(s));
  }
  // Compute cumulative distances
  function cumdist(coords) {
    const d = [0];
    let acc = 0;
    for (let i = 1; i < coords.length; i++) {
      acc += haversine(coords[i - 1], coords[i]);
      d.push(acc);
    }
    return d;
  }
  // Clamp angle to [-π, π]
  function wrapPi(rad) {
    const twoPi = 2 * Math.PI;
    let t = rad % twoPi;
    if (t > Math.PI) t -= twoPi;
    if (t <= -Math.PI) t += twoPi;
    return t;
  }

  // Convert hex colour to RGB object
  function hexToRgb(h) {
    const m = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(h);
    return m
      ? {
          r: parseInt(m[1], 16),
          g: parseInt(m[2], 16),
          b: parseInt(m[3], 16),
        }
      : null;
  }
  // Convert RGB object to hex string
  function rgbToHex({ r, g, b }) {
    const h = (n) => n.toString(16).padStart(2, "0");
    return "#" + h(r) + h(g) + h(b);
  }
  // Linear interpolation helper
  function lerp(a, b, t) {
    return a + (b - a) * t;
  }
  // Blend two colours by factor t
  function lerpColor(c1, c2, t) {
    const A = hexToRgb(c1),
      B = hexToRgb(c2);
    return rgbToHex({
      r: Math.round(lerp(A.r, B.r, t)),
      g: Math.round(lerp(A.g, B.g, t)),
      b: Math.round(lerp(A.b, B.b, t)),
    });
  }
  const stops = [
    {
      g: -30,
      c: "#0077b4",
    },
    {
      g: 0,
      c: "#3f77b4",
    },
    {
      g: 3,
      c: "#2ca02c",
    },
    {
      g: 8,
      c: "#ff7f0e",
    },
    {
      g: 10,
      c: "#d62728",
    },
    {
      g: 15,
      c: "#d62728",
    },
    {
      g: 15,
      c: "#000000",
    },
  ];
  // Determine colour based on gradient value
  function colorForGrad(g) {
    for (let i = 0; i < stops.length - 1; i++) {
      const a = stops[i],
        b = stops[i + 1];
      if (g <= a.g) return a.c;
      if (g < b.g) return lerpColor(a.c, b.c, (g - a.g) / (b.g - a.g));
    }
    return stops[stops.length - 1].c;
  }
  function colorForGrad(g) {
    if (!Number.isFinite(g)) return "#888";
    for (let i = 0; i < stops.length - 1; i++) {
      const a = stops[i],
        b = stops[i + 1];
      if (g <= a.g) return a.c;
      if (g < b.g) return lerpColor(a.c, b.c, (g - a.g) / (b.g - a.g));
    }
    return stops[stops.length - 1].c;
  }

  // Input data arrays from server
  const coords_osrm = V.coords_osrm || [];
  const coords_rs = V.coords_rs || [];
  const elev_rs = V.elev_rs || [];
  const grad_rs = V.grad_rs || [];
  const head_raw = V.heading_delta || [];
  const way_rs = V.way_rs || [];
  const speed_rs = V.speed_rs || [];
  const speed_u_skm = V.speed_u_skm || [];
  const speed_u_vals = V.speed_u_vals || [];
  const speed_u_ds_m = V.speed_u_ds_m;

  // Initialise Leaflet map if element present
  const mapEl = document.getElementById("map");
  if (mapEl && window.L) {
    const map = L.map(mapEl, {
      zoomControl: true,
    });
    const tiles = L.tileLayer(
      "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
      {
        attribution: "&copy; OpenStreetMap contributors",
      },
    ).addTo(map);
    L.control
      .scale({
        metric: true,
        imperial: false,
      })
      .addTo(map);
    // Add polyline to map
    function addPolyline(latlon, opts) {
      return L.polyline(
        latlon,
        Object.assign(
          {
            weight: 3,
            opacity: 0.9,
          },
          opts || {},
        ),
      ).addTo(map);
    }
    let bounds = null;
    if (coords_osrm.length > 1) {
      const osrmLine = addPolyline(coords_osrm, {
        color: "#d33",
      });
      bounds = osrmLine.getBounds();
    }
    if (coords_rs.length > 1) {
      const rsBase = addPolyline(coords_rs, {
        color: "#38f",
        weight: 3,
        opacity: 0.9,
      });
      bounds = bounds ? bounds.extend(rsBase.getBounds()) : rsBase.getBounds();

      if (Array.isArray(way_rs) && way_rs.length) {
        let segBounds = null;
        for (const seg of way_rs) {
          const i = Math.max(0, Math.min(coords_rs.length - 1, seg.i | 0));
          const j = Math.max(i, Math.min(coords_rs.length - 1, seg.j | 0));
          const col =
            typeof seg.color === "string" && seg.color ? seg.color : "#888";
          const pl = addPolyline(coords_rs.slice(i, j + 1), {
            color: col,
            weight: 4,
            opacity: 0.9,
          });
          if (pl)
            segBounds = segBounds
              ? segBounds.extend(pl.getBounds())
              : pl.getBounds();
        }
        if (segBounds) bounds = bounds ? bounds.extend(segBounds) : segBounds;
      }
    }
    if (bounds) map.fitBounds(bounds);
  }

  if (
    !(
      coords_rs.length > 1 &&
      elev_rs.length === coords_rs.length &&
      grad_rs.length === coords_rs.length &&
      head_raw.length === coords_rs.length
    )
  ) {
    const el = document.getElementById("plot");
    if (el)
      el.innerHTML =
        '<div style="padding:12px;color:#c00">Missing or mismatched arrays (coords/elev/grad/heading).</div>';
    return;
  }

  const dist_m = cumdist(coords_rs);
  const dist_km = dist_m.map((x) => x / 1e3);
  const n = elev_rs.length;

  const head = head_raw.map(wrapPi);
  const headingTraces = [];
  let runStart = 0;
  for (let i = 1; i < n; i++) {
    const prev = head[i - 1];
    const curr = head[i];
    if (!Number.isFinite(prev) || !Number.isFinite(curr)) continue;
    if (Math.abs(curr - prev) > Math.PI * 0.75) {
      const xseg = dist_km.slice(runStart, i);
      const yseg = head.slice(runStart, i);
      if (xseg.length && yseg.length) {
        headingTraces.push({
          x: xseg,
          y: yseg,
          mode: "lines",
          name: "heading Δ (rad)",
          hovertemplate: "d=%{x:.3f} km<br>θ=%{y:.2f} rad<extra></extra>",
          showlegend: runStart === 0 && headingTraces.length === 0,
        });
      }
      runStart = i;
    }
  }

  const finiteElev = elev_rs.filter(Number.isFinite);
  const eMin = finiteElev.length ? Math.min(...finiteElev) : 0;
  const eMax = finiteElev.length ? Math.max(...finiteElev) : 1;
  const pad = Math.max(2, (eMax - eMin) * 0.08);

  const absMaxGrad = grad_rs.reduce(
    (m, v) => (Number.isFinite(v) ? Math.max(m, Math.abs(v)) : m),
    0,
  );
  const grad_pct =
    absMaxGrad <= 1 ? grad_rs.map((g) => g * 100) : grad_rs.slice();
  const sgrad = new Array(n).fill(0);
  const W_M = 7;
  for (let i = 0; i < n; i++) {
    let sum = 0,
      wsum = 0;
    for (let k = -W_M; k <= W_M; k++) {
      const j = i + k;
      if (j < 0 || j >= n) continue;
      const wdist = Math.exp(-(Math.abs(k) / W_M));
      const wmag = 1 / (1 + Math.abs(grad_pct[j]));
      const w = wdist * wmag;
      sum += w * grad_pct[j];
      wsum += w;
    }
    sgrad[i] = wsum ? sum / wsum : grad_pct[i];
  }

  const SEG_LEN_MIN = 12;
  const SEG_LEN_MAX = 40;
  const G1 = 10;
  function segLenForGrad(g) {
    if (!Number.isFinite(g)) return (SEG_LEN_MIN + SEG_LEN_MAX) * 0.5;
    const t = Math.max(0, Math.min(1, Math.abs(g) / G1));
    return SEG_LEN_MAX - (SEG_LEN_MAX - SEG_LEN_MIN) * t;
  }

  let segStart = 0;
  const traces = [];
  while (segStart < n - 1) {
    const currLen = segLenForGrad(sgrad[segStart]);
    let i = segStart + 1;
    while (i < n && dist_m[i] - dist_m[segStart] < currLen) i++;
    if (i <= segStart) i = Math.min(segStart + 1, n - 1);
    const mid = Math.floor((segStart + i) / 2);
    const col = colorForGrad(sgrad[mid]);
    const xseg = dist_km.slice(segStart, i + 1);
    const yseg = elev_rs.slice(segStart, i + 1);
    const ybase = new Array(xseg.length).fill(eMin - pad);

    traces.push({
      x: xseg,
      y: yseg,
      mode: "lines",
      fill: "tozeroy",
      fillcolor: col,
      line: {
        color: col,
        width: 1,
      },
      name: `elev (m) - grad≈${sgrad[mid].toFixed(1)}%`,
      hovertemplate: "d=%{x:.3f} km<br>elev=%{y:.1f} m<extra></extra>",
      showlegend: false,
    });

    traces.push({
      x: xseg,
      y: ybase,
      mode: "lines",
      line: {
        color: "rgba(0,0,0,0)",
        width: 0.1,
      },
      hoverinfo: "skip",
      showlegend: false,
    });
    segStart = i;
  }

  traces.push({
    x: dist_km,
    y: elev_rs,
    mode: "lines",
    line: {
      width: 1.2,
      color: "#333",
    },
    name: "elev (outline)",
    hovertemplate: "d=%{x:.3f} km<br>elev=%{y:.1f} m<extra></extra>",
  });

  function ensureKmH(arr) {
    return (arr || []).map((v) =>
      Number.isFinite(v) ? (v > 30 ? v : v * 3.6) : v,
    );
  }
  if (Array.isArray(speed_rs) && speed_rs.length === dist_km.length) {
    traces.push({
      x: dist_km,
      y: ensureKmH(speed_rs),
      yaxis: "y2",
      mode: "lines",
      name: "Speed (km/h)",
      line: {
        width: 1.4,
        dash: "dot",
      },
      hovertemplate: "d=%{x:.3f} km<br>speed=%{y:.1f} km/h<extra></extra>",
    });
  }
  Plotly.newPlot("plot", traces, {
    title: "Elevation vs Distance (continuous-colour fill, high-grad emphasis)",
    xaxis: {
      title: "Distance (km)",
    },
    yaxis: {
      title: "Elevation (m)",
      range: [eMin - pad, eMax + pad],
    },
    yaxis2: {
      overlaying: "y",
      side: "right",
      title: "Speed (km/h)",
      showgrid: false,
    },
    legend: {
      orientation: "h",
    },
    margin: {
      t: 40,
      r: 50,
    },
  });

  const hoverMarker = window.L.marker(coords_rs[0], {
    opacity: 0,
  }).addTo(map);
  function showMarker(latlng) {
    hoverMarker.setLatLng(latlng).setOpacity(1);
  }
  function hideMarker() {
    hoverMarker.setOpacity(0);
  }
  function latLngAtDistance(d) {
    if (d <= 0) return coords_rs[0];
    const total = dist_m[dist_m.length - 1];
    if (d >= total) return coords_rs[coords_rs.length - 1];
    let lo = 0,
      hi = dist_m.length - 1;
    while (lo + 1 < hi) {
      const mid = (lo + hi) >> 1;
      if (dist_m[mid] <= d) lo = mid;
      else hi = mid;
    }
    const seg = dist_m[hi] - dist_m[lo] || 1;
    const t = (d - dist_m[lo]) / seg;
    const lat = coords_rs[lo][0] + t * (coords_rs[hi][0] - coords_rs[lo][0]);
    const lon = coords_rs[lo][1] + t * (coords_rs[hi][1] - coords_rs[lo][1]);
    return [lat, lon];
  }
  function wireHover(divId) {
    const el = document.getElementById(divId);
    if (!el || typeof el.on !== "function") return;
    el.on("plotly_hover", (ev) => {
      if (!ev.points || !ev.points.length) return;
      const xkm = ev.points[0].x;
      if (typeof xkm !== "number" || !isFinite(xkm)) return;
      showMarker(latLngAtDistance(xkm * 1e3));
    });
    el.on("plotly_unhover", () => hideMarker());
    el.on("plotly_click", (ev) => {
      if (!ev.points || !ev.points.length) return;
      const xkm = ev.points[0].x;
      if (typeof xkm !== "number" || !isFinite(xkm)) return;
      showMarker(latLngAtDistance(xkm * 1e3));
    });
  }

  wireHover("plot");

  function stepStats(coords) {
    if (!Array.isArray(coords) || coords.length < 2) return null;
    const steps = [];
    for (let i = 1; i < coords.length; i++)
      steps.push(haversine(coords[i - 1], coords[i]));
    const s = [...steps].sort((a, b) => a - b),
      q = (p) => s[Math.floor((s.length - 1) * p)];
    const avg = steps.reduce((a, b) => a + b, 0) / steps.length;
    return {
      n: steps.length,
      min: s[0],
      p50: q(0.5),
      p95: q(0.95),
      max: s[s.length - 1],
      avg: avg,
    };
  }
  const diagEl = document.getElementById("diag");
  if (diagEl) {
    const os = stepStats(coords_osrm) || {
      n: 0,
      min: 0,
      p50: 0,
      p95: 0,
      max: 0,
      avg: 0,
    };
    const rs = stepStats(coords_rs) || {
      n: 0,
      min: 0,
      p50: 0,
      p95: 0,
      max: 0,
      avg: 0,
    };
    diagEl.textContent = `OSRM: pts=${coords_osrm.length} steps=${os.n}  min=${(os.min || 0).toFixed(2)}m p50=${(os.p50 || 0).toFixed(2)}m p95=${(os.p95 || 0).toFixed(2)}m max=${(os.max || 0).toFixed(2)}m avg=${(os.avg || 0).toFixed(2)}m
RS  : pts=${coords_rs.length}   steps=${rs.n}  min=${(rs.min || 0).toFixed(2)}m p50=${(rs.p50 || 0).toFixed(2)}m p95=${(rs.p95 || 0).toFixed(2)}m max=${(rs.max || 0).toFixed(2)}m avg=${(rs.avg || 0).toFixed(2)}m`;
  }
})();
