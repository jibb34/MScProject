/* signal_lab.js (cleaned, Plotly-free; upload fix + canvas chart)
 * - Uploads list: polls /lab/list every 3s and remembers selection (localStorage 'lastMap')
 * - Upload: sends RAW JSON (Content-Type: application/json) expected by the server
 * - Resample: POST /lab/resamplee updates stats (text only), draws a canvas chart, emits 'resample-data'
 * - Tabs: simple tab switcher (Stats, Wavelet placeholder, Segments placeholder, Route map)
 * - Route tab: Leaflet map overlay using /view?map=... (OSRM match=red, RouteSignal=blue)
 * - Defensive coding: null checks; //NOTE markers for optional future hooks
 */

(() => {
  // ===== Elements =====
  const els = {
    mapSelect:  document.getElementById('mapSelect'),
    runBtn:     document.getElementById('runBtn'),
    dsInput:    document.getElementById('dsInput'),
    varSelect:  document.getElementById('varSelect'),
    kindSelect: document.getElementById('kindSelect'),
    uploadFile: document.getElementById('uploadFile'),
    uploadBtn:  document.getElementById('uploadBtn'),
    uploadMsg:  document.getElementById('uploadMsg'),
    plotUni:    document.getElementById('plotUni'),   // Canvas chart drawn here
    tabs:       document.getElementById('labTabs'),
    panels:     document.getElementById('labPanels'),
    statsBox:   document.getElementById('statsSummary'),
    routeMap:   document.getElementById('routeMap'),
    openView:   document.getElementById('openViewLink'),
    log:        document.getElementById('log')
  };

  function initTooltips() {
    const tips = {
      wf_ds:       'Sample spacing ds (m). Smaller = more detail, noisier.',
      wf_LT:       'Trend window L_T (m). Larger = smoother blue trend; smaller = faster response.',
      wf_LE:       'Energy window L_E (m). Larger = less spiky energy; smaller = sharper bursts.',
      wf_lam_min:  'Rolling band λ_min (m). Lower = include faster bumps; higher = ignore very short ones.',
      wf_lam_max:  'Rolling band λ_max (m). Lower = focus on short rollers; higher = include longer undulations.',
      wf_E_env:    'Envelope window (m). Larger merges nearby spikes into one plateau.',
      wf_hyst_en:  'Enable hysteresis grouping. Off = raw envelope only.',
      wf_E_hi:     'Start threshold (MAD). Higher = fewer starts; lower = starts easier.',
      wf_E_lo:     'Sustain threshold (MAD). Lower = stays ON longer; higher = ends sooner.',
      wf_E_gap:    'Gap close (m). Bridge dips shorter than this while ON.',
      wf_E_min:    'Min run (m). Discard ON runs shorter than this.',
      wf_fill:     'Fill background with energy shading.',
    };

    Object.entries(tips).forEach(([id, text]) => {
      const el = document.getElementById(id);
      if (!el) return;
      el.title = text;                         // tooltip on the input
      const row = el.closest('.row');          // also on the whole row and label
      if (row) row.title = text;
      const lab = row && row.querySelector('label');
      if (lab) lab.title = text;
    });
  }
  document.addEventListener('DOMContentLoaded', initTooltips);

  // ===== Utilities =====
  /** Read wavelet terrain params from the panel */
  /** Wavelet Terrain parameters (robust to missing inputs: falls back to defaults). */

  function setHystUI(on) {
    ['wf_E_hi','wf_E_lo','wf_E_gap','wf_E_min'].forEach(id => {
      const el = document.getElementById(id);
      if (el) el.disabled = !on;
    });
    document.querySelectorAll('[data-hyst-row]').forEach(r => {
      r.classList.toggle('disabled', !on);
    });
  }

  document.addEventListener('DOMContentLoaded', () => {
    const t = document.getElementById('wf_hyst_en');
    if (t) {
      setHystUI(t.checked);
      t.addEventListener('change', e => setHystUI(e.target.checked));
    }
  });

  function getTerrainParams() {
    const num = (id, def, clamp) => {
      const el = document.getElementById(id);
      let v = el ? Number(el.value) : NaN;
      if (!Number.isFinite(v)) v = def;
      if (clamp && Array.isArray(clamp)) {
        const [lo, hi] = clamp;
        if (Number.isFinite(lo)) v = Math.max(lo, v);
        if (Number.isFinite(hi)) v = Math.min(hi, v);
      }
      return v;
    };
    const bool = (id) => !!(document.getElementById(id)?.checked);
    const hyst = bool('wf_hyst_en');

    return {
      ds_m:  num('wf_ds', 5, [0.1, 10000]),
      L_T:   num('wf_LT', 150, [2, 100000]),
      fill:  bool('wf_fill'),
      L_E: num('wf_LE', 120, [4, 100000]),
      lambda_min: num('wf_lam_min', 50, [1, 1e6]),
      lambda_max: num('wf_lam_max', 200, [1, 1e6]),
      energy_mode: 0, // 0=FFT, 1=HAAR when server supports HAAR
      E_env_m:       num('wf_E_env', 200, [0, 5000]),
      E_use_hysteresis: hyst,
      // only read numbers when enabled (undefined drops from JSON)
      E_hyst_hi:     hyst ? num('wf_E_hi',  2.0, [0,10]) : undefined,
      E_hyst_lo:     hyst ? num('wf_E_lo',  1.0, [0,10]) : undefined,
      E_gap_close_m: hyst ? num('wf_E_gap', 30,  [0,2000]) : undefined,
      E_min_run_m:   hyst ? num('wf_E_min', 80,  [0,5000]) : undefined,

    };
  }


  function normalizeTerrainCodes(arr) {
    if (!Array.isArray(arr)) return [];
    const out = new Array(arr.length);
    for (let i = 0; i < arr.length; i++) {
      const v = Number(arr[i]);
      const c = Math.round(Number.isFinite(v) ? v : 0);
      out[i] = (c < 0) ? 0 : (c > 3 ? 3 : c);
    }
    return out;
  }

  
  /** Set the upload status text + color */
  function setUploadStatus(text, kind /* 'ok' | 'err' | 'info' */) {
    if (!els.uploadMsg) return;
    els.uploadMsg.textContent = text || '';
    els.uploadMsg.classList.remove('is-ok', 'is-err', 'is-info');
    els.uploadMsg.classList.add(kind ? `is-${kind}` : 'is-info');
  }


  /** Append a line to the debug log */
  function appendLog(msg) {
    if (!els.log) return;
    const ts = new Date().toISOString().replace('T',' ').replace('Z','');
    els.log.textContent += `[${ts}] ${msg}\n`;
    els.log.scrollTop = els.log.scrollHeight;
  }

  /** Human-friendly byte size */
  function humanSize(bytes) {
    if (typeof bytes !== 'number') return '';
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / 1024 / 1024).toFixed(2)} MB`;
  }

  /** Lightweight Leaflet loader (if not already included in HTML) */
  function ensureLeaflet() {
    if (window.L) return Promise.resolve();
    return new Promise((resolve, reject) => {
      const link = document.createElement('link');
      link.rel = 'stylesheet';
      link.href = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.css';
      document.head.appendChild(link);
      const s = document.createElement('script');
      s.src = 'https://unpkg.com/leaflet@1.9.4/dist/leaflet.js';
      s.onload = () => resolve();
      s.onerror = () => reject(new Error('Leaflet failed to load'));
      document.head.appendChild(s);
    });
  }

  // ===== Uploads list =====

  /** Replace #mapSelect options from server list [{file,name,bytes}] */
  function updateMapOptions(files) {
    if (!els.mapSelect) return;
    const prev = els.mapSelect.value || localStorage.getItem('lastMap') || '';
    els.mapSelect.innerHTML = '';
    (files || []).forEach(f => {
      if (!f || !f.file || !f.name) return;
      const opt = document.createElement('option');
      opt.value = String(f.file);            // e.g. "uploads/xxx.json"
      const size = humanSize(f.bytes);
      opt.textContent = size ? `${f.name} - ${size}` : f.name;
      els.mapSelect.add(opt);
    });
    if (prev && [...els.mapSelect.options].some(o => o.value === prev)) {
      els.mapSelect.value = prev;
    }
    if (els.mapSelect.value) {
      localStorage.setItem('lastMap', els.mapSelect.value);
    }
  }

  /** GET /lab/list and refresh dropdown */
  async function refreshUploads(preferPath = null) {
    try {
      const r = await fetch('/lab/list');
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      const data = await r.json();
      const files = Array.isArray(data?.files) ? data.files : [];
      updateMapOptions(files);
      if (preferPath && els.mapSelect && [...els.mapSelect.options].some(o => o.value === preferPath)) {
        els.mapSelect.value = preferPath;
        localStorage.setItem('lastMap', preferPath);
      }
    } catch (e) {
      appendLog(`[list] failed: ${e.message}`);
      console.error(e);
    }
  }

  /** Called by upload flow after server returns { file: "uploads/..." } */
  async function handleUploadResponse(json) {
    try {
      if (!json || typeof json.file !== 'string') throw new Error('Upload response missing "file"');
      await refreshUploads(json.file);
      if (els.uploadMsg) setUploadStatus(`Uploaded & selected ${json.file}`, 'ok');

    } catch (e) {
      setUploadStatus(`[upload] ${e.message}`, 'err');
      appendLog(`[upload] error: ${e.message}`);
    }
  }

  // ===== Stats (text only) =====

  function _finite(arr){ return (Array.isArray(arr) ? arr : []).filter(Number.isFinite); }
  function _quantile(sorted, q){
    if (!sorted.length) return NaN;
    const i = (sorted.length - 1) * q;
    const lo = Math.floor(i), hi = Math.ceil(i);
    return (lo === hi) ? sorted[lo] : (sorted[lo] * (hi - i) + sorted[hi] * (i - lo));
  }
  function _mean(a){ const v=_finite(a); return v.length ? v.reduce((s,x)=>s+x,0)/v.length : NaN; }
  function _variance(a){ const v=_finite(a); if(v.length<2) return NaN; const m=_mean(v); return v.reduce((s,x)=>s+(x-m)*(x-m),0)/(v.length-1); }
  function _std(a){ const vv=_variance(a); return Number.isFinite(vv) ? Math.sqrt(vv) : NaN; }
  function _deg(rad){ return rad * 180/Math.PI; }
  function _wrapPi(rad){
    const twoPi = 2*Math.PI;
    let t = rad % twoPi;
    if (t > Math.PI) t -= twoPi;
    if (t <= -Math.PI) t += twoPi;
    return t;
  }
  function _circularStats(radArray){
    const v = _finite(radArray).map(_wrapPi);
    const n = v.length;
    if (!n) return { mean: NaN, std: NaN, var: NaN, n: 0, Rbar: NaN };
    let C=0, S=0;
    for (const a of v){ C += Math.cos(a); S += Math.sin(a); }
    const mean = Math.atan2(S, C);
    const R = Math.hypot(C, S);
    const Rbar = R / n;
    const circVar = 1 - Rbar;
    const circStd = Math.sqrt(-2 * Math.log(Rbar || Number.EPSILON));
    return { mean, std: circStd, var: circVar, n, Rbar };
  }
  function computeStats(values, kind){
    const v = _finite(values);
    const n = v.length, missing = (Array.isArray(values) ? values.length : 0) - v.length;
    if (!n) return { n:0, missing, mean:NaN, std:NaN, var:NaN };
    if ((kind||'scalar') === 'angle') {
      const c = _circularStats(v);
      return { n, missing, mean: _deg(c.mean), std: _deg(c.std), var: c.var, rbar: c.Rbar };
    } else {
      const s = [...v].sort((a,b)=>a-b);
      const mean = _mean(v);
      const variance = _variance(v);
      const std = _std(v);
      const q25 = _quantile(s, 0.25);
      const med = _quantile(s, 0.50);
      const q75 = _quantile(s, 0.75);
      return { n, missing, min: s[0], q25, median: med, q75, max: s[s.length-1], mean, var: variance, std };
    }
  }
  function guessKind(name){ return /heading/.test(String(name||'')) ? 'angle' : 'scalar'; }

  /** Renders numeric stats only (no Plotly) */
  function renderStats(values, vname, maybeKind){
    const kind = (maybeKind && String(maybeKind).toLowerCase()) || guessKind(vname);
    const stats = computeStats(values, kind);
    if (els.statsBox) {
      const fmt = (x) => (Number.isFinite(x) ? (+x.toFixed(4)).toString() : '-');
      const cells = (pairs) => pairs.map(([label,val]) =>
        `<div class="badge"><span class="label">${label}</span><span class="value">${fmt(val)}</span></div>`
      ).join('');
      els.statsBox.innerHTML = (kind === 'angle')
        ? cells([ ['Samples', stats.n], ['Missing', stats.missing], ['Mean (°)', stats.mean], ['Std (°)', stats.std], ['CircVar', stats.var], ['R̄', stats.rbar] ])
        : cells([ ['Samples', stats.n], ['Missing', stats.missing], ['Mean', stats.mean], ['Std', stats.std], ['Var', stats.var], ['Min', stats.min], ['Q25', stats.q25], ['Median', stats.median], ['Q75', stats.q75], ['Max', stats.max] ]);
    }
    //NOTE: For a tiny sparkline without Plotly, we draw a canvas chart below.
  }

  // ===== Minimal Canvas Line Chart (in #plotUni) =====

  // ===== Minimal Canvas Line Chart (in #plotUni) with hover =====
  function renderLineChart(x, y, title = '') {
    if (!els.plotUni) return;

    // Create/clear canvas
    let canvas = els.plotUni.querySelector('canvas');
    if (!canvas) {
      canvas = document.createElement('canvas');
      canvas.width = els.plotUni.clientWidth || 800;
      canvas.height = els.plotUni.clientHeight || 240;
      els.plotUni.appendChild(canvas);
    } else {
      const w = els.plotUni.clientWidth || canvas.width;
      const h = els.plotUni.clientHeight || canvas.height;
      canvas.width = w;   // reset clears the canvas
      canvas.height = h;
    }
    const ctx = canvas.getContext('2d');

    // Filter finite pairs
    const pts = [];
    for (let i = 0; i < Math.min(x.length, y.length); i++) {
      const xi = x[i], yi = y[i];
      if (Number.isFinite(xi) && Number.isFinite(yi)) pts.push([xi, yi]);
    }
    if (!pts.length) return;

    // Bounds
    const xmin = Math.min(...pts.map(p => p[0]));
    const xmax = Math.max(...pts.map(p => p[0]));
    const ymin = Math.min(...pts.map(p => p[1]));
    const ymax = Math.max(...pts.map(p => p[1]));

    // Margins + scales
    const padL = 48, padR = 16, padT = 24, padB = 28;
    const W = canvas.width, H = canvas.height;
    const plotW = Math.max(10, W - padL - padR);
    const plotH = Math.max(10, H - padT - padB);
    const sx = (xv) => padL + (xmax === xmin ? 0 : ((xv - xmin) / (xmax - xmin)) * plotW);
    const sy = (yv) => padT + plotH - (ymax === ymin ? 0 : ((yv - ymin) / (ymax - ymin)) * plotH);

    // -- base draw (axes, line, ticks, title)
    function drawBase() {
      // Background
      ctx.fillStyle = '#fff';
      ctx.fillRect(0, 0, W, H);

      // Axes
      ctx.strokeStyle = '#333';
      ctx.lineWidth = 1;
      // x-axis
      ctx.beginPath(); ctx.moveTo(padL, H - padB); ctx.lineTo(W - padR, H - padB); ctx.stroke();
      // y-axis
      ctx.beginPath(); ctx.moveTo(padL, padT); ctx.lineTo(padL, H - padB); ctx.stroke();

      // Title
      if (title) {
        ctx.fillStyle = '#111';
        ctx.font = 'bold 12px system-ui, sans-serif';
        ctx.fillText(title, padL, padT - 8);
      }

      // Line
      ctx.strokeStyle = '#0a66ff';
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(sx(pts[0][0]), sy(pts[0][1]));
      for (let i = 1; i < pts.length; i++) ctx.lineTo(sx(pts[i][0]), sy(pts[i][1]));
      ctx.stroke();

      // Simple ticks (min/mid/max)
      ctx.fillStyle = '#555';
      ctx.font = '10px system-ui, sans-serif';
      const xticks = [xmin, (xmin + xmax) / 2, xmax];
      const yticks = [ymin, (ymin + ymax) / 2, ymax];
      xticks.forEach(v => {
        const px = sx(v);
        ctx.fillText(v.toFixed(2), px - 8, H - padB + 12);
        ctx.beginPath(); ctx.moveTo(px, H - padB); ctx.lineTo(px, H - padB + 4); ctx.stroke();
      });
      yticks.forEach(v => {
        const py = sy(v);
        ctx.fillText(v.toFixed(2), 6, py + 3);
        ctx.beginPath(); ctx.moveTo(padL - 4, py); ctx.lineTo(padL, py); ctx.stroke();
      });

      // Underlay: let callers paint inside the plot area BEFORE the line is drawn

      if (typeof window.__plotOverlayDrawer === 'function') {
        try {
          window.__plotOverlayDrawer(ctx, { sx, sy, padL, padR, padT, padB, W, H, plotW, plotH, xmin, xmax, ymin, ymax });
        } catch (e) { console.warn('overlay drawer error (base):', e); }
      }

    }

    // Precompute x pixels (assumes s_km mostly increases; if not, we fallback to linear scan)
    const xpx = pts.map(p => sx(p[0]));
    const monoIncreasing = pts.length > 2 && (pts[1][0] - pts[0][0]) * (pts[2][0] - pts[1][0]) >= 0;

    function nearestIndex(px) {
      if (monoIncreasing) {
        // binary search on xpx
        let lo = 0, hi = xpx.length - 1;
        while (lo < hi) {
          const mid = (lo + hi) >> 1;
          if (xpx[mid] < px) lo = mid + 1; else hi = mid;
        }
        // compare lo and lo-1
        const i2 = Math.max(0, lo - 1);
        return (Math.abs(xpx[lo] - px) < Math.abs(xpx[i2] - px)) ? lo : i2;
      }
      // fallback linear
      let best = 0, bestd = Infinity;
      for (let i = 0; i < xpx.length; i++) {
        const d = Math.abs(xpx[i] - px);
        if (d < bestd) { bestd = d; best = i; }
      }
      return best;
    }

    function drawHover(mx, my) {
      drawBase();

      // Find nearest point by x-pixel
      const idx = nearestIndex(mx);
      const [vx, vy] = pts[idx];
      const px = sx(vx), py = sy(vy);

      // Crosshair
      ctx.strokeStyle = 'rgba(0,0,0,.25)';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(px, padT); ctx.lineTo(px, H - padB); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(padL, py); ctx.lineTo(W - padR, py); ctx.stroke();

      // Point highlight
      ctx.fillStyle = '#0a66ff';
      ctx.beginPath(); ctx.arc(px, py, 3, 0, Math.PI * 2); ctx.fill();

      // Tooltip
      const text1 = `s: ${vx.toFixed(3)} km`;
      const text2 = `y: ${vy.toFixed(3)}`;
      ctx.font = '11px system-ui, sans-serif';
      const pad = 6;
      const tw = Math.max(ctx.measureText(text1).width, ctx.measureText(text2).width);
      const th = 14 + 14 + 6; // two lines + padding
      let bx = px + 10, by = py - th - 10;
      if (bx + tw + pad * 2 > W - padR) bx = px - 10 - tw - pad * 2;
      if (by < padT) by = py + 10;

      ctx.fillStyle = 'rgba(255,255,255,0.95)';
      ctx.fillRect(bx, by, tw + pad * 2, th);
      ctx.strokeStyle = '#333';
      ctx.strokeRect(bx, by, tw + pad * 2, th);

      ctx.fillStyle = '#111';
      ctx.fillText(text1, bx + pad, by + 14);
      ctx.fillText(text2, bx + pad, by + 28);
      // drawHover end - re-apply overlay on top
      // Underlay: let callers paint inside the plot area BEFORE the line is drawn
      if (typeof window.__plotUnderlayDrawer === 'function') {
        try {
          window.__plotUnderlayDrawer(ctx, { sx, sy, padL, padR, padT, padB, W, H, plotW, plotH, xmin, xmax, ymin, ymax });
        } catch (e) { console.warn('underlay drawer error (base):', e); }
      }

    }

    // Initial draw and wire hover handlers
    drawBase();

    canvas.onmousemove = (ev) => {
      const rect = canvas.getBoundingClientRect();
      const mx = ev.clientX - rect.left;
      const my = ev.clientY - rect.top;
      // only hover inside plot area
      if (mx >= padL && mx <= W - padR && my >= padT && my <= H - padB) {
        drawHover(mx, my);
      } else {
        drawBase();
      }
    };
    canvas.onmouseleave = () => drawBase();
  }
  function setOverlayMulti(lines) {
  // lines: [{x:[], y:[], dash:[4,3] | [], width: number, label: string}, ...]
  if (!Array.isArray(lines) || lines.length === 0) {
    window.__plotOverlayDrawer = null;
    return;
  }
  // compute shared y2 range
  let y2Min = +Infinity, y2Max = -Infinity;
  for (const L of lines) {
    if (!Array.isArray(L.y) || !L.y.length) continue;
    for (const v of L.y) { if (v < y2Min) y2Min = v; if (v > y2Max) y2Max = v; }
  }
  if (!isFinite(y2Min) || !isFinite(y2Max) || y2Min === y2Max) {
    y2Min -= 1; y2Max += 1;
  }
  window.__plotOverlayDrawer = function overlay(ctx, s) {
    const { sx, padL, padR, padT, padB, W, H } = s;
    const plotH = Math.max(10, H - padT - padB);
    const sy2 = (v) => H - padB - plotH * (v - y2Min) / ((y2Max - y2Min) || 1);

    // right axis
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(W - padR, padT); ctx.lineTo(W - padR, H - padB); ctx.stroke();

    // each series
    for (const L of lines) {
      if (!Array.isArray(L.x) || !Array.isArray(L.y) || L.x.length !== L.y.length) continue;
      ctx.save();
      ctx.setLineDash(L.dash || []);
      ctx.lineWidth = L.width || 1.1;
      ctx.strokeStyle = '#000';
      ctx.beginPath();
      ctx.moveTo(sx(L.x[0]), sy2(L.y[0]));
      for (let i = 1; i < L.x.length; i++) ctx.lineTo(sx(L.x[i]), sy2(L.y[i]));
      ctx.stroke();
      ctx.restore();
    }
  };
}


    /** Draw elevation + overlaid line (right axis). */
  function renderElevationWithOverlayLine(x, yElev, title, x2, y2, y2Label = 'Haar trend') {
    if (!els.plotUni) return;

    // canvas
    let canvas = els.plotUni.querySelector('canvas');
    if (!canvas) {
      canvas = document.createElement('canvas');
      canvas.width = els.plotUni.clientWidth || 800;
      canvas.height = els.plotUni.clientHeight || 260;
      els.plotUni.appendChild(canvas);
    } else {
      const w = els.plotUni.clientWidth || canvas.width;
      const h = els.plotUni.clientHeight || canvas.height;
      canvas.width = w; canvas.height = h;
    }
    const ctx = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    const padL = 44, padR = 50, padT = 24, padB = 26;

    // bg
    ctx.fillStyle = '#fff'; ctx.fillRect(0,0,W,H);

    const N  = Math.max(0, Math.min(x?.length||0, yElev?.length||0));
    const N2 = Math.max(0, Math.min(x2?.length||0, y2?.length||0));
    if (!N) return;

    const xMin = 0, xMax = Math.max(1e-9, Math.max(...x));
    const y1Min = Math.min(...yElev), y1Max = Math.max(...yElev);
    const y2Min = N2 ? Math.min(...y2) : 0, y2Max = N2 ? Math.max(...y2) : 1;

    const sx  = (v) => padL + (W - padL - padR) * (v - xMin) / (xMax - xMin || 1);
    const sy1 = (v) => H - padB - (H - padT - padB) * (v - y1Min) / ((y1Max - y1Min) || 1);
    const sy2 = (v) => H - padB - (H - padT - padB) * (v - y2Min) / ((y2Max - y2Min) || 1);

    // axes
    ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(padL, H - padB); ctx.lineTo(W - padR, H - padB); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(padL, padT); ctx.lineTo(padL, H - padB); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(W - padR, padT); ctx.lineTo(W - padR, H - padB); ctx.stroke();

    if (title) { ctx.fillStyle = '#111'; ctx.font = 'bold 12px system-ui,sans-serif'; ctx.fillText(title, padL, padT - 8); }

    // elevation
    ctx.strokeStyle = '#0a66ff'; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(sx(x[0]), sy1(yElev[0]));
    for (let i = 1; i < N; i++) ctx.lineTo(sx(x[i]), sy1(yElev[i]));
    ctx.stroke();

    // overlay
    if (N2) {
      ctx.strokeStyle = '#000'; ctx.setLineDash([4,3]); ctx.lineWidth = 1.2;
      ctx.beginPath(); ctx.moveTo(sx(x2[0]), sy2(y2[0]));
      for (let i = 1; i < N2; i++) ctx.lineTo(sx(x2[i]), sy2(y2[i]));
      ctx.stroke(); ctx.setLineDash([]);
      ctx.fillStyle = '#444'; ctx.font = '11px system-ui,sans-serif';
      ctx.fillText(y2Label, W - padR + 6, padT + 10);
    }
  }


  /** Draw elevation with a categorical overlay (terrain codes 0..3) using its own y2 scale.
 *  - Overlays a colored step line for codes 0=Flat,1=Uphill,2=Downhill,3=Rolling.
 *  - Draws a right-side y-axis labeled with those categories (separate from elevation scale).
 *  - No adjustable height or modes; always spans the full plot height.
 */

  /** Draw elevation with a categorical overlay (terrain codes 0..3) using its own y2 scale (right).
 *  Persists during hover by installing window.__plotOverlayDrawer.
 */


  /** Colorize the area under the elevation profile by terrain code (0..3).
  *  Paints as an UNDERLAY (before the line), so the line stays crisp on top.
  *  overlay = { codes: number[] } with values in {0,1,2,3}.
  */
  function renderElevationWithOverlay(x, yElev, title, overlay) {
    // Align lengths
    const N = Math.max(0, Math.min(
      Array.isArray(x) ? x.length : 0,
      Array.isArray(yElev) ? yElev.length : 0,
      overlay && Array.isArray(overlay.codes) ? overlay.codes.length : 0
    ));
    const X = (Array.isArray(x) ? x : []).slice(0, N);
    const Y = (Array.isArray(yElev) ? yElev : []).slice(0, N);
    const C = (overlay && Array.isArray(overlay.codes) ? overlay.codes : []).slice(0, N);

    // Install UNDERLAY (drawn before the line) and clear any old overlay hook
    window.__plotOverlayDrawer = null;
    window.__plotUnderlayDrawer = function underlayDrawer(ctx, s) {
      if (!X.length) return;

      const { sx, sy, padL, padR, padT, padB, W, H } = s;
      const plotW = Math.max(10, W - padL - padR);
      const plotH = Math.max(10, H - padT - padB);
      const yBase = padT + plotH;

      const COLORS = {
        0: '#9aa0a6', // Flat
        1: '#d33',    // Uphill
        2: '#38f',    // Downhill
        3: '#ff8c00', // Rolling
      };

      // Clip to plot area so we fill only under the curve
      ctx.save();
      ctx.beginPath();
      ctx.rect(padL, padT, plotW, plotH);
      ctx.clip();

      // Pre-normalize codes (defensive)
      const CN = normalizeTerrainCodes(C);

      // Draw contiguous runs as single polygons

      // Draw contiguous runs as single polygons
      let i = 0;
      while (i < X.length) {
        const code = CN[i];
        let j = i + 1;
        while (j < X.length && CN[j] === code) j++;

        const col = COLORS.hasOwnProperty(code) ? COLORS[code] : '#888';

        // Use save/restore to sandbox alpha & fill style (no need for oldAlpha var)
        ctx.save();
        ctx.globalAlpha = 0.28;        // adjust opacity to taste
        ctx.fillStyle = col;

        ctx.beginPath();
        ctx.moveTo(sx(X[i]), yBase);
        for (let k = i; k < j; k++) {
          const yk = Number.isFinite(Y[k]) ? sy(Y[k]) : yBase;
          ctx.lineTo(sx(X[k]), yk);
        }
        ctx.lineTo(sx(X[j - 1]), yBase);
        ctx.closePath();
        ctx.fill();

        ctx.restore();  // restores alpha & fillStyle for next run

        i = j;
      }


      ctx.restore();
    };


    // Draw base chart; underlay will render inside renderLineChart (before the line)
    renderLineChart(X, Y, title || 'Elevation');
  }


  // ===== Resample flow =====

  /** POST /lab/resample with selected file + params */
  async function runResample() {
    const mapPath = els.mapSelect ? els.mapSelect.value : '';
    if (!mapPath) { alert('Please select a file first.'); return; }

    const ds_m  = Number(els.dsInput?.value || 5);
    const vname = String(els.varSelect?.value || 'elev');
    const payload = { map: mapPath, ds_m, vars: [vname] };

    appendLog(`[run] map=${mapPath} var=${vname} ds=${ds_m}`);

    const r = await fetch('/lab/resample', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    const bodyText = await r.text();
    if (!r.ok) {
      appendLog(`[runResample] POST /lab/resample -> ${r.status}: ${bodyText}`);
      return;
    }

    let j = {};
    try { j = JSON.parse(bodyText); } catch { j = {}; }

    const x = Array.isArray(j.s_km) ? j.s_km : [];
    const y = (j.series && Array.isArray(j.series[vname])) ? j.series[vname] : [];

    // Stats (text only)
    renderStats(y, vname, els.kindSelect?.value);

    // Canvas chart
    renderLineChart(x, y, `Uniform ${vname} (Δs≈${Number(j.ds_m ?? ds_m).toFixed(1)} m)`);

    // Keep the last elevation for overlays (x in km, y in meters)
    window.__lastElevation = {
      x: (Array.isArray(j.s_km) ? j.s_km.slice() : []),
      y: (j.series && Array.isArray(j.series.elev) ? j.series.elev.slice() : []),
      title: 'Elevation'
    };


    // Emit for any external listeners
    document.dispatchEvent(new CustomEvent('resample-data', {
      detail: { x, y, vname, ds_m: j.ds_m, json: j }
    }));

    // If Route tab is visible, refresh Leaflet overlay
    const routePanel = document.getElementById('panel-route');
    if (routePanel && !routePanel.hidden && els.mapSelect?.value) {
      loadRouteLeaflet(els.mapSelect.value);
    }
  }

/** Draw elevation with a categorical overlay (terrain codes 0..3). */
/** Call /lab/resample for the wavelet terrain and overlay it on the elevation plot. */

  async function runWaveletTerrain() {
    const sel = els.mapSelect?.value;
    if (!sel) { alert('Please select a file first.'); return; }

    const p = getTerrainParams();
    const payload = { map: sel, fn: "terrain", params: { terrain: p } };
    appendLog(`[wavelet] POST /wavelet fn=terrain map=${sel}`);

    const r = await fetch('/wavelet', {
      method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(payload)
    });
    const txt = await r.text();
    if (!r.ok) { appendLog(`[wavelet] HTTP ${r.status}: ${txt}`); alert(`Wavelet request failed: ${txt}`); return; }
    let j; try { j = JSON.parse(txt); } catch { appendLog('[wavelet] invalid JSON'); alert('Wavelet response was not JSON'); return; }

    const xWave = Array.isArray(j.s_km_uniform) && j.s_km_uniform.length ? j.s_km_uniform
                : (Array.isArray(j.s_km) ? j.s_km : []);
    const base = window.__lastElevation || { x: xWave, y: new Array(xWave.length).fill(0), title: 'Elevation' };

    const series = j.series || {};
    const trend  = series.haar_trend;
    const energy = series.energy || series.energy_fft; // backend key either name
    const showE  = document.getElementById('wf_showE')?.checked;
    const lines  = [];

    if (Array.isArray(trend) && trend.length === xWave.length) {
      lines.push({ x: xWave, y: trend, dash: [4,3], width: 1.2, label: 'Haar trend' });
    }
    if (showE && Array.isArray(energy) && energy.length === xWave.length) {
      lines.push({ x: xWave, y: energy, dash: [], width: 1.0, label: 'Rolling energy' });
    }

    setOverlayMulti(lines);
    renderLineChart(base.x, base.y, base.title);


    if (p.fill && Array.isArray(series.terrain) && series.terrain.length === xWave.length) {
      renderElevationWithOverlay(base.x, base.y, base.title, { codes: normalizeTerrainCodes(series.terrain) });
    }


    appendLog('[wavelet] done');
  }


  // ===== Tabs: simple switcher =====

  function initTabs() {
    if (!els.tabs) return;
    els.tabs.addEventListener('click', (e) => {
      const btn = e.target.closest('button[data-tab]');
      if (!btn || btn.disabled) return;
      const name = btn.getAttribute('data-tab');

      // buttons
      els.tabs.querySelectorAll('button[data-tab]').forEach(b => b.classList.toggle('active', b === btn));
      // panels
      document.querySelectorAll('#labPanels .panel').forEach(p => {
        const active = p.id === `panel-${name}`;
        p.hidden = !active;
        p.classList.toggle('active', active);
      });

      // Route tab activation: ensure map renders
      if (name === 'route' && els.mapSelect?.value) {
        loadRouteLeaflet(els.mapSelect.value);
      }
    });
  }

  // ===== Route tab (Leaflet map overlay) =====

  async function loadRouteLeaflet(mapPath) {
    await ensureLeaflet();  // guarantees window.L exists
    if (els.openView) els.openView.href = `/view?map=${encodeURIComponent(mapPath)}`;

    const url = `/view?map=${encodeURIComponent(mapPath)}&t=${Date.now()}`;
    const r = await fetch(url);
    if (!r.ok) { appendLog(`[route] GET ${r.status}`); return; }
    const html = await r.text();

    function getArrayFromView(name) {
      const re = new RegExp(String.raw`(?:var|let|const)?\s*${name}\s*=\s*(\[[\s\S]*?\])\s*;`, 'g');
      let m, last = null;
      while ((m = re.exec(html)) !== null) last = m[1];
      if (!last) return null;
      const sanitized = last.replace(/,(\s*])/g, '$1'); // strip trailing commas
      try { return JSON.parse(sanitized); } catch { return null; }
    }

    let coords_osrm = getArrayFromView('coords_osrm');
    let coords_rs   = getArrayFromView('coords_rs');

    if (!coords_osrm || !coords_rs) {
      const vm = html.match(/window\.VIEW\s*=\s*(\{[\s\S]*?\});?/);
      if (vm) {
        try {
          const fixed = vm[1]
            .replace(/([{,]\s*)([A-Za-z_]\w*)(\s*:)/g, '$1"$2"$3')
            .replace(/,(\s*[}\]])/g, '$1');
          const V = JSON.parse(fixed);
          coords_osrm = coords_osrm || (Array.isArray(V.coords_osrm) ? V.coords_osrm : null);
          coords_rs   = coords_rs   || (Array.isArray(V.coords_rs)   ? V.coords_rs   : null);
        } catch {}
      }
    }

    if (!Array.isArray(coords_osrm)) coords_osrm = [];
    if (!Array.isArray(coords_rs))   coords_rs   = [];

    if (!coords_osrm.length && !coords_rs.length) {
      appendLog('[route] no coords in /view');
      return;
    }

    const latlngsOsrm = coords_osrm.map(p => [p[0], p[1]]);
    const latlngsRs   = coords_rs.map(p => [p[0], p[1]]);

    if (!window.__routeMap && els.routeMap) {
      const map = L.map(els.routeMap, { zoomControl: true });
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; OpenStreetMap contributors'
      }).addTo(map);
      L.control.scale({ metric: true, imperial: false }).addTo(map);
      window.__routeMap = { map, osrmLine: null, rsLine: null };
    }

    const { map } = window.__routeMap || {};
    if (!map) return;

    setTimeout(() => map.invalidateSize(), 0);

    if (window.__routeMap.osrmLine) map.removeLayer(window.__routeMap.osrmLine);
    if (window.__routeMap.rsLine)   map.removeLayer(window.__routeMap.rsLine);

    const osrmLine = latlngsOsrm.length > 1 ? L.polyline(latlngsOsrm, { color: '#d33', weight: 3, opacity: 0.9 }) : null;
    const rsLine   = latlngsRs.length   > 1 ? L.polyline(latlngsRs,   { color: '#38f', weight: 3, opacity: 0.9 }) : null;

    if (osrmLine) osrmLine.addTo(map);
    if (rsLine)   rsLine.addTo(map);

    window.__routeMap.osrmLine = osrmLine;
    window.__routeMap.rsLine   = rsLine;

    const layers = [osrmLine, rsLine].filter(Boolean);
    if (layers.length) {
      const group = L.featureGroup(layers);
      map.fitBounds(group.getBounds().pad(0.05), { animate: false });
    }
  }

  // ===== Boot =====
  document.addEventListener('DOMContentLoaded', async () => {
    // Populate list
    await refreshUploads();
    // Poll for external changes (curl upload etc.)
    setInterval(() => { refreshUploads().catch(()=>{}); }, 3000);

    // Run button
    els.runBtn?.addEventListener('click', () => {
      runResample().catch(err => appendLog(`[runResample] ${err.message||err}`));
    });

    // Wavelet Selection button
    document.getElementById('wfTerrainBtn')?.addEventListener('click', () => {
      runWaveletTerrain().catch(e => appendLog(`[wavelet] ${e.message||e}`));
    });
    document.getElementById('wfClearBtn')?.addEventListener('click', () => {
      window.__plotOverlayDrawer = null;
      const base = window.__lastElevation;
      if (base && Array.isArray(base.x) && Array.isArray(base.y)) {
        renderLineChart(base.x, base.y, base.title || 'Elevation');
      }
    });



    // Upload button - send RAW JSON (server expects JSON body, not multipart)
    els.uploadBtn?.addEventListener('click', async () => {
      try {
        const f = els.uploadFile?.files?.[0];
        if (!f) { alert('Choose a JSON file first'); return; }
        setUploadStatus('Uploading...','info');
        const text = await f.text();      // send raw JSON
        // Ensure it's valid JSON before sending (optional safety)
        try { JSON.parse(text); } 
        catch { alert('File is not valid JSON'); 
                setUploadStatus('Upload failed: invalid JSON', 'err');
                return; 
              }

        const r = await fetch('/upload', {
          method:'POST',
          headers: { 'Content-Type':'application/json' },
          body: text
        });
        if (!r.ok) {
          const msg = await r.text();
          appendLog(`[upload] HTTP ${r.status}: ${msg}`);
          setUploadStatus(`Upload failed: ${msg}`, 'err');
          alert(`Upload failed: ${msg}`);
          return;
        }
        const j = await r.json();
        await handleUploadResponse(j);
      } catch (e) {
        appendLog(`[upload] ${e.message||e}`);
        setUploadStatus(`Upload failed: ${e.message||e}`, 'err');
      }
    });

    // Update route map if tab is visible when file changes
    els.mapSelect?.addEventListener('change', () => {
      const panel = document.getElementById('panel-route');
      if (panel && !panel.hidden && els.mapSelect.value) loadRouteLeaflet(els.mapSelect.value);
      localStorage.setItem('lastMap', els.mapSelect.value);
    });

    initTabs();
  });
})();

