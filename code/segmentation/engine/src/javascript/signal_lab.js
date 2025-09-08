// signal_lab.js
// UI and processing logic for signal analysis lab

(() => {
  // Smoothing helpers
  const SMOOTH_LEVEL = 13;
  const SG_POLY = 3;

  function isFiniteNum(x) {
    return Number.isFinite(x);
  }

  function smoothSG(arr, window = SMOOTH_LEVEL, poly = SG_POLY) {
    if (!Array.isArray(arr) || arr.length === 0) return arr;
    if (window % 2 === 0) window += 1;
    if (window < 5) window = 5;
    if (poly >= window) poly = window - 1;
    const n = arr.length,
      half = Math.floor(window / 2);

    const A = [];
    for (let i = -half; i <= half; i++) {
      const row = [];
      for (let p = 0; p <= poly; p++) row.push(Math.pow(i, p));
      A.push(row);
    }
    const At = (M) => M[0].map((_, j) => M.map((r) => r[j]));
    const matMul = (X, Y) =>
      X.map((row, i) =>
        Y[0].map((_, j) => row.reduce((s, _, k) => s + X[i][k] * Y[k][j], 0)),
      );
    const inv = (M) => {
      const n = M.length;
      const I = [...Array(n)].map((_, i) =>
        [...Array(n)].map((__, j) => (i === j ? 1 : 0)),
      );
      const A = M.map((r) => r.slice());
      for (let i = 0; i < n; i++) {
        let p = i;
        for (let r = i + 1; r < n; r++)
          if (Math.abs(A[r][i]) > Math.abs(A[p][i])) p = r;
        if (Math.abs(A[p][i]) < 1e-12) return null;
        [A[i], A[p]] = [A[p], A[i]];
        [I[i], I[p]] = [I[p], I[i]];
        const f = A[i][i];
        for (let j = 0; j < n; j++) {
          A[i][j] /= f;
          I[i][j] /= f;
        }
        for (let r = 0; r < n; r++)
          if (r !== i) {
            const g = A[r][i];
            for (let j = 0; j < n; j++) {
              A[r][j] -= g * A[i][j];
              I[r][j] -= g * I[i][j];
            }
          }
      }
      return I;
    };
    const AT = At(A);
    const ATA = matMul(AT, A);
    const ATAinv = inv(ATA);
    if (!ATAinv) return arr;
    const pinv = matMul(ATAinv, AT);
    const centerWeights = pinv[0];

    const get = (k) => (k < 0 ? arr[-k] : k >= n ? arr[2 * n - 2 - k] : arr[k]);
    const out = new Array(n);
    for (let i = 0; i < n; i++) {
      let acc = 0,
        wsum = 0;
      for (let j = -half; j <= half; j++) {
        const v = get(i + j);
        const w = centerWeights[j + half];
        if (isFiniteNum(v)) {
          acc += w * v;
          wsum += w;
        }
      }
      out[i] = wsum !== 0 ? acc : arr[i];
    }
    return out;
  }

  function smoothEMA(arr, alpha = 0.25) {
    if (!Array.isArray(arr) || arr.length === 0) return arr;
    const n = arr.length,
      f = new Array(n),
      b = new Array(n);
    f[0] = isFiniteNum(arr[0]) ? arr[0] : 0;
    for (let i = 1; i < n; i++) {
      const x = isFiniteNum(arr[i]) ? arr[i] : f[i - 1];
      f[i] = f[i - 1] + alpha * (x - f[i - 1]);
    }
    b[n - 1] = isFiniteNum(arr[n - 1]) ? arr[n - 1] : f[n - 1];
    for (let i = n - 2; i >= 0; i--) {
      const x = isFiniteNum(arr[i]) ? arr[i] : b[i + 1];
      b[i] = b[i + 1] + alpha * (x - b[i + 1]);
    }
    return f.map((v, i) => 0.5 * (v + b[i]));
  }

  // UI element references
  const els = {
    mapSelect: document.getElementById("mapSelect"),
    runBtn: document.getElementById("runBtn"),
    dsInput: document.getElementById("dsInput"),
    varSelect: document.getElementById("varSelect"),
    kindSelect: document.getElementById("kindSelect"),
    uploadFile: document.getElementById("uploadFile"),
    uploadBtn: document.getElementById("uploadBtn"),
    uploadMsg: document.getElementById("uploadMsg"),
    plotUni: document.getElementById("plotUni"),
    tabs: document.getElementById("labTabs"),
    panels: document.getElementById("labPanels"),
    statsBox: document.getElementById("statsSummary"),
    routeMap: document.getElementById("routeMap"),
    openView: document.getElementById("openViewLink"),
    log: document.getElementById("log"),
    extPlot: document.getElementById("extPlot"),
  };

  // Attach tooltips to form controls
  function initTooltips() {
    const tips = {
      wf_ds: "Sample spacing ds (m). Smaller = more detail, noisier.",
      wf_LT:
        "Trend window L_T (m). Larger = smoother blue trend; smaller = faster response.",
      wf_LE:
        "Energy window L_E (m). Larger = less spiky energy; smaller = sharper bursts.",
      wf_E_env:
        "Envelope window (m). Larger merges nearby spikes into one plateau.",
      wf_hyst_en: "Enable hysteresis grouping. Off = raw envelope only.",
      wf_E_hi:
        "Start threshold (MAD). Higher = fewer starts; lower = starts easier.",
      wf_E_lo:
        "Sustain threshold (MAD). Lower = stays ON longer; higher = ends sooner.",
      wf_E_gap: "Gap close (m). Bridge dips shorter than this while ON.",
      wf_E_min: "Min run (m). Discard ON runs shorter than this.",
      wf_fill: "Fill background with energy shading.",
    };

    Object.entries(tips).forEach(([id, text]) => {
      const el = document.getElementById(id);
      if (!el) return;
      el.title = text;
      const row = el.closest(".row");
      if (row) row.title = text;
      const lab = row && row.querySelector("label");
      if (lab) lab.title = text;
    });
  }
  document.addEventListener("DOMContentLoaded", initTooltips);

  // Enable or disable hysteresis UI controls
  function setHystUI(on) {
    ["wf_E_hi", "wf_E_lo", "wf_E_gap", "wf_E_min"].forEach((id) => {
      const el = document.getElementById(id);
      if (el) el.disabled = !on;
    });
    document.querySelectorAll("[data-hyst-row]").forEach((r) => {
      r.classList.toggle("disabled", !on);
    });
  }

  document.addEventListener("DOMContentLoaded", () => {
    const t = document.getElementById("wf_hyst_en");
    if (t) {
      setHystUI(t.checked);
      t.addEventListener("change", (e) => setHystUI(e.target.checked));
    }
  });

  // Read terrain parameters from UI
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
    const bool = (id) => !!document.getElementById(id)?.checked;
    const hyst = bool("wf_hyst_en");

    const segMin = Number(document.getElementById("wf_seg_min").value) || 0;
    const mergeRight =
      document.getElementById("wf_merge_side")?.value === "right";

    return {
      ds_m: num("wf_ds", 5, [0.1, 10000]),
      L_T: num("wf_LT", 150, [2, 100000]),
      L_E: num("wf_LE", 120, [4, 100000]),

      k_g: num("wf_k_g", 3.0, [0, 100]),
      k_E: num("wf_k_E", 3.0, [0, 100]),
      tau_p: num("wf_tau_p", 1.5, [0, 100]),

      energy_mode: "rms",

      E_env_m: num("wf_E_env", 200, [0, 5000]),
      E_use_hysteresis: hyst,
      E_hyst_hi: hyst ? num("wf_E_hi", 2.0, [0, 10]) : undefined,
      E_hyst_lo: hyst ? num("wf_E_lo", 1.0, [0, 10]) : undefined,
      E_gap_close_m: hyst ? num("wf_E_gap", 30, [0, 2000]) : undefined,
      E_min_run_m: hyst ? num("wf_E_min", 80, [0, 5000]) : undefined,

      min_segment_length_m: segMin,
      merge_side_right: mergeRight,

      fill: bool("wf_fill"),
    };
  }

  function normalizeTerrainCodes(arr) {
    if (!Array.isArray(arr)) return [];
    const out = new Array(arr.length);
    for (let i = 0; i < arr.length; i++) {
      const v = Number(arr[i]);
      const c = Math.round(Number.isFinite(v) ? v : 0);
      out[i] = c < 0 ? 0 : c > 4 ? 4 : c;
    }
    return out;
  }

  function setUploadStatus(text, kind) {
    if (!els.uploadMsg) return;
    els.uploadMsg.textContent = text || "";
    els.uploadMsg.classList.remove("is-ok", "is-err", "is-info");
    els.uploadMsg.classList.add(kind ? `is-${kind}` : "is-info");
  }

  function appendLog(msg) {
    if (!els.log) return;
    const ts = new Date().toISOString().replace("T", " ").replace("Z", "");
    els.log.textContent += `[${ts}] ${msg}\n`;
    els.log.scrollTop = els.log.scrollHeight;
  }

  /** Human-friendly byte size */
  function humanSize(bytes) {
    if (typeof bytes !== "number") return "";
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(1)} KB`;
    return `${(bytes / 1024 / 1024).toFixed(2)} MB`;
  }

  /** Lightweight Leaflet loader (if not already included in HTML) */
  function ensureLeaflet() {
    if (window.L) return Promise.resolve();
    return new Promise((resolve, reject) => {
      const link = document.createElement("link");
      link.rel = "stylesheet";
      link.href = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.css";
      document.head.appendChild(link);
      const s = document.createElement("script");
      s.src = "https://unpkg.com/leaflet@1.9.4/dist/leaflet.js";
      s.onload = () => resolve();
      s.onerror = () => reject(new Error("Leaflet failed to load"));
      document.head.appendChild(s);
    });
  }

  function updateMapOptions(files) {
    if (!els.mapSelect) return;
    const prev = els.mapSelect.value || localStorage.getItem("lastMap") || "";
    els.mapSelect.innerHTML = "";
    (files || []).forEach((f) => {
      if (!f || !f.file || !f.name) return;
      const opt = document.createElement("option");
      opt.value = String(f.file);
      const size = humanSize(f.bytes);
      opt.textContent = size ? `${f.name} - ${size}` : f.name;
      els.mapSelect.add(opt);
    });
    if (prev && [...els.mapSelect.options].some((o) => o.value === prev)) {
      els.mapSelect.value = prev;
    }
    if (els.mapSelect.value) {
      localStorage.setItem("lastMap", els.mapSelect.value);
    }
  }

  async function refreshUploads(preferPath = null) {
    try {
      const r = await fetch("/lab/list");
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      const data = await r.json();
      const files = Array.isArray(data?.files) ? data.files : [];
      updateMapOptions(files);
      if (
        preferPath &&
        els.mapSelect &&
        [...els.mapSelect.options].some((o) => o.value === preferPath)
      ) {
        els.mapSelect.value = preferPath;
        localStorage.setItem("lastMap", preferPath);
      }
    } catch (e) {
      appendLog(`[list] failed: ${e.message}`);
      console.error(e);
    }
  }

  async function handleUploadResponse(json) {
    try {
      if (!json || typeof json.file !== "string")
        throw new Error('Upload response missing "file"');
      await refreshUploads(json.file);
      if (els.uploadMsg)
        setUploadStatus(`Uploaded & selected ${json.file}`, "ok");
    } catch (e) {
      setUploadStatus(`[upload] ${e.message}`, "err");
      appendLog(`[upload] error: ${e.message}`);
    }
  }

  function _finite(arr) {
    return (Array.isArray(arr) ? arr : []).filter(Number.isFinite);
  }
  function _quantile(sorted, q) {
    if (!sorted.length) return NaN;
    const i = (sorted.length - 1) * q;
    const lo = Math.floor(i),
      hi = Math.ceil(i);
    return lo === hi
      ? sorted[lo]
      : sorted[lo] * (hi - i) + sorted[hi] * (i - lo);
  }
  function _mean(a) {
    const v = _finite(a);
    return v.length ? v.reduce((s, x) => s + x, 0) / v.length : NaN;
  }
  function _variance(a) {
    const v = _finite(a);
    if (v.length < 2) return NaN;
    const m = _mean(v);
    return v.reduce((s, x) => s + (x - m) * (x - m), 0) / (v.length - 1);

  }
  function _std(a) {
    const vv = _variance(a);
    return Number.isFinite(vv) ? Math.sqrt(vv) : NaN;

  }
  function _deg(rad) {
    return (rad * 180) / Math.PI;
  }

  function _wrapPi(rad) {
    const twoPi = 2 * Math.PI;
    let t = rad % twoPi;
    if (t > Math.PI) t -= twoPi;
    if (t <= -Math.PI) t += twoPi;
    return t;
  }
  function _circularStats(radArray) {
    const v = _finite(radArray).map(_wrapPi);
    const n = v.length;
    if (!n) return { mean: NaN, std: NaN, var: NaN, n: 0, Rbar: NaN };
    let C = 0,
      S = 0;
    for (const a of v) {
      C += Math.cos(a);
      S += Math.sin(a);
    }
    const mean = Math.atan2(S, C);
    const R = Math.hypot(C, S);
    const Rbar = R / n;
    const circVar = 1 - Rbar;
    const circStd = Math.sqrt(-2 * Math.log(Rbar || Number.EPSILON));
    return { mean, std: circStd, var: circVar, n, Rbar };
  }
  function computeStats(values, kind) {
    const v = _finite(values);
    const n = v.length,
      missing = (Array.isArray(values) ? values.length : 0) - v.length;
    if (!n) return { n: 0, missing, mean: NaN, std: NaN, var: NaN };
    if ((kind || "scalar") === "angle") {
      const c = _circularStats(v);
      return {
        n,
        missing,
        mean: _deg(c.mean),
        std: _deg(c.std),
        var: c.var,
        rbar: c.Rbar,
      };
    } else {
      const s = [...v].sort((a, b) => a - b);
      const mean = _mean(v);
      const variance = _variance(v);
      const std = _std(v);
      const q25 = _quantile(s, 0.25);
      const med = _quantile(s, 0.5);
      const q75 = _quantile(s, 0.75);
      return {
        n,
        missing,
        min: s[0],
        q25,
        median: med,
        q75,
        max: s[s.length - 1],
        mean,
        var: variance,
        std,
      };
    }
  }
  function guessKind(name) {
    return /heading/.test(String(name || "")) ? "angle" : "scalar";
  }

  function renderStats(values, vname, maybeKind) {
    const kind =
      (maybeKind && String(maybeKind).toLowerCase()) || guessKind(vname);
    const stats = computeStats(values, kind);
    if (els.statsBox) {
      const fmt = (x) =>
        Number.isFinite(x) ? (+x.toFixed(4)).toString() : "-";
      const cells = (pairs) =>
        pairs
          .map(
            ([label, val]) =>
              `<div class="badge"><span class="label">${label}</span><span class="value">${fmt(val)}</span></div>`,
          )
          .join("");
      els.statsBox.innerHTML =
        kind === "angle"
          ? cells([
              ["Samples", stats.n],
              ["Missing", stats.missing],
              ["Mean (°)", stats.mean],
              ["Std (°)", stats.std],
              ["CircVar", stats.var],
              ["R̄", stats.rbar],
            ])
          : cells([
              ["Samples", stats.n],
              ["Missing", stats.missing],
              ["Mean", stats.mean],
              ["Std", stats.std],
              ["Var", stats.var],
              ["Min", stats.min],
              ["Q25", stats.q25],
              ["Median", stats.median],
              ["Q75", stats.q75],
              ["Max", stats.max],
            ]);
    }
  }

  function renderLineChart(x, y, title = "") {
    if (!els.plotUni) return;

    // Create/clear canvas
    let canvas = els.plotUni.querySelector("canvas");
    if (!canvas) {
      canvas = document.createElement("canvas");
      canvas.width = els.plotUni.clientWidth || 800;
      canvas.height = els.plotUni.clientHeight || 240;
      els.plotUni.appendChild(canvas);
    } else {
      const w = els.plotUni.clientWidth || canvas.width;
      const h = els.plotUni.clientHeight || canvas.height;
      canvas.width = w;
      canvas.height = h;
    }
    const ctx = canvas.getContext("2d");

    const pts = [];
    for (let i = 0; i < Math.min(x.length, y.length); i++) {
      const xi = x[i],
        yi = y[i];
      if (Number.isFinite(xi) && Number.isFinite(yi)) pts.push([xi, yi]);
    }
    if (!pts.length) return;

    const xmin = Math.min(...pts.map((p) => p[0]));
    const xmax = Math.max(...pts.map((p) => p[0]));
    const ymin = Math.min(...pts.map((p) => p[1]));
    const ymax = Math.max(...pts.map((p) => p[1]));

    const padL = 48,
      padR = 16,
      padT = 24,
      padB = 28;
    const W = canvas.width,
      H = canvas.height;
    const plotW = Math.max(10, W - padL - padR);
    const plotH = Math.max(10, H - padT - padB);
    const sx = (xv) =>
      padL + (xmax === xmin ? 0 : ((xv - xmin) / (xmax - xmin)) * plotW);
    const sy = (yv) =>
      padT +
      plotH -
      (ymax === ymin ? 0 : ((yv - ymin) / (ymax - ymin)) * plotH);

    function drawBase() {
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, W, H);

      ctx.strokeStyle = "#333";
      ctx.lineWidth = 1;

      ctx.beginPath();
      ctx.moveTo(padL, H - padB);
      ctx.lineTo(W - padR, H - padB);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(padL, padT);
      ctx.lineTo(padL, H - padB);
      ctx.stroke();

      if (title) {
        ctx.fillStyle = "#111";
        ctx.font = "bold 12px system-ui, sans-serif";
        ctx.fillText(title, padL, padT - 8);
      }

      ctx.strokeStyle = "#0a66ff";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      ctx.moveTo(sx(pts[0][0]), sy(pts[0][1]));
      for (let i = 1; i < pts.length; i++)
        ctx.lineTo(sx(pts[i][0]), sy(pts[i][1]));
      ctx.stroke();

      ctx.fillStyle = "#555";
      ctx.font = "10px system-ui, sans-serif";
      const xticks = [xmin, (xmin + xmax) / 2, xmax];
      const yticks = [ymin, (ymin + ymax) / 2, ymax];
      xticks.forEach((v) => {
        const px = sx(v);
        ctx.fillText(v.toFixed(2), px - 8, H - padB + 12);
        ctx.beginPath();
        ctx.moveTo(px, H - padB);
        ctx.lineTo(px, H - padB + 4);
        ctx.stroke();
      });
      yticks.forEach((v) => {
        const py = sy(v);
        ctx.fillText(v.toFixed(2), 6, py + 3);
        ctx.beginPath();
        ctx.moveTo(padL - 4, py);
        ctx.lineTo(padL, py);
        ctx.stroke();
      });

      if (typeof window.__plotOverlayDrawer === "function") {
        try {
          window.__plotOverlayDrawer(ctx, {
            sx,
            sy,
            padL,
            padR,
            padT,
            padB,
            W,
            H,
            plotW,
            plotH,
            xmin,
            xmax,
            ymin,
            ymax,
          });
        } catch (e) {
          console.warn("overlay drawer error (base):", e);
        }
      }
    }

    const xpx = pts.map((p) => sx(p[0]));
    const monoIncreasing =
      pts.length > 2 && (pts[1][0] - pts[0][0]) * (pts[2][0] - pts[1][0]) >= 0;

    function nearestIndex(px) {
      if (monoIncreasing) {
        let lo = 0,
          hi = xpx.length - 1;
        while (lo < hi) {
          const mid = (lo + hi) >> 1;
          if (xpx[mid] < px) lo = mid + 1;
          else hi = mid;
        }

        const i2 = Math.max(0, lo - 1);
        return Math.abs(xpx[lo] - px) < Math.abs(xpx[i2] - px) ? lo : i2;
      }

      let best = 0,
        bestd = Infinity;
      for (let i = 0; i < xpx.length; i++) {
        const d = Math.abs(xpx[i] - px);
        if (d < bestd) {
          bestd = d;
          best = i;
        }
      }
      return best;
    }

    function drawHover(mx, my) {
      drawBase();

      const idx = nearestIndex(mx);
      const [vx, vy] = pts[idx];
      const px = sx(vx),
        py = sy(vy);

      ctx.strokeStyle = "rgba(0,0,0,.25)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(px, padT);
      ctx.lineTo(px, H - padB);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(padL, py);
      ctx.lineTo(W - padR, py);
      ctx.stroke();

      ctx.fillStyle = "#0a66ff";
      ctx.beginPath();
      ctx.arc(px, py, 3, 0, Math.PI * 2);
      ctx.fill();

      const text1 = `s: ${vx.toFixed(3)} km`;
      const text2 = `y: ${vy.toFixed(3)}`;
      ctx.font = "11px system-ui, sans-serif";
      const pad = 6;
      const tw = Math.max(
        ctx.measureText(text1).width,
        ctx.measureText(text2).width,
      );
      const th = 14 + 14 + 6;
      let bx = px + 10,
        by = py - th - 10;
      if (bx + tw + pad * 2 > W - padR) bx = px - 10 - tw - pad * 2;
      if (by < padT) by = py + 10;

      ctx.fillStyle = "rgba(255,255,255,0.95)";
      ctx.fillRect(bx, by, tw + pad * 2, th);
      ctx.strokeStyle = "#333";
      ctx.strokeRect(bx, by, tw + pad * 2, th);

      ctx.fillStyle = "#111";
      ctx.fillText(text1, bx + pad, by + 14);
      ctx.fillText(text2, bx + pad, by + 28);

      if (typeof window.__plotUnderlayDrawer === "function") {
        try {
          window.__plotUnderlayDrawer(ctx, {
            sx,
            sy,
            padL,
            padR,
            padT,
            padB,
            W,
            H,
            plotW,
            plotH,
            xmin,
            xmax,
            ymin,
            ymax,
          });
        } catch (e) {
          console.warn("underlay drawer error (base):", e);
        }
      }
    }

    drawBase();

    canvas.onmousemove = (ev) => {
      const rect = canvas.getBoundingClientRect();
      const mx = ev.clientX - rect.left;
      const my = ev.clientY - rect.top;

      if (mx >= padL && mx <= W - padR && my >= padT && my <= H - padB) {
        drawHover(mx, my);
      } else {
        drawBase();
      }
    };
    canvas.onmouseleave = () => drawBase();
  }
  function setOverlayMulti(lines) {
    if (!Array.isArray(lines) || lines.length === 0) {
      window.__plotOverlayDrawer = null;
      return;
    }

    let y2Min = +Infinity,
      y2Max = -Infinity;
    for (const L of lines) {
      if (!Array.isArray(L.y) || !L.y.length) continue;
      for (const v of L.y) {
        if (v < y2Min) y2Min = v;
        if (v > y2Max) y2Max = v;
      }
    }
    if (!isFinite(y2Min) || !isFinite(y2Max) || y2Min === y2Max) {
      y2Min -= 1;
      y2Max += 1;
    }
    window.__plotOverlayDrawer = function overlay(ctx, s) {
      const { sx, padL, padR, padT, padB, W, H } = s;
      const plotH = Math.max(10, H - padT - padB);
      const sy2 = (v) =>
        H - padB - (plotH * (v - y2Min)) / (y2Max - y2Min || 1);

      ctx.strokeStyle = "#333";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(W - padR, padT);
      ctx.lineTo(W - padR, H - padB);
      ctx.stroke();

      for (const L of lines) {
        if (
          !Array.isArray(L.x) ||
          !Array.isArray(L.y) ||
          L.x.length !== L.y.length
        )
          continue;
        ctx.save();
        ctx.setLineDash(L.dash || []);
        ctx.lineWidth = L.width || 1.1;
        ctx.strokeStyle = "#000";
        ctx.beginPath();
        ctx.moveTo(sx(L.x[0]), sy2(L.y[0]));
        for (let i = 1; i < L.x.length; i++)
          ctx.lineTo(sx(L.x[i]), sy2(L.y[i]));
        ctx.stroke();
        ctx.restore();
      }
    };
  }

  function renderElevationWithOverlayLine(
    x,
    yElev,
    title,
    x2,
    y2,
    y2Label = "Haar trend",
  ) {
    if (!els.plotUni) return;

    let canvas = els.plotUni.querySelector("canvas");
    if (!canvas) {
      canvas = document.createElement("canvas");
      canvas.width = els.plotUni.clientWidth || 800;
      canvas.height = els.plotUni.clientHeight || 260;
      els.plotUni.appendChild(canvas);
    } else {
      const w = els.plotUni.clientWidth || canvas.width;
      const h = els.plotUni.clientHeight || canvas.height;
      canvas.width = w;
      canvas.height = h;
    }
    const ctx = canvas.getContext("2d");
    const W = canvas.width,
      H = canvas.height;
    const padL = 44,
      padR = 50,
      padT = 24,
      padB = 26;

    ctx.fillStyle = "#fff";
    ctx.fillRect(0, 0, W, H);

    const N = Math.max(0, Math.min(x?.length || 0, yElev?.length || 0));
    const N2 = Math.max(0, Math.min(x2?.length || 0, y2?.length || 0));
    if (!N) return;

    const xMin = 0,
      xMax = Math.max(1e-9, Math.max(...x));
    const y1Min = Math.min(...yElev),
      y1Max = Math.max(...yElev);
    const y2Min = N2 ? Math.min(...y2) : 0,
      y2Max = N2 ? Math.max(...y2) : 1;

    const sx = (v) =>
      padL + ((W - padL - padR) * (v - xMin)) / (xMax - xMin || 1);
    const sy1 = (v) =>
      H - padB - ((H - padT - padB) * (v - y1Min)) / (y1Max - y1Min || 1);
    const sy2 = (v) =>
      H - padB - ((H - padT - padB) * (v - y2Min)) / (y2Max - y2Min || 1);

    ctx.strokeStyle = "#333";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(padL, H - padB);
    ctx.lineTo(W - padR, H - padB);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(padL, padT);
    ctx.lineTo(padL, H - padB);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(W - padR, padT);
    ctx.lineTo(W - padR, H - padB);
    ctx.stroke();

    if (title) {
      ctx.fillStyle = "#111";
      ctx.font = "bold 12px system-ui,sans-serif";
      ctx.fillText(title, padL, padT - 8);
    }

    ctx.strokeStyle = "#0a66ff";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(sx(x[0]), sy1(yElev[0]));
    for (let i = 1; i < N; i++) ctx.lineTo(sx(x[i]), sy1(yElev[i]));
    ctx.stroke();

    if (N2) {
      ctx.strokeStyle = "#000";
      ctx.setLineDash([4, 3]);
      ctx.lineWidth = 1.2;
      ctx.beginPath();
      ctx.moveTo(sx(x2[0]), sy2(y2[0]));
      for (let i = 1; i < N2; i++) ctx.lineTo(sx(x2[i]), sy2(y2[i]));
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = "#444";
      ctx.font = "11px system-ui,sans-serif";
      ctx.fillText(y2Label, W - padR + 6, padT + 10);
    }
  }

  function renderElevationWithOverlay(x, yElev, title, overlay) {
    const N = Math.max(
      0,
      Math.min(
        Array.isArray(x) ? x.length : 0,
        Array.isArray(yElev) ? yElev.length : 0,
        overlay && Array.isArray(overlay.codes) ? overlay.codes.length : 0,
      ),
    );
    const X = (Array.isArray(x) ? x : []).slice(0, N);
    const Y = (Array.isArray(yElev) ? yElev : []).slice(0, N);
    const C = (
      overlay && Array.isArray(overlay.codes) ? overlay.codes : []
    ).slice(0, N);

    window.__plotOverlayDrawer = null;
    window.__plotUnderlayDrawer = function underlayDrawer(ctx, s) {
      if (!X.length) return;

      const { sx, sy, padL, padR, padT, padB, W, H } = s;
      const plotW = Math.max(10, W - padL - padR);
      const plotH = Math.max(10, H - padT - padB);
      const yBase = padT + plotH;

      const COLORS = {
        1: "#9aa0a6",
        2: "#d33",
        3: "#38f",
        4: "#ff8c00",
        0: "#8e44ad",
      };

      ctx.save();
      ctx.beginPath();
      ctx.rect(padL, padT, plotW, plotH);
      ctx.clip();

      const CN = normalizeTerrainCodes(C);

      let i = 0;
      while (i < X.length) {
        const code = CN[i];
        let j = i + 1;
        while (j < X.length && CN[j] === code) j++;

        const col = COLORS.hasOwnProperty(code) ? COLORS[code] : "#888";

        ctx.save();
        ctx.globalAlpha = 0.28;
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

        ctx.restore();

        i = j;
      }

      ctx.restore();
    };

    renderLineChart(X, Y, title || "Elevation");
  }

  async function runResample() {
    const mapPath = els.mapSelect ? els.mapSelect.value : "";
    if (!mapPath) {
      alert("Please select a file first.");
      return;
    }

    const ds_m = Number(els.dsInput?.value || 5);
    const vname = String(els.varSelect?.value || "elev");
    const payload = { map: mapPath, ds_m, vars: [vname] };

    appendLog(`[run] map=${mapPath} var=${vname} ds=${ds_m}`);

    const r = await fetch("/lab/resample", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const bodyText = await r.text();
    if (!r.ok) {
      appendLog(`[runResample] POST /lab/resample -> ${r.status}: ${bodyText}`);
      return;
    }

    let j = {};
    try {
      j = JSON.parse(bodyText);
    } catch {
      j = {};
    }

    const x = Array.isArray(j.s_km) ? j.s_km : [];
    const y = j.series && Array.isArray(j.series[vname]) ? j.series[vname] : [];

    renderStats(y, vname, els.kindSelect?.value);

    renderLineChart(
      x,
      y,
      `Uniform ${vname} (Δs≈${Number(j.ds_m ?? ds_m).toFixed(1)} m)`,
    );

    window.__lastElevation = {
      x: Array.isArray(j.s_km) ? j.s_km.slice() : [],
      y: j.series && Array.isArray(j.series.elev) ? j.series.elev.slice() : [],
      title: "Elevation",
    };

    document.dispatchEvent(
      new CustomEvent("resample-data", {
        detail: { x, y, vname, ds_m: j.ds_m, json: j },
      }),
    );

    const routePanel = document.getElementById("panel-route");
    if (routePanel && !routePanel.hidden && els.mapSelect?.value) {
      loadRouteLeaflet(els.mapSelect.value);
    }
  }

  async function runWaveletTerrain() {
    console.log("runWaveletTerrain() start");
    const sel = els.mapSelect?.value;
    if (!sel) {
      alert("Please select a file first.");
      return;
    }

    const persist = document.getElementById("persistToggle").checked;
    const p = getTerrainParams();
    const payload = {
      map: sel,
      fn: "terrain",
      params: { terrain: p },
      persist,
    };
    appendLog(`[wavelet] POST /wavelet fn=terrain map=${sel}`);

    const r = await fetch("/wavelet", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    if (!r.ok) {
      const errTxt = await r.text();
      appendLog(`[wavelet] HTTP ${r.status}: ${errTxt}`);
      alert(`Wavelet request failed: ${errTxt}`);
      return;
    }

    let j;
    try {
      j = await r.json();
      console.log(
        `[wavelet] parsed JSON object, keys: ${Object.keys(j).length}`,
      );
    } catch (e) {
      console.error("[wavelet] response.json() threw:", e);
      const textDump = await r.clone().text();
      console.log("[wavelet] raw response length:", textDump.length);
      alert("Failed to parse wavelet JSON—see console for details");
      return;
    }

    if (window.onWaveletResponse) window.onWaveletResponse(j);

    const xWave =
      Array.isArray(j.s_km_uniform) && j.s_km_uniform.length
        ? j.s_km_uniform
        : Array.isArray(j.s_km)
          ? j.s_km
          : [];
    const base = window.__lastElevation || {
      x: xWave,
      y: new Array(xWave.length).fill(0),
      title: "Elevation",
    };

    const series = j.series || {};
    console.log("[wavelet] series keys:", Object.keys(series));
    console.log(
      "[wavelet] terrain array sample:",
      series.terrain && series.terrain.slice(0, 10),
    );
    const trend = series.haar_trend;
    const energy = series.energy || series.energy_fft;
    const showE = document.getElementById("wf_showE")?.checked;
    const lines = [];

    if (Array.isArray(trend) && trend.length === xWave.length) {
      lines.push({
        x: xWave,
        y: trend,
        dash: [4, 3],
        width: 1.2,
        label: "Haar trend",
      });
    }
    if (showE && Array.isArray(energy) && energy.length === xWave.length) {
      lines.push({
        x: xWave,
        y: energy,
        dash: [],
        width: 1.0,
        label: "Rolling energy",
      });
    }

    setOverlayMulti(lines);
    renderLineChart(base.x, base.y, base.title);

    if (
      p.fill &&
      Array.isArray(series.terrain) &&
      series.terrain.length === xWave.length
    ) {
      renderElevationWithOverlay(base.x, base.y, base.title, {
        codes: normalizeTerrainCodes(series.terrain),
      });
    }

    appendLog("[wavelet] done");
  }

  function initTabs() {
    if (!els.tabs) return;
    els.tabs.addEventListener("click", (e) => {
      const btn = e.target.closest("button[data-tab]");
      if (!btn || btn.disabled) return;
      const name = btn.getAttribute("data-tab");
      els.tabs
        .querySelectorAll("button[data-tab]")
        .forEach((b) => b.classList.toggle("active", b === btn));

      document.querySelectorAll("#labPanels .panel").forEach((p) => {
        const active = p.id === `panel-${name}`;
        p.hidden = !active;
        p.classList.toggle("active", active);
      });
      if (name === "segments") {
        renderSegments(segments);

        if (window.render3DSegment && window.MAPBOX_TOKEN && segments.length) {
          document.getElementById("panel-segments").hidden = false;

          window.render3DSegment(segments[0]);

          requestAnimationFrame(() => {
            if (window.map3d) window.map3d.resize();
          });
        } else {
          initSegMap();
          if (segments.length) selectSegment(0);
        }
      } else if (name === "route" && els.mapSelect?.value) {
        loadRouteLeaflet(els.mapSelect.value);
      } else if (name === "wavelet") {
        if (window.__lastElevation) {
          runWaveletTerrain().catch((e) =>
            appendLog(`[wavelet] ${e.message || e}`),
          );
        }
      } else if (name === "extensions") {
        if (currentSegment >= 0 && segments[currentSegment])
          renderSegmentExtensions(segments[currentSegment]);
      }
    });
  }

  async function loadRouteLeaflet(mapPath) {
    await ensureLeaflet();
    if (els.openView)
      els.openView.href = `/view?map=${encodeURIComponent(mapPath)}`;

    const url = `/view?map=${encodeURIComponent(mapPath)}&t=${Date.now()}`;
    const r = await fetch(url);
    if (!r.ok) {
      appendLog(`[route] GET ${r.status}`);
      return;
    }
    const html = await r.text();

    function getArrayFromView(name) {
      const re = new RegExp(
        String.raw`(?:var|let|const)?\s*${name}\s*=\s*(\[[\s\S]*?\])\s*;`,
        "g",
      );
      let m,
        last = null;
      while ((m = re.exec(html)) !== null) last = m[1];
      if (!last) return null;
      const sanitized = last.replace(/,(\s*])/g, "$1"); // strip trailing commas
      try {
        return JSON.parse(sanitized);
      } catch {
        return null;
      }
    }

    let coords_osrm = getArrayFromView("coords_osrm");
    let coords_rs = getArrayFromView("coords_rs");

    if (!coords_osrm || !coords_rs) {
      const vm = html.match(/window\.VIEW\s*=\s*(\{[\s\S]*?\});?/);
      if (vm) {
        try {
          const fixed = vm[1]
            .replace(/([{,]\s*)([A-Za-z_]\w*)(\s*:)/g, '$1"$2"$3')
            .replace(/,(\s*[}\]])/g, "$1");
          const V = JSON.parse(fixed);
          coords_osrm =
            coords_osrm ||
            (Array.isArray(V.coords_osrm) ? V.coords_osrm : null);
          coords_rs =
            coords_rs || (Array.isArray(V.coords_rs) ? V.coords_rs : null);
        } catch {}
      }
    }

    if (!Array.isArray(coords_osrm)) coords_osrm = [];
    if (!Array.isArray(coords_rs)) coords_rs = [];

    if (!coords_osrm.length && !coords_rs.length) {
      appendLog("[route] no coords in /view");
      return;
    }

    const latlngsOsrm = coords_osrm.map((p) => [p[0], p[1]]);
    const latlngsRs = coords_rs.map((p) => [p[0], p[1]]);

    if (!window.__routeMap && els.routeMap) {
      const map = L.map(els.routeMap, { zoomControl: true });
      L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
        attribution: "&copy; OpenStreetMap contributors",
      }).addTo(map);
      L.control.scale({ metric: true, imperial: false }).addTo(map);
      window.__routeMap = { map, osrmLine: null, rsLine: null };
    }

    const { map } = window.__routeMap || {};
    if (!map) return;

    setTimeout(() => map.invalidateSize(), 0);

    if (window.__routeMap.osrmLine) map.removeLayer(window.__routeMap.osrmLine);
    if (window.__routeMap.rsLine) map.removeLayer(window.__routeMap.rsLine);

    const osrmLine =
      latlngsOsrm.length > 1
        ? L.polyline(latlngsOsrm, { color: "#d33", weight: 3, opacity: 0.9 })
        : null;
    const rsLine =
      latlngsRs.length > 1
        ? L.polyline(latlngsRs, { color: "#38f", weight: 3, opacity: 0.9 })
        : null;

    if (osrmLine) osrmLine.addTo(map);
    if (rsLine) rsLine.addTo(map);

    window.__routeMap.osrmLine = osrmLine;
    window.__routeMap.rsLine = rsLine;

    const layers = [osrmLine, rsLine].filter(Boolean);
    if (layers.length) {
      const group = L.featureGroup(layers);
      map.fitBounds(group.getBounds().pad(0.05), { animate: false });
    }
  }

  // ===== Boot =====
  document.addEventListener("DOMContentLoaded", async () => {
    // Populate list
    await refreshUploads();
    // Poll for external changes (curl upload etc.)
    setInterval(() => {
      refreshUploads().catch(() => {});
    }, 3000);

    // Run button
    els.runBtn?.addEventListener("click", () => {
      runResample().catch((err) =>
        appendLog(`[runResample] ${err.message || err}`),
      );
    });

    // Wavelet Selection button
    document.getElementById("wfTerrainBtn")?.addEventListener("click", () => {
      runWaveletTerrain().catch((e) =>
        appendLog(`[wavelet] ${e.message || e}`),
      );
    });
    document.getElementById("wfClearBtn")?.addEventListener("click", () => {
      window.__plotOverlayDrawer = null;
      const base = window.__lastElevation;
      if (base && Array.isArray(base.x) && Array.isArray(base.y)) {
        renderLineChart(base.x, base.y, base.title || "Elevation");
      }
    });

    // ---- Settings Save/Load ----
    const SETTINGS_KEY = "signalLab.settings.v1";
    const FIELDS = [
      "mapSelect",
      "varSelect",
      "dsInput",
      "kindSelect",
      "wf_ds",
      "wf_LT",
      "wf_LE",
      "wf_k_g",
      "wf_k_E",
      "wf_tau_p",
      "wf_hyst_en",
      "wf_E_hi",
      "wf_E_lo",
      "wf_E_gap",
      "wf_E_min",
      "wf_E_env",
      "wf_showE",
      "wf_fill",
      "wf_seg_min",
      "wf_merge_side",
    ];

    function collectSettings() {
      const data = {};
      for (const id of FIELDS) {
        const el = document.getElementById(id);
        if (!el) continue;
        if (el.type === "checkbox") data[id] = !!el.checked;
        else if (el.type === "number") data[id] = Number(el.value);
        else data[id] = el.value;
      }
      return data;
    }

    function applySettings(data) {
      if (!data) return;
      for (const [id, val] of Object.entries(data)) {
        const el = document.getElementById(id);
        if (!el) continue;
        if (el.type === "checkbox") el.checked = !!val;
        else el.value = val;
      }
      // keep hysteresis rows in sync
      const syncHyst = () => {
        const en = document.getElementById("wf_hyst_en")?.checked;
        document
          .querySelectorAll("[data-hyst-row]")
          .forEach((r) => r.classList.toggle("disabled", !en));
      };
      syncHyst();
    }

    document.getElementById("btnSave")?.addEventListener("click", () => {
      localStorage.setItem(SETTINGS_KEY, JSON.stringify(collectSettings()));
      console.log("[settings] saved");
    });

    document.getElementById("btnLoad")?.addEventListener("click", () => {
      const raw = localStorage.getItem(SETTINGS_KEY);
      if (!raw) return;
      applySettings(JSON.parse(raw));
      console.log("[settings] loaded");
    });

    // optional: auto-load on page open
    (() => {
      const raw = localStorage.getItem(SETTINGS_KEY);
      if (raw) applySettings(JSON.parse(raw));
    })();

    // ---- File save/load for settings ----
    (function () {
      const fileInput = document.getElementById("fileImport");

      // Save current UI settings to a .json file
      document.getElementById("btnExport")?.addEventListener("click", () => {
        const data = collectSettings(); // your existing function
        const blob = new Blob([JSON.stringify(data, null, 2)], {
          type: "application/json",
        });
        const a = document.createElement("a");
        a.href = URL.createObjectURL(blob);
        a.download = `signalLab-settings-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, "-")}.json`;
        a.click();
        URL.revokeObjectURL(a.href);
      });

      document.getElementById("btnImport")?.addEventListener("click", () => {
        fileInput?.click();
      });

      fileInput?.addEventListener("change", async (e) => {
        const f = e.target.files?.[0];
        if (!f) return;
        try {
          const text = await f.text();
          let obj = JSON.parse(text);

          const settings =
            obj && obj.settings && typeof obj.settings === "object"
              ? obj.settings
              : obj;

          applySettings(settings);

          console.log("[settings] loaded from file:", f.name);
        } catch (err) {
          console.error("Failed to load settings file:", err);
          alert("Invalid settings file (must be JSON).");
        } finally {
          e.target.value = ""; // reset so same file can be picked again
        }
      });
    })();

    // Upload button - send RAW JSON (server expects JSON body, not multipart)
    els.uploadBtn?.addEventListener("click", async () => {
      try {
        const f = els.uploadFile?.files?.[0];
        if (!f) {
          alert("Choose a JSON file first");
          return;
        }
        setUploadStatus("Uploading...", "info");
        const text = await f.text();

        try {
          JSON.parse(text);
        } catch {
          alert("File is not valid JSON");
          setUploadStatus("Upload failed: invalid JSON", "err");
          return;
        }

        const r = await fetch("/upload", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: text,
        });
        if (!r.ok) {
          const msg = await r.text();
          appendLog(`[upload] HTTP ${r.status}: ${msg}`);
          setUploadStatus(`Upload failed: ${msg}`, "err");
          alert(`Upload failed: ${msg}`);
          return;
        }
        const j = await r.json();
        await handleUploadResponse(j);
      } catch (e) {
        appendLog(`[upload] ${e.message || e}`);
        setUploadStatus(`Upload failed: ${e.message || e}`, "err");
      }
    });

    els.mapSelect?.addEventListener("change", () => {
      const panel = document.getElementById("panel-route");
      if (panel && !panel.hidden && els.mapSelect.value)
        loadRouteLeaflet(els.mapSelect.value);
      localStorage.setItem("lastMap", els.mapSelect.value);
    });

    initTabs();
  });

  let segments = [];
  let segMap = null;
  let currentSegment = -1;

  window.onWaveletResponse = function (out) {
    segments = Array.isArray(out.segments) ? out.segments : [];
    currentSegment = -1;
    const btn = document.querySelector('[data-tab="segments"]');
    if (btn) btn.disabled = segments.length === 0;
    const btnExt = document.querySelector('[data-tab="extensions"]');
    if (btnExt) btnExt.disabled = segments.every((s) => !s.extensions);
  };

  document.getElementById("labTabs").addEventListener("click", (e) => {
    const tab = e.target.dataset.tab;
    if (!tab) return;
  });

  function renderSegments(segs) {
    const container = document.querySelector(".segments-container");
    if (!container) return;
    container.innerHTML = "";
    segs.forEach((seg, i) => {
      const label = seg.label || `Segment ${i + 1}`;
      const len = seg.length != null ? `${seg.length}m` : "";
      const btn = document.createElement("button");
      btn.className = "segment-btn";
      btn.dataset.seg = i;
      btn.textContent = len ? `${label} (${len})` : label;
      btn.addEventListener("click", () => selectSegment(i));
      container.appendChild(btn);
    });
  }
  function renderSegmentExtensions(seg) {
    if (!els.extPlot) return;
    if (!seg || !seg.extensions) {
      els.extPlot.innerHTML = "<em>No extension data</em>";
      return;
    }
    const ext = seg.extensions;
    const x = Array.isArray(ext.s_km) ? ext.s_km : [];
    const traces = [];
    Object.entries(ext).forEach(([k, arr]) => {
      if (k === "s_km" || !Array.isArray(arr)) return;

      const yRaw = arr.map((v) =>
        v == null || typeof v === "object" ? null : v,
      );
      let y;
      try {
        y = smoothSG(yRaw);
      } catch {
        y = smoothEMA(yRaw, 0.25);
      }
      traces.push({ x, y, name: k, mode: "lines" });
    });
    if (!traces.length) {
      els.extPlot.innerHTML = "<em>No extension data</em>";
      return;
    }
    Plotly.newPlot(els.extPlot, traces, {
      margin: { t: 20 },
      xaxis: { title: "Distance (km)" },
    });
  }

  function initSegMap() {
    if (window.MAPBOX_TOKEN && typeof window.render3DSegment === "function")
      return;
    if (segMap) return;
    segMap = L.map("seg-map", { scrollWheelZoom: false });
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png").addTo(
      segMap,
    );
  }

  function selectSegment(i) {
    const seg = segments[i];
    if (!seg || !Array.isArray(seg.coordinates)) return;
    currentSegment = i;

    if (window.MAPBOX_TOKEN && typeof window.render3DSegment === "function") {
      window.render3DSegment(seg);
    } else {
      if (!segMap) initSegMap();
      if (segMap._poly) segMap.removeLayer(segMap._poly);
      segMap._poly = L.polyline(seg.coordinates, {
        weight: 4,
        color: "#2ad",
      }).addTo(segMap);
      segMap.fitBounds(segMap._poly.getBounds(), { padding: [10, 10] });
    }

    const container = document.querySelector(".segments-container");
    container.querySelectorAll(".segment-btn").forEach((b) => {
      b.classList.toggle("active", +b.dataset.seg === i);
    });
    const btn = container.querySelector(`.segment-btn[data-seg="${i}"]`);
    if (btn) {
      btn.scrollIntoView({ inline: "center", block: "nearest" });
    }
    const extPanel = document.getElementById("panel-extensions");
    if (extPanel && !extPanel.hidden) renderSegmentExtensions(seg);
  }


})();
