#include "http_handler.hpp"
#include "core/RouteSignalBuilder.hpp"
#include "debug/json_debug.hpp"
#include "debug/osrm_inspect.hpp"
#include "httplib.h"
#include "io/json_parser.hpp"
#include "json.hpp"
#include "models/params.hpp"
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

using json = nlohmann::json;

// Constructor

void HttpHandler::registerRoutes(httplib::Server &server) {
  // TODO: Implement route input for database matching
}

void HttpHandler::callPostHandler(std::string action,
                                  const httplib::Request &req,
                                  httplib::Response &res) {
  // NOTE: This section is where all selection of routing occurs based on the
  // endpoint
  std::cerr << "[Handler] Received handler request... action:" + action + ".\n";
  if (action == "segment") {
    handleSegment(req, res);
  } else if (action == "debug") {
    handleDebug(req, res);
  } else if (action == "upload") {
    handleUpload(req, res);
  }

  else {

    res.status = 404;
    res.set_content("Unknown action: " + action, "text/plain");
  }
}
void HttpHandler::callGetHandler(std::string action,
                                 const httplib::Request &req,
                                 httplib::Response &res) {
  if (action == "view") {
    handleView(req, res);
  }

  else {

    res.status = 404;
    res.set_content("Unknown action: " + action, "text/plain");
  }
}

void HttpHandler::handleDebug(const httplib::Request &req,
                              httplib::Response &res) {
  const bool validate =
      req.has_param("validate") && (req.get_param_value("validate") == "true");

  // Validation-only mode: just parse and report precise location on error
  if (validate) {
    try {
      auto _ = nlohmann::json::parse(req.body);
      nlohmann::json ok = {{"ok", true},
                           {"message", "JSON parsed successfully"}};
      res.set_content(ok.dump(), "application/json");
      return;
    } catch (const nlohmann::json::parse_error &e) {
      const size_t byte = e.byte; // byte offset where parse failed
      auto [line, col] = calc_line_col(req.body, byte);
      nlohmann::json err = {{"ok", false},
                            {"kind", "parse_error"},
                            {"what", e.what()},
                            {"byte", byte},
                            {"line", line},
                            {"column", col},
                            {"context", context_snippet(req.body, byte)}};
      res.status = 400;
      res.set_content(err.dump(), "application/json");
      return;
    } catch (const std::exception &e) {
      nlohmann::json err = {
          {"ok", false}, {"kind", "exception"}, {"what", e.what()}};
      res.status = 400;
      res.set_content(err.dump(), "application/json");
      return;
    }
  }

  const std::string inspect =
      req.has_param("inspect") ? req.get_param_value("inspect") : "";
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(req.body);
  } catch (const nlohmann::json::parse_error &e) {
    const size_t byte = e.byte;
    auto [line, col] = calc_line_col(req.body, byte);
    nlohmann::json err = {{"ok", false},
                          {"kind", "parse_error"},
                          {"what", e.what()},
                          {"byte", byte},
                          {"line", line},
                          {"column", col},
                          {"context", context_snippet(req.body, byte)}};
    res.status = 400;
    res.set_content(err.dump(2), "application/json");
    return;
  }

  if (inspect == "osrm") {
    try {
      // Use your model; if you prefer “first matching only”, use
      // OsrmMatchSingle instead
      OsrmResponse resp = body.get<OsrmResponse>();
      auto info = summarize(resp); // compact summary JSON
      res.set_content(info.dump(2), "application/json");
      return;
    } catch (const std::exception &e) {
      nlohmann::json err = {
          {"ok", false}, {"kind", "type_error"}, {"what", e.what()}};
      res.status = 400;
      res.set_content(err.dump(2), "application/json");
      return;
    }
  }

  if (inspect == "geojson") {
    // Old behavior: count coordinates from a FeatureCollection
    size_t point_count = 0;
    for (const auto &feat : body.value("features", nlohmann::json::array())) {
      const auto &coords = feat["geometry"]["coordinates"];
      if (!coords.is_array())
        continue;
      if (coords.size() > 0 && coords[0].is_array() &&
          coords[0][0].is_array()) {
        for (const auto &line : coords)
          point_count += line.size(); // MultiLineString
      } else {
        point_count += coords.size(); // LineString
      }
    }
    nlohmann::json out = {{"ok", true}, {"points", point_count}};
    res.set_content(out.dump(2), "application/json");
    return;
  }

  res.set_content(R"({"ok":true,"message":"debug alive"})", "application/json");
}

void HttpHandler::handleSegment(const httplib::Request &req,
                                httplib::Response &res) {
  // Parse parameters
  // NOTE: update parameters in here, and in /core/params.hpp to add or remove
  // any parameters.
  SegmentationParams params;
  try {
    if (req.has_param("min_segment_length_m"))
      params.min_segment_length_m =
          std::stod(req.get_param_value("min_segment_length_m"));
    if (req.has_param("split_on_highway_change"))
      params.split_on_highway_change =
          (req.get_param_value("split_on_highway_change") == "true");
    if (req.has_param("max_points"))
      params.max_points = std::stoi(req.get_param_value("max_points"));
    if (req.has_param("job_id"))
      params.job_id = req.get_param_value("job_id");
  } catch (...) {
    res.status = 400;
    res.set_content("Invalid query parameter types", "text/plain");
    return;
  }

  // Parse JSON file from input
  json body;
  try {
    body = json::parse(req.body);
  } catch (...) {
    res.status = 400;
    res.set_content("Invalid JSON body", "text/plain");
  }

  OsrmResponse osrm_object;
  try {
    osrm_object = body.get<OsrmResponse>();
  } catch (const std::exception &e) {
    res.status = 400;
    res.set_content(std::string("Parse error: ") + e.what(), "text/plain");
    return;
  }
  // TODO: Implement Statistics function, and send to segmentation machine.

  // Example usage of OsrmStatistics:
  // float tp_count = stats.get_total_tracepoints(osrm_object);
  // Bbox bbox = stats.get_bounds(osrm_object);
  //
  // Generate segments
  // SegmentEngine sm = new SegmentEngine(args_from_post)
  // std::vector<std::vector<Leg>> segments
  // segments = sm.segment(osrm_object)
  // array of leg arrays, each leg array = 1 segment
}

void HttpHandler::handleUpload(const httplib::Request &req,
                               httplib::Response &res) {
  // Validate JSON first
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(req.body);
  } catch (const std::exception &e) {
    res.status = 400;
    res.set_content(std::string("Invalid JSON: ") + e.what(), "text/plain");
    return;
  }
  std::cerr << "[HTTP] Receiving JSON payload..." << std::endl;

  // Create uploads dir if needed
  std::error_code ec;
  std::filesystem::create_directories("uploads", ec);

  // Generate a simple filename
  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  std::mt19937_64 rng{static_cast<unsigned long long>(ms)};
  unsigned long long r = rng();

  std::string filename =
      "uploads/map_" + std::to_string(ms) + "_" + std::to_string(r) + ".json";

  // Save pretty-printed (easier to eyeball)
  try {
    std::ofstream out(filename);
    out << body.dump(2);
    out.close();
  } catch (const std::exception &e) {
    res.status = 500;
    res.set_content(std::string("Failed to save: ") + e.what(), "text/plain");
    return;
  }

  nlohmann::json ok = {{"ok", true}, {"file", filename}};
  res.set_content(ok.dump(), "application/json");
}

static bool is_safe_basename(const std::string &s) {
  for (char c : s) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '_' ||
          c == '-' || c == '/'))
      return false;
  }
  return s.rfind("uploads/", 0) == 0; // must start with uploads/
}

// ===================== Handle View ===================
void HttpHandler::handleView(const httplib::Request &req,
                             httplib::Response &res) {
  if (!req.has_param("map")) {
    res.status = 400;
    res.set_content("Missing ?map=uploads/<file>.json", "text/plain");
    return;
  }
  std::string path = req.get_param_value("map");
  if (!is_safe_basename(path)) {
    res.status = 400;
    res.set_content("Bad map path", "text/plain");
    return;
  }

  // Load JSON file
  nlohmann::json body;
  try {
    std::ifstream in(path);
    if (!in) {
      res.status = 404;
      res.set_content("File not found", "text/plain");
      return;
    }
    in >> body;
  } catch (const std::exception &e) {
    res.status = 400;
    res.set_content(std::string("Cannot read JSON: ") + e.what(), "text/plain");
    return;
  }

  std::ostringstream js;
  js.setf(std::ios::fixed);
  js << std::setprecision(7);

  // Arrays for map + charts
  js << "var coords_osrm = [];\n";
  js << "var coords_rs  = [];\n";
  js << "var elev_rs    = [];\n";
  js << "var grad_rs    = [];\n";
  js << "var heading_delta = [];\n"; // NEW: headings in radians
  bool have_osrm = false, have_rs = false;
  std::string parse_error;

  // 1) OSRM GeoJSON geometry: matchings[0].geometry.coordinates
  try {
    if (body.contains("matchings") && body["matchings"].is_array() &&
        !body["matchings"].empty()) {
      const auto &geom = body["matchings"][0]["geometry"];
      if (geom.is_object() && geom.contains("coordinates") &&
          geom["coordinates"].is_array()) {
        const auto &coords = geom["coordinates"];
        if (!coords.empty()) {
          js << "coords_osrm = [";
          for (const auto &p : coords) {
            // GeoJSON [lon,lat] -> Leaflet [lat,lon]
            js << "[" << p[1].get<double>() << "," << p[0].get<double>()
               << "],";
          }
          js << "];\n";
          have_osrm = true;
        }
      }
    }
  } catch (const std::exception &e) {
    parse_error = e.what();
  }

  // 2) RouteSignal overlay + elevation + gradient + heading
  try {
    OsrmResponse osrm = body.get<OsrmResponse>();
    RouteSignalBuilder builder;
    RouteSignal rs = builder.build(osrm);

    if (!rs.points.empty()) {
      js << "coords_rs = [";
      for (const auto &dp : rs.points) {
        js << "[" << dp.coord.lat << "," << dp.coord.lon << "],";
      }
      js << "];\n";

      js << "elev_rs = [";
      for (const auto &dp : rs.points)
        js << dp.coord.elv << ",";
      js << "];\n";

      js << "grad_rs = [";
      for (const auto &dp : rs.points)
        js << dp.gradient << ",";
      js << "];\n";

      // way_ids aligned 1:1 with coords_rs
      js << "way_rs = [";
      for (const auto &dp : rs.points) {
        js << dp.way_id << ",";
      }
      js << "];\n";

      js << "heading_delta = [";
      for (const auto &dp : rs.points)
        js << dp.heading_delta << ",";
      js << "];\n";

      // after elev_rs / grad_rs
      js << "speed_rs = [";
      for (const auto &dp : rs.points)
        js << dp.speed << ",";
      js << "];\n";

      have_rs = true;
    }
  } catch (const std::exception &e) {
    if (parse_error.empty())
      parse_error = e.what();
  }

  if (!have_osrm && !have_rs) {
    res.status = 400;
    res.set_content("No coordinates to draw (no OSRM geometry and no "
                    "RouteSignal). Last error: " +
                        parse_error,
                    "text/plain");
    return;
  }

  // HTML: map + HEADING plot + elevation chart
  std::string html = R"(
<!doctype html><html><head>
<meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
<style>
  body{margin:0;font-family:sans-serif}
  #map{height:50vh;}
  #plotHeading{height:22vh;}  /* heading on top */
  #plot{height:28vh;}         /* squish elevation a bit */
  #diag{padding:10px; font:12px/1.4 monospace;}
</style>
</head><body>
<h3 style="margin:8px">Route Viewer (OSRM vs RouteSignal)</h3>
<div id="map"></div>
<div id="plotHeading"></div>
<div id="plot"></div>
<pre id="diag"></pre>
<script>
)" + js.str() + R"(

  // Helpers
  function haversine(a,b){
    const R=6371000, toRad=d=>d*Math.PI/180;
    const dφ=toRad(b[0]-a[0]), dλ=toRad(b[1]-a[1]);
    const φ1=toRad(a[0]), φ2=toRad(b[0]);
    const s=Math.sin(dφ/2)**2 + Math.cos(φ1)*Math.cos(φ2)*Math.sin(dλ/2)**2;
    return 2*R*Math.asin(Math.sqrt(s));
  }
  function cumdist(coords){
    const d=[0]; let acc=0;
    for (let i=1;i<coords.length;i++){ acc += haversine(coords[i-1], coords[i]); d.push(acc); }
    return d;
  }

  var map = window.L.map('map');
  window.L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);

  function addPolyline(coords, opts) {
    if (!coords || coords.length < 2) return null;
    return window.L.polyline(coords, Object.assign({weight:4, smoothFactor:0}, opts)).addTo(map);
  }

  var bounds=null;
  var osrmLine = (coords_osrm && coords_osrm.length>1) ? addPolyline(coords_osrm, {color:'#1f77b4'}) : null;
  if (osrmLine) bounds = osrmLine.getBounds();
  // Coloured RS segments by way_id, cycling RGB binary colours 0..7
// 000,001,010,011,100,101,110,111
const wayColors = [
  '#000000', // 0: 000 (black)
  '#0000FF', // 1: 001 (blue)
  '#00FF00', // 2: 010 (green)
  '#00FFFF', // 3: 011 (cyan)
  '#FF0000', // 4: 100 (red)
  '#FF00FF', // 5: 101 (magenta)
  '#FFFF00', // 6: 110 (yellow)
  '#FFFFFF'  // 7: 111 (white)
];

if (coords_rs && coords_rs.length > 1 && Array.isArray(way_rs) && way_rs.length === coords_rs.length) {
  let segStart = 0;
  let colourIndex = 0;
  let segBounds = null;

  for (let i = 1; i < coords_rs.length; i++) {
    if (way_rs[i] !== way_rs[i-1]) {
      const seg = coords_rs.slice(segStart, i+1);
      const color = (way_rs[i-1] < 0) ? '#888888' : wayColors[colourIndex % 8];
      const pl = addPolyline(seg, { color, weight: 4, smoothFactor: 0 });
      if (pl) segBounds = segBounds ? segBounds.extend(pl.getBounds()) : pl.getBounds();
      segStart = i;
      colourIndex++;
    }
  }
  // tail segment
  if (segStart < coords_rs.length - 1) {
    const seg = coords_rs.slice(segStart);
    const prevWay = way_rs[Math.max(0, segStart - 1)];
    const color = (prevWay < 0) ? '#888888' : wayColors[colourIndex % 8];
    const pl = addPolyline(seg, { color, weight: 4, smoothFactor: 0 });
    if (pl) segBounds = segBounds ? segBounds.extend(pl.getBounds()) : pl.getBounds();
  }

  if (segBounds) bounds = bounds ? bounds.extend(segBounds) : segBounds;
} else {
  // Fallback single-colour RS line if way_ids missing/misaligned
  var rsLine = (coords_rs && coords_rs.length>1) ? addPolyline(coords_rs, {color:'#d62728', dashArray:'4 4'}) : null;
  if (rsLine) bounds = bounds ? bounds.extend(rsLine.getBounds()) : rsLine.getBounds();
}

  if (bounds) map.fitBounds(bounds);

  if (coords_rs && coords_rs.length>1 &&
      elev_rs   && elev_rs.length===coords_rs.length &&
      grad_rs   && grad_rs.length===coords_rs.length &&
      heading_delta&& heading_delta.length===coords_rs.length) {

    const dist_m  = cumdist(coords_rs);
    const dist_km = dist_m.map(x => x/1000.0);
    const n = elev_rs.length;

  // ====== HEADING PLOT (−π .. π) ======
  const PI = Math.PI, TWO_PI = 2*PI;

  // Wrap any radians to (−π, π]
  function wrapPi(t){
    if (!isFinite(t)) return NaN;
    t = (t + PI) % TWO_PI;       // (-π, π] modulo
    if (t < 0) t += TWO_PI;
    return t - PI;
  }

  // Make a wrapped copy to plot
  const head = heading_delta.map(wrapPi);

  // Split into runs to avoid vertical jumps at the wrap
  const headingTraces = [];
  let runStart = 0;
  for (let i = 1; i < n; i++) {
    const dθ = Math.abs(head[i] - head[i-1]);
    if (!isFinite(dθ) || dθ > PI) {   // crossed the wrap
      if (i - runStart >= 2) {
        headingTraces.push({
          x: dist_km.slice(runStart, i),
          y: head.slice(runStart, i),
          mode: 'lines',
          name: 'Heading',
          line: { width: 1.6, color: '#7e57c2' },
          hovertemplate: 'd=%{x:.3f} km<br>θ=%{y:.2f} rad<extra></extra>',
          showlegend: (runStart === 0) // one legend entry
        });
      }
      runStart = i;
    }
  }
  // push last run
  if (runStart < n) {
    headingTraces.push({
      x: dist_km.slice(runStart),
      y: head.slice(runStart),
      mode: 'lines',
      name: 'Heading',
      line: { width: 1.6, color: '#7e57c2' },
      hovertemplate: 'd=%{x:.3f} km<br>θ=%{y:.2f} rad<extra></extra>',
      showlegend: (runStart === 0 && headingTraces.length === 0)
    });
  }

  Plotly.newPlot('plotHeading', headingTraces, {
    title: 'Heading (radians)',
    xaxis: { title: 'Distance (km)' },
    yaxis: {
      title: 'θ',
      range: [-PI, PI],
      tickvals: [-PI, -0.5*PI, 0, 0.5*PI, PI],
      ticktext: ['−π', '−π/2', '0', 'π/2', 'π']
    },
    margin: { t: 40 },
    showlegend: true,
    legend: { orientation: 'h' }
  });

    // ====== ELEVATION PLOT (your continuous-colour fill, unchanged) ======
    // If your grad is a fraction, convert to % here instead of slice():
    const grad_pct = grad_rs.slice(); // assume already %

    // Weighted smoothing that favours higher |grad|
    const SMOOTH_WIN_M = 50, SIGMA = SMOOTH_WIN_M/3, BOOST = 0.6, G0 = 10;
    const sgrad = new Array(n); let leftWin=0, rightWin=0;
    for (let i=0;i<n;i++) {
      while (leftWin<i && dist_m[i] - dist_m[leftWin] > SMOOTH_WIN_M) leftWin++;
      while (rightWin+1<n && dist_m[rightWin+1] - dist_m[i] <= SMOOTH_WIN_M) rightWin++;
      let sum=0, wsum=0;
      for (let k=leftWin;k<=rightWin;k++) {
        const dd = Math.abs(dist_m[k] - dist_m[i]);
        const w_dist = Math.exp(-(dd*dd)/(2*SIGMA*SIGMA));
        const g = grad_pct[k];
        const w_mag  = 1 + BOOST * Math.min(Math.abs(g)/G0, 1);
        const w = w_dist * w_mag;
        sum += w * g; wsum += w;
      }
      sgrad[i] = wsum ? (sum/wsum) : grad_pct[i];
    }

    // colour ramp
    function hexToRgb(h){const m=/^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(h);
      return m?{r:parseInt(m[1],16),g:parseInt(m[2],16),b:parseInt(m[3],16)}:null;}
    function rgbToHex({r,g,b}){const h=n=>n.toString(16).padStart(2,'0');return '#'+h(r)+h(g)+h(b);}
    function lerp(a,b,t){return a+(b-a)*t;}
    function lerpColor(c1,c2,t){const A=hexToRgb(c1),B=hexToRgb(c2);return rgbToHex({
      r:Math.round(lerp(A.r,B.r,t)), g:Math.round(lerp(A.g,B.g,t)), b:Math.round(lerp(A.b,B.b,t))});}
    const stops=[{g:-10,c:'#1f77b4'},{g:0,c:'#1f77b4'},{g:4,c:'#2ca02c'},{g:8,c:'#ffd700'},{g:12,c:'#d62728'},{g:15,c:'#d62728'},{g:25,c:'#000000'}];
    function colorForGrad(g){for (let i=0;i<stops.length-1;i++){const a=stops[i],b=stops[i+1];if (g<=a.g) return a.c; if (g<b.g) return lerpColor(a.c,b.c,(g-a.g)/(b.g-a.g));} return stops[stops.length-1].c;}

    // Adaptive segment lengths
    const SEG_LEN_MIN=12, SEG_LEN_MAX=40, G1=10;
    function segLenForGrad(g){const t=Math.max(0,Math.min(1,Math.abs(g)/G1));return SEG_LEN_MAX-(SEG_LEN_MAX-SEG_LEN_MIN)*t;}

    const traces=[];
    let segStart=0;
    while (segStart<n-1){
      const currLen=segLenForGrad(sgrad[segStart]);
      let i=segStart+1;
      while (i<n && (dist_m[i]-dist_m[segStart])<currLen) i++;
      if (i<=segStart) i=Math.min(segStart+1,n-1);
      const mid=Math.floor((segStart+i)/2);
      const col=colorForGrad(sgrad[mid]);
      traces.push({
        x: dist_km.slice(segStart,i+1),
        y: elev_rs.slice(segStart,i+1),
        mode:'lines', line:{width:0}, fill:'tozeroy', fillcolor:col,
        connectgaps:false,
        hovertemplate:'d=%{x:.3f} km<br>elev=%{y:.1f} m<br>grad~'+sgrad[mid].toFixed(1)+'%<extra></extra>',
        showlegend:false
      });
      segStart=i;
    }
    // outline on top
    traces.push({ x:dist_km, y:elev_rs, mode:'lines', name:'Elevation', line:{color:'#222',width:1.2}, hoverinfo:'skip' });

      // --- Speed overlay (right axis) ---
  const speed_kmh = (typeof speed_rs !== 'undefined' && speed_rs.length === dist_km.length)
    ? speed_rs.map(v => v * 3.6)    // m/s -> km/h
    : new Array(dist_km.length).fill(null); // safe fallback

  const speedTrace = {
    x: dist_km,
    y: speed_kmh,
    yaxis: 'y2',
    mode: 'lines',
    name: 'Speed',
    line: { width: 1.4, dash: 'dot' },
    hovertemplate: 'd=%{x:.3f} km<br>speed=%{y:.1f} km/h<extra></extra>'
  };

traces.push(speedTrace);

    Plotly.newPlot('plot', traces, {
      title: 'Elevation vs Distance (continuous-colour fill, high-grad emphasis)',
      xaxis: { title: 'Distance (km)' },
      yaxis: { title: 'Elevation (m)' },
      yaxis2: {
        title: 'Speed (km/h)',
        overlaying: 'y',
        side: 'right',
        rangemode: 'tozero'
      },
      legend: { orientation: 'h' },
      margin: { t: 40,r: 50 }
    });

    // --- Map-hover pin synced to BOTH charts ---
    const hoverMarker = window.L.marker(coords_rs[0], { opacity: 0 }).addTo(map);
    function showMarker(latlng){ hoverMarker.setLatLng(latlng).setOpacity(1); }
    function hideMarker(){ hoverMarker.setOpacity(0); }

    function latLngAtDistance(d) {
      if (d <= 0) return coords_rs[0];
      const total = dist_m[dist_m.length - 1];
      if (d >= total) return coords_rs[coords_rs.length - 1];
      let lo = 0, hi = dist_m.length - 1;
      while (lo + 1 < hi) {
        const mid = (lo + hi) >> 1;
        if (dist_m[mid] <= d) lo = mid; else hi = mid;
      }
      const seg = dist_m[hi] - dist_m[lo] || 1;
      const t = (d - dist_m[lo]) / seg;
      const lat = coords_rs[lo][0] + t * (coords_rs[hi][0] - coords_rs[lo][0]);
      const lon = coords_rs[lo][1] + t * (coords_rs[hi][1] - coords_rs[lo][1]);
      return [lat, lon];
    }

    function wireHover(divId){
      const el = document.getElementById(divId);
      el.on('plotly_hover', (ev) => {
        if (!ev.points || !ev.points.length) return;
        const xkm = ev.points[0].x;
        if (typeof xkm !== 'number' || !isFinite(xkm)) return;
        const d = xkm * 1000.0;
        showMarker(latLngAtDistance(d));
      });
      el.on('plotly_unhover', hideMarker);
    }
    wireHover('plotHeading');
    wireHover('plot');

  } else {
    document.getElementById('plot').innerHTML =
      '<div style="padding:12px;color:#c00">Missing or mismatched arrays (coords/elev/grad/heading).</div>';
  }

  // Diagnostics (optional)
  function stepStats(coords){
    if (!coords || coords.length<2) return null;
    const steps=[]; for (let i=1;i<coords.length;i++) steps.push(haversine(coords[i-1], coords[i]));
    const s=[...steps].sort((a,b)=>a-b), q=p=>s[Math.floor((s.length-1)*p)];
    const avg=steps.reduce((a,b)=>a+b,0)/steps.length;
    return {n:steps.length, min:s[0], p50:q(0.5), p95:q(0.95), max:s[s.length-1], avg};
  }
  const os = stepStats(coords_osrm) || {n:0,min:0,p50:0,p95:0,max:0,avg:0};
  const rs = stepStats(coords_rs)   || {n:0,min:0,p50:0,p95:0,max:0,avg:0};
  document.getElementById('diag').textContent =
`OSRM: pts=${coords_osrm.length} steps=${os.n}  min=${(os.min||0).toFixed(2)}m p50=${(os.p50||0).toFixed(2)}m p95=${(os.p95||0).toFixed(2)}m max=${(os.max||0).toFixed(2)}m avg=${(os.avg||0).toFixed(2)}m
RS  : pts=${coords_rs.length}   steps=${rs.n}  min=${(rs.min||0).toFixed(2)}m p50=${(rs.p50||0).toFixed(2)}m p95=${(rs.p95||0).toFixed(2)}m max=${(rs.max||0).toFixed(2)}m avg=${(rs.avg||0).toFixed(2)}m`;
</script>
</body></html>)";

  res.set_content(html, "text/html");
}
