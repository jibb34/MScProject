#include "http_handler.hpp"
#include "core/RouteSignalBuilder.hpp"
#include "debug/json_debug.hpp"
#include "debug/osrm_inspect.hpp"
#include "httplib.h"
#include "io/json_parser.hpp"
#include "json.hpp"
#include "models/OsrmResponse.hpp"
#include "models/params.hpp"
#include <cctype>     // std::isalnum in is_safe_basename
#include <chrono>     // timestamps for filenames
#include <filesystem> // std::filesystem::create_directories
#include <fstream>    // std::ifstream, std::ofstream
#include <iomanip>
#include <iostream>
#include <random> // rng for filenames

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

  // We’ll emit both sources for the map, plus elevation from RS
  js << "var coords_osrm = [];\n";
  js << "var coords_rs = [];\n";
  js << "var elev_rs = [];\n"; // elevation array matching coords_rs

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

  // 2) RouteSignal overlay + elevation
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
      // Elevation array aligned with coords_rs
      js << "elev_rs = [";
      for (const auto &dp : rs.points) {
        // Adjust field name if your struct uses something like dp.coord.alt or
        // dp.ele
        js << dp.coord.elv << ","; // <—— uses dp.coord.elv as you described
      }
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

  // HTML: map + elevation chart (Plotly), with in-page distance compute
  std::string html = R"(
<!doctype html><html><head>
<meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
<style>
  body{margin:0;font-family:sans-serif}
  #map{height:60vh;} #plot{height:35vh;}
  #diag{padding:10px; font:12px/1.4 monospace;}
</style>
</head><body>
<h3 style="margin:8px">Route Viewer (OSRM vs RouteSignal) + Elevation</h3>
<div id="map"></div>
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

  var map = L.map('map');
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19}).addTo(map);

  function addPolyline(coords, opts) {
    if (!coords || coords.length < 2) return null;
    return L.polyline(coords, Object.assign({weight:4, smoothFactor:0}, opts)).addTo(map);
  }

  var bounds=null;
  var osrmLine = (coords_osrm && coords_osrm.length>1) ? addPolyline(coords_osrm, {color:'#1f77b4'}) : null;
  if (osrmLine) bounds = osrmLine.getBounds();
  var rsLine   = (coords_rs   && coords_rs.length>1)   ? addPolyline(coords_rs,   {color:'#d62728', dashArray:'4 4'}) : null;
  if (rsLine)  bounds = bounds ? bounds.extend(rsLine.getBounds()) : rsLine.getBounds();
  if (bounds) map.fitBounds(bounds);

  const minElv = Math.min.apply(null, elev_rs);
  const maxElv = Math.max.apply(null, elev_rs);
  console.log('elev_rs stats:', { n: elev_rs.length, minElv, maxElv });
  // Elevation plot (from RS). We compute distance from coords_rs.
  if (coords_rs && coords_rs.length>1 && elev_rs && elev_rs.length===coords_rs.length) {
    const dist_m = cumdist(coords_rs);
    const dist_km = dist_m.map(x => x/1000.0);

    Plotly.newPlot('plot', [{
      x: dist_km,
      y: elev_rs,
      mode: 'lines',
      name: 'Elevation',
      line: {shape:'linear'}
    }], {
      title: 'Elevation vs Distance',
      xaxis: {title: 'Distance (km)'},
      yaxis: {title: 'Elevation (m)'},
      margin: {t: 30}
    });
  } else {
    document.getElementById('plot').innerHTML =
      '<div style="padding:12px;color:#666">No elevation data available for RouteSignal.</div>';
  }

  // Diagnostics
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
`OSRM: pts=${coords_osrm.length} steps=${os.n}  min=${os.min.toFixed(2)}m p50=${os.p50.toFixed(2)}m p95=${os.p95.toFixed(2)}m max=${os.max.toFixed(2)}m avg=${os.avg.toFixed(2)}m
RS  : pts=${coords_rs.length}   steps=${rs.n}  min=${rs.min.toFixed(2)}m p50=${rs.p50.toFixed(2)}m p95=${rs.p95.toFixed(2)}m max=${rs.max.toFixed(2)}m avg=${rs.avg.toFixed(2)}m`;
</script>
</body></html>)";

  res.set_content(html, "text/html");
}
