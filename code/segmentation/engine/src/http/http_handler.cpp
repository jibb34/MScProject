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

  // Load file
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

  // Try to parse & build your RouteSignal
  std::ostringstream js;
  bool have_signal = false;
  std::string parse_error;

  try {
    OsrmResponse osrm = body.get<OsrmResponse>();
    RouteSignalBuilder builder;
    RouteSignal rs = builder.build(
        osrm); // or osrm.matchings.front(), per your builder signature

    // Build JS arrays from the signal
    js << "var coords = [";
    for (const auto &dp : rs.points) {
      js << "[" << dp.coord.lat << "," << dp.coord.lon << "],";
    }
    js << "];\n";

    // Optional: curvature vs distance (adapt names to your DataPoint fields)
    js << "var dist = [";
    for (const auto &dp : rs.points)
      js << dp.cum_dist << ",";
    js << "];\nvar curvature = [";
    for (const auto &dp : rs.points)
      js << dp.curvature << ",";
    js << "];\n";

    have_signal = !rs.points.empty();
  } catch (const std::exception &e) {
    parse_error = e.what();
  }

  // If building failed, try fallback to geometry->coordinates
  if (!have_signal) {
    try {
      const auto &coords = body["matchings"][0]["geometry"]["coordinates"];
      js << "var coords = [";
      for (const auto &p : coords) {
        // GeoJSON is [lon, lat]; Leaflet expects [lat, lon]
        js << "[" << p[1].get<double>() << "," << p[0].get<double>() << "],";
      }
      js << "];\n";
      // leave dist/curvature undefined; we'll plot only the map
    } catch (...) {
      res.status = 400;
      res.set_content("Could not build signal or read geometry; last error: " +
                          parse_error,
                      "text/plain");
      return;
    }
  }

  // HTML: Leaflet map + (if arrays exist) Plotly curvature chart
  std::string html = R"(
<!doctype html><html><head>
<meta charset="utf-8"/>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
<style>body{margin:0;font-family:sans-serif} #map{height:60vh;} #plot{height:35vh;}</style>
</head><body>
<h3 style="margin:8px">Route Viewer</h3>
<div id="map"></div>
<div id="plot"></div>
<script>
)" + js.str() + R"(
  var map = L.map('map');
  var tile = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19});
  tile.addTo(map);
  var poly = L.polyline(coords, {weight:4}).addTo(map);
  map.fitBounds(poly.getBounds());

  // Only draw curvature if data exists
  if (typeof dist !== 'undefined' && typeof curvature !== 'undefined') {
    Plotly.newPlot('plot', [{x: dist, y: curvature, mode: 'lines', name: 'Curvature'}],
                   {title: 'Curvature vs Distance', margin: {t:30}});
  } else {
    document.getElementById('plot').innerHTML =
      '<div style="padding:12px;color:#666">No curvature data available (showing map only).</div>';
  }
</script>
</body></html>)";

  res.set_content(html, "text/html");
}
