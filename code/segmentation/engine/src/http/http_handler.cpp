#include "http_handler.hpp"
#include "core/RouteSignalBuilder.hpp"
#include "core/wavelets/WaveletFootprint.hpp"
#include "debug/json_debug.hpp"
#include "debug/osrm_inspect.hpp"
#include "httplib.h"
#include "io/json_parser.hpp"
#include "models/params.hpp"
#include "nlohmann/json.hpp"

#include "infra/MySQLSegmentDB.hpp"
#include <algorithm> // sort
#include <cctype>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits> // numeric_limits
#include <random>
#include <set> // set

using json = nlohmann::json;

// forward decl so we can use it before definition
static bool is_safe_basename(const std::string &);

// tiny helper for Signal Lab
static double pick_value(const DataPoint &d, const std::string &var) {
  if (var == "elev")
    return d.coord.elv;
  if (var == "speed_mps")
    return d.speed_smoothed;
  if (var == "gradient_pct")
    return d.gradient;
  if (var == "heading_delta")
    return d.heading_delta_norm;
  return std::numeric_limits<double>::quiet_NaN();
}

// ===== routes =====

// void HttpHandler::registerRoutes(httplib::Server &server) {
//   // (keep as-is or add routes here)
// }

void HttpHandler::callPostHandler(std::string action,
                                  const httplib::Request &req,
                                  httplib::Response &res) {
  if (action == "segment") {
    // handleSegment(req, res);
  } else if (action == "debug") {
    handleDebug(req, res);
  } else if (action == "upload") {
    handleUpload(req, res);
  } else if (action == "lab/resample") {
    handleLabResample(req, res);
  } else if (action == "wavelet") {
    handleWavelet(req, res);

  } else {
    res.status = 404;
    res.set_content("Unknown action: " + action, "text/plain");
  }
}
void HttpHandler::callGetHandler(std::string action,
                                 const httplib::Request &req,
                                 httplib::Response &res) {
  if (action == "view") {
    handleView(req, res);
  } else if (action == "viewLab") {
    handleSignalLabUI(req, res);
  } else if (action == "labMeta") {
    handleLabMeta(req, res);
  } else if (action == "dbping") {
    handleDBPing(req, res);
  } else if (action == "segments") {
    handleSegments(req, res);
  }
  // default
  else {
    res.status = 404;
    res.set_content("Unknown action: " + action, "text/plain");
  }
}

// ===== debug =====

void HttpHandler::handleDebug(const httplib::Request &req,
                              httplib::Response &res) {
  const bool validate =
      req.has_param("validate") && (req.get_param_value("validate") == "true");

  // Signal Lab POST endpoint: /debug?lab=signal
  if (req.method == "POST" && req.has_param("lab") &&
      req.get_param_value("lab") == "signal") {
    try {
      auto j = nlohmann::json::parse(req.body);
      const std::string map = j.value("map", "");
      const std::string var = j.value("var", "elev");
      const double ds = j.value("ds", 5.0);
      const std::string ks = j.value("kind", "scalar");
      const SeriesKind kind = (ks == "angle")         ? SeriesKind::Angle
                              : (ks == "categorical") ? SeriesKind::Categorical
                                                      : SeriesKind::Scalar;

      if (map.empty() || !is_safe_basename(map)) {
        res.status = 400;
        res.set_content(R"({"error":"bad map path"})", "application/json");
        return;
      }

      // load + build
      nlohmann::json body;
      {
        std::ifstream in(map);
        if (!in) {
          res.status = 404;
          res.set_content(R"({"error":"file not found"})", "application/json");
          return;
        }
        in >> body;
      }
      OsrmResponse osrm = body.get<OsrmResponse>();
      RouteSignalBuilder builder;
      RouteSignal rs = builder.build(osrm);

      WaveletFootprintEngine eng;
      auto sig = eng.make_wavelet_signal(
          rs, [&, var](const DataPoint &d) { return pick_value(d, var); }, kind,
          /*min_step_m*/ 0.0);

      UniformSignal uni;
      switch (kind) {
      case SeriesKind::Scalar:
        uni = eng.resample_uniform_scalar(sig, ds);
        break;
      case SeriesKind::Angle:
        uni = eng.resample_uniform_angle(sig, ds);
        break;
      case SeriesKind::Categorical:
        uni = eng.resample_uniform_categorical(sig, ds);
        break;
      }

      nlohmann::json out;
      out["s_km"] = nlohmann::json::array();
      out["y"] = nlohmann::json::array();
      for (size_t i = 0; i < uni.s.size(); ++i) {
        out["s_km"].push_back(uni.s[i] / 1000.0);
        out["y"].push_back(uni.y[i]);
      }
      res.set_content(out.dump(), "application/json");
      return;
    } catch (const std::exception &e) {
      nlohmann::json err = {{"error", e.what()}};
      res.status = 400;
      res.set_content(err.dump(), "application/json");
      return;
    }
  }

  if (validate) {
    try {
      auto _ = nlohmann::json::parse(req.body);
      nlohmann::json ok = {{"ok", true},
                           {"message", "JSON parsed successfully"}};
      res.set_content(ok.dump(), "application/json");
      return;
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
      OsrmResponse resp = body.get<OsrmResponse>();
      auto info = summarize(resp);
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

// ===== POST: /upload =====

void HttpHandler::handleUpload(const httplib::Request &req,
                               httplib::Response &res) {
  nlohmann::json body;
  try {
    body = nlohmann::json::parse(req.body);
  } catch (const std::exception &e) {
    res.status = 400;
    res.set_content(std::string("Invalid JSON: ") + e.what(), "text/plain");
    return;
  }

  std::error_code ec;
  std::filesystem::create_directories("uploads", ec);

  auto now = std::chrono::system_clock::now().time_since_epoch();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
  std::mt19937_64 rng{static_cast<unsigned long long>(ms)};
  unsigned long long r = rng();

  std::string filename =
      "uploads/map_" + std::to_string(ms) + "_" + std::to_string(r) + ".json";

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

// keep it file-scope static for this TU
static bool is_safe_basename(const std::string &s) {
  for (char c : s) {
    if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '.' || c == '_' ||
          c == '-' || c == '/'))
      return false;
  }
  return s.rfind("uploads/", 0) == 0;
}

// ===== GET: /view =====

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

  js << "var coords_osrm = [];\n";
  js << "var coords_rs  = [];\n";
  js << "var elev_rs    = [];\n";
  js << "var grad_rs    = [];\n";
  js << "var heading_delta = [];\n";
  bool have_osrm = false, have_rs = false;
  std::string parse_error;

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

  try {
    OsrmResponse osrm = body.get<OsrmResponse>();
    RouteSignalBuilder builder;
    RouteSignal rs = builder.build(osrm);

    if (!rs.points.empty()) {
      js << "coords_rs = [";
      for (const auto &dp : rs.points)
        js << "[" << dp.coord.lat << "," << dp.coord.lon << "],";
      js << "];\n";

      js << "elev_rs = [";
      for (const auto &dp : rs.points)
        js << dp.coord.elv << ",";
      js << "];\n";

      js << "grad_rs = [";
      for (const auto &dp : rs.points)
        js << dp.gradient << ",";
      js << "];\n";

      js << "way_rs = [";
      for (const auto &dp : rs.points)
        js << dp.way_id << ",";
      js << "];\n";

      js << "heading_delta = [";
      for (const auto &dp : rs.points)
        js << dp.heading_delta_norm << ",";
      js << "];\n";

      js << "speed_rs = [";
      for (const auto &dp : rs.points)
        js << dp.speed_smoothed << ",";
      js << "];\n";

      have_rs = true;

      WaveletFootprintEngine eng;
      auto sig_speed = eng.make_wavelet_signal(
          rs, [](const DataPoint &d) { return d.speed_smoothed; },
          SeriesKind::Scalar, 0.0);

      double ds_m = 5.0;
      if (rs.points.size() > 3) {
        std::vector<double> steps;
        steps.reserve(rs.points.size());
        for (size_t i = 1; i < rs.points.size(); ++i) {
          double d = rs.points[i].cum_dist - rs.points[i - 1].cum_dist;
          if (std::isfinite(d) && d > 0)
            steps.push_back(d);
        }
        if (!steps.empty()) {
          std::nth_element(steps.begin(), steps.begin() + steps.size() / 2,
                           steps.end());
          ds_m = std::max(1.0, std::min(10.0, steps[steps.size() / 2]));
        }
      }

      auto speed_u = eng.resample_uniform_scalar(sig_speed, ds_m);

      js << "var speed_u_skm = [";
      for (size_t k = 0; k < speed_u.s.size(); ++k)
        js << (speed_u.s[k] / 1000.0) << ",";
      js << "];\n";

      // keep values in m/s here; JS converts to km/h when plotting
      js << "var speed_u_vals = [";
      for (size_t k = 0; k < speed_u.y.size(); ++k)
        js << speed_u.y[k] << ",";
      js << "];\n";

      js << "var speed_u_ds_m = " << ds_m << ";\n";
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

  std::string html = R"(
  <!doctype html><html><head>
  <meta charset="utf-8"/>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
  <style>
    body{margin:0;font-family:sans-serif}
    #map{height:45vh;}
    #plotHeading{height:18vh;}
    #plot{height:22vh;}
    #plotSpeed{height:18vh;}
    #diag{padding:10px; font:12px/1.4 monospace;}
  </style>
  </head><body>
  <h3 style="margin:8px">Route Viewer (OSRM vs RouteSignal)</h3>
  <div id="map"></div>
  <div id="plotHeading"></div>
  <div id="plot"></div>
  <div id="plotSpeed"></div>
  <pre id="diag"></pre>
  <script>
  )" + js.str() + R"(

  window.VIEW = {
    coords_osrm, coords_rs, elev_rs, grad_rs, heading_delta, way_rs,
    speed_rs,
    speed_u_skm: (typeof speed_u_skm !== 'undefined') ? speed_u_skm : [],
    speed_u_vals: (typeof speed_u_vals !== 'undefined') ? speed_u_vals : [],
    speed_u_ds_m: (typeof speed_u_ds_m !== 'undefined') ? speed_u_ds_m : null
  };
  </script>
  <script src="/static/view.js?v=3"></script>
  </body></html>)";

  res.set_content(html, "text/html");
}

// 302 redirect to static Signal Lab UI
void HttpHandler::handleSignalLabUI(const httplib::Request &req,
                                    httplib::Response &res) {
  std::string url = "/static/signal_lab.html";
  if (req.has_param("map"))
    url += "?map=" + req.get_param_value("map");
  res.status = 302;
  res.set_header("Location", url);
  res.set_content("Redirecting to " + url, "text/plain");
}

// JSON: list uploads and optional preload
void HttpHandler::handleLabMeta(const httplib::Request &req,
                                httplib::Response &res) {
  nlohmann::json meta;
  meta["uploads"] = nlohmann::json::array();
  try {
    for (auto &p : std::filesystem::directory_iterator("uploads")) {
      if (!p.is_regular_file())
        continue;
      auto path = p.path().string();
      if (path.size() >= 5 && path.substr(path.size() - 5) == ".json")
        meta["uploads"].push_back(path);
    }
  } catch (...) {
  }

  if (req.has_param("map"))
    meta["preload"] = req.get_param_value("map");
  res.set_content(meta.dump(), "application/json");
}

// POST /lab/resample : build & resample requested series
void HttpHandler::handleLabResample(const httplib::Request &req,
                                    httplib::Response &res) {
  nlohmann::json in;
  try {
    in = nlohmann::json::parse(req.body);
  } catch (...) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"invalid json"})",
                    "application/json");
    return;
  }
  // etc.
  const std::string map = in.value("map", "");
  const double ds_m = in.value("ds_m", 5.0);
  const auto req_vars = in.value("vars", nlohmann::json::array());

  if (map.empty() || !std::filesystem::exists(map)) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"missing or bad 'map' path"})",
                    "application/json");
    return;
  }

  nlohmann::json body;
  try {
    std::ifstream f(map);
    f >> body;
  } catch (...) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"cannot read map file"})",
                    "application/json");
    return;
  }
  OsrmResponse osrm;
  try {
    osrm = body.get<OsrmResponse>();
  } catch (const std::exception &e) {
    nlohmann::json out = {
        {"ok", false},
        {"error", std::string("parse OsrmResponse: ") + e.what()}};
    res.status = 400;
    res.set_content(out.dump(), "application/json");
    return;
  }

  RouteSignalBuilder b;
  RouteSignal rs = b.build(osrm);

  WaveletFootprintEngine eng;

  nlohmann::json out;
  out["ok"] = true;

  nlohmann::json series = nlohmann::json::object();

  auto push_series = [&](const std::string &name, SeriesKind kind,
                         auto getter) {
    auto w = eng.make_wavelet_signal(rs, getter, kind, 0.0);
    UniformSignal u;
    switch (kind) {
    case SeriesKind::Scalar:
      u = eng.resample_uniform_scalar(w, ds_m);
      break;
    case SeriesKind::Angle:
      u = eng.resample_uniform_angle(w, ds_m);
      break;
    case SeriesKind::Categorical:
      u = eng.resample_uniform_categorical(w, ds_m);
      break;
    }

    if (!out.contains("s_km")) {
      std::vector<double> s_km;
      s_km.reserve(u.s.size());
      for (double s : u.s)
        s_km.push_back(s / 1000.0);
      out["s_km"] = s_km;
      out["ds_m"] = ds_m;
    }

    std::vector<double> y;
    y.reserve(u.y.size());
    if (name == "speed") {
      for (double v : u.y)
        y.push_back(v * 3.6); // m/s -> km/h
    } else {
      for (double v : u.y)
        y.push_back(v);
    }
    series[name] = y;
  };

  std::set<std::string> want;
  for (auto &v : req_vars)
    if (v.is_string())
      want.insert(v.get<std::string>());

  if (want.empty() || want.count("elev"))
    push_series("elev", SeriesKind::Scalar,
                [](const DataPoint &d) { return d.coord.elv; });
  if (want.empty() || want.count("speed"))
    push_series("speed", SeriesKind::Scalar,
                [](const DataPoint &d) { return d.speed_smoothed; });
  if (want.empty() || want.count("grad"))
    push_series("grad", SeriesKind::Scalar,
                [](const DataPoint &d) { return d.gradient; });
  if (want.empty() || want.count("heading_delta_norm"))
    push_series("heading_delta_norm", SeriesKind::Scalar,
                [](const DataPoint &d) { return d.heading_delta_norm; });

  out["series"] = series;
  res.set_content(out.dump(), "application/json");
}

// ============= Wavelets ==============
static inline bool starts_with(const std::string &s, const std::string &p) {
  return s.rfind(p, 0) == 0;
}

void HttpHandler::handleWavelet(const httplib::Request &req,
                                httplib::Response &res) {
  // Parse JSON body
  json in;
  try {
    in = json::parse(req.body);
  } catch (...) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"invalid json"})",
                    "application/json");
    return;
  }

  // Required: map path
  const std::string mapPath = in.value("map", "");
  if (mapPath.empty() || !std::filesystem::exists(mapPath)) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"missing or bad 'map' path"})",
                    "application/json");
    return;
  }

  // Only terrain mode supported
  const std::string fn = in.value("fn", "terrain");
  if (fn != "terrain") {
    res.status = 400;
    res.set_content(
        R"({"ok":false,"error":"only fn==\"terrain\" is supported"})",
        "application/json");
    return;
  }

  // Persist flag
  bool persist = in.value("persist", false);

  // Read map JSON
  json body;
  try {
    std::ifstream f(mapPath);
    f >> body;
  } catch (...) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"cannot read map file"})",
                    "application/json");
    return;
  }

  // Parse OSRM response
  OsrmResponse osrm;
  try {
    osrm = body.get<OsrmResponse>();
  } catch (const std::exception &e) {
    res.status = 400;
    json out = {{"ok", false},
                {"error", std::string("parse OsrmResponse: ") + e.what()}};
    res.set_content(out.dump(), "application/json");
    return;
  }

  // Build RouteSignal
  RouteSignalBuilder builder;
  RouteSignal rs = builder.build(osrm);

  // Prepare output
  json out;
  out["ok"] = true;

  // Raw distance axis
  {
    std::vector<double> s_km;
    s_km.reserve(rs.points.size());
    for (auto &dp : rs.points)
      s_km.push_back(dp.cum_dist / 1000.0);
    out["s_km"] = std::move(s_km);
  }

  // Get terrain params object
  json params = in.value("params", json::object());
  const json &tp = params.contains("terrain") ? params["terrain"] : params;

  // Run terrain segmentation
  WaveletFootprintEngine eng;
  WaveletFootprintEngine::TerrainParams tpar; // defaults

  // Apply overrides
  auto upd = [&](const char *key, auto &field) {
    if (tp.contains(key))
      field = tp[key].get<std::decay_t<decltype(field)>>();
  };
  upd("ds_m", tpar.dx);
  upd("L_T", tpar.L_T);
  upd("k_g", tpar.k_g);
  upd("L_E", tpar.L_E);
  upd("k_E", tpar.k_E);
  upd("tau_p", tpar.tau_p);
  upd("E_env_m", tpar.E_env_m);
  upd("E_use_hysteresis", tpar.E_use_hysteresis);
  upd("E_hyst_hi", tpar.E_hyst_hi);
  upd("E_hyst_lo", tpar.E_hyst_lo);
  upd("E_gap_close_m", tpar.E_gap_close_m);
  upd("E_min_run_m", tpar.E_min_run_m);
  upd("min_segment_length_m", tpar.min_segment_length_m);
  upd("merge_side_right", tpar.merge_side_right);

  // Compute
  std::vector<double> E;
  auto terrainUS = eng.terrain_states_from_elevation(rs, tpar, E);
  auto state_codes = eng.get_states();

  auto state_to_type = [](int s) -> SegmentKind {
    switch (s) {
    case 1:
      return SegmentKind::Flat;
    case 2:
      return SegmentKind::Uphill;
    case 3:
      return SegmentKind::Downhill;
    case 4:
      return SegmentKind::Rolling;
    default:
      return SegmentKind::Unknown;
    }
  };

  // Change points
  std::vector<int> change_points = eng.get_changepoint_points(rs);
  out["change_points"] = change_points;

  // Build spans
  int N = (int)rs.points.size();
  auto spans = SegmentUtils::make_segments_from_change_points(change_points, N);

  // helper: nearest uniform-sample index for a given route cumulative distance
  // (meters)
  auto nearest_uniform_idx = [&](double s_m) -> int {
    // terrainUS.s is ascending in meters
    auto it = std::lower_bound(terrainUS.s.begin(), terrainUS.s.end(), s_m);
    if (it == terrainUS.s.begin())
      return 0;
    if (it == terrainUS.s.end())
      return (int)terrainUS.s.size() - 1;
    size_t j = (size_t)(it - terrainUS.s.begin());
    // pick closer of j and j-1
    return (std::abs(terrainUS.s[j] - s_m) < std::abs(terrainUS.s[j - 1] - s_m))
               ? (int)j
               : (int)j - 1;
  };

  // route-index → state code via midpoint sampling
  auto state_at_span = [&](int a, int b) -> int {
    int mid = a + (std::max(1, b - a) / 2);
    if (mid < 0)
      mid = 0;
    if (mid >= (int)rs.points.size())
      mid = (int)rs.points.size() - 1;
    double s_m = rs.points[mid].cum_dist; // meters
    int j = nearest_uniform_idx(s_m);
    int code =
        (j >= 0 && j < (int)state_codes.size()) ? (int)state_codes[j] : 0;
    if (code < 0 || code > 4)
      code = 0;
    return code;
  };

  // Collect way_ids once
  std::vector<long long> way_ids;
  way_ids.reserve(N);
  for (auto &p : rs.points)
    way_ids.push_back(p.way_id);

  // Build segments
  std::vector<SegmentInstance> segs;
  segs.reserve(spans.size());
  for (auto [a, b] : spans) {
    SegmentInstance si;
    si.def = SegmentUtils::build_segment_def_forward(rs.points, a, b);
    si.start_idx = a;
    si.end_idx = b;
    si.runs = SegmentUtils::way_runs_in_slice(way_ids, a, b);
    si.def.kind = state_to_type(state_at_span(a, b));
    segs.push_back(std::move(si));
  }

  std::vector<Coordinate> route_coords;
  route_coords.reserve(rs.points.size());
  for (auto &p : rs.points) {
    route_coords.push_back(p.coord);
  }
  SegmentUtils::overlay_db_segments(segs, route_coords, way_ids);

  // Serialize segments
  json jsegs = json::array();
  for (auto &s : segs) {
    json jr;
    jr["segment_uid"] = s.def.uid_hex;
    jr["start_idx"] = s.start_idx;
    jr["end_idx"] = s.end_idx;
    jr["length_m"] = s.def.length_m;
    jr["point_count"] = s.def.point_count;
    jr["coordinates"] = json::parse(s.def.coords_json);
    jr["runs"] = json::array();
    jr["type"] = [&] {
      switch (s.def.kind) {
      case SegmentKind::Rolling:
        return "rolling";
      case SegmentKind::Flat:
        return "flat";
      case SegmentKind::Uphill:
        return "uphill";
      case SegmentKind::Downhill:
        return "downhill";
      default:
        return "unknown";
      }
    }();
    for (size_t i = 0; i < s.runs.size(); ++i) {
      const auto &r = s.runs[i];
      jr["runs"].push_back({{"seq", int(i)},
                            {"way_id", r.way_id},
                            {"from", r.from_idx},
                            {"to", r.to_idx}});
    }
    jsegs.push_back(std::move(jr));
  }
  // Axis for uniform result (for precise alignment)
  std::vector<double> s_km_u;

  s_km_u.reserve(terrainUS.s.size());
  for (double sm : terrainUS.s)
    s_km_u.push_back(sm / 1000.0);
  json series;
  series["haar_trend"] = terrainUS.y;
  series["energy"] = E;
  series["terrain"] = state_codes;

  out["segments"] = std::move(jsegs);
  out["series"] = std::move(series);
  out["s_km_uniform"] = s_km_u;
  out["ds_m"] = terrainUS.ds;

  if (persist) {
    std::cerr << "Sending segments to database" << "\n";
    // Load DB connection info from environment (with defaults)
    const char *db_host = std::getenv("DB_HOST") ?: "127.0.0.1";
    const char *db_user = std::getenv("DB_USER") ?: "routeseg_user";
    const char *db_pass = std::getenv("DB_PASS") ?: "changeme-user";
    const char *db_name = std::getenv("DB_NAME") ?: "routeseg";
    unsigned int db_port =
        std::getenv("DB_PORT") ? std::atoi(std::getenv("DB_PORT")) : 3306;

    MySQLSegmentDB db(std::string("tcp://") + db_host + ":" +
                          std::to_string(db_port),
                      db_user, db_pass, db_name);
    try {
      db.begin();
      for (auto &s : segs) {
        db.upsert_segment_def(s.def);
        auto seg_id = db.insert_route_segment(s);
        db.insert_segment_runs(seg_id, s.runs);
      }
      db.commit();
    } catch (const std::exception &e) {
      std::cerr << "[DB ERROR] " << e.what() << "\n";
      // rollback on error, but don’t fail the entire request
      try {
        db.rollback();
      } catch (...) {
      }
      out["db_error"] = e.what();
    }
  }

  // Send response
  res.set_content(out.dump(), "application/json");
}

// ===== segmentation (placeholder) =====

// helper: parse bbox from query string
static bool parse_bbox(const httplib::Request &req, double &west, double &south,
                       double &east, double &north) {
  // option A: bbox=west,south,east,north
  if (auto it = req.get_param_value("bbox", 0); !it.empty()) {
    std::string s = it;
    std::stringstream ss(s);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ',')) {
      try {
        vals.push_back(std::stod(tok));
      } catch (...) {
        return false;
      }
    }
    if (vals.size() != 4)
      return false;
    west = vals[0];
    south = vals[1];
    east = vals[2];
    north = vals[3];
    return true;
  }
  // option B: minLon, minLat, maxLon, maxLat
  try {
    std::string sWest = req.get_param_value("minLon", 0);
    std::string sSouth = req.get_param_value("minLat", 0);
    std::string sEast = req.get_param_value("maxLon", 0);
    std::string sNorth = req.get_param_value("maxLat", 0);
    if (sWest.empty() || sSouth.empty() || sEast.empty() || sNorth.empty())
      return false;
    west = std::stod(sWest);
    south = std::stod(sSouth);
    east = std::stod(sEast);
    north = std::stod(sNorth);
    return true;
  } catch (...) {
    return false;
  }
}

std::string to_hex(const unsigned char *data, size_t len) {
  static const char *hex = "0123456789abcdef";
  std::string out;
  out.reserve(len * 2);
  for (size_t i = 0; i < len; ++i) {
    out.push_back(hex[data[i] >> 4]);
    out.push_back(hex[data[i] & 0x0F]);
  }
  return out;
}
// handleSegment: fetch segments from `segment_defs` JSON+bbox columns only
void HttpHandler::handleSegments(const httplib::Request &req,
                                 httplib::Response &res) {
  // 1) Parse bbox (same helper)
  double west = 0, south = 0, east = 0, north = 0;
  if (!parse_bbox(req, west, south, east, north)) {
    res.status = 400;
    res.set_content(
        R"({"error":"invalid bbox. use bbox=west,south,east,north or minLon/minLat/maxLon/maxLat"})",
        "application/json");
    return;
  }
  if (!(west < east && south < north)) {
    res.status = 400;
    res.set_content(
        R"({"error":"bbox must satisfy west<east and south<north"})",
        "application/json");
    return;
  }

  // 2) Query segment_defs using precomputed bbox columns
  static const std::string sql = R"(
    SELECT
      segment_uid,
      direction,
      length_m,
      coords_json
    FROM segment_defs
    WHERE bbox_min_lon <= ?
      AND bbox_max_lon >= ?
      AND bbox_min_lat <= ?
      AND bbox_max_lat >= ?;
  )";

  if (!db_) {
    std::cerr << "[handleSegments] FATAL: db_ is null!\n";
    res.status = 500;
    res.set_content(R"({"error":"internal db connection not initialized"})",
                    "application/json");
    return;
  }
  MYSQL_STMT *stmt = mysql_stmt_init(db_);
  if (!stmt) {
    res.status = 500;
    res.set_content(R"({"error":"db statement init failed"})",
                    "application/json");
    return;
  }
  if (mysql_stmt_prepare(stmt, sql.c_str(),
                         static_cast<unsigned long>(sql.size())) != 0) {
    std::string e = mysql_error(db_);
    mysql_stmt_close(stmt);
    res.status = 500;
    res.set_content(std::string(R"({"error":"prepare failed: )") + e + "\"}",
                    "application/json");
    return;
  }

  // 3) Bind bbox params: (east, west, north, south)
  MYSQL_BIND pbind[4];
  memset(pbind, 0, sizeof(pbind));
  double params[4] = {east, west, north, south};
  for (int i = 0; i < 4; ++i) {
    pbind[i].buffer_type = MYSQL_TYPE_DOUBLE;
    pbind[i].buffer = &params[i];
    pbind[i].is_null = nullptr;
    pbind[i].length = nullptr;
  }
  if (mysql_stmt_bind_param(stmt, pbind) != 0) {
    std::string e = mysql_error(db_);
    mysql_stmt_close(stmt);
    res.status = 500;
    res.set_content(std::string(R"({"error":"bind params failed: )") + e +
                        "\"}",
                    "application/json");
    return;
  }

  if (mysql_stmt_execute(stmt) != 0) {
    std::string e = mysql_error(db_);
    mysql_stmt_close(stmt);
    res.status = 500;
    res.set_content(std::string(R"({"error":"execute failed: )") + e + "\"}",
                    "application/json");
    return;
  }

  // 4) Bind result columns and fetch into GeoJSON FeatureCollection
  MYSQL_BIND rbind[4];
  memset(rbind, 0, sizeof(rbind));
  unsigned char uid_buf[32]; // BINARY(32)
  char dir_buf[8];
  unsigned long dir_len = 0;
  double length_m = 0;
  // coords_json: assume <64KB, adjust if necessary
  std::vector<char> json_buf(1 << 16);
  unsigned long json_len = 0;

  // UID
  rbind[0].buffer_type = MYSQL_TYPE_STRING;
  rbind[0].buffer = uid_buf;
  rbind[0].buffer_length = sizeof(uid_buf);
  rbind[0].length = nullptr;
  // direction
  rbind[1].buffer_type = MYSQL_TYPE_STRING;
  rbind[1].buffer = dir_buf;
  rbind[1].buffer_length = sizeof(dir_buf);
  rbind[1].length = &dir_len;
  // length_m
  rbind[2].buffer_type = MYSQL_TYPE_DOUBLE;
  rbind[2].buffer = &length_m;
  // coords_json
  rbind[3].buffer_type = MYSQL_TYPE_STRING;
  rbind[3].buffer = json_buf.data();
  rbind[3].buffer_length = json_buf.size();
  rbind[3].length = &json_len;

  if (mysql_stmt_bind_result(stmt, rbind) != 0 ||
      mysql_stmt_store_result(stmt) != 0) {
    std::string e = mysql_error(db_);
    mysql_stmt_close(stmt);
    res.status = 500;
    res.set_content(std::string(R"({"error":"bind/store result failed: )") + e +
                        "\"}",
                    "application/json");
    return;
  }

  nlohmann::json fc = {{"type", "FeatureCollection"},
                       {"features", nlohmann::json::array()}};

  while (true) {
    int rc = mysql_stmt_fetch(stmt);
    if (rc == MYSQL_NO_DATA)
      break;
    if (rc == 1) {
      std::string e = mysql_error(db_);
      mysql_stmt_close(stmt);
      res.status = 500;
      res.set_content(std::string(R"({"error":"fetch failed: )") + e + "\"}",
                      "application/json");
      return;
    }

    // Build a Feature
    nlohmann::json feat;
    feat["type"] = "Feature";
    // properties
    nlohmann::json props = {{"uid", to_hex(uid_buf, sizeof(uid_buf))},
                            {"direction", std::string(dir_buf, dir_len)},
                            {"length_m", length_m}};
    feat["properties"] = std::move(props);
    // geometry
    feat["geometry"] = {{"type", "LineString"},
                        {"coordinates", nlohmann::json::parse(std::string(
                                            json_buf.data(), json_len))}};
    fc["features"].push_back(std::move(feat));
  }

  mysql_stmt_free_result(stmt);
  mysql_stmt_close(stmt);

  // 5) Return the GeoJSON
  res.set_header("Access-Control-Allow-Origin", "*");
  res.set_content(fc.dump(), "application/json");
}

void HttpHandler::handleDBPing(const httplib::Request &req,
                               httplib::Response &res) {

  // read credentials from ENV or config
  std::cerr << "hit ping" << "\n";
  const char *host = std::getenv("DB_HOST") ?: "127.0.0.1";
  const char *user = std::getenv("DB_USER") ?: "routeseg_user";
  const char *pass = std::getenv("DB_PASS") ?: "changeme-user";
  const char *db = std::getenv("DB_NAME") ?: "routeseg";
  unsigned int port =
      std::getenv("DB_PORT") ? std::atoi(std::getenv("DB_PORT")) : 3306;

  MYSQL *conn = db_;
  if (!conn) {
    res.status = 500;
    res.set_content(R"({"ok":false,"error":"mysql_init failed"})",
                    "application/json");
    return;
  }

  if (!mysql_real_connect(conn, host, user, pass, db, port, nullptr, 0)) {
    std::string err = mysql_error(conn);
    mysql_close(conn);
    res.status = 500;
    res.set_content(
        json{{"ok", false}, {"error", "connect failed: " + err}}.dump(),
        "application/json");
    return;
  }

  mysql_close(conn);
  res.set_content(R"({"ok":true,"message":"DB connection successful"})",
                  "application/json");
}
