#include "http_handler.hpp"
#include "core/RouteSignalBuilder.hpp"
#include "core/wavelets/WaveletFootprint.hpp"
#include "debug/json_debug.hpp"
#include "debug/osrm_inspect.hpp"
#include "httplib.h"
#include "io/json_parser.hpp"
#include "json.hpp"
#include "models/params.hpp"

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
  std::cerr << "[Handler] Received handler request... action:" + action + ".\n";
  if (action == "segment") {
    handleSegment(req, res);
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
  } else {
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
  std::cerr << "[HTTP] Receiving JSON payload..." << std::endl;

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
  std::cerr << "Hit Hanlde Lab Resample";
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
  std::cerr << "[lab/resample] parsed input\n";
  const std::string map = in.value("map", "");
  const double ds_m = in.value("ds_m", 5.0);
  const auto req_vars = in.value("vars", nlohmann::json::array());

  if (map.empty() || !std::filesystem::exists(map)) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"missing or bad 'map' path"})",
                    "application/json");
    return;
  }
  std::cerr << "[lab/resample] loaded map\n";

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
  std::cerr << "Define route builder" << std::endl;
  RouteSignal rs = b.build(osrm);

  std::cerr << "[lab/resample] built route signal\n";
  WaveletFootprintEngine eng;

  nlohmann::json out;
  out["ok"] = true;

  nlohmann::json series = nlohmann::json::object();

  auto push_series = [&](const std::string &name, SeriesKind kind,
                         auto getter) {
    auto w = eng.make_wavelet_signal(rs, getter, kind, 0.0);
    std::cerr << "pushing series: " << name << std::endl;
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
  std::cerr << "[lab/resample] push_series(elev)\n";

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
  std::cerr << "[lab/resample] Created series array response";
  res.set_content(out.dump(), "application/json");
}

// ============= Wavelets ==============
static inline bool starts_with(const std::string &s, const std::string &p) {
  return s.rfind(p, 0) == 0;
}

void HttpHandler::handleWavelet(const httplib::Request &req,
                                httplib::Response &res) {
  std::cerr << "[wavelet] received request\n";

  // --- Parse JSON body ---
  json in;
  try {
    in = json::parse(req.body);
  } catch (...) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"invalid json"})",
                    "application/json");
    return;
  }
  std::cerr << "[wavelet] parsed input\n";

  // Required: map path (same contract as /lab/resample)
  const std::string map = in.value("map", "");
  if (map.empty() || !std::filesystem::exists(map)) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"missing or bad 'map' path"})",
                    "application/json");
    return;
  }
  std::cerr << "[wavelet] loaded map path\n";

  // Optional: which wavelet function to compute (default: "terrain")
  const std::string fn = in.value("fn", std::string("terrain"));

  // Optional: params object (we look for params.terrain.* for terrain function)
  const json params = in.value("params", json::object());
  const json &tp = params.contains("terrain") ? params["terrain"] : params;

  // --- Read the uploaded map JSON ---
  json body;
  try {
    std::ifstream f(map);
    f >> body;
  } catch (...) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"cannot read map file"})",
                    "application/json");
    return;
  }

  // --- Parse OSRM response ---
  OsrmResponse osrm;
  try {
    osrm = body.get<OsrmResponse>();
  } catch (const std::exception &e) {
    json out = {{"ok", false},
                {"error", std::string("parse OsrmResponse: ") + e.what()}};
    res.status = 400;
    res.set_content(out.dump(), "application/json");
    return;
  }
  std::cerr << "[wavelet] parsed OsrmResponse\n";

  // --- Build RouteSignal the same way as in /lab/resample ---
  RouteSignalBuilder b;
  std::cerr << "Define route builder" << std::endl;
  RouteSignal rs = b.build(osrm);
  std::cerr << "[wavelet] built route signal\n";

  // --- Prepare response ---
  json out;
  out["ok"] = true;

  // For convenience we include the raw (non-uniform) axis too
  {
    std::vector<double> s_km;
    s_km.reserve(rs.points.size());
    for (const auto &dp : rs.points)
      s_km.push_back(dp.cum_dist / 1000.0);
    out["s_km"] = s_km;
  }

  json series = json::object();

  // --- Dispatch on function name ---
  if (fn == "terrain") {
    WaveletFootprintEngine eng;
    WaveletFootprintEngine::TerrainParams tpar; // defaults

    auto upd = [&](const char *key, auto &field) {
      if (!tp.contains(key))
        return;
      const auto &v = tp[key];
      using T = std::decay_t<decltype(field)>;

      if constexpr (std::is_enum_v<T>) {
        // accept int or string
        if (v.is_number_integer()) {
          int x = v.get<int>();
          switch (x) {
          case 0:
            field = WaveletFootprintEngine::EnergyAlgorithm::FFT;
            break;
          case 1:
            field = WaveletFootprintEngine::EnergyAlgorithm::RMS;
            break;
          default: /* ignore bad value */
            break;
          }
        } else if (v.is_string()) {
          std::string s = v.get<std::string>();
          std::transform(s.begin(), s.end(), s.begin(), ::tolower);
          if (s == "fft" || s == "fftw")
            field = WaveletFootprintEngine::EnergyAlgorithm::FFT;
          else if (s == "haar" || s == "wavelet")
            field = WaveletFootprintEngine::EnergyAlgorithm::RMS;
        }
      } else {
        // doubles/ints/etc
        field = v.get<T>();
      }
    };
    upd("ds_m", tpar.dx);
    upd("L_T", tpar.L_T);
    upd("k_g", tpar.k_g);
    upd("L_E", tpar.L_E);

    upd("lambda_min", tpar.lambda_min);
    upd("lambda_max", tpar.lambda_max);
    upd("energy_mode", tpar.energy_mode);

    upd("E_env_m", tpar.E_env_m);

    upd("E_use_hysteresis", tpar.E_use_hysteresis);
    upd("E_hyst_hi", tpar.E_hyst_hi);
    upd("E_hyst_lo", tpar.E_hyst_lo);
    upd("E_gap_close_m", tpar.E_gap_close_m);
    upd("E_min_run_m", tpar.E_min_run_m);

    std::vector<double> E;
    std::vector<double> pseries;
    auto terrainUS = eng.terrain_states_from_elevation(rs, tpar, E, pseries);
    E = pseries;

    // Axis for uniform result (for precise alignment)
    std::vector<double> s_km_u;
    s_km_u.reserve(terrainUS.s.size());
    for (double sm : terrainUS.s)
      s_km_u.push_back(sm / 1000.0);

    out["s_km_uniform"] = s_km_u;
    out["ds_m"] = terrainUS.ds;

    // Series: 0..3 codes
    series["haar_trend"] = terrainUS.y;

    series["energy"] = E;
    // TODO: series["terrain"] = state_codes;

  } else {
    // Unknown function name
    res.status = 400;
    res.set_content(std::string(R"({"ok":false,"error":"unknown fn: )") + fn +
                        R"("})",
                    "application/json");
    return;
  }

  out["series"] = series;

  // DEBUG list keys
  std::cerr << "[wavelet] returning series keys:";
  for (auto it = series.begin(); it != series.end(); ++it)
    std::cerr << " " << it.key();
  std::cerr << "\n";

  res.set_content(out.dump(), "application/json");
}

// ===== segmentation (placeholder) =====

void HttpHandler::handleSegment(const httplib::Request &req,
                                httplib::Response &res) {
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

  json body;
  try {
    body = json::parse(req.body);
  } catch (...) {
    res.status = 400;
    res.set_content("Invalid JSON body", "text/plain");
    return;
  }

  OsrmResponse osrm_object;
  try {
    osrm_object = body.get<OsrmResponse>();
  } catch (const std::exception &e) {
    res.status = 400;
    res.set_content(std::string("Parse error: ") + e.what(), "text/plain");
    return;
  }

  // TODO: segmentation engine
  res.set_content(R"({"ok":true})", "application/json");
}
