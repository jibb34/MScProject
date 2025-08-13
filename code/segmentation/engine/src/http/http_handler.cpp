#include "http_handler.hpp"
#include "debug/json_debug.hpp"
#include "debug/osrm_inspect.hpp"
#include "io/json_parser.hpp"
#include "json.hpp"
#include "models/OsrmResponse.hpp"
#include "models/params.hpp"
#include <iostream>

using json = nlohmann::json;

// Constructor

void HttpHandler::registerRoutes(httplib::Server &server) {
  // TODO: Implement route input for database matching
}

void HttpHandler::callHandler(std::string action, const httplib::Request &req,
                              httplib::Response &res) {
  // NOTE: This section is where all selection of routing occurs based on the
  // endpoint
  if (action == "segment") {
    handleSegment(req, res);
  } else if (action == "debug") {
    handleDebug(req, res);
  } else {
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
