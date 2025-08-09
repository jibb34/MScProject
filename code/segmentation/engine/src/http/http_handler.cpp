#include "http_handler.hpp"
#include "json.hpp"
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
  json doc;
  try {
    doc = json::parse(req.body);
  } catch (const json::parse_error &e) {
    res.status = 400;
    res.set_content(std::string("Invalid JSON: ") + e.what(), "text/plain");
    return;
  }

  // Count points across OSRM /match results
  size_t point_count = 0;

  if (doc.contains("matchings") && doc["matchings"].is_array()) {
    for (const auto &m : doc["matchings"]) {
      if (!m.contains("geometry"))
        continue;

      const auto &geom = m["geometry"];

      // If you didn't request geometries=geojson, this will be a string
      // (polyline)
      if (geom.is_string()) {
        // Can't count points without decoding; bail with a helpful message
        res.status = 400;
        res.set_content("geometry is a string (polyline). Re-run OSRM with "
                        "geometries=geojson "
                        "or implement polyline decoding.",
                        "text/plain");
        return;
      }

      if (!geom.is_object())
        continue;

      const std::string type = geom.value("type", "");
      const auto &coords =
          geom.contains("coordinates") ? geom["coordinates"] : json();

      if (type == "LineString" && coords.is_array()) {
        point_count += coords.size();
      } else if (type == "MultiLineString" && coords.is_array()) {
        for (const auto &line : coords) {
          if (line.is_array())
            point_count += line.size();
        }
      }
      // else: ignore other geometry types
    }
  } else {
    // Not an OSRM /match result – maybe a FeatureCollection? (handle that here
    // if needed)
    res.status = 400;
    res.set_content("Expected OSRM match result with 'matchings' array.",
                    "text/plain");
    return;
  }

  std::cout << "[DEBUG] /debug -> points=" << point_count << std::endl;
  json reply = {{"status", "ok"}, {"points", point_count}};
  res.set_content(reply.dump(2), "application/json");
}

void HttpHandler::handleSegment(const httplib::Request &req,
                                httplib::Response &res) {
  // stub
  res.set_content("Not implemented", "text/plain");
}
