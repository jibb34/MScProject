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
  json geojson;
  try {
    geojson = json::parse(req.body);
  } catch (...) {
    // Return 400 error if could not parse jason (who even is jason?)
    res.status = 400;
    res.set_content("Invalid JSON", "text/plain");
    return;
  }
  // count total number of Waypoints
  size_t point_count = 0;
  if (geojson.contains("matchings")) {
    for (auto &feat : geojson["matchings"]) {
      for (auto &geo : feat["geometry"]) {
        auto coords = geo["coordinates"];
        if (coords.size() > 0) {
          point_count++;
        }
      }
    }
  }
  std::cout << "[DEBUG] Received " << point_count << " points in /debug"
            << std::endl;
  res.set_content("Points received: " + std::to_string(point_count),
                  "text/plain");
}

void HttpHandler::handleSegment(const httplib::Request &req,
                                httplib::Response &res) {
  // stub
  res.set_content("Not implemented", "text/plain");
}
