#pragma once
#include "json.hpp"
#include <array>
#include <string>
#include <vector>

using Coord = std::array<double, 2>; // [lon, lat]
using Json = nlohmann::json;

// ---------- Geometry ----------
struct Geometry {
  std::string type; // e.g., "LineString"
  std::vector<Coord> coordinates;
};
// define type: Run = a run of consecutive OSM way IDs, has a way_id and count
// of the amount of segments in a row
struct Run {
  int64_t way_id = 0; // OSM way ID
  size_t count = 0;   // Number of consecutive segments in this run
};

// ---------- Legs --------------
struct Leg {
  Json annotation;
  double weight = 0.0;
  double duration = 0.0;
  double distance = 0.0;
  std::vector<Run> runs; // List of ID:count pairs, where count
                         // is number of consecutive OSM way IDs
};

// ------------ Matching ------------
struct Matching {
  double confidence = 0.0;
  Geometry geometry;
  std::vector<Leg> legs;
  std::string weight_name;
  double weight = 0.0;
  double duration = 0.0;
  double distance = 0.0;
};

// ----- GPX -------
struct GpxPoint {
  double lat = 0.0;
  double lon = 0.0;
  double elv = 0.0;
  int time = 0;
  std::string time_iso;
  Json extensions;
};

struct Tracepoint {
  bool matched = false;
  int alternatives_count = 0;
  int waypoint_index = 0;
  int matchings_index = 0;
  Coord location{0.0, 0.0}; // [lon, lat]
  std::string name;
  std::vector<GpxPoint> gpx_list;
};

struct OsrmResponse {
  std::string code;
  Matching matching;
  std::vector<Tracepoint> tracepoints;
};

// Define from_json() overloads for structs to work with json::get() function

// --- Geometry ----
inline void from_json(const Json &j, Geometry &g) {
  g.type = j.value("type", "");
  g.coordinates.clear();
  if (j.contains("coordinates") && j["coordinates"].is_array()) {
    for (const auto &pt : j["coordinates"]) {
      if (pt.is_array() && pt.size() >= 2) {
        // for each pair of coordinates, check the pt has at least 2 values (x
        // and y), and add pair to new geometry g, in coordinates field
        g.coordinates.push_back({pt[0].get<double>(), pt[1].get<double>()});
      }
    }
  }
}

// --- Legs ----
inline void from_json(const Json &j, Leg &l) {
  l.annotation = j.value("annotation", Json::object());
  l.weight = j.value("weight", 0.0);
  l.duration = j.value("duration", 0.0);
  l.distance = j.value("distance", 0.0);
}

// --- Matching ----
inline void from_json(const Json &j, Matching &m) {
  m.confidence = j.value("confidence", 0.0);
  if (j.contains("geometry") && j["geometry"].is_object())
    m.geometry = j["geometry"].get<Geometry>();
  m.weight_name = j.value("weight_name", "");
  m.weight = j.value("weight", 0.0);
  m.duration = j.value("duration", 0.0);
  m.distance = j.value("distance", 0.0);

  // add legs
  m.legs.clear();
  if (j.contains("legs") && j["legs"].is_array()) {
    for (const auto &L : j["legs"])
      m.legs.push_back(L.get<Leg>());
  }
}

// --- GpxPoint----
inline void from_json(const Json &j, GpxPoint &p) {
  p.lat = j.value("lat", 0.0);
  p.lon = j.value("lon", 0.0);
  p.elv = j.value("elv", 0.0);
  p.time = j.value("time", 0);
  p.time_iso = j.value("time_iso", "");
  p.extensions = j.value("extensions", Json::object());
}

// --- Tracepoint ----
inline void from_json(const Json &j, Tracepoint &t) {
  if (j.is_null()) {
    // create placeholder struct
    t.matched = false;
    t.alternatives_count = 0;
    t.waypoint_index = -1;
    t.matchings_index = -1;
    t.location = {0.0, 0.0};
    t.name.clear();
    t.gpx_list.clear();
    return;
  }
  t.matched = true;
  t.alternatives_count = j.value("alternatives_count", 0);
  t.waypoint_index = j.value("waypoint_index", 0);
  if (j.contains("location") && j["location"].is_array() &&
      j["location"].size() >= 2) {
    t.location = {j["location"][0].get<double>(),
                  j["location"][1].get<double>()};
  }
  t.name = j.value("name", "");
  t.gpx_list.clear();
  if (j.contains("gpx_list") && j["gpx_list"].is_array()) {
    for (const auto &gp : j["gpx_list"])
      t.gpx_list.push_back(gp.get<GpxPoint>());
  }
}

// --- OsrmResponse ----
inline void from_json(const Json &j, OsrmResponse &r) {
  r.code = j.value("code", "Error");
  const auto &arr = j.value("matchings", Json::array());
  if (arr.empty())
    throw std::runtime_error("No matchings in OSRM response");
  r.matching = arr.front().get<Matching>(); // Get first matching by default
  if (j.contains("matchings") && j["matchings"].is_array()) {
    for (const auto &tp : j["tracepoints"])
      r.tracepoints.push_back(tp.get<Tracepoint>());
  }
}
