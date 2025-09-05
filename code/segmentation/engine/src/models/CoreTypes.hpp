#pragma once

#include <inttypes.h>
#include <limits>
#include <nlohmann/json.hpp>
#include <sys/types.h>
#include <vector>

using Json = nlohmann::json;

// Basic spatial coordinate with elevation.
struct Coordinate {
  double lat;
  double lon;
  double elv;
};

// Simplified set of OSM highway classes used by the engine.
enum class HighwayType : u_int8_t {
  Motorway,
  Trunk,
  Primary,
  Secondary,
  Tertiary,
  Unknown,
  Unclassified,
  Residential,
  Service,
  Cycleway,
  Footway,
  Track,
  Path,
  Ferry
};

// A single sampled point along a route with derived metrics for segmentation.
struct DataPoint {
  Coordinate coord; // raw location
  double time_rel;  // time from point 0
  uint32_t way_id;  // OSM way identifier
  HighwayType highway_type;

  // Derived quantities
  double heading_radians;
  double heading_delta;
  double heading_delta_norm;
  double speed = 0;
  double speed_smoothed = 0;
  double gradient;
  double curvature;       // radians per metre
  int tracepoint_idx = 0; // index back to OSRM tracepoint
  double cum_dist = 0;    // cumulative distance in metres

  Json extensions = Json::object(); // GPX extensions
};

// Convenience container for a full route.
struct RouteSignal {
  std::vector<DataPoint> points;
};
