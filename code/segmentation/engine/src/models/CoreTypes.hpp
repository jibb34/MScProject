#pragma once
#include <inttypes.h>
#include <sys/types.h>
#include <vector>

struct Coordinate {
  double lat;
  double lon;
  double elv;
};

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

struct DataPoint {
  // Raw Data
  Coordinate coord;
  double time_rel; // time from point 0
  uint32_t way_id;
  HighwayType highway_type;

  // Derived
  double heading_radians;
  double heading_delta;
  double heading_delta_norm;
  double speed = 0;
  double speed_smoothed = 0;
  double gradient;
  double curvature; // in rads per metre
  // For obtaining future values double cum_dist;
  int tracepoint_idx = 0;
  double cum_dist = 0;
};

struct RouteSignal {
  std::vector<DataPoint> points;
};
