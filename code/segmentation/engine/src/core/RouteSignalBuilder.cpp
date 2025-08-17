#include "RouteSignalBuilder.hpp"
#include "core/SegmentUtils.hpp"
#include <limits>

RouteSignal RouteSignalBuilder::build(const OsrmResponse &osrm) {
  RouteSignal signal;
  if (osrm.matching.legs.empty()) {
    return signal;
  }
  size_t total_points = 0;
  double total_dist = 0.0;
  double start_time = get_start_time(osrm);
  for (const auto &tp : osrm.tracepoints) {
    total_points += tp.gpx_list.size();
  }
  // pre-size vector to correct size
  signal.points.resize(total_points);

  // Populate Coordinates
  size_t point_counter = 0;
  for (size_t tp_idx = 0; tp_idx < osrm.tracepoints.size(); ++tp_idx) {
    const auto &tp = osrm.tracepoints[tp_idx];
    for (const auto &gpx : tp.gpx_list) {
      DataPoint &dp = signal.points[point_counter++];
      dp.coord = {gpx.lat, gpx.lon, gpx.elv};
      dp.tracepoint_idx = tp_idx;
      dp.time_rel = gpx.time - start_time; // get time offset
    }
  }
  assignWayIDs(osrm.matching.legs, osrm.tracepoints, signal);
  // TODO: implement curvature, heading, and gradient/variation

  // Curvature: Decide on a window size, or make it dynamic
  //
  // heading:
  for (size_t dp_idx = 0; dp_idx < signal.points.size() - 1; ++dp_idx) {
    DataPoint &dp1 = signal.points[dp_idx];
    DataPoint &dp2 = signal.points[dp_idx + 1];
    signal.points[dp_idx].heading_radians =
        SegmentUtils::calculateHeadingDegToRad(dp1.coord, dp2.coord);
    // calculate heading delta
    if (dp_idx > 0) {
      signal.points[dp_idx].heading_delta =
          SegmentUtils::ang_diff(signal.points[dp_idx - 1].heading_radians,
                                 signal.points[dp_idx].heading_radians);
    }
  }
  if (!signal.points.empty()) { // Guard
    // Assigns last heading to be same as second-to-last since we have no
    // further reference (close enough)
    signal.points.back().heading_radians =
        std::prev(signal.points.end(), 2)->heading_radians;
  }
  // Normalize heading delta by speed:

  // instantaneous speed:
  if (!signal.points.empty()) {
    signal.points[0].speed = 0.0;
    for (size_t i = 1; i < signal.points.size(); ++i) {
      const DataPoint &A = signal.points[i - 1];
      const DataPoint &B = signal.points[i];
      const double dt = std::max( // max is to avoid divide by 0 errors
          1e-3, double(B.time_rel) - double(A.time_rel)); // should be 1 second
      const double ds = SegmentUtils::haversine(A.coord, B.coord);
      signal.points[i].speed = ds / dt;
    }
  }
  // gradient:

  SegmentUtils::compute_windowed_gradients(signal.points, 20.0, 20.0);

  return signal;
}

void RouteSignalBuilder::assignWayIDs(const std::vector<Leg> &legs,
                                      const std::vector<Tracepoint> &tps,
                                      RouteSignal &rs) {
  // For each tracepoint, we iterate through it's gpx list, calcualting distance
  // as we go, when distance gets over the length of the current way_run,
  // subtract next way_run's distance from gpx distance accumulated, and reset

  double total_dist = 0.0;
  size_t point_counter = 0;

  for (size_t tp_idx = 0; tp_idx < tps.size(); ++tp_idx) {
    const Tracepoint &tp = tps[tp_idx];
    const auto &gpx_list = tp.gpx_list;

    if (!tp.matched) {
      // still advance to keep point_counter aligned with build()
      point_counter += gpx_list.size();
      continue;
    }

    // Map this tracepoint to its leg/runs (guard indices)
    int leg_idx = tp.waypoint_index;
    const std::vector<Run> *runs_ptr = nullptr;
    if (leg_idx >= 0 && static_cast<size_t>(leg_idx) < legs.size()) {
      runs_ptr = &legs[leg_idx].runs;
    }
    const auto &runs = runs_ptr ? *runs_ptr : std::vector<Run>{};

    size_t run_idx = 0;
    double run_dist_remaining = runs.empty()
                                    ? std::numeric_limits<double>::infinity()
                                    : runs[0].length_m;

    for (size_t gpx_idx = 0; gpx_idx < gpx_list.size();
         ++gpx_idx, ++point_counter) {
      DataPoint &dp = rs.points[point_counter];

      // DO NOT overwrite dp.coord with a 2D value; if you want to refresh,
      // include elv: dp.coord = {gpx_list[gpx_idx].lat, gpx_list[gpx_idx].lon,
      // gpx_list[gpx_idx].elv};

      dp.tracepoint_idx = tp_idx;
      dp.way_id = runs.empty() ? -1 : runs[run_idx].way_id;
      dp.cum_dist = total_dist;

      // advance distance and run position
      if (gpx_idx + 1 < gpx_list.size()) {
        double edge_len =
            SegmentUtils::haversine(gpx_list[gpx_idx], gpx_list[gpx_idx + 1]);
        total_dist += edge_len;

        if (!runs.empty()) {
          run_dist_remaining -= edge_len;
          while (run_dist_remaining <= 0 && run_idx + 1 < runs.size()) {
            run_idx++;
            run_dist_remaining += runs[run_idx].length_m;
          }
        }
      }
    }
  }
}
