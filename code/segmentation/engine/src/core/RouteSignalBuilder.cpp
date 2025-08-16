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
      dp.coord = {gpx.lat, gpx.lon};
      dp.tracepoint_idx = tp_idx;
      dp.time_rel = gpx.time - start_time; // get time offset
    }
  }
  assignWayIDs(osrm.matching.legs, osrm.tracepoints, signal);
  // TODO: implement curvature, heading, and gradient/variation

  return signal;
}

void RouteSignalBuilder::assignWayIDs(const std::vector<Leg> &legs,
                                      const std::vector<Tracepoint> &tps,
                                      RouteSignal &rs) {
  // For each tracepoint, we iterate through it's gpx list, calcualting distance
  // as we go, when distance gets over the length of the current way_run,
  // subtract next way_run's distance from gpx distance accumulated, and reset

  Coordinate ref;
  bool ref_set = false;
  double total_dist = 0;
  size_t point_counter = 0;
  // Iterate over tracepoints
  for (size_t tp_idx = 0; tp_idx < tps.size(); ++tp_idx) {
    const Tracepoint &tp = tps[tp_idx];
    if (!tp.matched) {
      continue;
    }
    const std::vector<GpxPoint> &gpx_list = tp.gpx_list;
    int leg_idx = tp.waypoint_index;

    const std::vector<Run> &runs = legs[leg_idx].runs;
    size_t run_idx = 0;
    // if no runs, distance is infinity so we always add current waypoint until
    // next tracepoint
    double run_dist_remaining = runs.empty()
                                    ? std::numeric_limits<double>::infinity()
                                    : runs[0].length_m;
    // set reference for distance
    if (!ref_set && !gpx_list.empty()) {
      ref = {gpx_list.front().lat, gpx_list.front().lon};
      ref_set = true;
    }
    rs.points[tp_idx].cum_dist = total_dist;
    // iterate over gpx points in tracepoint
    for (size_t gpx_idx = 0; gpx_idx < gpx_list.size();
         ++gpx_idx, ++point_counter) {
      DataPoint &dp = rs.points[point_counter];
      dp.coord = {gpx_list[gpx_idx].lat, gpx_list[gpx_idx].lon};
      dp.tracepoint_idx = tp_idx;
      // assign way id at current run index
      dp.way_id = runs.empty() ? -1 : runs[run_idx].way_id;

      // check distances
      if (gpx_idx + 1 < gpx_list.size() && !runs.empty()) {
        double edge_len =
            SegmentUtils::haversine(gpx_list[gpx_idx], gpx_list[gpx_idx + 1]);
        total_dist += edge_len;
        run_dist_remaining -= edge_len; // subtract from run dist remaining;
        // if run remaining is less than 0, add next run length onto remaining.
        while (run_dist_remaining <= 0 && run_idx + 1 < runs.size()) {
          run_idx++;
          run_dist_remaining += runs[run_idx].length_m;
        }
      }
    }
  }
}
