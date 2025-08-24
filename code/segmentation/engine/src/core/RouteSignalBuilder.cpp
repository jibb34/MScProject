#include "RouteSignalBuilder.hpp"
#include "core/SegmentUtils.hpp"
#include <iostream>
#include <limits>

RouteSignal RouteSignalBuilder::build(const OsrmResponse &osrm) {
  RouteSignal signal;
  try {
    if (osrm.matching.legs.empty()) {
      std::cerr << "[RouteSignalBuilder] OSRM response is empty.\n";
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
    if (signal.points.size() < 2) {
      std::cerr << "[RouteSignalBuilder] Not enough points ("
                << signal.points.size() << ")\n";
      return signal; // empty -> caller can decide how to respond
    }

    std::cerr << "Populating coordinates...\n";
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
    std::cerr << "Assigning Way ID's\n";
    assignWayIDs(osrm.matching.legs, osrm.tracepoints, signal);

    std::cerr << "Computing Cumulative distance...\n";
    // compute cumulative distance
    double acc_m = 0.0;
    for (size_t i = 0; i < signal.points.size(); ++i) {
      if (i == 0) {
        signal.points[i].cum_dist = 0.0;
      } else {
        const auto &a = signal.points[i - 1].coord;
        const auto &b = signal.points[i].coord;
        acc_m += SegmentUtils::haversine(a, b); // meters
        signal.points[i].cum_dist = acc_m;
      }
    }
    // TODO: implement curvature, heading, and gradient/variation

    // Curvature: Decide on a window size, or make it dynamic
    //
    // heading:
    std::cerr << "calulating heading...\n";
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
    SegmentUtils::compute_and_smooth_speed(signal.points, 2.0, 5, 3);
    // TEST:
    // normalize heading deltas to speed
    SegmentUtils::normalize_heading_to_speed(signal.points);
    //  gradient:
    SegmentUtils::compute_windowed_gradients(signal.points, 20.0, 20.0);

  } catch (std::exception e) {
    std::cerr << "Route Signal Build Failure: " << e.what() << std::endl;
    signal = {};
  }
  return signal;
}

void RouteSignalBuilder::assignWayIDs(const std::vector<Leg> &legs,
                                      const std::vector<Tracepoint> &tps,
                                      RouteSignal &rs) {
  size_t point_counter = 0;
  for (size_t tp_idx = 0; tp_idx < tps.size(); ++tp_idx) {
    const Tracepoint &tp = tps[tp_idx];
    if (!tp.matched)
      continue;

    const auto &gpx_list = tp.gpx_list;
    const int leg_idx = tp.waypoint_index;
    const auto &runs = (leg_idx >= 0 && leg_idx < (int)legs.size())
                           ? legs[leg_idx].runs
                           : std::vector<Run>{};

    size_t run_idx = 0;
    double run_dist_remaining = runs.empty()
                                    ? std::numeric_limits<double>::infinity()
                                    : runs[0].length_m;

    for (size_t gpx_idx = 0; gpx_idx < gpx_list.size();
         ++gpx_idx, ++point_counter) {
      DataPoint &dp = rs.points[point_counter];

      // DO NOT reassign dp.coord; you already set {lat, lon, elv} earlier.
      dp.tracepoint_idx = tp_idx;
      dp.way_id = runs.empty() ? -1 : runs[run_idx].way_id;

      // If you want to move run_idx using distances, do it here,
      // but DO NOT change cum_dist here.
      if (!runs.empty() && gpx_idx + 1 < gpx_list.size()) {
        const double edge_len = SegmentUtils::haversine(
            gpx_list[gpx_idx].lat, gpx_list[gpx_idx + 1].lat,
            gpx_list[gpx_idx].lon, gpx_list[gpx_idx + 1].lon);
        run_dist_remaining -= edge_len;
        while (run_dist_remaining <= 0 && run_idx + 1 < runs.size()) {
          run_idx++;
          run_dist_remaining += runs[run_idx].length_m;
        }
      }
    }
  }
}
