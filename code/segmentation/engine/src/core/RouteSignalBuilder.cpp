// This file is an updated version of the segmentation engine’s
// RouteSignalBuilder.
//
// The original implementation populated each DataPoint’s coordinates directly
// from the input GPX tracepoints and used those coordinates to compute
// cumulative distance, heading, speed and gradient for a route.  In practice
// the OSRM matching step also returns an enriched geometry polyline (a sequence
// of longitude/latitude pairs) describing the snapped route.  Because the GPX
// points often contain GPS noise, the segments produced by the engine can
// appear jagged and imprecise when visualised.  To tidy the segments we
// replace each DataPoint’s latitude/longitude with the nearest coordinate
// sampled from the OSRM geometry.  Elevation is preserved from the original
// tracepoint.  After adjusting the coordinates we recompute the cumulative
// distance and derive heading, speed and gradient as before.

#include "RouteSignalBuilder.hpp"
#include "core/SegmentUtils.hpp"
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <random>
#include <vector>

// --- helper: compute bearings on geometry (radians)
static inline double geom_heading_rad(const Coordinate &a,
                                      const Coordinate &b) {
  return SegmentUtils::calculateHeadingDegToRad(a, b);
}

// --- helper: absolute wrapped angle difference in [0,pi]
static inline double ang_diff_abs(double a, double b) {
  return std::fabs(SegmentUtils::ang_diff(a, b));
}

// --- quick nearest-vertex distance on geometry (linear scan; fine for tens of
// k points)
static inline double nearest_geom_vertex_m(const Coordinate &p,
                                           const std::vector<Coordinate> &g) {
  double best = std::numeric_limits<double>::infinity();
  for (const auto &q : g) {
    best = std::min(best, SegmentUtils::haversine(p, q));
  }
  return best;
}

// --- local ENU (equirect) projection around an origin (good enough for short
// spans)
struct Enu {
  double x, y;
};
static inline Enu to_enu(const Coordinate &origin, const Coordinate &p) {
  // meters per deg near origin
  const double lat0 = origin.lat * M_PI / 180.0;
  const double m_per_deg_lat = 111132.954; // approx
  const double m_per_deg_lon = 111132.954 * std::cos(lat0);
  return {(p.lon - origin.lon) * m_per_deg_lon,
          (p.lat - origin.lat) * m_per_deg_lat};
}

// --- signed cross-track to segment AB (positive on left of A->B in ENU)
static inline double signed_xt_m(const Coordinate &A, const Coordinate &B,
                                 const Coordinate &P,
                                 const Coordinate &origin) {
  Enu a = to_enu(origin, A), b = to_enu(origin, B), p = to_enu(origin, P);
  const double vx = b.x - a.x, vy = b.y - a.y;
  const double wx = p.x - a.x, wy = p.y - a.y;
  const double L2 = vx * vx + vy * vy;
  if (L2 <= 1e-6)
    return 0.0;
  // projection parameter t in [0,1]
  double t = (wx * vx + wy * vy) / L2;
  t = std::max(0.0, std::min(1.0, t));
  // vector from foot to P
  const double fx = a.x + t * vx, fy = a.y + t * vy;
  const double rx = p.x - fx, ry = p.y - fy;
  // signed area / length = signed cross-track (left +, right -)
  const double cross = (vx * ry - vy * rx);
  const double seg_len = std::sqrt(L2);
  return cross / (seg_len > 1e-6 ? 1.0 : 1.0); // already in meters since ENU
}

// --- windowed stats for heading (circular mean + dispersion proxy)
struct HeadingStats {
  double mean = 0.0;     // radians
  double var_approx = 0; // 0..pi (use std of small diffs as proxy)
};
static HeadingStats window_heading_stats_rad(const std::deque<double> &hwin) {
  if (hwin.empty())
    return {};
  // circular mean via unit vectors
  double sx = 0, sy = 0;
  for (double a : hwin) {
    sx += std::cos(a);
    sy += std::sin(a);
  }
  double mean = std::atan2(sy, sx);
  // variance proxy: mean absolute angular deviation
  double mad = 0.0;
  for (double a : hwin)
    mad += ang_diff_abs(a, mean);
  mad /= static_cast<double>(hwin.size());
  return {mean, mad};
}
// --- bootstrap mean distance over first INIT_N points, used as global
// feasibility
static bool bootstrap_geometry_ok(const RouteSignal &rs,
                                  const std::vector<Coordinate> &g,
                                  int INIT_N = 120,
                                  double MAX_MEAN_M = 22.0) { // tight bias
  const int N = static_cast<int>(rs.points.size());
  if (N == 0 || g.size() < 2)
    return false;
  const int n = std::min(N, INIT_N);
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += nearest_geom_vertex_m(rs.points[i].coord, g);
  }
  const double mean = sum / std::max(1, n);
  return mean <= MAX_MEAN_M;
}

// Snap rs.points to OSRM geometry monotonically forward
static void snap_points_monotone(RouteSignal &rs, const Geometry &geom) {
  if (geom.coordinates.size() < 2 || rs.points.size() < 2)
    return;

  // Build geometry as lat/lon and cumulative distances
  const size_t G = geom.coordinates.size();
  std::vector<Coordinate> g(G);
  for (size_t i = 0; i < G; ++i) {
    // NOTE: Geometry is [lon, lat]
    g[i] = Coordinate{geom.coordinates[i][1], geom.coordinates[i][0],
                      rs.points.front().coord.elv};
  }
  std::vector<double> gcum(G, 0.0), ghead(G, 0.0);
  for (size_t j = 1; j < G; ++j) {
    gcum[j] = gcum[j - 1] + SegmentUtils::haversine(g[j - 1], g[j]);
  }
  for (size_t j = 0; j + 1 < G; ++j) {
    ghead[j] = geom_heading_rad(g[j], g[j + 1]);
  }
  ghead.back() = ghead[G - 2];

  // ---------- NEW: global guard ----------
  // ---------- NEW: bootstrap feasibility (tight) ----------
  if (!bootstrap_geometry_ok(rs, g)) {
    return; // keep original GPX coords
  }

  // Tunables
  const int WINDOW_FWD = 18;      // fewer candidates → less zig-zag
  const double SLACK_M = 18.0;    // tighter search around prediction
  const double ALPHA = 8.0;       // weigh heading more
  const int MAX_MISS_STREAK = 25; // stricter kill-switch
  // Adaptive gate parameters
  const int W = 50;             // running window length
  const double K_MULT = 1.25;   // threshold multiplier on avg
  const double MIN_ALLOW = 6.0; // never require below this
  const double HARD_CAP = 25.0; // never allow above this
  const double ABORT_MULT =
      2.5;                  // if d_best > ABORT_MULT * avg repeatedly → abort
                            //
  const int HWIN = 25;      // heading window (samples)
  const int GHEAD_WIN = 20; // geometry heading window
  const double HEADING_TOL = 10.0 * M_PI / 180.0; // 10 deg
  const double VAR_STRAIGHT = 5.0 * M_PI / 180.0; // “stable” if MAD < 5 deg
  const double LANE_SEP_M = 12.0; // treat offsets below this as same corridor
  const double SIDE_FLIP_PEN =
      1.5; // multiply cost if attempting side flip under stability

  // Ensure DP headings exist (coarse pre-pass)
  for (size_t i = 0; i + 1 < rs.points.size(); ++i) {
    rs.points[i].heading_radians = SegmentUtils::calculateHeadingDegToRad(
        rs.points[i].coord, rs.points[i + 1].coord);
  }
  if (!rs.points.empty())
    rs.points.back().heading_radians =
        rs.points[rs.points.size() - 2].heading_radians;

  size_t last_j = 0;
  double last_gdist = 0.0;
  int miss_streak = 0;
  // running average of nearest distances (to accepted snaps)
  std::deque<double> window_d;
  double window_sum = 0.0;
  auto avg_d = [&]() {
    if (window_d.empty())
      return 12.0; // sane default if not warmed up
    return window_sum / static_cast<double>(window_d.size());
  };
  // heading windows (GPX and geometry)
  std::deque<double> hwin_gpx;
  // track last accepted side sign (+left / -right) to enforce side consistency
  int last_side_sign = 0; // 0 unknown, +1 left, -1 right
  const Coordinate origin = rs.points.front().coord;
  for (size_t i = 0; i < rs.points.size(); ++i) {
    // Predict along-geometry distance using DP inter-point distance
    if (i > 0) {
      double ds =
          SegmentUtils::haversine(rs.points[i - 1].coord, rs.points[i].coord);
      last_gdist += ds;
    }
    // Find window start near predicted distance, but never before last_j
    size_t j0 = last_j;
    if (last_gdist > gcum[last_j]) {
      // lower_bound in [last_j, G)
      auto it = std::lower_bound(gcum.begin() + static_cast<long>(last_j),
                                 gcum.end(), last_gdist - SLACK_M);
      j0 = static_cast<size_t>(std::distance(gcum.begin(), it));
      if (j0 < last_j)
        j0 = last_j;
    }
    size_t j1 = std::min(G - 1, j0 + static_cast<size_t>(WINDOW_FWD));

    // Evaluate candidates forward-only
    double best_cost = std::numeric_limits<double>::infinity();
    size_t best_j = last_j;
    const double h_dp = rs.points[i].heading_radians;
    for (size_t j = j0; j <= j1; ++j) {
      double d = SegmentUtils::haversine(rs.points[i].coord, g[j]);
      double ghd = ghead[j == G - 1 ? G - 2 : j];
      double angc = ang_diff_abs(h_dp, ghd);
      double cost = d + ALPHA * angc;
      // If attempting to switch “side” while the corridor is straight, penalize
      if (last_side_sign != 0 && j + 1 < G) {
        double xt = signed_xt_m(g[j], g[j + 1], rs.points[i].coord, origin);
        int side_now = (xt >= 0.0) ? +1 : -1;
        // geometry local stability over small window
        double g_mad = 0.0;
        {
          int cnt = 0;
          double sx = 0, sy = 0;
          for (size_t k = j;
               k < std::min(G - 1, j + static_cast<size_t>(GHEAD_WIN)); ++k) {
            sx += std::cos(ghead[k]);
            sy += std::sin(ghead[k]);
            ++cnt;
          }
          double mean = std::atan2(sy, sx);
          for (size_t k = j;
               k < std::min(G - 1, j + static_cast<size_t>(GHEAD_WIN)); ++k) {
            g_mad += ang_diff_abs(ghead[k], mean);
          }
          g_mad /= std::max(1, cnt);
        }
        // If both GPX and geometry are straight-ish and side flips, penalize
        if (side_now != last_side_sign && std::abs(xt) < LANE_SEP_M) {
          // defer to GPX heading stability window
          if (hwin_gpx.size() >= static_cast<size_t>(HWIN)) {
            auto hs = window_heading_stats_rad(hwin_gpx);
            if (hs.var_approx < VAR_STRAIGHT && g_mad < VAR_STRAIGHT) {
              cost *= SIDE_FLIP_PEN;
            }
          }
        }
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_j = j;
      }
    }
    // Adaptive acceptance gate based on running avg of nearest distances
    if (best_j >= last_j) {
      double d_best = SegmentUtils::haversine(rs.points[i].coord, g[best_j]);
      // dynamic threshold = clamp(MIN_ALLOW, K_MULT * avg_d, HARD_CAP)
      const double thr =
          std::max(MIN_ALLOW, std::min(K_MULT * avg_d(), HARD_CAP));
      // heading-consistency gate when corridor is stable
      bool heading_ok = true;
      // maintain gpx heading window
      hwin_gpx.push_back(h_dp);
      if (static_cast<int>(hwin_gpx.size()) > HWIN)
        hwin_gpx.pop_front();
      if (hwin_gpx.size() >= static_cast<size_t>(HWIN)) {
        auto hs = window_heading_stats_rad(hwin_gpx);
        // compute geometry window stats around best_j
        double g_mad = 0.0;
        int cnt = 0;
        double sx = 0, sy = 0;
        for (size_t k = best_j;
             k < std::min(G - 1, best_j + static_cast<size_t>(GHEAD_WIN));
             ++k) {
          sx += std::cos(ghead[k]);
          sy += std::sin(ghead[k]);
          ++cnt;
        }
        double g_mean = std::atan2(sy, sx);
        for (size_t k = best_j;
             k < std::min(G - 1, best_j + static_cast<size_t>(GHEAD_WIN));
             ++k) {
          g_mad += ang_diff_abs(ghead[k], g_mean);
        }
        g_mad /= std::max(1, cnt);
        // if both are straight-ish, enforce tight heading tolerance to mean
        if (hs.var_approx < VAR_STRAIGHT && g_mad < VAR_STRAIGHT) {
          heading_ok = (ang_diff_abs(hs.mean, g_mean) <= HEADING_TOL);
        }
      }
      if (d_best <= thr && heading_ok) {
        // keep original elevation
        rs.points[i].coord.lat = g[best_j].lat;
        rs.points[i].coord.lon = g[best_j].lon;
        last_j = best_j;
        last_gdist = gcum[best_j];
        miss_streak = 0;
        // update running window with the actually achieved nearest distance
        window_d.push_back(d_best);
        window_sum += d_best;
        if (static_cast<int>(window_d.size()) > W) {
          window_sum -= window_d.front();
          window_d.pop_front();
        }
        // update side sign
        if (best_j + 1 < G) {
          double xt =
              signed_xt_m(g[best_j], g[best_j + 1], rs.points[i].coord, origin);
          last_side_sign = (xt >= 0.0) ? +1 : -1;
        }
        continue;
      }
    }
    // miss: leave GPX coord
    // if we keep missing while avg_d is small, abort snapping (clearly
    // mismatching)
    if (++miss_streak >= MAX_MISS_STREAK) {
      break;
    }
    // hard drift detector: repeated egregious misses vs avg
    if (!window_d.empty()) {
      const double thr_abort = ABORT_MULT * avg_d();
      double d_best_now =
          SegmentUtils::haversine(rs.points[i].coord, g[best_j]);
      if (d_best_now > thr_abort && miss_streak > (MAX_MISS_STREAK / 2)) {
        break;
      }
      // Too many consecutive fails -> revert remaining points to GPX (abort)
      break;
    }
  }
}

RouteSignal RouteSignalBuilder::build(const OsrmResponse &osrm) {
  RouteSignal signal;
  try {
    if (osrm.matching.legs.empty()) {
      return signal;
    }
    size_t total_points = 0;
    double start_time = get_start_time(osrm);
    // Determine how many datapoints we will have in the output signal and
    // allocate that many.  Each tracepoint holds a list of GPX samples.
    for (const auto &tp : osrm.tracepoints) {
      total_points += tp.gpx_list.size();
    }
    signal.points.resize(total_points);
    if (signal.points.size() < 2) {
      return signal;
    }

    // Populate DataPoints from the raw GPX trace.  We initialise lat/lon/elv
    // using the GPX values and store the relative time from the start of the
    // recording.  Additional fields (way_id, heading etc.) will be filled
    // later.
    size_t point_counter = 0;
    for (size_t tp_idx = 0; tp_idx < osrm.tracepoints.size(); ++tp_idx) {
      const auto &tp = osrm.tracepoints[tp_idx];
      for (const auto &gpx : tp.gpx_list) {
        DataPoint &dp = signal.points[point_counter++];
        dp.coord = {gpx.lat, gpx.lon, gpx.elv};
        dp.tracepoint_idx = tp_idx;
        dp.time_rel = gpx.time - start_time;
      }
    }

    // Assign the way identifiers for each point.  This relies on the OSRM
    // matchings to tell us which OpenStreetMap way each GPX sample belongs to.
    assignWayIDs(osrm.matching.legs, osrm.tracepoints, signal);

    // monotone geometry snapping (prevents “against the grain” jumps)
    if (!osrm.matching.geometry.coordinates.empty()) {
      snap_points_monotone(signal, osrm.matching.geometry);
    }

    // ---------------------------------------------------------------------
    // Neaten coordinates using OSRM geometry
    //
    // The OSRM response contains a geometry polyline on the matched route.
    // Each coordinate is an array of [lon, lat] values.  We want each
    // DataPoint’s latitude/longitude to correspond to the closest point on
    // this geometry rather than the raw GPS coordinate.  This step helps
    // smooth out noise in the input and makes resulting segments look
    // cleaner when plotted.
    if (!osrm.matching.geometry.coordinates.empty()) {
      const auto &geom_coords = osrm.matching.geometry.coordinates;
      for (auto &dp : signal.points) {
        // Preserve the original altitude.  Record the original lat/lon for
        // distance computations.
        const double orig_lat = dp.coord.lat;
        const double orig_lon = dp.coord.lon;
        double best_dist = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        // Iterate through each geometry coordinate and find the one with
        // smallest haversine distance to the current GPX point.  The
        // haversine() overload with four doubles uses (x1,x2,y1,y2) where x
        // denotes longitude and y denotes latitude.
        for (size_t i = 0; i < geom_coords.size(); ++i) {
          const double g_lon = geom_coords[i][0];
          const double g_lat = geom_coords[i][1];
          const double d =
              SegmentUtils::haversine(orig_lon, g_lon, orig_lat, g_lat);
          if (d < best_dist) {
            best_dist = d;
            best_idx = i;
          }
        }
        // Update the DataPoint’s latitude and longitude to the closest
        // geometry coordinate.  Elevation remains unchanged.
        dp.coord.lat = geom_coords[best_idx][1];
        dp.coord.lon = geom_coords[best_idx][0];
      }
    }

    // ---------------------------------------------------------------------
    // Cumulative distance
    //
    // Compute the cumulative distance along the route using the updated
    // coordinates.  We initialise the first point’s cum_dist to zero and
    // accumulate the haversine distance between successive coordinates.  The
    // result is stored in each DataPoint’s cum_dist field.
    double acc_m = 0.0;
    for (size_t i = 0; i < signal.points.size(); ++i) {
      if (i == 0) {
        signal.points[i].cum_dist = 0.0;
      } else {
        const auto &a = signal.points[i - 1].coord;
        const auto &b = signal.points[i].coord;
        acc_m += SegmentUtils::haversine(a, b);
        signal.points[i].cum_dist = acc_m;
      }
    }

    // ---------------------------------------------------------------------
    // Heading and derivative computations
    //
    // Calculate instantaneous heading between consecutive points and the
    // change in heading (delta) across samples.  We then normalise the
    // heading changes by speed and compute gradients.  All these methods
    // operate on the updated coordinates stored in signal.points.
    for (size_t dp_idx = 0; dp_idx < signal.points.size() - 1; ++dp_idx) {
      DataPoint &dp1 = signal.points[dp_idx];
      DataPoint &dp2 = signal.points[dp_idx + 1];
      signal.points[dp_idx].heading_radians =
          SegmentUtils::calculateHeadingDegToRad(dp1.coord, dp2.coord);
      if (dp_idx > 0) {
        signal.points[dp_idx].heading_delta =
            SegmentUtils::ang_diff(signal.points[dp_idx - 1].heading_radians,
                                   signal.points[dp_idx].heading_radians);
      }
    }
    if (!signal.points.empty()) {
      // Assign the last point’s heading equal to the penultimate one.
      signal.points.back().heading_radians =
          std::prev(signal.points.end(), 2)->heading_radians;
    }

    // Smooth speed and gradients
    SegmentUtils::compute_and_smooth_speed(signal.points, 2.0, 5, 3);
    SegmentUtils::normalize_heading_to_speed(signal.points);
    SegmentUtils::compute_windowed_gradients(signal.points, 20.0, 20.0);
  } catch (std::exception e) {
    signal = {};
  }
  return signal;
}

// assignWayIDs is unchanged from the original implementation.  It associates
// each DataPoint with a run of consecutive OpenStreetMap way IDs based on
// OSRM matchings.  The caller must ensure that the number of DataPoints
// matches the combined size of all tracepoints’ GPX lists.
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
    const auto &runs = (leg_idx >= 0 && leg_idx < static_cast<int>(legs.size()))
                           ? legs[leg_idx].runs
                           : std::vector<Run>{};
    size_t run_idx = 0;
    double run_dist_remaining = runs.empty()
                                    ? std::numeric_limits<double>::infinity()
                                    : runs[0].length_m;
    for (size_t gpx_idx = 0; gpx_idx < gpx_list.size();
         ++gpx_idx, ++point_counter) {
      DataPoint &dp = rs.points[point_counter];
      // dp.coord has already been set above; do not overwrite it here.
      dp.tracepoint_idx = tp_idx;
      dp.way_id = runs.empty() ? -1 : runs[run_idx].way_id;
      // If we want to advance run_idx based on GPX distance, do it here.
      if (!runs.empty() && gpx_idx + 1 < gpx_list.size()) {
        const double edge_len = SegmentUtils::haversine(
            gpx_list[gpx_idx].lon, gpx_list[gpx_idx + 1].lon,
            gpx_list[gpx_idx].lat, gpx_list[gpx_idx + 1].lat);
        run_dist_remaining -= edge_len;
        while (run_dist_remaining <= 0 && run_idx + 1 < runs.size()) {
          run_idx++;
          run_dist_remaining += runs[run_idx].length_m;
        }
      }
    }
  }
}
