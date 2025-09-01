#include "core/SegmentUtils.hpp"
#include "models/OsrmResponse.hpp"
#include <math.h>
#include <numeric>
#include <vector>

double checkForJunction() {
  // TODO: implement function to find the location of the junction as a
  // segmentation node
  return 0.0;
}

double SegmentUtils::haversine(double x1, double x2, double y1, double y2) {
  double phi1 = y1 * (M_PI / 180);
  double phi2 = y2 * (M_PI / 180);
  double delta_phi = (y2 - y1) * (M_PI / 180);
  double delta_gamma = (x2 - x1) * (M_PI / 180);
  double h = pow(sin(delta_phi / 2), 2) +
             cos(phi1) * cos(phi2) * pow(sin(delta_gamma / 2), 2);
  return 2 * 6371000 * asin(sqrt(h));
}
double SegmentUtils::haversine(const GpxPoint &p1, const GpxPoint &p2) {
  double phi1 = p1.lat * (M_PI / 180);
  double phi2 = p2.lat * (M_PI / 180);
  double delta_phi = (p2.lat - p1.lat) * (M_PI / 180);
  double delta_gamma = (p2.lon - p1.lon) * (M_PI / 180);
  double h = pow(sin(delta_phi / 2), 2) +
             cos(phi1) * cos(phi2) * pow(sin(delta_gamma / 2), 2);
  return 2 * 6371000 * asin(sqrt(h));
}
double SegmentUtils::haversine(const Coordinate &p1, const Coordinate &p2) {
  double phi1 = p1.lat * (M_PI / 180);
  double phi2 = p2.lat * (M_PI / 180);
  double delta_phi = (p2.lat - p1.lat) * (M_PI / 180);
  double delta_gamma = (p2.lon - p1.lon) * (M_PI / 180);
  double h = pow(sin(delta_phi / 2), 2) +
             cos(phi1) * cos(phi2) * pow(sin(delta_gamma / 2), 2);
  return 2 * 6371000 * asin(sqrt(h));
}
// double SegmentUtils::calculateGradient(const Coordinate &coord1,
//                                        const Coordinate &coord2) {
//   double dist =
//       SegmentUtils::haversine(coord1.lon, coord2.lon, coord1.lat,
//       coord2.lat);
//   return (coord2.elv - coord1.elv) / dist;
// }

// Helpers local to this TU
static inline std::size_t seek_left_by_dist(const std::vector<DataPoint> &pts,
                                            std::size_t i, double back_m) {
  const double target = pts[i].cum_dist - back_m;
  while (i > 0 && pts[i - 1].cum_dist >= target)
    --i;
  return i;
}

static inline std::size_t seek_right_by_dist(const std::vector<DataPoint> &pts,
                                             std::size_t i, double fwd_m) {
  const double target = pts[i].cum_dist + fwd_m;
  const std::size_t n = pts.size();
  while (i + 1 < n && pts[i + 1].cum_dist <= target)
    ++i;
  return i;
}

// Windowed gradient in percent: (elev[R]-elev[L]) / (dist[R]-dist[L]) * 100
void SegmentUtils::compute_windowed_gradients(std::vector<DataPoint> &pts,
                                              double back_window_m,
                                              double fwd_window_m,
                                              double eps_m) {
  const std::size_t n = pts.size();
  if (n == 0)
    return;
  if (n == 1) {
    pts[0].gradient = 0.0;
    return;
  }

  for (std::size_t i = 0; i < n; ++i) {
    std::size_t L = seek_left_by_dist(pts, i, back_window_m);
    std::size_t R = seek_right_by_dist(pts, i, fwd_window_m);

    // collapse guard: ensure at least a span
    if (R <= L) {
      L = (i > 0) ? i - 1 : i;
      R = (i + 1 < n) ? i + 1 : i;
    }

    const double base = pts[R].cum_dist - pts[L].cum_dist;
    double g = 0.0;
    if (std::isfinite(base) && base > eps_m) {
      const double dh = pts[R].coord.elv - pts[L].coord.elv; // metres
      g = (dh / base) * 100.0;                               // percent
    }

    if (!std::isfinite(g))
      g = 0.0;
    pts[i].gradient = g;
  }
}

double
SegmentUtils::calculateGradientVariation(const std::vector<DataPoint> &points) {
  std::vector<double> grades;
  // push first value so we only need to keep pushing 2nd value
  // loop until penultimate value
  for (int i = 0; i + 1 < points.size(); i++) {
    DataPoint point1 = points.at(i);
    DataPoint point2 = points.at(i + 1);
    double gradient = (point2.coord.elv - point1.coord.elv) /
                      haversine(point1.coord, point2.coord);
    grades.push_back(gradient);
  }
  // if no gradients found
  if (grades.empty()) {
    return 0.0;
  }
  // Calculate mean through (sum all values / size)
  double mean = std::reduce(grades.begin(), grades.end(), 0.0) / grades.size();
  int size = grades.size();
  auto variance_func = [&mean, &size](double accumulator, const double &val) {
    return accumulator + ((val - mean) * (val - mean) / (size - 1));
  };
  return std::accumulate(grades.begin(), grades.end(), 0.0, variance_func);
}

ENU to_local(const Coordinate &ref, const Coordinate &p) {
  constexpr double R = 6371008.8; // mean Earth radius (m)
  double lat0 = ref.lat * M_PI / 180.0;
  double dlat = (p.lat - ref.lat) * M_PI / 180.0;
  double dlon = (p.lon - ref.lon) * M_PI / 180.0;
  double x = R * dlon * std::cos(lat0);
  double y = R * dlat;
  return {0, 0};
}
double dist(const ENU &a, const ENU &b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

double heading_ENU(const ENU &a, const ENU &b) {
  return std::atan2(b.y - a.y, b.x - a.x);
}

double unwrap_angle(double d) {
  while (d > M_PI)
    d -= 2 * M_PI;
  while (d <= -M_PI)
    d += 2 * M_PI;
  return d;
}

Curvature SegmentUtils::calculateCurvature(const std::vector<DataPoint> &points,
                                           double critical_radius) {
  /* Takes the input array of gpx points, and returns a Curvature struct,
   * detailing the curvature of the segment
   * > kappa = instantaneous curvature differentials for signal analysis
   * > pct_twisty = the percent of the GPX trace that is considered beyond the
   * twistiness threshold.
   * > cbi = curvature burden index, how much the twistiness
   * of the road causes a burden on the rider, based on the input threshold
   * NOTE: The smaller the size of points, the more detailed the data, but
   * also the less wide scale it will affect.
   */
  Curvature result;
  size_t n = points.size();
  if (n < 3) {
    result.cbi = 0;
    result.pct_twisty = 0;
    return result;
  }
  // Convert each point to a distance from the first point.
  Coordinate ref = points.front().coord;
  std::vector<ENU> pts(n);
  for (size_t i = 0; i < n; ++i) {
    pts[i] = to_local(ref, points[i].coord);
  }

  result.kappa.resize(n, 0.0);
  double total_dist = 0;
  double twisty_dist = 0;
  double excess_integral = 0.0;

  // calcluate differential of each point: |dθ| / ds
  for (size_t i; i + 1 < n; ++i) {
    // angle differential
    double theta_1 = heading_ENU(pts[i - 1], pts[i]);
    double theta_2 = heading_ENU(pts[i], pts[i + 1]);
    double delta_theta =
        unwrap_angle(theta_2 - theta_1); // bi-directional reference point

    // distance differential
    double ds = dist(pts[i - 1], pts[i + 1]);
    if (ds > 1e-6) {

      double kappa_val = std::abs(delta_theta) / ds; // in rads/m
      result.kappa[i] = kappa_val;
      double partial_dist = dist(pts[i], pts[i + 1]);
      total_dist += partial_dist;

      if (kappa_val > critical_radius) {
        twisty_dist +=
            partial_dist; // if we exceed our threshold for twistyness (as
                          // decided by our average speed) we add the distance
                          // of the partial to a running counter.
        excess_integral += (kappa_val - critical_radius) * partial_dist;
      }
    }
  }
  if (total_dist > 0) {
    result.cbi = excess_integral / total_dist;
    result.pct_twisty = (twisty_dist / total_dist) * 100;
  } else {
    result.cbi = 0;
    result.pct_twisty = 0;
  }
  return result;
}

double SegmentUtils::calculateHeadingDegToRad(const Coordinate &point_from,
                                              const Coordinate &point_to) {
  /* Calculates the heading of two GPX points from the first to the second.
   * Takes the result in degrees, and converts to radians.
   */
  return atan2((point_to.lat - point_from.lat) * (M_PI / 180),
               (point_to.lon - point_from.lon) * (M_PI / 180));
}
double SegmentUtils::ang_diff(double a, double b) {
  const double d = b - a;
  return std::atan2(std::sin(d), std::cos(d)); // returns (−π, π]
}
