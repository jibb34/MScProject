#include "core/SegmentUtils.hpp"
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

double SegmentUtils::calculateGradient(const std::vector<GpxPoint> &points) {
  GpxPoint start_point = points.at(0);
  GpxPoint end_point = points.at(points.size() - 1);
  double dist = SegmentUtils::haversine(start_point.lat, end_point.lat,
                                        start_point.lon, end_point.lon);
  return (end_point.elv - start_point.elv) / dist;
}

double
SegmentUtils::calculateGradientVariation(const std::vector<GpxPoint> &points) {
  std::vector<double> grades;
  // push first value so we only need to keep pushing 2nd value
  // loop until penultimate value
  for (int i = 0; i < points.size() - 1; i++) {
    GpxPoint point1 = points.at(i);
    GpxPoint point2 = points.at(i + 1);
    double gradient = (point2.elv - point1.elv) / haversine(point1, point2);
    grades.push_back(gradient);
  }
  // if no gradients found
  if (grades.empty()) {
    return 0.0;
  }
  // Calculate mean through (sum all values / size)
  double mean = std::reduce(grades.begin(), grades.end(), 0) / grades.size();
  int size = grades.size();
  auto variance_func = [&mean, &size](double accumulator, const double &val) {
    return accumulator + ((val - mean) * (val - mean) / (size - 1));
  };
  return std::accumulate(grades.begin(), grades.end(), 0.0, variance_func);
}

ENU to_local(const GpxPoint &ref, const GpxPoint &p) {
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

Curvature SegmentUtils::calculateCurvature(const std::vector<GpxPoint> &points,
                                           double critical_radius) {
  /* Takes the input array of gpx points, and returns a Curvature struct,
   * detailing the curvature of the segment
   * > kappa = instantaneous curvature differentials for signal analysis
   * > pct_twisty = the percent of the GPX trace that is considered beyond the
   * twistiness threshold.
   * > cbi = curvature burden index, how much the twistiness
   * of the road causes a burden on the rider, based on the input threshold
   * NOTE: The smaller the size of points, the more detailed the data, but also
   * the less wide scale it will affect.
   */
  Curvature result;
  size_t n = points.size();
  if (n < 3) {
    result.cbi = 0;
    result.pct_twisty = 0;
    return result;
  }
  // Convert each point to a distance from the first point.
  GpxPoint ref = points.front();
  std::vector<ENU> pts(n);
  for (size_t i = 0; i < n; ++i) {
    pts[i] = to_local(ref, points[i]);
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

double SegmentUtils::calculateHeadingDegToRad(const GpxPoint &point_from,
                                              const GpxPoint &point_to) {
  /* Calculates the heading of two GPX points from the first to the second.
   * Takes the result in degrees, and converts to radians.
   */
  return atan2((point_to.lat - point_from.lat) * (M_PI / 180),
               (point_to.lon - point_from.lon) * (M_PI / 180));
}
