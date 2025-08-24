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
  for (size_t i = 0; i + 1 < n; ++i) {
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
static inline double safe_dt(double dt) {
  return (dt > 1e-3 && std::isfinite(dt)) ? dt : 1e-3;
}
// Small helper: median of a tiny vector (copy; window is small)
static double median_small(std::vector<double> v) {
  if (v.empty())
    return 0.0;
  size_t m = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + m, v.end());
  double med = v[m];
  if ((v.size() & 1) == 0) {
    // even → average two middles
    auto it = std::max_element(v.begin(), v.begin() + m);
    med = 0.5 * (med + *it);
  }
  return med;
}
static void ema_forward_backward(const std::vector<double> &t,
                                 const std::vector<double> &x,
                                 std::vector<double> &y, double tau_s) {
  const size_t n = x.size();
  if (n == 0)
    return;
  y.resize(n);
  y[0] = x[0];
  for (size_t i = 1; i < n; ++i) {
    const double dt = safe_dt(t[i] - t[i - 1]);
    const double alpha = 1.0 - std::exp(-dt / std::max(1e-6, tau_s));
    y[i] = y[i - 1] + alpha * (x[i] - y[i - 1]);
  }
  // backward pass to reduce lag (zero-phase)
  for (size_t i = n - 1; i-- > 0;) {
    const double dt = safe_dt(t[i + 1] - t[i]);
    const double alpha = 1.0 - std::exp(-dt / std::max(1e-6, tau_s));
    y[i] = y[i + 1] + alpha * (y[i] - y[i + 1]); // filter the filtered series
  }
}
// Enforce |dv| <= a_max * dt (both directions)
static void enforce_accel_cap(const std::vector<double> &t,
                              std::vector<double> &v, double a_max) {
  const size_t n = v.size();
  if (n == 0)
    return;
  // forward
  for (size_t i = 1; i < n; ++i) {
    const double dt = safe_dt(t[i] - t[i - 1]);
    const double vmax = v[i - 1] + a_max * dt;
    const double vmin = v[i - 1] - a_max * dt;
    v[i] = std::min(std::max(v[i], vmin), vmax);
  }
  // backward
  for (size_t i = n - 1; i-- > 0;) {
    const double dt = safe_dt(t[i + 1] - t[i]);
    const double vmax = v[i + 1] + a_max * dt;
    const double vmin = v[i + 1] - a_max * dt;
    v[i] = std::min(std::max(v[i], vmin), vmax);
  }
}

void SegmentUtils::compute_and_smooth_speed(std::vector<DataPoint> &pts,
                                            double tau_s, int med_win,
                                            double a_max) {
  const size_t n = pts.size();
  if (n == 0)
    return;

  // 1) RAW speed (m/s)
  pts[0].speed = 0.0;
  for (size_t i = 1; i < n; ++i) {
    const double dt =
        safe_dt(double(pts[i].time_rel) - double(pts[i - 1].time_rel));
    const double ds =
        SegmentUtils::haversine(pts[i - 1].coord.lon, pts[i].coord.lon,
                                pts[i - 1].coord.lat, pts[i].coord.lat);
    pts[i].speed = (std::isfinite(ds) ? ds : 0.0) / dt;
  }
  if (n >= 2)
    pts[0].speed = pts[1].speed;

  // 2) SMOOTH
  SegmentUtils::smooth_speed(pts, tau_s, med_win, a_max);
}

void SegmentUtils::smooth_speed(std::vector<DataPoint> &pts, double tau_s,
                                int med_win, double a_max) {
  const size_t n = pts.size();
  if (n == 0)
    return;

  // Collect time and a working copy of raw speeds
  std::vector<double> t(n), v(n);
  for (size_t i = 0; i < n; ++i) {
    t[i] = double(pts[i].time_rel);
    v[i] = pts[i].speed;
  }

  // 2a) Median pre-filter (odd window length; clamp to available)
  if (med_win < 1)
    med_win = 1;
  if ((med_win & 1) == 0)
    med_win += 1;
  const int half = med_win / 2;
  std::vector<double> v_med(n);
  for (int i = 0; i < (int)n; ++i) {
    const int a = std::max(0, i - half);
    const int b = std::min<int>(n - 1, i + half);
    std::vector<double> w;
    w.reserve(b - a + 1);
    for (int k = a; k <= b; ++k)
      w.push_back(v[k]);
    v_med[i] = median_small(std::move(w));
  }

  // 2b) Forward-backward EMA (adaptive by dt)
  std::vector<double> v_filt;
  ema_forward_backward(t, v_med, v_filt, tau_s);

  // 2c) Acceleration cap (physically plausible)
  enforce_accel_cap(t, v_filt, a_max);

  // 2d) Non-negativity + write back
  for (size_t i = 0; i < n; ++i) {
    pts[i].speed_smoothed = std::max(0.0, v_filt[i]);
  }
}

void SegmentUtils::normalize_heading_to_speed(std::vector<DataPoint> &pts,
                                              double v0,  // knee (~9 km/h)
                                              double vhi, // falloff (~20 km/h)
                                              double p, double q) {
  // NOTE:
  // For each sample, we need to calculate the normalized heading delta to
  // speed, as speed grows above 10kph, the normalization decreases, and really
  // falls off avove 20kph

  // X(v) * delta_theta would equal our weighted heading delta
  // X(v) = v^p / (v^p + v_l^p) * 1/(1+(v/v_h)^q)
  // v = smoothed speed at current point
  // v0 = 2.5m/s (10kph)
  // vhi = 5.5m/s (20kph)
  // p = 2 smoothing factor for low speed
  // q = 2 smoothing factor for high speed
  for (auto &pt : pts) {
    double v = pt.speed_smoothed;

    double low_bound = pow(v, p) / (pow(v, p) + pow(v0, p));
    double up_bound = 1 / (1 + pow(v / vhi, q));

    double Xv = low_bound * up_bound;
    pt.heading_delta_norm = pt.heading_delta * Xv;
  }
}
