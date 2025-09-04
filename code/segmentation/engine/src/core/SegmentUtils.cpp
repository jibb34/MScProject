#include "core/SegmentUtils.hpp"
#include "infra/MySQLSegmentDB.hpp"
#include "models/OsrmResponse.hpp"
#include "models/SegmentModel.hpp"
#include <iomanip>
#include <math.h>
#include <nlohmann/json.hpp>
#include <numeric>
#include <openssl/sha.h>
#include <sstream>
#include <vector>
using json = nlohmann::json;

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

static std::string to_hex(const uint8_t *p, size_t n) {
  std::ostringstream oss;
  oss << std::hex << std::setfill('0');
  for (size_t i = 0; i < n; ++i)
    oss << std::setw(2) << (int)p[i];
  return oss.str();
}

static inline bool near_eq(double a, double b, double eps = 1e-7) {
  return std::abs(a - b) <= eps;
}

std::vector<std::pair<int, int>>
SegmentUtils::make_segments_from_change_points(std::vector<int> cp, int N) {
  if (cp.empty() || cp.front() != 0)
    cp.insert(cp.begin(), 0);
  if (cp.back() != N)
    cp.push_back(N);
  std::sort(cp.begin(), cp.end());
  cp.erase(std::unique(cp.begin(), cp.end()), cp.end());
  std::vector<std::pair<int, int>> segs;
  for (size_t i = 0; i + 1 < cp.size(); ++i) {
    const int a = std::max(0, std::min(N, cp[i]));
    const int b = std::max(0, std::min(N, cp[i + 1]));
    if (a < b)
      segs.emplace_back(a, b);
  }
  return segs;
}

std::vector<SegmentRun>
SegmentUtils::way_runs_in_slice(const std::vector<long long> &way_ids, int a,
                                int b) {
  std::vector<SegmentRun> runs;
  if (a >= b)
    return runs;
  int i = a;
  while (i < b) {
    long long w = (i < (int)way_ids.size() ? way_ids[i] : 0LL);
    int j = i + 1;
    while (j < b) {
      long long wj = (j < (int)way_ids.size() ? way_ids[j] : 0LL);
      if (wj != w)
        break;
      ++j;
    }
    runs.push_back(SegmentRun{w, i, j});
    i = j;
  }
  return runs;
}

SegmentDef
SegmentUtils::build_segment_def_forward(const std::vector<DataPoint> &pts,
                                        int a, int b) {
  // 1) normalize geometry in forward order
  std::vector<Coordinate> norm;
  norm.reserve(std::max(0, b - a));
  auto push_norm = [&](double lat, double lon) {
    double rlat = std::round(lat * 1e6) / 1e6;
    double rlon = std::round(lon * 1e6) / 1e6;
    if (norm.empty() || !near_eq(norm.back().lat, rlat) ||
        !near_eq(norm.back().lon, rlon))
      norm.push_back({rlat, rlon, 0.0});
  };
  for (int i = a; i < b; ++i)
    push_norm(pts[i].coord.lat, pts[i].coord.lon);

  // guard: at least 2 points for length; still fingerprint singletons
  double minLat = +1e9, minLon = +1e9, maxLat = -1e9, maxLon = -1e9, len = 0.0;
  for (size_t i = 0; i < norm.size(); ++i) {
    minLat = std::min(minLat, norm[i].lat);
    minLon = std::min(minLon, norm[i].lon);
    maxLat = std::max(maxLat, norm[i].lat);
    maxLon = std::max(maxLon, norm[i].lon);
    if (i)
      len += SegmentUtils::haversine(norm[i - 1], norm[i]); // meters
  }

  // 2) materialize fingerprint payload: "v1|EPSG:4326|F|lat,lon;lat,lon;..."
  std::string csv;
  csv.reserve(norm.size() * 24);
  for (size_t i = 0; i < norm.size(); ++i) {
    if (i)
      csv.push_back(';');
    csv += std::to_string(norm[i].lat);
    csv.push_back(',');
    csv += std::to_string(norm[i].lon);
  }
  std::string material = "v1|EPSG:4326|F|" + csv;

  // 3) SHA-256
  std::array<uint8_t, 32> uid{};
  SHA256(reinterpret_cast<const unsigned char *>(material.data()),
         material.size(), uid.data());

  // 4) encode normalized points (use your existing encoder if present)
  // Here we fallback to a trivial "csv" — replace with your polyline encoder
  std::string enc = csv;
  // build a nlohmann::json array first…
  nlohmann::json tmp = nlohmann::json::array();
  for (auto &c : norm) {
    tmp.push_back({{"lat", c.lat}, {"lon", c.lon}});
  }
  // then serialize into your string field

  SegmentDef def;
  def.uid = uid;
  def.uid_hex = to_hex(uid.data(), uid.size());
  def.coords_json = tmp.dump();
  def.point_count = (int)norm.size();
  def.bbox_min_lat = minLat;
  def.bbox_min_lon = minLon;
  def.bbox_max_lat = maxLat;
  def.bbox_max_lon = maxLon;
  def.length_m = len;
  return def;
}
SegmentUtils::BBox
SegmentUtils::compute_bbox(const std::vector<Coordinate> &pts) {
  SegmentUtils::BBox b{+90, +180, -90, -180};
  for (auto &c : pts) {
    b.min_lat = std::min(b.min_lat, c.lat);
    b.max_lat = std::max(b.max_lat, c.lat);
    b.min_lon = std::min(b.min_lon, c.lon);
    b.max_lon = std::max(b.max_lon, c.lon);
  }
  return b;
}

static inline double approx_m_per_deg_lat() { return 111132.954; }
static inline double approx_m_per_deg_lon(double lat_rad) {
  return 111132.954 * std::cos(lat_rad);
}

SegmentUtils::BBox SegmentUtils::inflate_bbox(const SegmentUtils::BBox &b,
                                              double pad_m) {
  const double lat0 = ((b.min_lat + b.max_lat) * 0.5) * M_PI / 180.0;
  const double dlat = pad_m / approx_m_per_deg_lat();
  const double dlon = pad_m / approx_m_per_deg_lon(lat0);
  return {b.min_lat - dlat, b.min_lon - dlon, b.max_lat + dlat,
          b.max_lon + dlon};
}

std::vector<Coordinate>
SegmentUtils::parse_coords_json(const std::string &coords_json) {
  std::vector<Coordinate> out;
  if (coords_json.empty())
    return out;
  auto arr = json::parse(coords_json);
  out.reserve(arr.size());
  for (auto &v : arr) {
    // expect [lon, lat] (OSRM/GeoJSON)
    double lon = v[0].get<double>();
    double lat = v[1].get<double>();
    out.push_back({lat, lon, 0.0});
  }
  return out;
}

double SegmentUtils::avg_directed_distance_m(const std::vector<Coordinate> &A,
                                             const std::vector<Coordinate> &B) {
  if (A.empty() || B.empty())
    return std::numeric_limits<double>::infinity();
  double acc = 0.0;
  for (auto &a : A) {
    double best = std::numeric_limits<double>::infinity();
    for (auto &b : B)
      best = std::min(best, SegmentUtils::haversine(a, b));
    acc += best;
  }
  return acc / static_cast<double>(A.size());
}

double SegmentUtils::symmetric_distance_m(const std::vector<Coordinate> &A,
                                          const std::vector<Coordinate> &B) {
  double d1 = SegmentUtils::avg_directed_distance_m(A, B);
  double d2 = SegmentUtils::avg_directed_distance_m(B, A);
  return std::max(d1, d2);
}

SegmentUtils::Alignment
SegmentUtils::best_window_alignment(const std::vector<Coordinate> &route,
                                    const std::vector<Coordinate> &seg,
                                    double thr_m, int min_pts) {
  if (route.size() < static_cast<size_t>(min_pts) || seg.size() < 2) {
    return {-1, -1, std::numeric_limits<double>::infinity()};
  }
  // sliding window over route with same length (by point count) as seg,
  // using average directed distance route_window -> seg
  const int W = static_cast<int>(seg.size());
  const int N = static_cast<int>(route.size());
  int best_i = -1;
  double best_d = std::numeric_limits<double>::infinity();
  for (int i = 0; i + W <= N; ++i) {
    std::vector<Coordinate> sub(route.begin() + i, route.begin() + i + W);
    double d = SegmentUtils::avg_directed_distance_m(sub, seg);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }
  if (best_i >= 0 && best_d <= thr_m) {
    return {best_i, best_i + W, best_d};
  }
  return {-1, -1, best_d};
}

// Replace spans with DB-matched segments
void SegmentUtils::overlay_db_segments(
    std::vector<SegmentInstance> &segs,
    const std::vector<Coordinate> &route_coords,
    const std::vector<long long> &way_ids) {
  // 1. Compute padded bbox
  auto bbox = SegmentUtils::compute_bbox(route_coords);
  auto padded = SegmentUtils::inflate_bbox(bbox, 50.0);

  // 2. Connect DB
  MySQLSegmentDB db("tcp://127.0.0.1:3306", "routeseg_user", "changeme-user",
                    "routeseg");

  auto candidates = db.query_defs_in_bbox(padded.min_lat, padded.min_lon,
                                          padded.max_lat, padded.max_lon);

  for (auto &def : candidates) {
    // 3. Parse coords
    std::vector<Coordinate> seg_coords;
    auto arr = nlohmann::json::parse(def.coords_json);
    for (auto &el : arr) {
      seg_coords.push_back(
          {el[0].template get<double>(), el[1].template get<double>(), 0.0});
    }

    // 4. Align
    auto al =
        SegmentUtils::best_window_alignment(route_coords, seg_coords, 50.0, 30);
    if (al.start_idx < 0)
      continue;

    // 5. Wipe out overlapping spans in segs
    segs.erase(std::remove_if(segs.begin(), segs.end(),
                              [&](const SegmentInstance &si) {
                                return si.start_idx >= al.start_idx &&
                                       si.end_idx <= al.end_idx;
                              }),
               segs.end());

    // 6. Insert DB instance
    SegmentInstance si;
    si.def = def;
    si.start_idx = al.start_idx;
    si.end_idx = al.end_idx;
    si.runs =
        SegmentUtils::way_runs_in_slice(way_ids, si.start_idx, si.end_idx);
    si.kind = SegmentKind::Existing;
    segs.push_back(std::move(si));
  }
}
