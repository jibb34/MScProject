#include "WaveletFootprint.hpp"
#include "models/CoreTypes.hpp"

#include "RollingEnergyFFT.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

std::vector<WaveletFootprintEngine::TerrainState>
WaveletFootprintEngine::get_states() const {
  return states_;
}
WaveletSignal WaveletFootprintEngine::make_wavelet_signal(
    const RouteSignal &rs, const GetterFn &getter, SeriesKind kind,
    double min_step_m) const {
  WaveletSignal out;
  out.reserve(rs.points.size());
  if (rs.points.empty())
    return out;

  double last_s = -1.0;
  bool have_prev = false;
  double prev_unwrapped = 0.0;

  for (const auto &dp : rs.points) {
    double s = dp.cum_dist;
    if (!std::isfinite(s))
      continue;

    // enforce non-decreasing s
    if (s < last_s)
      s = last_s;

    // down-sampling by distance spacing
    if (min_step_m > 0.0 && last_s >= 0.0 && (s - last_s) < min_step_m) {
      continue;
    }

    double y = getter(dp); // <- works for lambdas & function pointers now
    if (!std::isfinite(y)) {
      last_s = s;
      continue;
    }

    if (kind == SeriesKind::Angle) {
      // unwrap wrapped radians in (-pi,pi] to continuous series
      if (!have_prev) {
        prev_unwrapped = y;
        have_prev = true;
      } else {
        prev_unwrapped = unwrap_angle_step(prev_unwrapped, y);
      }
      y = prev_unwrapped;
    }

    out.push_back({s, y});
    last_s = s;
  }
  return out;
}

UniformSignal
WaveletFootprintEngine::resample_uniform_scalar(const WaveletSignal &sig,
                                                double ds) const {
  UniformSignal out;
  out.ds = ds;
  if (sig.empty() || !(ds > 0))
    return out;

  // 0) Sort by distance (stable, non-decreasing)
  WaveletSignal srt = sig;
  std::stable_sort(srt.begin(), srt.end(),
                   [](const WaveletPoint &a, const WaveletPoint &b) {
                     return a.s_m < b.s_m;
                   });

  // 1) Use MAX distance, not back()
  double s_end = 0.0;
  for (const auto &p : srt)
    if (std::isfinite(p.s_m))
      s_end = std::max(s_end, p.s_m);
  if (!(s_end > 0)) { // degenerate case
    out.s = {0.0};
    out.y = {srt.front().y};
    return out;
  }

  // 2) Grid (ceil to hit the end)
  const std::size_t m = static_cast<std::size_t>(std::ceil(s_end / ds)) + 1;
  out.s.resize(m);
  out.y.resize(m);
  for (std::size_t k = 0; k < m; ++k)
    out.s[k] = k * ds;

  // 3) Two-pointer sweep, guarded against zero-length segments
  std::size_t i = 0;
  for (std::size_t k = 0; k < m; ++k) {
    const double g = out.s[k];
    if (g <= srt.front().s_m) {
      out.y[k] = srt.front().y;
      continue;
    }
    if (g >= s_end) {
      out.y[k] = srt.back().y;
      continue;
    }

    while (i + 1 < srt.size() && srt[i + 1].s_m < g)
      ++i;

    std::size_t j = i + 1;
    while (j < srt.size() && !(srt[j].s_m > srt[i].s_m))
      ++j; // skip duplicates
    if (j == srt.size()) {
      out.y[k] = srt.back().y;
      continue;
    }

    const double s0 = srt[i].s_m, s1 = srt[j].s_m;
    const double y0 = srt[i].y, y1 = srt[j].y;
    const double seg = s1 - s0;
    const double t = (seg > 0) ? (g - s0) / seg : 0.0;
    out.y[k] = y0 + t * (y1 - y0);
  }
  return out;
}

static inline double wrapPi(double angle) {
  // Wraps to (-π, π]
  double t = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (t < 0) {
    t += 2.0 * M_PI;
  }
  return t - M_PI;
}

static inline double shortestArcDelta(double a, double b) {
  // delta from a->b on shortest arc
  return wrapPi(b - a);
}

UniformSignal
WaveletFootprintEngine::resample_uniform_angle(const WaveletSignal &sig,
                                               double ds) const {
  UniformSignal out;
  out.ds = ds;
  if (sig.empty() || ds <= 0)
    return out;

  // last distance signal
  const double s_end = sig.back().s_m;
  // what is most amount of times ds goes into the last distance value ()
  const std::size_t m = static_cast<size_t>(std::floor(s_end / ds)) + 1;
  // resize output signal to our new size
  out.s.resize(m);
  out.y.resize(m);
  for (size_t k = 0; k < m; ++k) {
    out.s[k] = k * ds; // create uniform increasing distances (cumulative)
  }
  size_t i = 0;
  for (size_t k = 0; k < m; ++k) {
    const double g = out.s[k]; // get our current distance

    while (i + 1 < sig.size() && sig[i + 1].s_m < g)
      ++i;

    if (g <= sig.front().s_m) {
      out.y[k] = wrapPi(sig.front().y);
      continue;
    }
    if (g >= sig.back().s_m) {
      out.y[k] = wrapPi(sig.back().y);
      continue;
    }

    std::size_t j = i + 1;
    while (j < sig.size() && sig[j].s_m <= sig[i].s_m)
      ++j;
    if (j == sig.size()) {
      out.y[k] = wrapPi(sig.back().y);
      continue;
    }

    const double s0 = sig[i].s_m, s1 = sig[j].s_m;
    const double y0 = wrapPi(sig[i].y), y1 = wrapPi(sig[j].y);
    const double seg = s1 - s0;
    const double t = seg > 0 ? (g - s0) / seg : 0.0;

    const double d = shortestArcDelta(y0, y1);
    out.y[k] = wrapPi(y0 + t * d);
  }
  return out;
}

UniformSignal
WaveletFootprintEngine::resample_uniform_categorical(const WaveletSignal &sig,
                                                     double ds) const {
  UniformSignal out;
  out.ds = ds;
  if (sig.empty() || ds <= 0)
    return out;

  const double s_end = sig.back().s_m;
  const std::size_t m = static_cast<std::size_t>(std::floor(s_end / ds)) + 1;

  out.s.resize(m);
  out.y.resize(m);
  for (std::size_t k = 0; k < m; ++k)
    out.s[k] = k * ds;

  std::size_t i = 0;
  for (std::size_t k = 0; k < m; ++k) {
    const double g = out.s[k];

    while (i + 1 < sig.size() && sig[i + 1].s_m <= g)
      ++i;

    if (g <= sig.front().s_m) {
      out.y[k] = sig.front().y;
      continue;
    }
    if (g >= sig.back().s_m) {
      out.y[k] = sig.back().y;
      continue;
    }

    out.y[k] = sig[i].y; // step/nearest-left
  }
  return out;
}

std::vector<NamedUniform> WaveletFootprintEngine::make_uniform_bundle(
    const RouteSignal &rs, double ds_m, std::initializer_list<NamedGetter> defs,
    double min_step_m) const {
  std::vector<NamedUniform> out;
  out.reserve(defs.size());

  for (const auto &def : defs) {
    auto w =
        this->make_wavelet_signal(rs, def.get, SeriesKind::Scalar, min_step_m);

    UniformSignal u;
    switch (def.kind) {
    case SeriesKind::Scalar:
      u = this->resample_uniform_scalar(w, ds_m);
      break;
    case SeriesKind::Angle:
      u = this->resample_uniform_angle(w, ds_m);
      break;
    case SeriesKind::Categorical:
      u = this->resample_uniform_categorical(w, ds_m);
      break;
    }

    out.push_back(NamedUniform{def.name, def.kind, std::move(u)});
  }
  return out;
}

//==================== Wavelet Terrain Functions =======================

// Clamp an integer to within [lo,hi]
static inline int clamp_int(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Calcuate moving average with an odd window length (>=3)
static std::vector<double> movavg(const std::vector<double> &y, int win) {

  const int n = (int)y.size();
  if (n == 0 || win <= 1)
    return y;
  if (win % 2 == 0)
    ++win; // force window length to be odd so we have a centre value
  const int h = win / 2; // amount of values on either side of middle

  // calulate sum of values in window
  std::vector<double> s(n + 1, 0.0);
  // starting at index 0 = 0, add y[i] and current s[i] to next s[i(+1)] (if
  // value is finite)
  for (int i = 0; i < n; ++i)
    s[i + 1] = s[i] + (std::isfinite(y[i]) ? y[i] : 0.0);
  std::vector<double> m(n, std::numeric_limits<double>::quiet_NaN());
  for (int i = 0; i < n; ++i) {
    int a = clamp_int(i - h, 0, n - 1);
    int b = clamp_int(i + h, 0, n - 1);
    int len = b - a + 1;
    // if length isn't zero calculate mean
    if (len > 0)
      m[i] = (s[b + 1] - s[a]) / (double)len;
  }
  return m;
}

// Fast prefix sum (for means in O(1) per query)
static std::vector<double> prefix_sum(const std::vector<double> &y) {
  std::vector<double> s(y.size() + 1, 0.0);
  for (size_t i = 0; i < y.size(); ++i)
    s[i + 1] = s[i] + (std::isfinite(y[i]) ? y[i] : 0.0);
  return s;
}
static inline double mean_range(const std::vector<double> &pref, int a, int b) {
  if (b < a)
    return 0.0;
  a = std::max(a, 0);
  b = std::min(b, (int)pref.size() - 2);
  const int len = b - a + 1;
  if (len <= 0)
    return 0.0;
  return (pref[b + 1] - pref[a]) / (double)len;
}
static std::vector<double> haar_trend_slope(const UniformSignal &U, int scale) {
  // return g[i] = (mean_right - mean_left)/(seff* U.ds) at each point
  // auto shrinks window at edges
  const int n = static_cast<int>(U.y.size());
  std::vector<double> g(n, 0.0);
  std::vector<double> ps = prefix_sum(U.y);

  auto range_sum = [&](int a, int b) -> double {
    // sum all from a->b-1, clamped to [0,n]
    a = std::clamp(a, 0, n);
    b = std::clamp(b, 0, n);
    if (b <= a)
      return 0.0;
    return ps[b] - ps[a];
  };

  // get available samples left and right of i
  for (int i = 0; i < n; ++i) {
    const int sL = std::min(scale, i);         // up to i-1 (inclusive)
    const int sR = std::min(scale, n - 1 - i); // from i+1 to i+sR (inclusive)
                                               //
    if (sL == 0 || sR == 0) {
      g[i] = 0; // not enough context at edges.
      continue;
    }
    const double meanL =
        range_sum(i - sL, i) / static_cast<double>(sL); // left half mean
    const double meanR = range_sum(i + 1, i + 1 + sR) /
                         static_cast<double>(sR); // right half mean
    // normalize by the effective half-width * the spacing
    const double denom = std::max(
        0.5 * (static_cast<double>(sL) + static_cast<double>(sR)) * U.ds,
        1e-12);

    g[i] = (meanR - meanL) / denom; // slope-like haar detail at position i.
  }
  return g;
}

double median(std::vector<double> v) {
  if (v.empty())
    return 0.0;
  const size_t mid = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + mid, v.end());
  double m = v[mid];
  if ((v.size() & 1) == 0) {
    std::nth_element(v.begin(), v.begin() + mid - 1, v.begin() + mid);
    m = 0.5 * (m + v[mid - 1]);
  }
  return m;
}

double mad(const std::vector<double> &a) {
  if (a.empty())
    return 0.0;
  std::vector<double> d;
  d.reserve(a.size());
  double med = median(std::vector<double>(a.begin(), a.end()));
  for (double x : a)
    d.push_back(std::abs(x - med));
  return median(std::move(d)); // unscaled MAD
};

void computeThresholds(double &tau_g, double &tau_E, double &tau_p,
                       const WaveletFootprintEngine::TerrainParams &p,
                       const std::vector<double> &g,
                       const std::vector<double> &E) {

  tau_g = p.k_g * mad(g);
  tau_E = p.k_E * mad(E);
  tau_p = p.tau_p;
}
static std::vector<double> movavg_boxcar(const std::vector<double> &x,
                                         int win) {
  win = std::max(1, win | 1); // force odd
  int h = win / 2;
  const size_t n = x.size();
  std::vector<double> y(n), c(n + 1, 0.0);
  for (size_t i = 0; i < n; i++)
    c[i + 1] = c[i] + x[i];
  for (size_t i = 0; i < n; i++) {
    int a = std::max<int>(0, (int)i - h);
    int b = std::min<int>(n - 1, (int)i + h);
    y[i] = (c[b + 1] - c[a]) / (b - a + 1);
  }
  return y;
}

// optional function to create plateaus from sections
void hysteresis(std::vector<double> &E, const UniformSignal &U,
                const WaveletFootprintEngine::TerrainParams &p) {
  auto med = median(E);
  double mad_val = mad(E);
  const double th_hi = med + p.E_hyst_hi * mad_val; // e.g. 2.0
  const double th_lo = med + p.E_hyst_lo * mad_val; // e.g. 1.0

  const int gap_close =
      std::max(0, (int)std::round(p.E_gap_close_m / U.ds)); // e.g. 30 m
  const int min_run =
      std::max(0, (int)std::round(p.E_min_run_m / U.ds)); // e.g. 80 m

  std::vector<uint8_t> mask(E.size(), 0);
  bool on = false;
  int start = 0, gap = 0;
  for (size_t i = 0; i < E.size(); ++i) {
    if (!on) {
      if (E[i] >= th_hi) {
        on = true;
        start = (int)i;
        gap = 0;
      }
    } else {
      if (E[i] >= th_lo) {
        gap = 0;
      } else {
        if (++gap > gap_close) {
          int end = (int)i - gap;
          if (end - start + 1 >= min_run)
            std::fill(mask.begin() + start, mask.begin() + end + 1, 1);
          on = false;
          gap = 0;
        }
      }
    }
  }
  if (on) {
    int end = (int)E.size() - 1;
    if (end - start + 1 >= min_run)
      std::fill(mask.begin() + start, mask.begin() + end + 1, 1);
  }

  // flatten to plateaus (use segment median)
  for (size_t i = 0; i < E.size();) {
    if (!mask[i]) {
      ++i;
      continue;
    }
    size_t j = i;
    while (j < E.size() && mask[j])
      ++j;
    size_t a = i, b = j - 1;
    std::vector<double> seg(E.begin() + a, E.begin() + b + 1);
    double mseg = median(seg);
    std::fill(E.begin() + a, E.begin() + b + 1, mseg);
    i = j;
  }
}
// ============= Main function =================

// NOTE: Terrain Main
UniformSignal WaveletFootprintEngine::terrain_states_from_elevation(
    const RouteSignal &rs, const TerrainParams &p,
    std::vector<double> &E_out) const {

  /* TODO:
   */
  // 1) Build Wavelet Signal from elevation
  const GetterFn get_elev = [](const DataPoint &d) { return d.coord.elv; };
  WaveletSignal w = this->make_wavelet_signal(rs, get_elev, SeriesKind::Scalar);

  // 2) Resample to uniform spacing
  double dx = (p.dx > 0 ? p.dx : 5.0);
  UniformSignal U =
      this->resample_uniform_scalar(w, dx); // Returns Distance series U[n]
  dx = U.ds;
  const int n = (int)U.y.size();
  if (n == 0)
    return U;

  /* Every point i in our signal U[n] should contain 2 separate features:
   * 1.) Trend at L_T (described above): g[i]
   * 2.) Rolling energy from smaller scales L_H: E[i]
   */

  // NOTE:
  // =========== Calculating g[n] L_T wavelet size ========
  const int s_T = std::max(3, (int)std::lround(p.L_T / std::max(U.ds, 1e-12)));
  auto g = haar_trend_slope(U, s_T);
  std::vector<double> g_abs;
  for (double g_i : g) {
    g_abs.push_back(std::abs(g_i)); // |g[i]|
  }
  // NOTE:
  //=================calculate E[n]=====================
  /*   * Mathmatically, we can define Energy of an oscillatory signal with
   * fourier transform. High frequency, high amplitude oscillations contribute
   * more to the "energy" of rolling terrain than lower freqency or amplitude.
   * Energy in this case being how difficult the terrain is. For oscillatory
   * energy of the scale T = s:
   * ------------------------------------------
   * E[i] = (8*N_s/3) * sum(abs(FFT{h[n] * U_detrended[n],k})^2) for k: B_Roll
   * -----------------------------------------------------------------------------
   * B_Roll = {K: f_k is an element within freqency band of "rolling terrain"}
   * f_k is a spatial frequency = k / N_s * dx
   * N_s = sample count in window W_i, 2*s + 1.
   * FFT = Fast fourier transform of U_detrended for k = [0, N_s - 1]
   * U_detrended = uses the overall W_i slope m = (U[i+s] - U[i-s]) / 2s + 1
   * === === === === === === === === === === === === === === === === === ===
   */

  std::vector<double> E(g.size(), 0.0);
  const int N_E = std::max(4, (int)std::lround(p.L_E / std::max(U.ds, 1e-12)));
  wf::RollingEnergyCalculator stft(N_E, U.ds);
  stft.compute_RMS(U, E);
  if (n >= 3) {
    // 1) curvature ≈ second derivative of elevation
    std::vector<double> k(n, 0.0);
    const double inv2ds = 1.0 / std::max(2.0 * U.ds, 1e-12);
    k[0] = (U.y[1] - 2 * U.y[0] + U.y[1]) * inv2ds; // one-sided safe
    for (int i = 1; i < n - 1; ++i)
      k[i] = (U.y[i + 1] - 2 * U.y[i] + U.y[i - 1]) * inv2ds;
    k[n - 1] = (U.y[n - 2] - 2 * U.y[n - 1] + U.y[n - 2]) * inv2ds;

    // 2) robust scale -> clipped z in [0,1]
    auto median_vec = [&](std::vector<double> v) {
      if (v.empty())
        return 0.0;
      size_t m = v.size() / 2;
      std::nth_element(v.begin(), v.begin() + m, v.end());
      double med = v[m];
      if (v.size() % 2 == 0) {
        std::nth_element(v.begin(), v.begin() + m - 1, v.end());
        med = 0.5 * (med + v[m - 1]);
      }
      return med;
    };
    std::vector<double> ak(n);
    for (int i = 0; i < n; ++i)
      ak[i] = std::abs(k[i]);
    const double med = median_vec(ak);
    for (int i = 0; i < n; ++i)
      ak[i] = std::abs(ak[i] - med);
    const double mad = std::max(1e-9, median_vec(ak));

    std::vector<double> z(n);
    for (int i = 0; i < n; ++i) {
      double v = (std::abs(k[i]) - med) / mad; // positive excursions only
      if (v < 0)
        v = 0;
      if (v > 3)
        v = 3;
      z[i] = v / 3.0; // 0..1
    }

    // 4) light smoothing for plateaus
  }
  // precompute for O(n) sliding windows
  const int w2 = std::max(1, (int)std::round(0.5 * p.L_E / U.ds));

  std::vector<double> dabs(n - 1, 0.0), ps(n, 0.0);
  for (int i = 0; i < n - 1; ++i)
    dabs[i] = std::abs(U.y[i + 1] - U.y[i]);
  for (int i = 0; i < n - 1; ++i)
    ps[i + 1] = ps[i] + dabs[i];

  auto gross_in = [&](int a, int b) { return ps[b] - ps[a]; }; // sum |dy|
  auto net_in = [&](int a, int b) { return std::abs(U.y[b] - U.y[a]); };

  for (int i = 0; i < n; ++i) {
    int a = std::max(0, i - w2);
    int b = std::min(n - 1, i + w2);
    double gross = std::max(1e-9, gross_in(a, b));         // avoid 0
    double r = std::clamp(net_in(a, b) / gross, 0.0, 1.0); // 0..1
    double scale = (1.0 - r); // monotonicity penalty
    E[i] *= scale * scale;    // smooth, no extra knobs
  }

  // --- after RollingEnergyFFT_FFTW().compute(U, E);
  const int w_env = std::max(1, (int)std::round(p.E_env_m / U.ds));

  // triangular smoothing = boxcar twice
  if (w_env > 1) {
    auto E1 = movavg_boxcar(E, w_env);
    E = movavg_boxcar(E1, w_env);
  }
  //

  // If toggle set, smooth with hysteresis
  if (p.E_use_hysteresis) {
    hysteresis(E, U, p);
  }

  E_out = E;
  // NOTE:============== Calculating thresholds to compare =============
  double tau_g, tau_E, tau_p;
  computeThresholds(tau_g, tau_E, tau_p, p, g, E);
  std::vector<double> rho; // rho[i] = E[i] / |g[i]| + ε
                           // ε = 1e-3 * median(|g|) + 1e-9
  double epsilon = 1e-3 * median(g_abs) + 1e-9;
  for (size_t i = 0; i < g.size(); ++i)
    rho.push_back(E[i] / (g_abs[i] + epsilon));

  //=======================================================
  // comparison:
  std::vector<WaveletFootprintEngine::TerrainState> terrain;
  for (int i = 0; i < U.y.size(); ++i) {
    // Is flat
    if (std::abs(g[i]) < tau_g && E[i] >= tau_E && rho[i] >= tau_p) {
      terrain.push_back(TerrainState::Rolling);
    } else if (g[i] >= tau_g && rho[i] < tau_p) {
      terrain.push_back(TerrainState::Uphill);
    } else if (g[i] <= -tau_g && rho[i] < tau_p) {
      terrain.push_back(TerrainState::Downhill);
    } else if (std::abs(g[i]) < tau_g && E[i] < tau_E) {
      terrain.push_back(TerrainState::Flat);
    } else {
      terrain.push_back(TerrainState::Unknown);
    }
  }
  double min_length = p.min_segment_length_m;
  MergeSide side = (p.merge_side_right) ? MergeSide::Right : MergeSide::Left;
  smoothSegments(terrain, U, min_length, side);

  states_ = terrain;

  std::vector<double> changepoint_dists =
      indices_to_distances(U, find_changepoint_indices(states_));
  // convert states changepoints to

  changepoints_ = mapChangepointsToRouteIndices(rs, changepoint_dists);
  UniformSignal X;
  X.s = U.s;
  X.y = std::move(g);

  X.ds = U.ds;
  return X;
}

std::vector<int> WaveletFootprintEngine::mapChangepointsToRouteIndices(
    const RouteSignal &rs, const std::vector<double> &changepoint_dists) const {

  std::vector<double> S;
  S.reserve(rs.points.size());
  for (const auto &p : rs.points)
    S.push_back(p.cum_dist);

  std::vector<int> out;
  out.reserve(changepoint_dists.size());
  for (double d : changepoint_dists) {
    auto it = std::lower_bound(S.begin(), S.end(), d);

    if (it == S.begin()) {
      out.push_back(0);
    } else if (it == S.end()) {
      out.push_back(static_cast<int>(S.size() - 1));
    } else {
      size_t hi = static_cast<size_t>(it - S.begin());
      size_t lo = hi - 1;
      // push closest of the two points
      out.push_back((d - S[lo] <= S[hi] - d) ? static_cast<int>(lo)
                                             : static_cast<int>(hi));
    }
  }
  return out;
}
std::vector<int> WaveletFootprintEngine::find_changepoint_indices(
    const std::vector<WaveletFootprintEngine::TerrainState> &st) {
  std::vector<int> idx;
  if (st.empty())
    return idx;
  for (int i = 1, n = (int)st.size(); i < n; ++i) {
    if (st[i] != st[i - 1]) {
      idx.push_back(i);
    }
  }
  return idx;
}

std::vector<double>
WaveletFootprintEngine::indices_to_distances(const UniformSignal &U,
                                             const std::vector<int> &idx) {
  std::vector<double> out;
  out.reserve(idx.size());
  const bool have_s = (U.s.size() == U.y.size() && !U.s.empty());
  for (int i : idx)
    out.push_back(have_s ? U.s[i] : i * std::max(U.ds, 0.0));
  return out;
}

void WaveletFootprintEngine::smoothSegments(
    std::vector<WaveletFootprintEngine::TerrainState> &states,
    const UniformSignal &U, double min_len_m, MergeSide side) const {
  if (states.empty())
    return;
  const size_t n = states.size();

  // 1) Build runs from per-sample states
  struct Run {
    int i0, i1;
    WaveletFootprintEngine::TerrainState t;
  };
  std::vector<Run> runs;
  runs.reserve(n);
  for (size_t i = 0; i < n;) {
    size_t j = i + 1;
    while (j < n && states[j] == states[i])
      ++j;
    runs.push_back({(int)i, (int)j - 1, states[i]});
    i = j;
  }

  // 2) Per-run lengths (use U.s if available, else counts*ds)
  auto run_len = [&](const Run &r) -> double {
    if (!U.s.empty() && U.s.size() == n)
      return std::max(0.0, U.s[r.i1] - U.s[r.i0]);
    return std::max(0, r.i1 - r.i0) * std::max(U.ds, 0.0);
  };
  std::vector<double> lens(runs.size());
  for (size_t r = 0; r < runs.size(); ++r)
    lens[r] = run_len(runs[r]);

  // 3) Merge loop on RUN arrays (sizes stay in sync)
  for (size_t r = 0; r < runs.size();) {
    if (lens[r] >= min_len_m) {
      ++r;
      continue;
    }
    const bool hasL = r > 0, hasR = r + 1 < runs.size();

    if (hasL && hasR && runs[r - 1].t == runs[r + 1].t) {
      runs[r - 1].i1 = runs[r + 1].i1;
      lens[r - 1] += lens[r] + lens[r + 1];
      runs.erase(runs.begin() + r, runs.begin() + r + 2);
      lens.erase(lens.begin() + r, lens.begin() + r + 2);
      if (r > 0)
        --r;
    } else if (hasL && (side == MergeSide::Left || !hasR)) {
      runs[r - 1].i1 = runs[r].i1;
      lens[r - 1] += lens[r];
      runs.erase(runs.begin() + r);
      lens.erase(lens.begin() + r);
      if (r > 0)
        --r;
    } else if (hasR) {
      runs[r + 1].i0 = runs[r].i0;
      lens[r + 1] += lens[r];
      runs.erase(runs.begin() + r);
      lens.erase(lens.begin() + r);
      // r stays at same index
    } else {
      ++r;
    }
  }

  // 4) Expand back to per-sample states
  for (const auto &run : runs)
    std::fill(states.begin() + run.i0, states.begin() + run.i1 + 1, run.t);
}
