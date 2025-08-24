#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

// Your project headers:
#include "core/RouteSignalBuilder.hpp" // RouteSignal, DataPoint
#include "core/wavelets/WaveletFootprint.hpp" // WaveletFootprintEngine, WaveletSignal, UniformSignal

// ------- Small helpers -------
inline double deg(double r) { return r * 180.0 / M_PI; }
inline const char *kind_str(SeriesKind k) {
  switch (k) {
  case SeriesKind::Scalar:
    return "scalar";
  case SeriesKind::Angle:
    return "angle";
  case SeriesKind::Categorical:
    return "categorical";
  }
  return "?";
}

inline void print_uniform_stats(const NamedUniform &nu) {
  const auto &s = nu.sig.s;
  const auto &y = nu.sig.y;
  std::size_t n = y.size();

  double ymin = std::numeric_limits<double>::infinity();
  double ymax = -std::numeric_limits<double>::infinity();
  double sum = 0.0;
  std::size_t nfinite = 0, nnan = 0, ninf = 0;

  for (double v : y) {
    if (std::isnan(v)) {
      nnan++;
      continue;
    }
    if (!std::isfinite(v)) {
      ninf++;
      continue;
    }
    ymin = std::min(ymin, v);
    ymax = std::max(ymax, v);
    sum += v;
    nfinite++;
  }
  double mean =
      nfinite ? (sum / nfinite) : std::numeric_limits<double>::quiet_NaN();

  std::cout << "  [" << nu.name << "] kind=" << kind_str(nu.kind)
            << "  points=" << n << "  ds=" << nu.sig.ds << " m"
            << "  s_end=" << (s.empty() ? 0.0 : s.back()) << " m\n"
            << "      y{min=" << ymin << ", max=" << ymax << ", mean=" << mean
            << ", nan=" << nnan << ", inf=" << ninf << "}\n";

  auto sample = [&](std::size_t k) {
    if (k < n) {
      std::cout << "      sample@" << k << "  s=" << (s[k] / 1000.0)
                << " km, y=" << y[k] << "\n";
    }
  };
  sample(0);
  sample(std::min<std::size_t>(5, n ? n - 1 : 0));
  sample(n ? n - 1 : 0);
}

// Compute median step (m) from RouteSignal.cum_dist, clamped [1..10]
inline double choose_ds_from_route(const RouteSignal &rs) {
  if (rs.points.size() < 3)
    return 5.0;
  std::vector<double> steps;
  steps.reserve(rs.points.size());
  for (std::size_t i = 1; i < rs.points.size(); ++i) {
    double d = rs.points[i].cum_dist - rs.points[i - 1].cum_dist;
    if (std::isfinite(d) && d > 0)
      steps.push_back(d);
  }
  if (steps.empty())
    return 5.0;
  std::nth_element(steps.begin(), steps.begin() + steps.size() / 2,
                   steps.end());
  double med = steps[steps.size() / 2];
  return std::max(1.0, std::min(10.0, med));
}

// Build an explicit list of getters by short name
inline std::map<std::string, NamedGetter> default_getters() {
  return {
      {"elev",
       {"elev", SeriesKind::Scalar,
        [](const DataPoint &d) { return d.coord.elv; }}},
      {"speed",
       {"speed", SeriesKind::Scalar,
        [](const DataPoint &d) { return d.speed_smoothed; }}},
      {"grad",
       {"grad", SeriesKind::Scalar,
        [](const DataPoint &d) { return d.gradient; }}},
      {"head",
       {"head", SeriesKind::Angle,
        [](const DataPoint &d) { return d.heading_delta_norm; }}},
      {"way",
       {"way", SeriesKind::Categorical,
        [](const DataPoint &d) { return static_cast<double>(d.way_id); }}},
  };
}

// Expand a comma list like "speed,elev,head" into a vector of getters.
// "all" returns everything in map order.
inline std::vector<NamedGetter>
select_getters(const std::string &csv,
               const std::map<std::string, NamedGetter> &universe) {
  if (csv == "all") {
    std::vector<NamedGetter> v;
    v.reserve(universe.size());
    for (auto &kv : universe)
      v.push_back(kv.second);
    return v;
  }
  std::vector<NamedGetter> out;
  std::string cur;
  cur.reserve(32);
  auto flush = [&]() {
    if (cur.empty())
      return;
    auto it = universe.find(cur);
    if (it != universe.end())
      out.push_back(it->second);
    else
      std::cerr << "[warn] unknown signal name: '" << cur << "' (skipped)\n";
    cur.clear();
  };
  for (char c : csv) {
    if (c == ',' || c == ' ')
      flush();
    else
      cur.push_back(c);
  }
  flush();
  return out;
}

// Compute a bundle of uniform series with appropriate resampler per kind.
inline std::vector<NamedUniform>
compute_uniform_bundle(const RouteSignal &rs, WaveletFootprintEngine &eng,
                       double ds_m, const std::vector<NamedGetter> &defs,
                       double min_step_m = 0.0) {
  std::vector<NamedUniform> out;
  out.reserve(defs.size());
  for (const auto &def : defs) {
    auto w =
        eng.make_wavelet_signal(rs, def.get, SeriesKind::Scalar, min_step_m);
    UniformSignal u;
    switch (def.kind) {
    case SeriesKind::Scalar:
      u = eng.resample_uniform_scalar(w, ds_m);
      break;
    case SeriesKind::Angle:
      u = eng.resample_uniform_angle(w, ds_m);
      break;
    case SeriesKind::Categorical:
      u = eng.resample_uniform_categorical(w, ds_m);
      break;
    }
    out.push_back(NamedUniform{def.name, def.kind, std::move(u)});
  }
  return out;
}
