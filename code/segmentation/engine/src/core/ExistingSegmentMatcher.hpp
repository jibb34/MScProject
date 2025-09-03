#pragma once
#include "core/SegmentDB.hpp"
#include "core/SegmentUtils.hpp"
#include "models/CoreTypes.hpp"
#include "models/SegmentModel.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <nlohmann/json.hpp>
#include <vector>

struct ExistingMatchResult {
  std::vector<SegmentInstance> instances;
  std::vector<std::pair<int, int>> masked_ranges; // [start,end)
};

class ExistingSegmentMatcher {
public:
  explicit ExistingSegmentMatcher(SegmentDB &db) : db_(db) {}

  // route: full RouteSignal points
  ExistingMatchResult find(const std::vector<Coordinate> &route_coords,
                           double bbox_pad_m = 100.0,
                           double max_mean_m = 20.0) {
    ExistingMatchResult out;

    if (route_coords.size() < 5)
      return out;

    // Local bbox helpers to avoid static/non-static symbol ambiguity
    auto compute_bbox_local = [](const std::vector<Coordinate> &pts) {
      struct {
        double min_lat, min_lon, max_lat, max_lon;
      } b{+90, +180, -90, -180};
      for (auto &c : pts) {
        b.min_lat = std::min(b.min_lat, c.lat);
        b.max_lat = std::max(b.max_lat, c.lat);
        b.min_lon = std::min(b.min_lon, c.lon);
        b.max_lon = std::max(b.max_lon, c.lon);
      }
      return b;
    };
    auto inflate_bbox_local = [](auto b, double pad_m) {
      constexpr double kPi = 3.14159265358979323846;
      const double lat0 = ((b.min_lat + b.max_lat) * 0.5) * kPi / 180.0;
      const double m_per_deg_lat = 111132.954;
      const double m_per_deg_lon = 111132.954 * std::cos(lat0);
      const double dlat = pad_m / m_per_deg_lat;
      const double dlon = pad_m / m_per_deg_lon;
      return decltype(b){b.min_lat - dlat, b.min_lon - dlon, b.max_lat + dlat,
                         b.max_lon + dlon};
    };
    auto rb = compute_bbox_local(route_coords);
    auto rbp = inflate_bbox_local(rb, bbox_pad_m);
    auto cands = db_.query_defs_in_bbox(rbp.min_lat, rbp.min_lon, rbp.max_lat,
                                        rbp.max_lon);
    // local helpers for distance
    auto hav = [](const Coordinate &a, const Coordinate &b) {
      // mirror of SegmentUtils::haversine without symbol issues
      auto deg2rad = [](double d) {
        return d * 3.14159265358979323846 / 180.0;
      };
      const double phi1 = deg2rad(a.lat), phi2 = deg2rad(b.lat);
      const double dphi = deg2rad(b.lat - a.lat);
      const double dlmb = deg2rad(b.lon - a.lon);
      const double h =
          std::pow(std::sin(dphi / 2), 2) +
          std::cos(phi1) * std::cos(phi2) * std::pow(std::sin(dlmb / 2), 2);
      return 2.0 * 6371000.0 * std::asin(std::sqrt(h));
    };
    auto avg_directed_distance_m_local = [&](const std::vector<Coordinate> &A,
                                             const std::vector<Coordinate> &B) {
      if (A.empty() || B.empty())
        return std::numeric_limits<double>::infinity();
      double acc = 0.0;
      for (const auto &a : A) {
        double best = std::numeric_limits<double>::infinity();
        for (const auto &b : B)
          best = std::min(best, hav(a, b));
        acc += best;
      }
      return acc / static_cast<double>(A.size());
    };
    auto best_window_alignment_local = [&](const std::vector<Coordinate> &route,
                                           const std::vector<Coordinate> &seg,
                                           double thr_m, int min_pts) {
      if (route.size() < static_cast<size_t>(min_pts) || seg.size() < 2) {
        return std::tuple<int, int, double>{
            -1, -1, std::numeric_limits<double>::infinity()};
      }
      const int W = static_cast<int>(seg.size());
      const int N = static_cast<int>(route.size());
      int best_i = -1;
      double best_d = std::numeric_limits<double>::infinity();
      for (int i = 0; i + W <= N; ++i) {
        std::vector<Coordinate> sub(route.begin() + i, route.begin() + i + W);
        double d = avg_directed_distance_m_local(sub, seg);
        if (d < best_d) {
          best_d = d;
          best_i = i;
        }
      }
      if (best_i >= 0 && best_d <= thr_m) {
        return std::tuple<int, int, double>{best_i, best_i + W, best_d};
      }
      return std::tuple<int, int, double>{-1, -1, best_d};
    };

    for (const auto &def : cands) {
      // local JSON parser to avoid any SegmentUtils symbol ambiguity
      auto parse_coords_json_local = [](const std::string &s) {
        std::vector<Coordinate> out;
        if (s.empty())
          return out;
        nlohmann::json arr = nlohmann::json::parse(s, /*cb=*/nullptr,
                                                   /*allow_exceptions=*/false);
        if (!arr.is_array())
          return out;
        out.reserve(arr.size());
        for (auto &v : arr) {
          if (!v.is_array() || v.size() < 2)
            continue;
          // Expect [lon, lat]
          double lon = v[0].get<double>();
          double lat = v[1].get<double>();
          out.push_back(Coordinate{lat, lon, 0.0});
        }
        return out;
      };
      auto segCoords = parse_coords_json_local(def.coords_json);
      if (segCoords.size() < 5)
        continue;

      auto [sidx, eidx, mean_d] =
          best_window_alignment_local(route_coords, segCoords, max_mean_m, 30);
      if (sidx >= 0) {
        SegmentInstance inst;
        inst.def = def;
        inst.start_idx = sidx;
        inst.end_idx = eidx;
        inst.kind = SegmentKind::Existing;
        out.instances.push_back(inst);
        out.masked_ranges.push_back({inst.start_idx, inst.end_idx});
      }
    }

    // merge overlapping masked ranges
    std::sort(out.masked_ranges.begin(), out.masked_ranges.end());
    std::vector<std::pair<int, int>> merged;
    for (auto r : out.masked_ranges) {
      if (merged.empty() || r.first > merged.back().second)
        merged.push_back(r);
      else
        merged.back().second = std::max(merged.back().second, r.second);
    }
    out.masked_ranges.swap(merged);

    return out;
  }

private:
  SegmentDB &db_;
};
