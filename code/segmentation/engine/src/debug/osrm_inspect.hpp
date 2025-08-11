#pragma once
#include "json.hpp"
#include "models/osrm_enriched.hpp" // your structs

inline nlohmann::json summarize(const OsrmResponse &r,
                                std::size_t sample_n = 3) {
  using nlohmann::json;
  json out;
  out["code"] = r.code;

  const auto &m = r.matching; // you said “use first matching”
  out["matching"] = {{"legs", m.legs.size()},
                     {"geometry_points", m.geometry.coordinates.size()},
                     {"distance", m.distance},
                     {"duration", m.duration},
                     {"weight_name", m.weight_name}};

  json samples = json::array();
  const auto N = std::min(sample_n, m.geometry.coordinates.size());
  for (std::size_t i = 0; i < N; ++i) {
    const auto &c = m.geometry.coordinates[i];
    samples.push_back({{"lon", c[0]}, {"lat", c[1]}});
  }
  out["matching"]["sample_coords"] = samples;

  std::size_t unmatched = 0, with_gpx = 0, gpx_total = 0;
  nlohmann::json tp_samples = nlohmann::json::array();
  const auto tpN = std::min(sample_n, r.tracepoints.size());
  for (std::size_t i = 0; i < tpN; ++i) {
    const auto &tp = r.tracepoints[i];
    const bool matched =
        (tp.waypoint_index >= 0); // or tp.matched if you added it
    unmatched += matched ? 0 : 1;
    with_gpx += tp.gpx_list.empty() ? 0 : 1;
    gpx_total += tp.gpx_list.size();

    nlohmann::json tpj = {
        {"i", static_cast<int>(i)},
        {"matched", matched},
        {"waypoint_index", tp.waypoint_index},
        {"location", {{"lon", tp.location[0]}, {"lat", tp.location[1]}}},
        {"gpx_points", tp.gpx_list.size()}};
    if (!tp.gpx_list.empty()) {
      const auto &gp = tp.gpx_list.front();
      tpj["gpx_sample"] = {
          {"lat", gp.lat}, {"lon", gp.lon}, {"elv", gp.elv}, {"time", gp.time}};
    }
    tp_samples.push_back(tpj);
  }

  out["tracepoints"] = {{"count", r.tracepoints.size()},
                        {"unmatched", unmatched},
                        {"with_gpx", with_gpx},
                        {"gpx_total", gpx_total},
                        {"samples", tp_samples}};
  return out;
}
