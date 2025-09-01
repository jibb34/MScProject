#pragma once
#include "models/OsrmResponse.hpp"

class RouteSignalBuilder {
public:
  RouteSignalBuilder() = default;
  RouteSignal build(const OsrmResponse &osrm);

private:
  void assignWayIDs(const std::vector<Leg> &legs,
                    const std::vector<Tracepoint> &tps, RouteSignal &rs);
  double get_start_time(const OsrmResponse &osrm) {
    if (!osrm.tracepoints.empty()) {
      return osrm.tracepoints.front().gpx_list.front().time;
    }
    return 0.0;
  };
  // std::vector<HighwayType> extract_highways(const Leg &l);
};
