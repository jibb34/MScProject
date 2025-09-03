#include "segmenter.hpp"

#include "core/ExistingSegmentMatcher.hpp"
#include "core/SegmentUtils.hpp"
#include "segmenter.hpp"
#include <algorithm>

std::vector<Coordinate>
SegmentEngine::buildPolyline(const RouteSignal &s) const {
  std::vector<Coordinate> out;
  out.reserve(s.points.size());
  for (auto &dp : s.points)
    out.push_back(dp.coord);
  return out;
}

std::vector<SegmentInstance>
SegmentEngine::processSegmentation(RouteSignal &signal, SegmentDB &db) const {
  std::vector<SegmentInstance> existing;
  if (signal.points.size() < 5)
    return existing;

  // 1) Existing-segment pass
  ExistingSegmentMatcher matcher(db);
  auto routeCoords = buildPolyline(signal);
  auto match = matcher.find(routeCoords,
                            /*bbox_pad_m=*/120.0,
                            /*max_mean_m=*/18.0); // tight bias
  existing = match.instances;

  // TODO: surface 'match.instances' to your HTTP/JSON layer so the UI can draw
  // them (green) e.g., stash into a member or return value (depends on your
  // current flow) For now we just “mask” their ranges for downstream
  // segmentation.

  // 2) Compute segmentation only on uncovered spans
  // build set of allowed index ranges
  std::vector<std::pair<int, int>> allowed;
  int N = static_cast<int>(signal.points.size());
  int cur = 0;
  for (auto r : match.masked_ranges) {
    if (cur < r.first)
      allowed.push_back({cur, r.first});
    cur = std::max(cur, r.second);
  }
  if (cur < N)
    allowed.push_back({cur, N});

  // Your existing change-point detector would run per allowed span.
  // Pseudocode:
  // for (auto span : allowed) {
  //   run_wavelet_or_rms(signal.points, span.first, span.second, ...);
  //   collect SegmentInstance(s) with kind = Flat/Uphill/...
  // }
  return existing;
}
