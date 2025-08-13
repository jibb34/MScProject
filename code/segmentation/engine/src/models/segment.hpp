#pragma once
#include "models/MergeCostManager.hpp" // for MergeCostManager
#include "models/OsrmResponse.hpp"     // for GpxPoint, Leg, Tracepoint, etc.
#include "models/SegStats.hpp"         // for SegStats
#include <cmath>
#include <cstdint>
#include <vector>

//------------------------------------------------------------------------------
// Segment class
//------------------------------------------------------------------------------
class Segment {
public:
  std::vector<std::pair<Leg, Tracepoint>> leg_trace;
  std::vector<int64_t> way_list; // List of OSM way IDs

  // Constructors
  Segment() = default;
  // Main constructor from a Leg and Tracepoint
  Segment(const Leg &leg, const Tracepoint &tp);

  // Merge cost: compute Δ cost for merging `other` into this segment
  double mergeCost(const Segment &other) const;

  // Perform merge if cost within threshold; returns true if merged
  bool mergeWith(const Segment &other, double threshold);

  // Accessors
  const SegStats &getStats() const noexcept { return stats; }

private:
  SegStats stats; // Segment statistics
  // Compute base cost of a segment
};

//------------------------------------------------------------------------------
// Implementation notes (to be moved to .cpp):
// - MergeCostManager::loadFromFile: parse JSON or key=val pairs into costs_.
// - MergeCostManager::getCost: lookup in costs_, throw if missing.
// - SegStats::initLeg: populate stats fields from leg data.
// - Segment::segmentCost: sum merge costs by retrieving attribute costs:
//     cost = MergeCostManager::getCost("grade_variance") * var_g + ...
//   where each term uses costs_ lookup instead of hard-coded weights.
// - mergeCost: builds combined SegStats M, then computes
//     Δ = segmentCost(M) - segmentCost(stats) - segmentCost(other.stats)
//       + boundaryPenalty(boundary_tier)
//   where boundaryPenalty uses MergeCostManager costs (e.g. "tier_med").
// - mergeWith: calls mergeCost, compares to editable threshold, and if OK
//   appends samples, coalesces way_runs, updates stats and tiers.
