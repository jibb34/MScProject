#pragma once
#include "models/CoreTypes.hpp"
#include <cstddef>
#include <unordered_map>
#include <vector>
#include <string>

// Basic statistics derived from numeric extension values.
struct ExtensionStats {
  std::size_t count{0};
  double mean{0.0};
  double std{0.0};
  double min{0.0};
  double max{0.0};
};

// Training suitability score for a segment, keyed by extension name.
struct TrainingSuitabilityScore {
  std::unordered_map<std::string, ExtensionStats> extensions;
};

// Compute a TrainingSuitabilityScore for the provided segment's extension values.
TrainingSuitabilityScore compute_tss(const std::vector<DataPoint> &segment);

