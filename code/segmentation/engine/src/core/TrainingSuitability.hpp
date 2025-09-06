#pragma once
#include "models/CoreTypes.hpp"
#include <nlohmann/json.hpp>
#include <vector>

// Compute basic statistics for each numeric extension key present in a segment.
// The result is a JSON object mapping extension names to their statistics
// (count, mean, std, min, max).
nlohmann::json extension_stats(const std::vector<DataPoint> &segment);

