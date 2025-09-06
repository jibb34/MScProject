#include "TrainingSuitability.hpp"
#include <unordered_map>
#include <numeric>
#include <algorithm>
#include <cmath>
#include <string>

using std::string;
using std::vector;
using std::unordered_map;

nlohmann::json extension_stats(const vector<DataPoint> &segment) {
  unordered_map<string, vector<double>> buckets;
  for (const auto &dp : segment) {
    if (!dp.extensions.is_object())
      continue;
    for (auto it = dp.extensions.begin(); it != dp.extensions.end(); ++it) {
      if (it->is_number()) {
        buckets[it.key()].push_back(it->get<double>());
      }
    }
  }

  nlohmann::json result = nlohmann::json::object();
  for (auto &kv : buckets) {
    const auto &vals = kv.second;
    if (vals.empty())
      continue;
    double sum = std::accumulate(vals.begin(), vals.end(), 0.0);
    double mean = sum / vals.size();
    double min = *std::min_element(vals.begin(), vals.end());
    double max = *std::max_element(vals.begin(), vals.end());
    double sq = 0.0;
    for (double v : vals) {
      double d = v - mean;
      sq += d * d;
    }
    double stddev = std::sqrt(sq / vals.size());
    result[kv.first] = {{"count", vals.size()},
                        {"mean", mean},
                        {"std", stddev},
                        {"min", min},
                        {"max", max}};
  }
  return result;
}

