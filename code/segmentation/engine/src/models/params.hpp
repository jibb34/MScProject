#pragma once

#include "json.hpp"
#include <string>

// User-supplied parameters controlling the segmentation algorithm.
struct SegmentationParams {
  double min_segment_length_m = 50.0;
  bool split_on_highway_change = true;
  int max_points = 100000;
  std::string job_id;

  static SegmentationParams from_json(const nlohmann::json &j) {
    SegmentationParams p;
    if (j.contains("min_segment_length_m"))
      p.min_segment_length_m = j.at("min_segment_length_m").get<double>();
    if (j.contains("split_on_highway_change"))
      p.split_on_highway_change = j.at("split_on_highway_change").get<bool>();
    if (j.contains("max_points"))
      p.max_points = j.at("max_points").get<int>();
    if (j.contains("job_id"))
      p.job_id = j.at("job_id").get<std::string>();
    return p;
  }
};
