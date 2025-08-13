#pragma once
#include <cmath>
#include <functional>
#include <string>

// All meaningful attributes for a segment
struct SegStats {
  // struct of all significant way attributes for a given leg
  // ================== Features useful for segmentation: ==================
  double leg_length = 0.0;
  double leg_weight = 0.0; // ratio of leg length to total segment length
  // Costs from MergeCostManager: handles heuristic cost analysis
  double highway_cost = 0.0;
  double surface_cost = 0.0;
  double smoothness_cost = 0.0;
  // Boolean checks
  bool has_cycleway = false;
  bool one_way = false;
  // =============== Features useful for classification (TSS) ===============
  double avg_power = 0.0;  // in watts
  double norm_power = 0.0; // Normalized power calculated by formula by Andrew
                           // Coggan. insert rest of the attributes here:
  double power_variance = 0.0;
  double cadence_variation = 0.0;

  // ================== Shared use features ==================
  double grade = 0.0;      // Average grade (slope) of the way
  double way_length = 0.0; // Length of the way in meters
  bool curvature = 0.0; // Sum of absolute bearing deltas over run's gpx points
  int max_speed = 0;    // Max speed in km/h
  bool is_mountain_pass = false; // mountain pass = summit of a mountain

  // ================== OSM Tags ==================
  // as segment grows, more ways/highways/surface may be added. We only care
  // about % contribution of the values, so a hash map would be a good fit for
  // this.
  std::hash<std::string> name;    // Way names
  std::hash<std::string> surface; // Surface type.
  //

  // First Pass variable checks for tier 1 candidates
  // if a segment has any of these, it will never merge.
  bool contains_junction = false;
  bool has_blocking_barrier = false;
  bool is_connected =
      true; // Segment is connected at both ends (within distance bounds)
  double heading_delta = 0.0; // Angle change between heading of first and last
                              // point in segment. Wrapped at 180 degrees.
                              // Only really useful in first pass
};
