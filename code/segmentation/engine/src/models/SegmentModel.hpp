#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <vector>

struct SegmentRun {
  long long way_id; // nullable in DB; here use 0 for "unknown"
  int from_idx;     // inclusive (global route index)
  int to_idx;       // exclusive
};
enum class SegmentKind : uint8_t {
  Existing = 0, // matched from DB (draw green)
  Flat,
  Uphill,
  Downhill,
  Rolling
};

struct SegmentDef {
  std::array<uint8_t, 32> uid; // SHA-256 bytes
  std::string uid_hex;         // hex for convenience
  std::string coords_json;
  int point_count = 0;
  double bbox_min_lat = 0, bbox_min_lon = 0, bbox_max_lat = 0, bbox_max_lon = 0;
  double length_m = 0.0;
  bool isSnapped = true;
};

struct SegmentInstance {
  // definition
  SegmentDef def;
  // instance mapping back to route
  long long route_id = 0; // supplied by caller if persisting
  int start_idx = 0;      // inclusive
  int end_idx = 0;        // exclusive
  // ordered list of way runs composing this span
  std::vector<SegmentRun> runs;
  SegmentKind kind = SegmentKind::Existing; // default for DB matches
};
