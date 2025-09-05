#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

// Run of consecutive OSM way IDs that compose a segment.
struct SegmentRun {
  long long way_id; // nullable in DB; 0 denotes "unknown"
  int from_idx;     // inclusive (global route index)
  int to_idx;       // exclusive
};

// High level classification for a segment instance.
enum class SegmentKind : uint8_t {
  Unknown = 0, // matched from DB (draw green)
  Flat,
  Uphill,
  Downhill,
  Rolling
};

inline const char *SegmentTypeToString(SegmentKind type) {
  switch (type) {
  case SegmentKind::Rolling:
    return "rolling";
  case SegmentKind::Flat:
    return "flat";
  case SegmentKind::Uphill:
    return "uphill";
  case SegmentKind::Downhill:
    return "downhill";
  default:
    return "unknown";
  }
}

inline SegmentKind SegmentTypeFromCode(int code) {
  switch (code) {
  case 1:
    return SegmentKind::Flat;
  case 2:
    return SegmentKind::Uphill;
  case 3:
    return SegmentKind::Downhill;
  case 4:
    return SegmentKind::Rolling;
  default:
    return SegmentKind::Unknown;
  }
}

// Canonical definition of a segment stored in the database.
struct SegmentDef {
  std::array<uint8_t, 32> uid; // SHA-256 bytes
  std::string uid_hex;         // hex for convenience
  std::string coords_json;
  int point_count = 0;
  double bbox_min_lat = 0, bbox_min_lon = 0, bbox_max_lat = 0, bbox_max_lon = 0;
  double length_m = 0.0;
  bool isSnapped = true;
  SegmentKind kind = SegmentKind::Unknown;
};

// A segment mapped back onto a particular route.
struct SegmentInstance {
  SegmentDef def;               // canonical definition
  long long route_id = 0;       // supplied by caller if persisting
  int start_idx = 0;            // inclusive
  int end_idx = 0;              // exclusive
  std::vector<SegmentRun> runs; // ordered list of runs composing this span
};
