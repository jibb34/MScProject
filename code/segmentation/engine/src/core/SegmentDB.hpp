#pragma once
#include "models/SegmentModel.hpp"
#include <string>
#include <vector>

// You can wire this to MySQL Connector/C++ or your own DB layer.
class SegmentDB {
public:
  virtual ~SegmentDB() = default;
  virtual void begin() = 0;
  virtual void commit() = 0;
  virtual void rollback() = 0;

  // Upsert segment definition (idempotent by uid)
  virtual void upsert_segment_def(const SegmentDef &def) = 0;

  // Insert concrete instance for a route; returns generated segment_id
  virtual long long insert_route_segment(const SegmentInstance &inst) = 0;

  // Bulk insert ordered way runs for a segment_id
  virtual void insert_segment_runs(long long segment_id,
                                   const std::vector<SegmentRun> &runs) = 0;
  // NEW: coarse candidate fetch using bbox overlap
  // Padding (metres) is applied externally when computing bbox to be
  // conservative.
  // Default: return empty; concrete DBs can override when ready.
  virtual std::vector<SegmentDef> query_defs_in_bbox(double bbox_min_lat,
                                                     double bbox_min_lon,
                                                     double bbox_max_lat,
                                                     double bbox_max_lon) {
    return {};
  }
};
