#pragma once
#include "core/SegmentDB.hpp"
#include <mysql/mysql.h>

class MySQLSegmentDB final : public SegmentDB {
public:
  MySQLSegmentDB(const std::string &uri, const std::string &user,
                 const std::string &pass, const std::string &schema);
  ~MySQLSegmentDB();

  void begin() override;
  void commit() override;
  void rollback() override;
  void upsert_segment_def(const SegmentDef &def) override;
  long long insert_route_segment(const SegmentInstance &inst) override;
  void insert_segment_runs(long long segment_id,
                           const std::vector<SegmentRun> &runs) override;
  std::vector<SegmentDef> query_defs_in_bbox(double bbox_min_lat,
                                             double bbox_min_lon,
                                             double bbox_max_lat,
                                             double bbox_max_lon) override;

private:
  // hold your connection/session here
  MYSQL *conn_ = nullptr; // replace with actual type
};
