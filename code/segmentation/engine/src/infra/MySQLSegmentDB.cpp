// MySQLSegmentDB wraps basic transactional operations for segment storage.

#include "MySQLSegmentDB.hpp"
#include "models/SegmentModel.hpp"
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// Establish connection using URI and credentials
MySQLSegmentDB::MySQLSegmentDB(const std::string &uri, const std::string &user,
                               const std::string &pass,
                               const std::string &schema) {
  conn_ = mysql_init(nullptr);
  if (!conn_)
    throw std::runtime_error("mysql_init failed");
  // Parse URI "tcp://host:port"
  std::string host = uri, port = "3306";
  if (auto pos = uri.find("://"); pos != std::string::npos) {
    host = uri.substr(pos + 3);
  }
  if (auto p = host.find(':'); p != std::string::npos) {
    port = host.substr(p + 1);
    host = host.substr(0, p);
  }
  if (!mysql_real_connect(conn_, host.c_str(), user.c_str(), pass.c_str(),
                          schema.c_str(), std::stoi(port), nullptr, 0)) {
    std::string err = mysql_error(conn_);
    mysql_close(conn_);
    throw std::runtime_error("connect failed: " + err);
  }
}

MySQLSegmentDB::~MySQLSegmentDB() { mysql_close(conn_); }

// Begin transaction
void MySQLSegmentDB::begin() {
  if (mysql_query(conn_, "START TRANSACTION"))
    throw std::runtime_error(mysql_error(conn_));
}

// Commit transaction
void MySQLSegmentDB::commit() {
  if (mysql_query(conn_, "COMMIT"))
    throw std::runtime_error(mysql_error(conn_));
}

// Roll back transaction
void MySQLSegmentDB::rollback() {
  if (mysql_query(conn_, "ROLLBACK"))
    throw std::runtime_error(mysql_error(conn_));
}

// Insert or update segment definition
void MySQLSegmentDB::upsert_segment_def(const SegmentDef &d) {
  static const char *SQL = R"SQL(
      INSERT INTO segment_defs
        (segment_uid, direction, srid, point_count, coords_json,
         bbox_min_lat, bbox_min_lon, bbox_max_lat, bbox_max_lon, length_m, kind)
      VALUES (?, 'F', 4326, ?, ?, ?, ?, ?, ?, ?, ?)
      ON DUPLICATE KEY UPDATE
        point_count = VALUES(point_count),
        coords_json = VALUES(coords_json),
        bbox_min_lat = VALUES(bbox_min_lat),
        bbox_min_lon = VALUES(bbox_min_lon),
        bbox_max_lat = VALUES(bbox_max_lat),
        bbox_max_lon = VALUES(bbox_max_lon),
        length_m = VALUES(length_m),
        kind = VALUES(kind)
    )SQL";

  MYSQL_STMT *stmt = mysql_stmt_init(conn_);
  if (!stmt)
    throw std::runtime_error("mysql_stmt_init failed");

  if (mysql_stmt_prepare(stmt, SQL, strlen(SQL)))
    throw std::runtime_error(mysql_stmt_error(stmt));

  // Bind parameters
  MYSQL_BIND b[11];
  memset(b, 0, sizeof(b));

  // segment_uid (binary 32)
  b[0].buffer_type = MYSQL_TYPE_BLOB;
  b[0].buffer = (void *)d.uid.data();
  b[0].buffer_length = d.uid.size();
  unsigned long uid_len = d.uid.size();
  b[0].length = &uid_len;

  // point_count
  b[1].buffer_type = MYSQL_TYPE_LONG;
  int pc = d.point_count;
  b[1].buffer = &pc;

  // coords_json (string)
  b[2].buffer_type = MYSQL_TYPE_STRING;
  b[2].buffer = (void *)d.coords_json.c_str();
  unsigned long json_len = d.coords_json.size();
  b[2].buffer_length = json_len;
  b[2].length = &json_len;

  // bbox_min_lat
  b[3].buffer_type = MYSQL_TYPE_DOUBLE;
  b[3].buffer = (void *)&d.bbox_min_lat;
  // bbox_min_lon
  b[4].buffer_type = MYSQL_TYPE_DOUBLE;
  b[4].buffer = (void *)&d.bbox_min_lon;
  // bbox_max_lat
  b[5].buffer_type = MYSQL_TYPE_DOUBLE;
  b[5].buffer = (void *)&d.bbox_max_lat;
  // bbox_max_lon
  b[6].buffer_type = MYSQL_TYPE_DOUBLE;
  b[6].buffer = (void *)&d.bbox_max_lon;
  // length_m
  b[7].buffer_type = MYSQL_TYPE_DOUBLE;
  b[7].buffer = (void *)&d.length_m;

  // kind (enum -> tinyint)
  b[8].buffer_type = MYSQL_TYPE_TINY;
  int kind_code = static_cast<int>(d.kind);
  b[8].buffer = &kind_code;

  if (mysql_stmt_bind_param(stmt, b))
    throw std::runtime_error(mysql_stmt_error(stmt));

  if (mysql_stmt_execute(stmt))
    throw std::runtime_error(mysql_stmt_error(stmt));

  mysql_stmt_close(stmt);
}

// Record segment instance for a route
long long MySQLSegmentDB::insert_route_segment(const SegmentInstance &inst) {
  static const char *SQL = R"SQL(
      INSERT INTO route_segments (segment_uid, route_id, start_idx, end_idx)
      VALUES (?, ?, ?, ?)
    )SQL";

  MYSQL_STMT *stmt = mysql_stmt_init(conn_);
  if (!stmt)
    throw std::runtime_error("mysql_stmt_init failed");

  if (mysql_stmt_prepare(stmt, SQL, strlen(SQL)))
    throw std::runtime_error(mysql_stmt_error(stmt));

  MYSQL_BIND b[4];
  memset(b, 0, sizeof(b));

  // segment_uid
  b[0].buffer_type = MYSQL_TYPE_BLOB;
  b[0].buffer = (void *)inst.def.uid.data();
  unsigned long uid_len = inst.def.uid.size();
  b[0].buffer_length = uid_len;
  b[0].length = &uid_len;

  // route_id
  b[1].buffer_type = MYSQL_TYPE_LONGLONG;
  long long rid = inst.route_id;
  b[1].buffer = &rid;

  // start_idx
  b[2].buffer_type = MYSQL_TYPE_LONG;
  int si = inst.start_idx;
  b[2].buffer = &si;

  // end_idx
  b[3].buffer_type = MYSQL_TYPE_LONG;
  int ei = inst.end_idx;
  b[3].buffer = &ei;

  if (mysql_stmt_bind_param(stmt, b))
    throw std::runtime_error(mysql_stmt_error(stmt));

  if (mysql_stmt_execute(stmt))
    throw std::runtime_error(mysql_stmt_error(stmt));

  mysql_stmt_close(stmt);
  return mysql_insert_id(conn_);
}

// Persist contiguous way runs for a segment
void MySQLSegmentDB::insert_segment_runs(long long segment_id,
                                         const std::vector<SegmentRun> &runs) {
  static const char *SQL = R"SQL(
      INSERT INTO segment_way_runs
        (segment_id, seq, way_id, run_start_idx, run_end_idx)
      VALUES (?, ?, ?, ?, ?)
    )SQL";

  MYSQL_STMT *stmt = mysql_stmt_init(conn_);
  if (!stmt)
    throw std::runtime_error("mysql_stmt_init failed");

  if (mysql_stmt_prepare(stmt, SQL, strlen(SQL)))
    throw std::runtime_error(mysql_stmt_error(stmt));

  for (size_t i = 0; i < runs.size(); ++i) {
    MYSQL_BIND b[5];
    memset(b, 0, sizeof(b));

    long long wid = runs[i].way_id;
    int seq = i;
    int from = runs[i].from_idx;
    int to = runs[i].to_idx;

    // segment_id
    b[0].buffer_type = MYSQL_TYPE_LONGLONG;
    b[0].buffer = &segment_id;
    // seq
    b[1].buffer_type = MYSQL_TYPE_LONG;
    b[1].buffer = &seq;
    // way_id
    b[2].buffer_type = MYSQL_TYPE_LONGLONG;
    b[2].buffer = &wid;
    // run_start_idx
    b[3].buffer_type = MYSQL_TYPE_LONG;
    b[3].buffer = &from;
    // run_end_idx
    b[4].buffer_type = MYSQL_TYPE_LONG;
    b[4].buffer = &to;

    if (mysql_stmt_bind_param(stmt, b))
      throw std::runtime_error(mysql_stmt_error(stmt));
    if (mysql_stmt_execute(stmt))
      throw std::runtime_error(mysql_stmt_error(stmt));
  }

  mysql_stmt_close(stmt);
}
