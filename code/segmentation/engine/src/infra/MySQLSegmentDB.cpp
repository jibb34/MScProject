// SPDX-License-Identifier: MIT
#include "MySQLSegmentDB.hpp"
#include "models/SegmentModel.hpp"
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// std::vector<SegmentDef> MySQLSegmentDB::query_defs_in_bbox(double min_lat,
//                                                            double min_lon,
//                                                            double max_lat,
//                                                            double max_lon) {
//   std::vector<SegmentDef> out;
//   // NOTE: assumes table `segments` with columns matching SegmentDef fields
//   //   uid_hex VARCHAR(64) PK, coords_json MEDIUMTEXT,
//   //   point_count INT, bbox_min_lat DOUBLE, bbox_min_lon DOUBLE,
//   //   bbox_max_lat DOUBLE, bbox_max_lon DOUBLE, length_m DOUBLE
//   const char *sql =
//       "SELECT uid_hex, coords_json, point_count, "
//       "       bbox_min_lat, bbox_min_lon, bbox_max_lat, bbox_max_lon,
//       length_m " "FROM segments " "WHERE bbox_min_lat <= ? AND bbox_max_lat
//       >= ? " "  AND bbox_min_lon <= ? AND bbox_max_lon >= ? " "LIMIT 500";
//
//   mariadb::statement_ref stmt = conn_->create_statement(sql);
//   stmt->set_double(0, max_lat);
//   stmt->set_double(1, min_lat);
//   stmt->set_double(2, max_lon);
//   stmt->set_double(3, min_lon);
//   auto res = stmt->query();
//   while (res->next()) {
//     SegmentDef d;
//     d.uid_hex = res->get_string(0);
//     d.coords_json = res->get_string(1);
//     d.point_count = static_cast<int>(res->get_unsigned32(2));
//     d.bbox_min_lat = res->get_double(3);
//     d.bbox_min_lon = res->get_double(4);
//     d.bbox_max_lat = res->get_double(5);
//     d.bbox_max_lon = res->get_double(6);
//     d.length_m = res->get_double(7);
//     out.push_back(std::move(d));
//   }
//   return out;
// }

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

void MySQLSegmentDB::begin() {
  if (mysql_query(conn_, "START TRANSACTION"))
    throw std::runtime_error(mysql_error(conn_));
}

void MySQLSegmentDB::commit() {
  if (mysql_query(conn_, "COMMIT"))
    throw std::runtime_error(mysql_error(conn_));
}

void MySQLSegmentDB::rollback() {
  if (mysql_query(conn_, "ROLLBACK"))
    throw std::runtime_error(mysql_error(conn_));
}

void MySQLSegmentDB::upsert_segment_def(const SegmentDef &d) {
  static const char *SQL = R"SQL(
      INSERT INTO segment_defs
        (segment_uid, direction, srid, point_count, coords_json,
         bbox_min_lat, bbox_min_lon, bbox_max_lat, bbox_max_lon, length_m)
      VALUES (?, 'F', 4326, ?, ?, ?, ?, ?, ?, ?)
      ON DUPLICATE KEY UPDATE
        point_count = VALUES(point_count),
        coords_json = VALUES(coords_json),
        bbox_min_lat = VALUES(bbox_min_lat),
        bbox_min_lon = VALUES(bbox_min_lon),
        bbox_max_lat = VALUES(bbox_max_lat),
        bbox_max_lon = VALUES(bbox_max_lon),
        length_m = VALUES(length_m)
    )SQL";

  MYSQL_STMT *stmt = mysql_stmt_init(conn_);
  if (!stmt)
    throw std::runtime_error("mysql_stmt_init failed");

  if (mysql_stmt_prepare(stmt, SQL, strlen(SQL)))
    throw std::runtime_error(mysql_stmt_error(stmt));

  // Bind parameters
  MYSQL_BIND b[10];
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

  if (mysql_stmt_bind_param(stmt, b))
    throw std::runtime_error(mysql_stmt_error(stmt));

  if (mysql_stmt_execute(stmt))
    throw std::runtime_error(mysql_stmt_error(stmt));

  mysql_stmt_close(stmt);
}

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
