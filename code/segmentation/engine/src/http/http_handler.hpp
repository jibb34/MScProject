#pragma once

#include "core/RouteSignalBuilder.hpp" // RouteSignalBuilder
#include "core/SegmentDB.hpp"          // SegmentInstance, SegmentDef
#include "core/SegmentUtils.hpp"       // SegmentUtils helpers
#include "httplib.h"
#include <mysql/mysql.h>
#include <nlohmann/json.hpp>

// Thin wrapper around httplib callbacks.  The main server forwards requests to
// these member functions based on the action string parsed from the URL.
class HttpHandler {
public:
  explicit HttpHandler(MYSQL *db) : db_(db) {}

  void registerRoutes(httplib::Server &server);
  void callPostHandler(std::string action, const httplib::Request &req,
                       httplib::Response &res);
  void callGetHandler(std::string action, const httplib::Request &req,
                      httplib::Response &res);

private:
  std::string endpoint_;
  MYSQL *db_;

  // Individual request handlers
  void handleSegment(const httplib::Request &req, httplib::Response &res);
  void handleWavelet(const httplib::Request &req, httplib::Response &res);
  void handleDebug(const httplib::Request &req, httplib::Response &res);
  void handleUpload(const httplib::Request &req, httplib::Response &res);
  void handleView(const httplib::Request &req, httplib::Response &res);
  void handleSignalLabUI(const httplib::Request &req, httplib::Response &res);
  void handleSignalLabMeta(const httplib::Request &req, httplib::Response &res);
  void handleLabMeta(const httplib::Request &req, httplib::Response &res);
  void handleLabResample(const httplib::Request &req, httplib::Response &res);
  void handleDBPing(const httplib::Request &req, httplib::Response &res);
  void handleSegments(const httplib::Request &req, httplib::Response &res);
};
