#pragma once
#include "httplib.h"

class HttpHandler {
public:
  HttpHandler() = default;

  void registerRoutes(httplib::Server &server);
  void callPostHandler(std::string action, const httplib::Request &req,
                       httplib::Response &res);
  void callGetHandler(std::string action, const httplib::Request &req,
                      httplib::Response &res);

private:
  std::string endpoint_;
  // Add handlers here as we create them. /segment should be the only main one
  // though
  void handleSegment(const httplib::Request &req, httplib::Response &res);
  void handleWavelet(const httplib::Request &req, httplib::Response &res);
  void handleDebug(const httplib::Request &req, httplib::Response &res);
  void handleUpload(const httplib::Request &req, httplib::Response &res);
  void handleView(const httplib::Request &req, httplib::Response &res);

  void handleSignalLabUI(const httplib::Request &req, httplib::Response &res);
  void handleSignalLabMeta(const httplib::Request &req, httplib::Response &res);
  void handleLabMeta(const httplib::Request &req, httplib::Response &res);
  void handleLabResample(const httplib::Request &req, httplib::Response &res);
};
