#include "http/http_handler.hpp"
#include "json.hpp" // nlohmann/json
#include <fstream>
#include <iostream>

using json = nlohmann::json;

int main() {
  // Load configuration
  std::ifstream cfg("config/settings.json");
  if (!cfg) {
    std::cerr << "[ERROR] Cannot open config/settings.json\n";
    return 1;
  }
  json settings;
  cfg >> settings;
  int port = settings["server"]["port"];
  auto logging = settings.value("logging", json::object());
  std::string logLevel = logging.value("level", "info");
  std::string logFile = logging.value("file", "");

  // Simple debug log to stdout/file
  std::cout << "[DEBUG] Starting server on port " << port << std::endl;

  // HTTP server setup
  httplib::Server server;
  // allow big uploads
  server.set_payload_max_length(1024ull * 1024ull * 512ull); // 512MB

  // optional: longer timeouts for big uploads
  server.set_read_timeout(60, 0);
  server.set_write_timeout(60, 0);

  server.Post("/_echo", [](const auto &req, auto &res) {
    nlohmann::json j = {
        {"method", req.method},
        {"path", req.path},
        {"content_type", req.get_header_value("Content-Type")},
        {"bytes", req.body.size()},
        {"snippet",
         req.body.substr(0, std::min<size_t>(req.body.size(), 256))}};
    res.set_content(j.dump(2), "application/json");
  });

  HttpHandler handler;
  for (const auto &ep : settings["server"]["endpoints"]) {
    std::string path = ep.get<std::string>();
    // get name of endpoint to call the handler
    std::string action =
        (!path.empty() && path[0] == '/') ? path.substr(1) : path;
    std::cerr << "Listening on: " << action << "\n";
    server.Post(path, [action, &handler](const auto &req, auto &res) {
      std::cerr << "[POST λ] ENTER path=" << req.path << " action=" << action
                << " ct=" << req.get_header_value("Content-Type")
                << " bytes=" << req.body.size() << "\n";
      try {
        handler.callPostHandler(action, req, res);
        std::cerr << "[POST λ] LEAVE status=" << res.status << "\n";
      } catch (const std::exception &e) {
        std::cerr << "[POST λ] EXCEPTION: " << e.what() << "\n";
        res.status = 500;
        res.set_content(std::string("exception: ") + e.what(), "text/plain");
      } catch (...) {
        std::cerr << "[POST λ] EXCEPTION: unknown\n";
        res.status = 500;
        res.set_content("exception: unknown", "text/plain");
      }
    });
    server.Get(path, [action, &handler](const auto &req, auto &res) {
      handler.callGetHandler(action, req, res);
    });
  }
  server.listen("0.0.0.0", port);
  return 0;
}
