// #include "httplib.h" // single‑header HTTP library
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
  std::string logLevel = settings["logging"]["level"];
  std::string logFile = settings["logging"]["file"];

  // Simple debug log to stdout/file
  std::cout << "[DEBUG] Starting server on port " << port << std::endl;

  // HTTP server setup
  httplib::Server server;
  // allow big uploads
  server.set_payload_max_length(1024ull * 1024ull * 512ull); // 512MB

  // optional: longer timeouts for big uploads
  server.set_read_timeout(60, 0);
  server.set_write_timeout(60, 0);
  server.set_logger([](const auto &req, const auto &res) {
    std::cout << "[HTTP] " << req.method << " " << req.path << " -> "
              << res.status << std::endl;
  });
  HttpHandler handler;
  for (const auto &ep : settings["server"]["endpoints"]) {
    std::string path = ep.get<std::string>();
    // get name of endpoint to call the handler
    std::string action =
        (!path.empty() && path[0] == '/') ? path.substr(1) : path;
    server.Post(path, [action, &handler](const auto &req, auto &res) {
      handler.callHandler(action, req, res);
    });
  }

  server.listen("0.0.0.0", port);
  return 0;
}
