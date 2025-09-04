#include "http/http_handler.hpp"
#include "json.hpp" // nlohmann/json
#include <execinfo.h>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unistd.h>

using json = nlohmann::json;
inline json list_uploads_json(const std::string &dir = "uploads");

static void bt_handler(int sig) {
  void *bt[64];
  int n = backtrace(bt, 64);
  dprintf(2, "\n=== FATAL SIG %d ===\n", sig);
  backtrace_symbols_fd(bt, n, 2);
  _exit(128 + sig);
}
static void install_bt_handlers() {
  signal(SIGSEGV, bt_handler);
  signal(SIGABRT, bt_handler);
  signal(SIGFPE, bt_handler);
  signal(SIGILL, bt_handler);
  signal(SIGBUS, bt_handler);
}
static void bt(int sig) {
  void *a[64];
  int n = backtrace(a, 64);
  dprintf(2, "\nSIG %d\n", sig);
  backtrace_symbols_fd(a, n, 2);
  _exit(128 + sig);
}

int main() {
  install_bt_handlers();
  // Load configuration
  std::ifstream cfg("config/settings.json");
  if (!cfg) {
    std::cerr << "[ERROR] Cannot open config/settings.json\n";
    return 1;
  }
  json settings;
  cfg >> settings;
  int port = settings["server"]["port"];
  // int port = 5005;
  // Simple debug log to stdout/file
  std::cout << "[DEBUG] Starting server on port " << port << std::endl;

  // HTTP server setup
  httplib::Server server;
  server.set_mount_point("/static",
                         "./public"); // serves files in ./public at /static/*
  server.set_file_extension_and_mimetype_mapping("js",
                                                 "application/javascript");

  server.set_file_extension_and_mimetype_mapping("css", "text/css");
  // Optional configuration – comment these out initially
  server.set_payload_max_length(1024ull * 1024ull * 512ull); // 512MB
  server.set_read_timeout(60, 0);
  server.set_write_timeout(60, 0);

  // Test echo endpoint – comment out for a minimal server
  server.Post("/_echo", [](const auto &req, auto &res) {
    // nlohmann::json j = {
    //     {"method", req.method},
    //     {"path", req.path},
    //     {"content_type", req.get_header_value("Content-Type")},
    //     {"bytes", req.body.size()},
    //     {"snippet",
    //      req.body.substr(0, std::min<size_t>(req.body.size(), 256))}};
    // res.set_content(j.dump(2), "application/json");
    std::string reply = "Received POST to " + req.path +
                        ", content‑length=" + std::to_string(req.body.size());
    res.set_content(reply, "text/plain");
  });

  // Instantiate HttpHandler and register endpoints – comment out initially
  MYSQL *db = mysql_init(nullptr);
  if (!db) {
    std::cerr << "[main] mysql_init failed\n";
    return 1;
  }

  const char *host = "127.0.0.1";
  const char *user = "routeseg_user";
  const char *password = "changeme-user";
  const char *database = "routeseg";

  if (!mysql_real_connect(db, host, user, password, database, 3306, nullptr,
                          0)) {
    std::cerr << "[main] mysql_real_connect failed: " << mysql_error(db)
              << "\n";
    return 1;
  }
  HttpHandler handler(db);

  for (const auto &ep : settings["server"]["endpoints"]) {
    std::string path = ep.get<std::string>();
    std::string action =
        (!path.empty() && path[0] == '/') ? path.substr(1) : path;
    std::cerr << "Listening on: " << action << "\n";
    server.Post(path, [action, &handler](const auto &req, auto &res) {
      try {
        handler.callPostHandler(action, req, res);
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
  }
  std::string action = "view";
  server.Get("/view", [action, &handler](const auto &req, auto &res) {
    handler.callGetHandler("view", req, res);
  });
  server.Get("/viewLab", [action, &handler](const auto &req, auto &res) {
    handler.callGetHandler("viewLab", req, res);
  });
  server.Get("/lab/meta", [&handler](const auto &req, auto &res) {
    handler.callGetHandler("labMeta", req, res);
  });
  server.Get("/dbping", [&handler](const auto &req, auto &res) {
    handler.callGetHandler("dbping", req, res);
  });
  server.Get("/segments", [&handler](const auto &req, auto &res) {
    handler.callGetHandler("segments", req, res);
  });

  server.Get("/lab/list", [](const httplib::Request &, httplib::Response &res) {
    const auto arr = list_uploads_json();
    nlohmann::json out = {{"files", arr}}; // <-- wrap it
    res.set_content(out.dump(), "application/json");
  });

  // Start listening – this blocks until the server stops
  server.listen("0.0.0.0", port);
  return 0;
}

namespace fs = std::filesystem;

inline nlohmann::json list_uploads_json(const std::string &dir) {
  nlohmann::json arr = nlohmann::json::array();
  if (!fs::exists(dir))
    return arr;

  // newest first
  std::vector<fs::directory_entry> entries;
  for (auto &de : fs::directory_iterator(dir)) {
    if (de.is_regular_file())
      entries.push_back(de);
  }
  std::sort(entries.begin(), entries.end(), [](auto &a, auto &b) {
    return fs::last_write_time(a) > fs::last_write_time(b);
  });

  for (auto &de : entries) {
    const auto p = de.path();
    const auto bytes = (uint64_t)fs::file_size(p);
    const auto rel = (fs::path(dir) / p.filename())
                         .generic_string(); // e.g. "uploads/map_....json"
    nlohmann::json j{{"file", rel},         // <- value to POST back as "map"
                     {"name", p.filename().string()}, // for display
                     {"bytes", bytes}};
    arr.push_back(j);
  }
  return arr;
}
