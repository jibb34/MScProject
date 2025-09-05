// Entry point for the segmentation engine HTTP server.  It wires up the
// httplib server, loads configuration and exposes the REST endpoints handled by
// `HttpHandler`.

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

  // ---------------------- Load configuration ------------------------------
  std::ifstream cfg("config/settings.json");
  if (!cfg) {
    std::cerr << "[ERROR] Cannot open config/settings.json\n";
    return 1;
  }
  json settings;
  cfg >> settings;
  int port = settings["server"].value("port", 5005);
  std::cout << "[DEBUG] Starting server on port " << port << std::endl;

  // ---------------------- HTTP server setup -------------------------------
  httplib::Server server;
  server.set_mount_point("/static", "./public");
  server.set_file_extension_and_mimetype_mapping("js",
                                                 "application/javascript");
  server.set_file_extension_and_mimetype_mapping("css", "text/css");
  server.set_payload_max_length(1024ull * 1024ull * 512ull); // 512MB
  server.set_read_timeout(60, 0);
  server.set_write_timeout(60, 0);

  // Simple echo endpoint useful during development
  server.Post("/_echo", [](const auto &req, auto &res) {
    std::string reply = "Received POST to " + req.path +
                        ", content-length=" + std::to_string(req.body.size());
    res.set_content(reply, "text/plain");
  });

  // ---------------------- Database connection -----------------------------
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

  // ---------------------- Register POST endpoints -------------------------
  for (const auto &ep : settings["server"]["post_endpoints"]) {
    std::string path = ep.get<std::string>();
    std::string action =
        (!path.empty() && path[0] == '/') ? path.substr(1) : path;
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


  // ---------------------- Register GET endpoints --------------------------
  for (const auto &ep : settings["server"]["get_endpoints"]) {
    std::string path = ep.get<std::string>();
    std::string action =
        (!path.empty() && path[0] == '/') ? path.substr(1) : path;
    server.Get(path, [action, &handler](const auto &req, auto &res) {
      handler.callGetHandler(action, req, res);
    });
  }

  // Non-configurable helper endpoint listing uploaded files
  server.Get("/lab/list", [](const httplib::Request &, httplib::Response &res) {
    const auto arr = list_uploads_json();
    nlohmann::json out = {{"files", arr}};
    res.set_content(out.dump(), "application/json");
  });

  // ---------------------- Start server ------------------------------------
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
