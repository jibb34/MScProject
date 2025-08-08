#include "http_server.hpp"
#include "segmentor.hpp"
#include <fstream>

crow::SimpleApp app;
CROW_ROUTE(app, "/upload")
    .methods("POST"_method)([&](const crow::request &req) {
      std::ofstream out("/input/uploaded_file.json");
      out << req.body;
      out.close();

      std::string response = process_file("input/uploaded_file.json");
      return crow::response{200, response};
    });

app.port(5005).run();
