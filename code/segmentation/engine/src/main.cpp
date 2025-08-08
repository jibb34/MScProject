#include "http_server.hpp"
#include "segmentor.hpp"
#include <iostream>
#include <yaml-cpp/yaml.h>

int main() {
  YAML::Node config = YAML::LoadFile("settings/config.yml");
  start_http_server(config);
  return 0;
}
