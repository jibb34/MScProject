cmake_minimum_required(VERSION 3.10)
project(segmentation-engine)

set(CMAKE_CXX_STANDARD 17)

include_directories(
  src
  third_party/crow/include
  third_party/yaml-cpp/include
)
add_subdirectory(
  third_party/yaml-cpp
)

add_executable(segmentation-engine
    src/main.cpp
    src/http_server.cpp
    src/segmenter.cpp
)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

target_link_libraries(segmentation-engine
    yaml-cpp
    pthread
)

