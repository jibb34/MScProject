cmake_minimum_required(VERSION 3.10)
project(segmentation_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen (header-only)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenSSL REQUIRED)
set(OPENSSL_USE_STATIC_LIBS TRUE)

# FFTW via pkg-config (no find_package(FFTW3) at all)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED
fftw3
mariadb
)

# Threads
find_package(Threads REQUIRED)

# Sources
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cpp)

add_executable(segmentation_engine ${SRC_FILES})

add_subdirectory(thirdparty/json EXCLUDE_FROM_ALL)
# Include paths
target_include_directories(segmentation_engine PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  # external/json/single_include
  ${FFTW3_INCLUDE_DIRS}
  ${OPENSSL_INCUDE_DIR}
  ${MARIADB_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(segmentation_engine PRIVATE
  Eigen3::Eigen           # Eigen headers
  ${FFTW3_LIBRARIES}      # FFTW3 via pkg-config
  Threads::Threads        # pthread
  m                       # math lib
  OpenSSL::Crypto
  ${MARIADB_LIBRARIES}
  nlohmann_json::nlohmann_json
)

# Any extra FFTW CFLAGS (e.g. -fopenmp)
target_compile_options(segmentation_engine PRIVATE ${FFTW3_CFLAGS_OTHER})

