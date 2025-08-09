set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
cmake_minimum_required(VERSION 3.10)
project(segmentation_engine)
set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS src/*.cpp)
# Executable target
add_executable(segmentation_engine ${SRC_FILES})

# Include directories for headers
target_include_directories(segmentation_engine PRIVATE include src)

# Link any libraries here (e.g., pthread)
target_link_libraries(segmentation_engine PRIVATE pthread)
