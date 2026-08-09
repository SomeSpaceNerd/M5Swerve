cmake_minimum_required(VERSION 3.22)
project(bellman LANGUAGES C CXX VERSION 0.2.0)
set(PROJECT_SEMVER "${PROJECT_VERSION}-M5Swerve")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/out/bin)

add_subdirectory(bellman-framework/utils)
add_subdirectory(bellman-framework/hal)
add_subdirectory(bellman-framework/runtime)
add_subdirectory(bellman-framework/bellman-protobuf)
add_subdirectory(robot)