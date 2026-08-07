cmake_minimum_required(VERSION 3.20)
project(robot_proto LANGUAGES C CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/nanopb/extra)
find_package(Nanopb REQUIRED)

nanopb_generate_cpp(TARGET robot_proto
    RELPATH proto
    proto/robot_advertisement.proto
    proto/robot_log.proto
    proto/robot_to_ds.proto
    proto/ds_to_robot.proto
)

