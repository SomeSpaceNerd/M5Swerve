add_executable(robot_runtime
    src/main.cpp
)

target_include_directories(robot_runtime PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(robot_runtime
    PRIVATE
        robot_hal
        robot_utils
        robot_code
)