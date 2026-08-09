add_library(robot_code STATIC
    src/robot.cpp
    )

target_include_directories(robot_code PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(robot_code
    PRIVATE
        robot_hal
        robot_utils
)