add_library(robot_hal STATIC)

target_compile_definitions(robot_hal PUBLIC
    FEMTOBOT_VERSION="${PROJECT_SEMVER}"
)

target_link_libraries(robot_hal PUBLIC 
    robot_proto)

target_include_directories(robot_hal
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(robot_hal PUBLIC cxx_std_20)

target_sources(robot_hal PRIVATE
    src/hal.cpp
    src/state_control.cpp
    src/logger.cpp
    src/gpio.cpp
    src/buses/uart.cpp
    src/buses/i2c.cpp
    src/driver_station.cpp
    src/device.cpp
    src/components/roller485.cpp
    src/components/sc_servo.cpp
    src/components/stamp_io.cpp
    src/components/ina228.cpp
    src/components/bno055.cpp)