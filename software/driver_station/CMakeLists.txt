cmake_minimum_required(VERSION 3.19)
project(driver_station LANGUAGES C CXX)

add_subdirectory(bellman-protobuf)

find_package(Qt6 6.5 REQUIRED COMPONENTS Core Widgets Network LinguistTools)
find_package(SDL3 CONFIG REQUIRED)

qt_standard_project_setup()

qt_add_executable(driver_station
    WIN32 MACOSX_BUNDLE
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    receiver.h receiver.cpp
    sender.h sender.cpp
    logger.h logger.cpp
    networkworker.h networkworker.cpp
    settingsmanager.h settingsmanager.cpp
)

qt_add_translations(
    TARGETS driver_station
    TS_FILES driver_station_en_US.ts
)

#target_compile_options(driver_station PRIVATE /fsanitize=address)

target_link_libraries(driver_station
    PRIVATE
        robot_proto
        SDL3::SDL3
        Qt::Core
        Qt::Widgets
        Qt::Network
)

install(TARGETS driver_station
    BUNDLE  DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

qt_generate_deploy_app_script(
    TARGET driver_station
    OUTPUT_SCRIPT deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
)
install(SCRIPT ${deploy_script})


