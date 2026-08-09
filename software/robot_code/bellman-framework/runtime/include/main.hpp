#pragma once

#include "hal.hpp"
#include "state_control.hpp"
#include "logger.hpp"
#include "device.hpp"
#include "robot.hpp"
#include <chrono>
#include <iostream>
#include <thread>
#include <csignal>
#include <string>
#include <memory>
#include <atomic>

#ifdef __linux__
#include <sys/mman.h>
#endif

constexpr int looptime = 10; // Define a 10ms looptime for the robot periodic code