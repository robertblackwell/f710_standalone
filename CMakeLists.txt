cmake_minimum_required(VERSION 3.8)
project(cpp_serial_bridge LANGUAGES CXX C  VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -std=gnu++20)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(f710
    VERSION 0.0.1
    DESCRIPTION "Standalone (non-ROS) Logitech F710 gampad controller"
    LANGUAGES CXX
)
set(CMAKE_BUILD_TYPE Debug)
add_executable(f710_controller
        src/main.cpp
        src/outputter.cpp
        src/outputter.h
        f710/src/f710_time.h
        f710/src/f710.h
        f710/src/f710.cpp
        f710/src/f710_helpers.cpp
        f710/src/f710_helpers.h
        f710/rbl/logger.cpp
        f710/rbl/logger.h

        ros2-serial-bridge/src/bridge_lib/fd_functions.h
        ros2-serial-bridge/src/bridge_lib/fd_functions.cpp
        ros2-serial-bridge/src/bridge_lib/iobuffer.h
        ros2-serial-bridge/src/bridge_lib/iobuffer.cpp
        ros2-serial-bridge/src/bridge_lib/queue.h
        ros2-serial-bridge/src/bridge_lib/iobuffer.h
        ros2-serial-bridge/src/bridge_lib/serial_link.h
        ros2-serial-bridge/src/bridge_lib/serial_link.cpp
        ros2-serial-bridge/src/bridge_lib/serial_settings.h
        ros2-serial-bridge/src/bridge_lib/serial_settings.cpp

)
target_include_directories(f710_controller  PUBLIC ./  ./src  ./ros2-serial-bridge/src  ./ros2-serial-bridge/src/bridge_lib ./f710 ./f710/src)
#target_compile_definitions(f710 PUBLIC RBL_LOG_ENABLED RBL_LOG_ALLOW_GLOBAL)

add_subdirectory(tests)