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
set(F710_DIR ${PROJECT_SOURCE_DIR}/f710)
set(RBL_DIR ${PROJECT_SOURCE_DIR}/f710/rbl)
set(MOTION_CONTROL_DIR ${PROJECT_SOURCE_DIR}/modioint_control)
set(MOTION_CONTROL_DIR ${PROJECT_SOURCE_DIR}/modioint_control)
set(SERIAL_BRIDGE_DIR ${PROJECT_SOURCE_DIR}/ros2-serial-bridge)
set(JSONCONS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/ros2-serial-bridge/include)
MESSAGE("PROJECT_SOURCE_DIR   ${PROJECT_SOURCE_DIR}")

set(SERIAL_BRIDGE_DIR ${PROJECT_SOURCE_DIR}/ros2-serial-bridge)

add_subdirectory(f710_demo)
add_subdirectory(f710_control_v1)
add_subdirectory(f710_control_v2)
add_subdirectory(serial_test_app)
add_subdirectory(tests)