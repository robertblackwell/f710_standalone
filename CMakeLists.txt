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
add_subdirectory(src)
add_subdirectory(tests)