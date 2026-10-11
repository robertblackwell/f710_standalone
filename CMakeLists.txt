cmake_minimum_required(VERSION 3.16)
project(F710Project LANGUAGES CXX C  VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_VERBOSE_MAKEFILE ON)

message(" c compiler ${CMAKE_C_COMPILER}")

find_package(PkgConfig REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
#set(F710_DIR ${PROJECT_SOURCE_DIR}/f710-v1)
set(RBL_DIR ${PROJECT_SOURCE_DIR}/src/rbl)
#set(BRIDGE_LIB_DIR ${PROJECT_SOURCE_DIR}/bridge_lib)
#set(MOTION_CONTROL_DIR ${PROJECT_SOURCE_DIR}/motion_control)
#set(TWIST_DRIVE_DIR ${PROJECT_SOURCE_DIR}/twist_drive)
set(PICO_MESSAGES_DIR ${PROJECT_SOURCE_DIR}/src/pico_messages)
set(JSONCONS_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/vendor/include)
set(SRC_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)
message("PICO_MESSAGES_DIR     ${PICO_MESSAGES_DIR}")
message("JSONCONS_INCLUDE_DIR  ${JSONCONS_INCLUDE_DIR}")
message("SRC_INCLUDE_DIR       ${SRC_INCLUDE_DIR}")
# ============================================================================
# LOCAL BOOST DEPENDENCY INTEGRATION (ASIO & FIBER)
# ============================================================================

# Enable Boost's internal Native CMake configuration architecture
set(BOOST_ENABLE_CMAKE ON CACHE INTERNAL "")

# Restrict the submodule to build/expose ONLY Asio, Context, and Fiber
set(BOOST_INCLUDE_LIBRARIES "asio" "context" "fiber" CACHE INTERNAL "")

# Force Boost's own internal unit testing framework to shut off
set(BUILD_TESTING_SAVED ${BUILD_TESTING}) # Save your setting
set(BUILD_TESTING OFF CACHE INTERNAL "")

# Parse the local submodule source directory tree
if(EXISTS "${PROJECT_SOURCE_DIR}/external/boost/CMakeLists.txt")
    add_subdirectory(external/boost)
else()
    message(FATAL_ERROR "Boost submodule not found! Please run your setup_boost.sh script first.")
endif()

# Restore your project's testing configuration state
#set(BUILD_TESTING ${BUILD_TESTING_SAVED} CACHE BOOL "" FORCE)

# ============================================================================
# PROJECT SUBDIRECTORIES & TESTING CONFIGURATIONS
# ============================================================================

# 1. Enable CTest testing
include(CTest)
enable_testing()

add_subdirectory(src)
add_subdirectory(tests)


# 3. Build the apps
# add_subdirectory(test_apps)

# Build unit tests 
if(BUILD_TESTIING)
    add_subdirectory(tests)
endif()

