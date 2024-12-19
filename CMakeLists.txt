cmake_minimum_required(VERSION 3.8)
project(cpp_serial_bridge LANGUAGES CXX C  VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -std=gnu++20)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_VERBOSE_MAKEFILE ON)

message(" c compiler ${CMAKE_C_COMPILER}")

project(f710
    VERSION 0.0.1
    DESCRIPTION "Standalone (non-ROS) Logitech F710 gampad controller"
    LANGUAGES CXX
)
set(CMAKE_CXX_ABI_COMPILED  Off)
set(CMAKE_C_ABI_COMPILED  Off)

# set(CMAKE_C_COMPILER clang)
# set(CMAKE_CXX_COMPILER clang)

set(CMAKE_BUILD_TYPE Debug)
set(F710_DIR ${PROJECT_SOURCE_DIR}/f710)
set(RBL_DIR ${PROJECT_SOURCE_DIR}/rbl)
set(BRIDGE_LIB_DIR ${PROJECT_SOURCE_DIR}/bridge_lib)
set(MOTION_CONTROL_DIR ${PROJECT_SOURCE_DIR}/motion_control)
set(NON_ROS_MSGS_DIR ${PROJECT_SOURCE_DIR}/non_ros_messages)
set(JSONCONS_INCLUDE_DIR vendor/include)

set(F710_SRC
        ${F710_DIR}/src/f710.cpp
        ${F710_DIR}/src/f710.h
        ${F710_DIR}/src/f710_exceptions.h
        ${F710_DIR}/src/f710_helpers.cpp
        ${F710_DIR}/src/f710_helpers.h
#        ${F710_DIR}/src/f710_original.cpp
        ${F710_DIR}/src/f710_time.h
)
set(RBL_SRC
        ${RBL_DIR}/logger.cpp
        ${RBL_DIR}/logger.h
        ${RBL_DIR}/iobuffer.h
        ${RBL_DIR}/iobuffer.cpp
        ${RBL_DIR}/queue.h
        ${RBL_DIR}/std_format.cpp
        ${RBL_DIR}/std_format.h
        ${RBL_DIR}/std_format.cpp
        ${RBL_DIR}/std_format.h
)
set(BRIDGE_LIB_SRC
        ${BRIDGE_LIB_DIR}/fd_functions.h
        ${BRIDGE_LIB_DIR}/fd_functions.cpp
        ${BRIDGE_LIB_DIR}/serial_link.h
        ${BRIDGE_LIB_DIR}/serial_link.cpp
        ${BRIDGE_LIB_DIR}/serial_settings.h
        ${BRIDGE_LIB_DIR}/serial_settings.cpp
)
set(MOTION_CONTROL_SRC
        ${MOTION_CONTROL_DIR}/helpers.h
        ${MOTION_CONTROL_DIR}/helpers.cpp
        ${MOTION_CONTROL_DIR}/motion_control.h
        ${MOTION_CONTROL_DIR}/motion_control.cpp
)
set(NON_ROS_MSGS_SRC
        ${NON_ROS_MSGS_DIR}/msg_struct.h
        ${NON_ROS_MSGS_DIR}/msgs.h
        ${NON_ROS_MSGS_DIR}/msgs.cpp
)
set(LIB_SRC
        ${F710_SRC}
        ${RBL_SRC}
        ${BRIDGE_LIB_SRC}
        ${NON_ROS_MSGS_SRC}
)
set(APP_INCLUDE_DIRS
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/f710/src
        ${PROJECT_SOURCE_DIR}/vendor/include
)


MESSAGE("PROJECT_SOURCE_DIR   ${PROJECT_SOURCE_DIR}")

MESSAGE("LIB_SRC   ${LIB_SRC}")
MESSAGE("APP_INCLUDE_DIRS   ${PROJECT_SOURCE_DIR}")

set(SERIAL_BRIDGE_DIR ${PROJECT_SOURCE_DIR}/serial-bridge)

set(APP_LIB applib)

add_library(${APP_LIB} STATIC ${LIB_SRC})
target_include_directories(${APP_LIB} PUBLIC ${APP_INCLUDE_DIRS})


add_subdirectory(f710_demo)
add_subdirectory(f710_control_v1)
add_subdirectory(f710_control_v2)
add_subdirectory(serial_test_app)
add_subdirectory(tests)
