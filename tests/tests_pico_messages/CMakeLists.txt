cmake_minimum_required(VERSION 3.27)

project(robot_linux_tests CXX C)
find_package(Threads REQUIRED)
find_program(BASH_PROG bash)

message("BASH_PROG-NOTFOUND       ${BASH_PROG-NOTFOUND}")
message("BASH_PROG                ${BASH_PROG}")

##
## options
##
option(LOCAL_TESTS_BuildTests "Build the tests when enabled" ON )
option(LOCAL_TESTS_Install_Targets "Installs targets as well as files if set" OFF)
option(LOCAL_TESTS_Package "Install as a CMake package - not implemented" OFF)
option(LOCAL_TESTS_Verbose "Print a lot of diagnostic stuff" ON)
option(LOCAL_TESTS_DebugBuild "Perform build as type Debug" ON)
option(LOCAL_TESTS_BuildDocs "Build doxygen documentation")
set(CMAKE_BUILD_TYPE Debug)
## ============================================================================
## The stuff below - I am not sure about ======================================
##

# this is a fix for a problem with boost libraries see https://github.com/Microsoft/vcpkg/issues/4497
# also a hack and breaks "best practices" as per Damiel Pfeiffer's presentation https://www.youtube.com/watch?v=rLopVhns4Zs&feature=youtu.be
set(other_flags "${other_flags} -frtti -fvisibility-inlines-hidden")
set(other_flags "${other_flags} -fvisibility=hidden")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${other_flags} -pthread -g -O0")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")
set(CTEST_CUSTOM_PRE_TEST "echo THIS IS PRE TEST")
set(THREADS_PREFER_PTHREAD_FLAG ON)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##
## Configuration
##
include (GNUInstallDirs)

set(TESTS_DIR  ${CMAKE_CURRENT_LIST_DIR})
message("TESTS_DIR       ${TESTS_DIR}")
cmake_path(GET           TESTS_DIR           PARENT_PATH          PKG_DIR)
cmake_path(APPEND_STRING PKG_DIR             "/f710_control_v1"               OUTPUT_VARIABLE PKG_SRC_DIR)
cmake_path(GET           PKG_DIR             PARENT_PATH          WS_SRC_DIR)
cmake_path(GET           WS_SRC_DIR          PARENT_PATH          WS_DIR)
cmake_path(APPEND_STRING PKG_SRC_DIR         "/bridge_lib"        OUTPUT_VARIABLE BRIDGE_LIB_DIR )
cmake_path(APPEND_STRING PKG_SRC_DIR         "/fbcontrol"         OUTPUT_VARIABLE FBCONTROL_LIB_DIR )
cmake_path(APPEND_STRING PKG_DIR             "/jsoncons/include"  OUTPUT_VARIABLE JSON_LIB_DIR )
cmake_path(APPEND_STRING TESTS_DIR           "/common"            OUTPUT_VARIABLE TESTS_COMMON_DIR)
cmake_path(APPEND_STRING WS_DIR              "install"            OUTPUT_VARIABLE WS_INSTALL_DIR)
cmake_path(APPEND_STRING WS_DIR              "build"              OUTPUT_VARIABLE WS_BUILD_DIR)
cmake_path(APPEND_STRING WS_DIR              "install/sample_interfaces/include/sample_interfaces" OUTPUT_VARIABLE SAMPLE_INTERFACES_INCLUDE_DIR)

cmake_path(GET           WS_DIR              PARENT_PATH          ROBOT_DIR)
cmake_path(APPEND_STRING ROBOT_DIR           "/pico_firmware"      OUTPUT_VARIABLE FIRMWARE_DIR)
cmake_path(APPEND_STRING FIRMWARE_DIR        "/f710_control_v1"                OUTPUT_VARIABLE FIRMWARE_SRC_DIR)
cmake_path(APPEND_STRING FIRMWARE_SRC_DIR    "/common"             OUTPUT_VARIABLE FIRMWARE_COMMON_DIR)

set(TESTS_INCLUDE_DIRS  
	${PKG_SRC_DIR} 
	${BRIDGE_LIB_DIR} 
	${FBCONTROL_LIB_DIR} 
	${JSON_LIB_DIR}
	${TESTS_COMMON_DIR} 
	${SAMPLE_INTERFACES_INCLUDE_DIR}
	${FIRMWARE_COMMON_DIR}
	/opt/ros/iron/include/rclcpp
	/opt/ros/iron/include/rcl
	/opt/ros/iron/include/rcutils
	/opt/ros/iron/include/rosidl_runtime_cpp
	/opt/ros/iron/include/rosidl_runtime_c

)
message("Project Name                      ${CMAKE_PROJECT_NAME}")
message("Build Type                        ${CMAKE_BUILD_TYPE}")
message("CMAKE_CXX_STANDARD                ${CMAKE_CXX_STANDARD}")
message("TESTS_DIR               ${TESTS_DIR}")
message("PKG_DIR                 ${PKG_DIR}")
message("PKG_SRC_DIR             ${PKG_SRC_DIR}")
message("WS_SRC_DIR              ${WS_SRC_DIR}")
message("BRIDGE_LIB_DIR          ${BRIDGE_LIB_DIR}")
message("JSON_LIB_DIR            ${JSON_LIB_DIR}")
message("TESTS_COMMON_DIR        ${TESTS_COMMON_DIR}")
message("TESTS_INCLUDE_DIRS      ${TESTS_INCLUDE_DIRS}")
message("ROBOT_DIR               ${ROBOT_DIR}")
message("FIRMWARE_DIR            ${FIRMWARE_DIR}")
message("FIRMWARE_SRC_DIR        ${FIRMWARE_SRC_DIR}")
message("FIRMWARE_COMMON_DIR     ${FIRMWARE_COMMON_DIR}")

include(CTest) 
enable_testing()

add_subdirectory(test_first_guess)
add_subdirectory(test_with_pico)

add_subdirectory(test_msgs)
add_subdirectory(test_buffers)
add_subdirectory(test_queue)
