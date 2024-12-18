cmake_minimum_required(VERSION 3.8)
project(cpp_serial_bridge LANGUAGES CXX C  VERSION 0.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sample_interfaces REQUIRED)

set(SRC_LIST
		../bridge_lib/queue.h
		../bridge_lib/serial_link.h
		../bridge_lib/serial_link.cpp
		../bridge_lib/serial_settings.h
		../bridge_lib/serial_settings.cpp
		../bridge_lib/iobuffer.h
		../bridge_lib/iobuffer.cpp
		../bridge_lib/fd_functions.h
		../bridge_lib/fd_functions.cpp
		../non_ros_messages/msgs.h
		../non_ros_messages/msgs.cpp
)
set(INCLUDE_DIR ../include)
set(SRC_DIR ../src)
set(JSON_DIR    ${CMAKE_CURRENT_LIST_DIR}/jsoncons/include)

set(PKG_INCLUDE_DIRS
		../include
		../src
	${CMAKE_CURRENT_LIST_DIR}/jsoncons/include

)

set(INCLUDE_DIRS)
# bridge
add_executable(bridge bridge.cpp ${SRC_LIST})
target_include_directories(bridge PUBLIC
		../include
		../src
	${CMAKE_CURRENT_LIST_DIR}/jsoncons/include
)
ament_target_dependencies(bridge rclcpp std_msgs geometry_msgs sample_interfaces)

# bridge_test_cmds
add_executable(bridge_test_cmds bridge_test_cmds.cpp ${SRC_LIST})
target_include_directories(bridge_test_cmds PUBLIC
		../include
		../src
	${CMAKE_CURRENT_LIST_DIR}/jsoncons/include
)
ament_target_dependencies(bridge_test_cmds rclcpp std_msgs geometry_msgs sample_interfaces)

# bridge_speed_test
add_executable(bridge_speed_test bridge_speed_test.cpp ${SRC_LIST})
target_include_directories(bridge_speed_test PUBLIC
		../include
		../src
	${CMAKE_CURRENT_LIST_DIR}/jsoncons/include
)
ament_target_dependencies(bridge_speed_test rclcpp std_msgs geometry_msgs sample_interfaces)
target_compile_definitions(bridge_speed_test 
            PUBLIC
            BRIDGE_SPEED_TEST_MESSAGE_COUNT=30000
            BRIDGE_SPEED_TEST_VERBOSE=2
)

# publish_test_cmds
add_executable(publisher_test_cmds publisher_test_cmds.cpp ${SRC_LIST})
target_include_directories(publisher_test_cmds PUBLIC
		../include
		../src
	${CMAKE_CURRENT_LIST_DIR}/jsoncons/include
)
ament_target_dependencies(publisher_test_cmds rclcpp std_msgs geometry_msgs sample_interfaces)
target_compile_definitions(publisher_test_cmds 
            PUBLIC
            BRIDGE_SPEED_TEST_MESSAGE_COUNT=30000
            BRIDGE_SPEED_TEST_VERBOSE=2
)


install(TARGETS 
	bridge
	bridge_speed_test
	bridge_test_cmds
	publisher_test_cmds
	DESTINATION lib/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}_test_01 test/test_01.cpp)
#   target_include_directories(${PROJECT_NAME}_test_01 PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
#   )
#   ament_target_dependencies(${PROJECT_NAME}_test_01  std_msgs)
#   # ament_target_link_libraries(${PROJECT_NAME}_test_01)# name_of_local_library)
# endif()

ament_package()
