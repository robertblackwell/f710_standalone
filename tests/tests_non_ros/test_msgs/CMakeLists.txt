message("test_msgs CmakeList.txt  BRIDGE_LIB_DIR ${BRIDGE_LIB_DIR}")
set(TARGET test_msgs)
add_executable(${TARGET} 
	test_msgs.cpp
	${BRIDGE_LIB_DIR}/msgs.h 
	${BRIDGE_LIB_DIR}/msgs.cpp
	${BRIDGE_LIB_DIR}/iobuffer.h 
	${BRIDGE_LIB_DIR}/iobuffer.cpp
)
target_compile_definitions(${TARGET} 
	PUBLIC 
		LOCAL_TEST=1 
		PLATFORM_IS_LINUX=1, 
		TRANSPORT_MODE_PACKET=0
)

set(INSTALL_DIR ${WS_DIR}/install)

target_include_directories(${TARGET} 
	PUBLIC 
		/opt/ros/iron/include/rosidl_runtime_cpp
		/opt/ros/iron/include/rosidl_runtime_c
		${TESTS_INCLUDE_DIRS}
		${JSON_LIB_DIR}
		${PKG_SRC_DIR}
		${INSTALL_DIR}/sample_interfaces/include/sample_interfaces
		${LOCAL_TESTS_INCLUDE_PATHS})

add_test(${TARGET} ${TARGET})

