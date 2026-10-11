message("test_msgs CmakeList.txt  BRIDGE_LIB_DIR ${BRIDGE_LIB_DIR}")
set(TARGET test_msgs)
add_executable(${TARGET} 
	test_msgs.cpp
	${PICO_MESSAGES_DIR}/msg_struct.h
	${PICO_MESSAGES_DIR}/serialize_command_msgs.cpp
		${PICO_MESSAGES_DIR}/deserialize_response_msgs.cpp
	${RBL_DIR}/iobuffer.h
	${RBL_DIR}/iobuffer.cpp
)
target_compile_definitions(${TARGET} 
	PUBLIC 
		LOCAL_TEST=1 
		PLATFORM_IS_LINUX=1, 
		TRANSPORT_MODE_PACKET=0
)
#
#set(INSTALL_DIR ${WS_DIR}/install)
#

target_include_directories(${TARGET}
	PUBLIC
		${JSONCONS_INCLUDE_DIR}
		${SRC_INCLUDE_DIR}
)
get_target_property(MY_INCLUDES ${TARGET} INCLUDE_DIRECTORIES)
message("JSONCONS_INCLUDE_DIR  [${JSONCONS_INCLUDE_DIR}]")
message("SRC_INCLUDE_DIR       [${SRC_INCLUDE_DIR}]")
message("Include Dirs [${MY_INCLUDES}]")
add_test(${TARGET} ${TARGET})

