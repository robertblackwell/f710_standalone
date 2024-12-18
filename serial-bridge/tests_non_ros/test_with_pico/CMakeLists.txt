
set(TARGET test_with_pico)
add_executable(${TARGET}
		test_with_pico.cpp
		${FBCONTROL_LIB_DIR}/custom.h
		${FBCONTROL_LIB_DIR}/custom.cpp
		${FBCONTROL_LIB_DIR}/value_with_direction.h
		${FBCONTROL_LIB_DIR}/motor_profile.h
		${FBCONTROL_LIB_DIR}/motor_profile.cpp
		${TESTS_COMMON_DIR}/unittest.h
		${TESTS_COMMON_DIR}/unittest.c
		${BRIDGE_LIB_DIR}/serial_settings.h
		${BRIDGE_LIB_DIR}/serial_settings.cpp
		${FIRMWARE_COMMON_DIR}/trace.h
		${FIRMWARE_COMMON_DIR}/trace.cpp
)
target_compile_definitions(${TARGET}
		PUBLIC
		FBCONTROL_JSON_OUTPUT_PATH="${CMAKE_CURRENT_LIST_DIR}/json.txt"
)
target_include_directories(${TARGET}
		PUBLIC
		${JSON_LIB_DIR}
		${FIRMWARE_COMMON_DIR}
		${TESTS_INCLUDE_DIRS}
)
target_link_libraries(${TARGET} /usr/lib/x86_64-linux-gnu/libserial.so)

add_test(${TARGET} ${TARGET})

