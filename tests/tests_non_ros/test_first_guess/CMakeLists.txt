# CHECK WE HAVE ALL THE VARIABLES WE NEED
#cmake_minimum_required(VERSION 3.27)
#project(serialio)

set(CMAKE_CXX_STANDARD 20)
set( CMAKE_VERBOSE_MAKEFILE on )

set(TARGET test_first_guess)
add_executable(${TARGET}
		test_first_guess.cpp
		${FBCONTROL_LIB_DIR}/custom.h
		${FBCONTROL_LIB_DIR}/custom.cpp
		${TESTS_COMMON_DIR}/unittest.h
		${TESTS_COMMON_DIR}/unittest.c
		)

target_include_directories(${TARGET}
		PUBLIC
		${TESTS_INCLUDE_DIRS}
)

add_test(${TARGET} ${TARGET})


