add_executable(serial_test_app
        main.cpp
        outputter.cpp
        outputter.h
#        ${MOTION_CONTROL_SRC}
)
target_link_libraries(serial_test_app PUBLIC ${APP_LIB})
target_include_directories(serial_test_app
        PUBLIC
        ${APP_INCLUDE_DIRS}
)
