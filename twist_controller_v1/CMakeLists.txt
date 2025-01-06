add_executable(f710_controller_v2
        main.cpp
        outputter.cpp
        outputter.h
        ${MOTION_CONTROL_SRC}
)
target_link_libraries(f710_controller_v2 PUBLIC ${APP_LIB})
target_include_directories(f710_controller_v2
        PUBLIC
        ${APP_INCLUDE_DIRS}
)
