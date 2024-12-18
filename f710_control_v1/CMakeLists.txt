add_executable(f710_controller_v1
        main.cpp
        outputter.cpp
        outputter.h
)
target_link_libraries(f710_controller_v1 PUBLIC ${APP_LIB})
target_include_directories(f710_controller_v1
        PUBLIC
        ${APP_INCLUDE_DIRS}
)
