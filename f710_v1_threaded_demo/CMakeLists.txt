add_executable(f710_demo
        main.cpp
        outputter.cpp
        outputter.h
)
target_link_libraries(f710_demo PUBLIC ${APP_LIB})
target_include_directories(f710_demo
        PUBLIC
        ${APP_INCLUDE_DIRS}
)
