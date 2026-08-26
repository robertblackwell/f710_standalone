set(TARGET f710_v1_threaded_demo)
add_executable(${TARGET}
        main.cpp
)
target_link_libraries(${TARGET} PUBLIC applib)
#target_include_directories(${TARGET} PUBLIC ${APP_INCLUDE_DIRS})
