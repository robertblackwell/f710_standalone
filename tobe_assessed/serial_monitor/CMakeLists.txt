
add_executable(serial_monitor main.cpp
        app_context.h
        background_thread.cpp
        background_thread.h
)
target_compile_options(serial_monitor PUBLIC ${GTK4_CFLAGS_OTHER})
target_include_directories(serial_monitor
        SYSTEM PUBLIC
        ${GTK4_INCLUDE_DIRS}
        ${APP_INCLUDE_DIRS}
)
target_link_directories(serial_monitor PUBLIC ${GTK4_LIBRARY_DIRS})
target_link_libraries(serial_monitor ${APP_LIB}  ${GTK4_LIBRARIES})

