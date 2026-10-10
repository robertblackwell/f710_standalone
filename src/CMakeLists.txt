# 1. Define the library without source files initially
add_library(applib STATIC "")
add_library(F710Project::f710lib ALIAS applib)

# 2. Configure public include directories for the target
target_include_directories(applib 
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    SYSTEM PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/include"
)

target_link_libraries(applib
    PUBLIC
        Boost::asio
        Boost::fiber
        Boost::context
        Boost::headers
)

target_compile_options(applib
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# 3. Add the subdirectories containing the actual source code
# add_subdirectory(bridge_lib)
# add_subdirectory(f710-v1)
# add_subdirectory(f710-v2)
add_subdirectory(pico_messages)
add_subdirectory(rbl)
add_subdirectory(app_asio)
add_subdirectory(app_threaded)