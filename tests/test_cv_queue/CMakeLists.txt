add_executable(test_cv_queue
        test_main.cpp
        ../../src/cv_queue.cpp
        ../../src/cv_queue.h
        ../../ros2-serial-bridge/src/bridge_lib/std_format.cpp
)

target_include_directories(test_cv_queue
        PUBLIC SYSTEM
        ./
        ../../
        ../../src
        ../../../
        ../../../ros2-serial-bridge/src
)

add_test(test_cv_queue test_cv_queue)
