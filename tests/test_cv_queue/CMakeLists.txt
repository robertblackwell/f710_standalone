add_executable(test_cv_queue
        test_main.cpp
        ../../src/cv_queue.cpp
        ../../src/cv_queue.h
)

target_include_directories(test_cv_queue
        PUBLIC SYSTEM
        ./
        ../../src
)

add_test(test_cv_queue test_cv_queue)
