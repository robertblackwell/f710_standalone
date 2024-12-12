add_executable(test_cv_queue
        test_main.cpp
        ${F710_DIR}/rbl/cv_queue.cpp
        ${F710_DIR}/rbl/cv_queue.h
        ${F710_DIR}/rbl/std_format.cpp
)

target_include_directories(test_cv_queue
        PUBLIC SYSTEM
        ${PROJECT_SOURCE_DIR}/
        ${PROJECT_SOURCE_DIR}/f710_control_v1
        ${PROJECT_SOURCE_DIR}/f710
        ${PROJECT_SOURCE_DIR}/f710/rbl
        ${PROJECT_SOURCE_DIR}/ros2-serial-bridge
)

add_test(test_cv_queue test_cv_queue)
