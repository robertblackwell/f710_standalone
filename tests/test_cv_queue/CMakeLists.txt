add_executable(test_cv_queue
        test_main.cpp
        ${RBL_DIR}/cv_queue.cpp
        ${RBL_DIR}/cv_queue.h
        ${RBL_DIR}/std_format.cpp
)

target_include_directories(test_cv_queue
        PUBLIC SYSTEM
        ${APP_INCLUDE_DIRS}
)

add_test(test_cv_queue test_cv_queue)
