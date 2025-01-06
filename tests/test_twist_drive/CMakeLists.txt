add_executable(test_twist_drive
        test_twist_drive.cpp
        ${TWIST_DRIVE_DIR}/interpret_f710_helpers.h
        ${TWIST_DRIVE_DIR}/interpret_f710_helpers.cpp
)

target_include_directories(test_twist_drive
        PUBLIC SYSTEM
        ${APP_INCLUDE_DIRS}
)

add_test(test_twist_drive test_cv_queue)
