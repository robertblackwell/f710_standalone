add_executable(serial_test_app
        main.cpp
        outputter.cpp
        outputter.h
        ${F710_DIR}/src/f710_time.h
        ${F710_DIR}/src/f710.h
        ${F710_DIR}/src/f710.cpp
        ${F710_DIR}/src/f710_helpers.cpp
        ${F710_DIR}/src/f710_helpers.h

        ${F710_DIR}/rbl/logger.cpp
        ${F710_DIR}/rbl/logger.h
        ${F710_DIR}/rbl/iobuffer.h
        ${F710_DIR}/rbl/iobuffer.cpp
        ${F710_DIR}/rbl/queue.h
        ${F710_DIR}/rbl/iobuffer.h
        ${F710_DIR}/rbl/std_format.cpp
        ${F710_DIR}/rbl/std_format.h

        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_link.h
        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_link.cpp
        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_settings.h
        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_settings.cpp
        ${SERIAL_BRIDGE_DIR}/bridge_lib/fd_functions.h
        ${SERIAL_BRIDGE_DIR}/bridge_lib/fd_functions.cpp

)
target_include_directories(serial_test_app
        PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/f710_control_v1
        ${PROJECT_SOURCE_DIR}/f710
        ${PROJECT_SOURCE_DIR}/f710/src
        ${PROJECT_SOURCE_DIR}/ros2-serial-bridge
        ${PROJECT_SOURCE_DIR}/ros2-serial-bridge/bridge_lib
)
