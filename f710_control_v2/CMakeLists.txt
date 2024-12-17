add_executable(f710_controller_v2
        main.cpp
        outputter.cpp
        outputter.h
        ${PROJECT_SOURCE_DIR}/motion_control/motion_control.cpp
        ${PROJECT_SOURCE_DIR}/motion_control/motion_control.h

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

        ${SERIAL_BRIDGE_DIR}/non_ros_messages/msg_struct.h
        ${SERIAL_BRIDGE_DIR}/non_ros_messages/msgs.h
        ${SERIAL_BRIDGE_DIR}/non_ros_messages/msgs.cpp


        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_link.h
        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_link.cpp
        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_settings.h
        ${SERIAL_BRIDGE_DIR}/bridge_lib/serial_settings.cpp
        ${SERIAL_BRIDGE_DIR}/bridge_lib/fd_functions.h
        ${SERIAL_BRIDGE_DIR}/bridge_lib/fd_functions.cpp

        ${MOTION_CONTROL_DIR}/motion_control.cpp
        ${MOTION_CONTROL_DIR}/motion_control.h
        ${MOTION_CONTROL_DIR}/helpers.cpp
        ${MOTION_CONTROL_DIR}/helpers.h

)
target_include_directories(f710_controller_v2
        PUBLIC
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/f710_control_v1
        ${PROJECT_SOURCE_DIR}/f710
        ${PROJECT_SOURCE_DIR}/f710/src
        ${PROJECT_SOURCE_DIR}/ros2-serial-bridge
        ${JSONCONS_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/ros2-serial-bridge/bridge_lib
)
