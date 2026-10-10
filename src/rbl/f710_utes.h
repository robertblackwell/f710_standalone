#ifndef H_F710_UTES_H
#define H_F710_UTES_H
#define GNU_SOURCE
#include <cstdio>
#include <thread>
#include <climits>
#include <rbl/f710_time.h>
#include <rbl/f710_exceptions.h>
#include <rbl/std_format.h>
#include <rbl/iobuffer.h>
#include <pico_messages/msg_struct.h>
#include <pico_messages/msgs.h>
#include <bridge_lib/threaded/serial_link.h>
#include <bridge_lib/threaded/outputter.h>
#include <bridge_lib/serial_settings.h>
#include <rbl/simple_exit_guard.h>

using namespace bridge::threaded;
using namespace rbl;
namespace f710 {
    IoBuffer::UPtr make_robot_pid_message();
    IoBuffer::UPtr make_robot_pwm_message(int left, int right);
    IoBuffer::UPtr make_robot_velocity_message(int left, int right);
    IoBuffer::UPtr make_robot_read_encoder_message();
    std::string make_outputter_message(int left, int right);
    bool change_is_material(int old_left, int old_right, int new_left, int new_right, float margin);
    // void handle_incoming_msg(Outputter& outputter, rbl::IoBuffer::UPtr iob_uptr);
    void handle_incoming_msg(std::function<void(std::string)> send_threadsafe, IoBuffer::UPtr iob_uptr);
} // namespace
#endif