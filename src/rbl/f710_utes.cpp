#define GNU_SOURCE
#include <cstdio>
#include <thread>
#include <climits>
#include <rbl/f710_time.h>
#include <rbl/f710_exceptions.h>
#include "f710_utes.h"
#include <rbl/std_format.h>
#include <rbl/iobuffer.h>
#include <pico_messages/msg_struct.h>
#include <pico_messages/msgs.h>
#include <bridge_lib/threaded/serial_link.h>
#include <bridge_lib/threaded/outputter.h>
#include <bridge_lib/serial_settings.h>
#include <rbl/simple_exit_guard.h>

using namespace bridge::threaded;
// using namespace rbl;
// namespace f710 {
//     IoBuffer::UPtr make_robot_pid_message();
//     IoBuffer::UPtr make_robot_pwm_message(int left, int right);
//     IoBuffer::UPtr make_robot_velocity_message(int left, int right);
//     IoBuffer::UPtr make_robot_read_encoder_message();
//     std::string make_outputter_message(int left, int right);
//     bool change_is_material(int old_left, int old_right, int new_left, int new_right, float margin);
//     void handle_incoming_msg(Outputter& outputter, rbl::IoBuffer::UPtr iob_uptr);
// } // namespace

IoBuffer::UPtr f710::make_robot_pwm_message(int left, int right)
{
    const auto left_percent = static_cast<float>(round((static_cast<float>(left) / static_cast<float>(INT16_MAX)) * 100.0));
    const auto right_percent = static_cast<float>(round((static_cast<float>(right) / static_cast<float>(INT16_MAX)) * 100.0));
    auto iob = std::make_unique<IoBuffer>(256);
    const int len = snprintf(iob->get_first_char_ptr(), 256, "pwm %f %f \n ", left_percent, right_percent);
    iob->setSize(len);
    return iob;
}
IoBuffer::UPtr f710::make_robot_velocity_message(int left, int right)
{
    auto left_vel = ((float)(left) / (float)INT16_MAX) * 0.100;
    auto right_vel = ((float)(right) / (float)INT16_MAX) * 0.100;
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "v %.6f %.6f \n ", left_vel, right_vel);
    printf("v %.6f %f \n ", left_vel, right_vel);
    iob->setSize(len);
    return iob;
}
IoBuffer::UPtr f710::make_robot_pid_message()
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "p \n ");
    printf("pid  \n ");
    iob->setSize(len);
    return iob;
}
IoBuffer::UPtr f710::make_robot_read_encoder_message()
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "e \n");
    printf("encoder_read  \n ");
    iob->setSize(len);
    return iob;
}

std::string f710::make_outputter_message(int left, int right)
{
    char* bufptr; 
    asprintf(&bufptr, "left: %d  right: %d", left, right);
    auto s = std::string(bufptr);
    free(bufptr);
    return s;
//    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
//    int len = snprintf(iob->get_first_char_ptr(), 256, "received %d %d  ", left, right);
//    iob->setSize(len);
//    return iob;
}
bool f710::change_is_material(int old_left, int old_right, int new_left, int new_right, float margin)
{
    const auto left_flag = fabs(static_cast<double>(old_left - new_left)/static_cast<double>(old_left)) > margin;
    const auto right_flag = fabs(static_cast<double>(old_right - new_right)/static_cast<double>(old_right)) > margin;
    return left_flag || right_flag;
}
namespace {
    template<class... Ts> struct msg_overload : Ts... {using Ts::operator()...;};
}

void f710::handle_incoming_msg(std::function<void(std::string)> send_threadsafe, IoBuffer::UPtr iob_uptr)

{
    
    auto msg_uptr = deserialize(*iob_uptr);
    if(!msg_uptr) {
        throw std::runtime_error("invalid pico input message");
    }
    auto default_action = [&](){
        std::string msg = std::string(iob_uptr->get_first_char_ptr());
        send_threadsafe(msg);
    };
    // c++'s complicated pattern matching
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-parameter"
    // NOLINTBEGIN(misc-unused-parameters))
    std::visit(msg_overload{
        [&](non_ros_msgs::FirmwareStartupResponse::UPtr& msg){default_action();},
        [&](non_ros_msgs::TwoEncoderStatus::UPtr& msg){
            auto fmtmsg = std_format("wheel speed m/s left: {} right: {} precent diff:  {}\n",
                msg->left.wheel_speed_mps, msg->right.wheel_speed_mps, 
                100.0*((msg->left.wheel_speed_mps - msg->right.wheel_speed_mps)/(msg->left.wheel_speed_mps)));
                send_threadsafe(fmtmsg);
            },
        [&](non_ros_msgs::F710LeftRight::UPtr& ){default_action();},
        [&](non_ros_msgs::MotorPwmCmd::UPtr& ){default_action();},
        [&](non_ros_msgs::MotorRpmCmd    ::UPtr& ){default_action();},
        [&](non_ros_msgs::CmdResponse::UPtr& ){default_action();},
        [&](non_ros_msgs::EncoderStatus::UPtr& ){default_action();},
        [&](non_ros_msgs::LoadTestCmd::UPtr& ){default_action();},
        [&](non_ros_msgs::TextMsg::UPtr& ){default_action();},
        [&](non_ros_msgs::EchoCmd::UPtr& ){default_action();},
        [&](non_ros_msgs::ReadEncodersCmd::UPtr& ){default_action();},
    }, *msg_uptr);
    #pragma GCC diagnostic pop
    // NOLINTEND(misc-unused-parameters))

    // std::string msg = std::string(iob_uptr->get_first_char_ptr());
    iob_uptr = nullptr;
    // outputter.send_threadsafe(msg);

}
