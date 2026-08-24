#define GNU_SOURCE
#include <stdio.h>
#include <thread>
#include <print>
#include <climits>
#include <f710.h>
#include <format>
#include <rbl/std_format.h>
#include <rbl/iobuffer.h>
#include <non_ros_messages/msg_struct.h>
#include <non_ros_messages/msgs.h>
#include <bridge_lib/serial_link.h>
#include <bridge_lib/serial_settings.h>
#include <rbl/simple_exit_guard.h>
#include "outputter.h"



using namespace serial_bridge;
using namespace f710;
using namespace rbl;

static IoBuffer::UPtr make_robot_pid_message();
static IoBuffer::UPtr make_robot_pwm_message(int left, int right);
static IoBuffer::UPtr make_robot_velocity_message(int left, int right);
static IoBuffer::UPtr make_robot_read_encoder_message();
static std::string make_outputter_message(int left, int right);
static bool change_is_material(int old_left, int old_right, int new_left, int new_right, float margin);
static void handle_incoming_msg(Outputter& outputter, IoBuffer::UPtr iob_uptr);

int main()
{
    auto velocity_messages = true;
    auto pid_active = false;
    try {
        SerialLink serial_link{};
        F710       f710{"js0"};
        Outputter  outputter{};
        
        int left_saved = 0;
        int right_saved = 0;

        /** BEWARE This callback will always run on thread tf710 */
        std::function<void(int, int)> on_f710_update_callback = [&](int left, int right) -> void {
            printf("left: %d right: %d\n", left, right);
            if(!pid_active) {
                pid_active = true;
                auto pid_msg = make_robot_pid_message();
                serial_link.send_threadsafe(std::move(pid_msg));
            }
            if(change_is_material(left_saved, right_saved, left, right, 0.001)) {
                left_saved = left;
                right_saved = right;
                auto iob_uptr = (velocity_messages)
                                ? make_robot_velocity_message(left, right)
                                : make_robot_pwm_message(left, right);
                auto outputter_msg = make_outputter_message(left, right);
                outputter.send_threadsafe(outputter_msg);
                serial_link.send_threadsafe(std::move(iob_uptr));
            }
            auto encoder_info_msg = make_robot_read_encoder_message();
            serial_link.send_threadsafe(std::move(encoder_info_msg));
        };

        /** BEWARE this function is called on the tserial thread*/
        SerialLink::OnRecvCallback on_incoming_message = [&](IoBuffer::UPtr iob_uptr) -> void {
            #if 1
            handle_incoming_msg(outputter, std::move(iob_uptr));
            #else
            std::string msg = std::string(iob_uptr->get_first_char_ptr());
            outputter.send_threadsafe(msg);
            #endif
        };
        
        std::thread toutputter{[&]() -> void { outputter.run(); }};
        std::thread tserial{[&]() -> void { serial_link.run(on_incoming_message); }};
        std::thread tf710{[&]() -> void { f710.run(on_f710_update_callback); }};
        tserial.join();
        tf710.join();
        toutputter.join();
        printf("after all joins \n");
    } catch(...) {
        printf("main catch \n");
    };

	printf("after serial thread start about to spin\n");
	return 0;
}

static IoBuffer::UPtr make_robot_pwm_message(int left, int right)
{
    auto left_percent = (float)round(((float)(left) / (float)INT16_MAX) * 100.0);
    auto right_percent = (float)round(((float)(right) / (float)INT16_MAX) * 100.0);
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "pwm %f %f \n ", left_percent, right_percent);
    iob->setSize(len);
    return iob;
}
static IoBuffer::UPtr make_robot_velocity_message(int left, int right)
{
    auto left_vel = ((float)(left) / (float)INT16_MAX) * 0.100;
    auto right_vel = ((float)(right) / (float)INT16_MAX) * 0.100;
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "v %.6f %.6f \n ", left_vel, right_vel);
    printf("v %.6f %f \n ", left_vel, right_vel);
    iob->setSize(len);
    return iob;
}
static IoBuffer::UPtr make_robot_pid_message()
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "p \n ");
    printf("pid  \n ");
    iob->setSize(len);
    return iob;
}
static IoBuffer::UPtr make_robot_read_encoder_message()
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "e \n");
    printf("encoder_read  \n ");
    iob->setSize(len);
    return iob;
}

static std::string make_outputter_message(int left, int right)
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
static bool change_is_material(int old_left, int old_right, int new_left, int new_right, float margin)
{
    auto left_flag = fabs((float)(old_left - new_left)/(float)old_left) > margin;
    auto right_flag = fabs((float)(old_right - new_right)/(float)old_right) > margin;
    return left_flag || right_flag;
}
template<class... Ts> struct msg_overload : Ts... {using Ts::operator()...;};

static void handle_incoming_msg(Outputter& outputter, IoBuffer::UPtr iob_uptr)
{
    
    auto msg_uptr = deserialize(*iob_uptr);
    if(!msg_uptr) {
        throw std::runtime_error("invalid pico input message");
    }
    auto default_action = [&](){
        std::string msg = std::string(iob_uptr->get_first_char_ptr());
        outputter.send_threadsafe(msg);
    };
    // c++'s complicated pattern matching
    std::visit(msg_overload{
        [&](non_ros_msgs::FirmwareStartupResponse::UPtr& msg){default_action();},
        [&](non_ros_msgs::TwoEncoderStatus::UPtr& msg){
            auto fmtmsg = std::format("wheel speed m/s left: {} right: {} precent diff: {}\n", 
                msg->left.wheel_speed_mps, msg->right.wheel_speed_mps, 
                100.0*((msg->left.wheel_speed_mps - msg->right.wheel_speed_mps)/(msg->left.wheel_speed_mps)));
            outputter.send_threadsafe(fmtmsg);
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

    // std::string msg = std::string(iob_uptr->get_first_char_ptr());
    iob_uptr = nullptr;
    // outputter.send_threadsafe(msg);

}
