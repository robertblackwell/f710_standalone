#define GNU_SOURCE
#include <cstdio>
#include <thread>
#include <exception>
#include <climits>
#include <f710.h>
#include <non_ros_messages/msg_struct.h>
#include <non_ros_messages/msgs.h>
#include <twist_drive/twist_drive.h>
#include <rbl/std_format.h>
#include <rbl/iobuffer.h>
#include <bridge_lib/serial_link.h>
#include <bridge_lib/serial_settings.h>
#include <rbl/simple_exit_guard.h>
#include "outputter.h"



using namespace serial_bridge;
using namespace f710;
using namespace non_ros_msgs;
using namespace twist_drive;

IoBuffer::UPtr make_robot_message(int left, int right);
std::string make_outputter_message(int left, int right);

template<class... Ts> struct msg_overload : Ts... {using Ts::operator()...;};

int main()
{
    try {
        SerialLink      serial_link{};
        F710            f710{"js0"};
        Outputter       outputter{};
        TwistDrive      twist_control{};
        /**
         * BEWARE This callback will always run on thread tf710
         *
         * Takes left/right messages from the f710 thread:
         *
         * -    converts them to f710_message and passes them to the motion control thread.
        */
        std::function<void(int, int)> on_f710_update_callback = [&](int left, int right) -> void {
//            printf("left: %d right: %d\n", left, right);
            auto iob_uptr = make_robot_message(left, right);
            auto outputter_msg = make_outputter_message(left, right);
            std::unique_ptr<F710LeftRight> msg = std::make_unique<F710LeftRight>(left, right);
            twist_control.send_threadsafe(std::move(msg));
        };

        /****************************************************************************************************
         * BEWARE this function is called on the tserial thread
         *
         * Receives all messages comming from the robot micro controller.
         ****************************************************************************************************/
        SerialLink::OnRecvCallback on_incoming_message = [&](IoBuffer::UPtr iob_uptr) -> void {
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
                [&](non_ros_msgs::FirmwareStartupResponse::UPtr& msg){
                    twist_control.send_threadsafe(std::move(msg));
                    },
                [&](non_ros_msgs::TwoEncoderStatus::UPtr& msg){
                    twist_control.send_threadsafe(std::move(msg));
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

            std::string msg = std::string(iob_uptr->get_first_char_ptr());
            iob_uptr = nullptr;
            outputter.send_threadsafe(msg);
        };

        /**
         * BEWARE this function runs on the motion control thread
         */
        auto motion_control_callback = [&](IoBuffer::UPtr iobuptr){
            /* The motion controller gets IO Buffers to send to the serial thread.
             */
            RBL_LOG_FMT("%s", iobuptr->c_str())
            serial_link.send_threadsafe(std::move(iobuptr));
        };

        std::thread toutputter{[&]() -> void { outputter.run(); }};
        std::thread tmotion_controller{
            [&](){
                twist_control.run(motion_control_callback);
            }
        };
        std::thread tserial{[&]() -> void { serial_link.run(on_incoming_message); }};
        std::thread tf710{[&]() -> void { f710.run(on_f710_update_callback); }};

        tserial.join();
        tmotion_controller.join();
        tf710.join();
        toutputter.join();
        printf("after all joins \n");
    } catch(...) {
        printf("main catch \n");
    };

	printf("after serial thread start about to spin\n");
	return 0;
}

IoBuffer::UPtr make_robot_message(int left, int right)
{
    auto left_percent = (float)round(((float)(left) / (float)INT16_MAX) * 100.0);
    auto right_percent = (float)round(((float)(right) / (float)INT16_MAX) * 100.0);
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "pwm %f %f \n ", left_percent, right_percent);
    iob->setSize(len);
    return iob;
}
std::string make_outputter_message(int left, int right)
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
