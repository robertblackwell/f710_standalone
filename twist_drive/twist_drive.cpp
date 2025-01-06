#include <cmath>
#include <cassert>
#define GNU_SOURCE
#include <cstdio>
#include "twist_drive.h"
#include "helpers.h"

//enum class TwistState {
//STARTUP,
//WAITING_FOR_PICO_TO_BOOT,
//STATIONARY,
//FORWARD_STRAIGHT,
//REVERSE_STRAIGHT,
//FORWARD_TURNING_RIGHT,
//FORWARD_TURNING_LEFT,
//REVERSE_TURNING_RIGHT,
//REVERSE_TURNING_LEFT,
//TRANSITION_TURNING_TO_GS
//
//};
//#define GS_STARTUP 9
//#define GS_WAITING_FOR_PICO_TO_BOOT 11
//#define GS_STATIONARY 1
//#define GS_FORWARD_STRAIGHT 2
//#define GS_REVERSE_STRAIGHT 5
//#define GS_FORWARD_TURNING_RIGHT 3
//#define GS_FORWARD_TURNING_LEFT 3
//#define GS_REVERSE_TURNING_RIGHT 3
//#define GS_REVERSE_TURNING_LEFT 3
//#define GS_TRANSITION_TURNING_TO_GS 4

#define M_SKIP_ENCODER_STATUS_MAX_COUNT 1
#define ENCODER_STATUS_DELAY_MS 2000
#define ENCODER_STATUS_INTERVAL_MILLI_SECS 500
#define F710_MAX_FORWARD 32767    // INT16_MAX
#define F710_MAX_REVERSE -32767   // -32767 this is not INT16_MIN

using namespace non_ros_msgs;
using namespace rbl;
namespace twist_drive {
    void TwistDrive::send_threadsafe(F710LeftRight::UPtr msg) {
        auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
        m_queue.put(std::move(item));
    }

    void TwistDrive::send_threadsafe(TwoEncoderStatus::UPtr msg) {
        auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
        m_queue.put(std::move(item));
    }

    void TwistDrive::send_threadsafe(FirmwareStartupResponse::UPtr msg) {
        auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
        m_queue.put(std::move(item));
    }

    bool TwistDrive::is_throttle_change(F710LeftRight &lr) const {
        return (lr.m_left != f710_target_throttle_left) || (lr.m_right != f710_target_throttle_right);
    };

    void TwistDrive::handle_turn_while_turning(F710LeftRight &lr) {
        m_state = TwistState::GS_TURNING;
        PwmResult r = calculate_first_pwm(lr.m_left, lr.m_right);
        actual_left_throttle = r.left;
        actual_right_throttle = r.right;
        m_ratio = r.ratio;
        m_error = r.error;
        auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
        m_buffer_callback(std::move(iobu));
    };

    void TwistDrive::handle_turn_while_going_straight(F710LeftRight &lr) {
        m_state = GS_TURNING;

        auto iob = make_robot_stream_samples_off_command();
        m_buffer_callback(std::move(iob));

        f710_target_throttle_left = lr.m_left;
        f710_target_throttle_right = lr.m_right;
        PwmResult r = calculate_first_pwm(lr.m_left, lr.m_right);
        actual_left_throttle = r.left;
        actual_right_throttle = r.right;
        m_ratio = r.ratio;
        m_error = r.error;
        auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
        m_buffer_callback(std::move(iobu));
    };

    void TwistDrive::handle_turn_while_stopped(F710LeftRight &lr) {
        m_state = GS_TURNING;
        PwmResult r = calculate_first_pwm(lr.m_left, lr.m_right);
        actual_left_throttle = r.left;
        actual_right_throttle = r.right;
        m_ratio = r.ratio;
        m_error = r.error;
        auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
        m_buffer_callback(std::move(iobu));
    };

    void TwistDrive::handle_gostraight_while_turning(F710LeftRight &lr) {
        handle_gostraight_while_stopped(lr);
    };

    void TwistDrive::handle_gostraight_while_going_straight(F710LeftRight &lr) {
        if (is_throttle_change(lr) && is_max_straight(lr)) {
            // forward reverse or visa versa
        } else {
            // come back to this
        }
    };

    void TwistDrive::handle_gostraight_while_stopped(F710LeftRight &lr) {
        m_state = GS_GO_STRAIGHT;
        f710_target_throttle_left = lr.m_left;
        f710_target_throttle_right = lr.m_right;
        PwmResult r = calculate_first_pwm(lr.m_left, lr.m_right);
        actual_left_throttle = r.left;
        actual_right_throttle = r.right;
        m_ratio = r.ratio;
        m_error = r.error;
        auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
        m_buffer_callback(std::move(iobu));
        auto iob2 = make_robot_stream_samples_command(ENCODER_STATUS_INTERVAL_MILLI_SECS);
        m_buffer_callback(std::move(iob2));
        m_skip_encoder_status_count = ENCODER_STATUS_DELAY_MS / ENCODER_STATUS_INTERVAL_MILLI_SECS;
    };

    void TwistDrive::handle_stop_while_going_straight() {
        m_state = GS_STATIONARY;
        auto iob = make_robot_stream_samples_off_command();
        m_buffer_callback(std::move(iob));
        PwmResult r = calculate_first_pwm(0, 0);
        actual_left_throttle = r.left;
        actual_right_throttle = r.right;
        m_ratio = r.ratio;
        m_error = r.error;
        auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
        m_buffer_callback(std::move(iobu));
    };

    void TwistDrive::handle_stop_while_turning() {
        m_state = GS_STATIONARY;
        PwmResult r = calculate_first_pwm(0, 0);
        actual_left_throttle = r.left;
        actual_right_throttle = r.right;
        m_ratio = r.ratio;
        m_error = r.error;
        auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
        m_buffer_callback(std::move(iobu));
    };

    void TwistDrive::handle_encoder_update_while_going_straight(TwoEncoderStatus &ec) {
        if (m_skip_encoder_status_count > 0) {
            m_skip_encoder_status_count--;
            dump_encoder_status(ec);
            return;
        }
        auto left_rpm = ec.left.motor_rpm_estimate;
        auto right_rpm = ec.right.motor_rpm_estimate;
        printf("about to call calculate next pwm\n");
        PwmResult r = calculate_next_pwm(f710_target_throttle_left, f710_target_throttle_right,
                                         actual_left_throttle, actual_right_throttle,
                                         left_rpm, right_rpm);
        actual_left_throttle = r.left;
        actual_right_throttle = r.right;
        m_ratio = r.ratio;
        m_error = r.error;
        auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
        m_buffer_callback(std::move(iobu));

    };

    void TwistDrive::handle_reboot_pico() {
        m_state = TwistState::STARTUP;
        auto iobu = make_robot_reboot_command();
        m_buffer_callback(std::move(iobu));
    }

    void TwistDrive::handle_pico_has_rebooted() {
        m_state = TwistState::WAITING_FOR_PICO_TO_BOOT;
    }

    TwistCommand f710LeftRightToTwist(F710LeftRight::UPtr lr)
    {
        if(lr->m_left == 0 && lr->m_right == 0) {
            return {TwistCategory::TC_STOP, 0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        } else if(lr->m_left == lr->m_right && lr->m_left > 0) {
            // go straight foward
            auto pwm = calculate_first_pwm(lr->m_left, lr->m_right);
            return {TwistCategory::TC_FORWARD_STRAIGHT, lr->m_left, lr->m_right, 0.0,
                    0.0, 0.0, 0.0, pwm.left, pwm.right};
        } else if(lr->m_left == lr->m_right && lr->m_left < 0) {
            // reverese straight
            auto pwm = calculate_first_pwm(lr->m_left, lr->m_right);
            return {TwistCategory::TC_FORWARD_STRAIGHT, lr->m_left, lr->m_right, 0.0,
                    0.0, 0.0, 0.0, pwm.left, pwm.right};
        } else if(lr->m_left > 0 && lr->m_right >= 0) {
            return {TwistCategory::TC_FORWARD_TURNING_RIGHT, lr->m_left, lr->m_right};
        } else if(lr->m_left < 0 && lr->m_right >= 0) {
            return {TwistCategory::TC_FORWARD_TURNING_LEFT, lr->m_left, lr->m_right};
        } else if(lr->m_left < 0 && lr->m_right < 0) {
            return {TwistCategory::TC_REVERSE_TURNING_LEFT, lr->m_left, lr->m_right};
        } else /*if(lr->m_left > 0 && lr->m_right < 0)*/ {
            return {TwistCategory::TC_REVERSE_TURNING_RIGHT,lr->m_left, lr->m_right};
        }
    }
    void TwistDrive::handle_firmware_startup()
    {
        switch (m_state) {
            case TwistState::STARTUP: {
                handle_reboot_pico();
            }
                break;
            case TwistState::WAITING_FOR_PICO_TO_BOOT: {
                handle_pico_has_rebooted();
            }
                break;
            default:
                throw std::runtime_error("");
        }
    }
    void TwistDrive::handle_leftright()
    {
        switch (m_state) {
            case TwistState::STARTUP:
            case TwistState::WAITING_FOR_PICO_TO_BOOT:
                throw std::runtime_error("");
                break;
            case TwistState::STATIONARY: {
            }
                break;
            case TwistState::FORWARD_STRAIGHT: {
            }
                break;
            case TwistState::REVERSE_STRAIGHT: {
            }
                break;
            case TwistState::FORWARD_TURNING_LEFT: {
            }
                break;
            case TwistState::FORWARD_TURNING_RIGHT: {
            }
                break;
            case TwistState::REVERSE_TURNING_LEFT:{
            }
                break;
            case TwistState::REVERSE_TURNING_RIGHT:{
            }
                break;
            case TwistState::TRANSITION_TURNING_TO_GS:
                break;
        }

    }
    void TwistDrive::handle_encoder_status()
    {
        switch (m_state) {
            case TwistState::STARTUP:
            case TwistState::WAITING_FOR_PICO_TO_BOOT:
                throw std::runtime_error("");
                break;
            case TwistState::STATIONARY: {
            }
                break;
            case TwistState::FORWARD_STRAIGHT: {
            }
                break;
            case TwistState::REVERSE_STRAIGHT: {
            }
                break;
            case TwistState::FORWARD_TURNING_LEFT: {
            }
                break;
            case TwistState::FORWARD_TURNING_RIGHT: {
            }
                break;
            case TwistState::REVERSE_TURNING_LEFT:{
            }
                break;
            case TwistState::REVERSE_TURNING_RIGHT:{
            }
                break;
            case TwistState::TRANSITION_TURNING_TO_GS:
                break;
        }

    }
    void TwistDrive::run(std::function<void(IoBuffer::UPtr iobuptr)> buffer_callback) {
        m_buffer_callback = std::move(buffer_callback);
        m_state = TwistState::WAITING_FOR_PICO_TO_BOOT;
        auto iobu = make_robot_reboot_command();
        m_buffer_callback(std::move(iobu));
        while (true) {
            auto item = m_queue.get_wait();
            switch(item->type) {
                case HackUnion::Type::FirmwareStartup:
                    handle_firmware_startup();
                    break;
                case HackUnion::Type::LeftRight:
                    handle_leftright();
                    break;
                case HackUnion::Type::EncoderStatus:
                    handle_encoder_status();
                    break;
            }
        }
    }

    void TwistDrive::dump_encoder_status(TwoEncoderStatus &ec) {
        auto left_latest_rpm = ec.left.motor_rpm_estimate;
        auto right_latest_rpm = ec.right.motor_rpm_estimate;
        auto left_latest_actual = actual_left_throttle;
        auto right_latest_actual = actual_right_throttle;

        RBL_LOG_FMT("calculate_next_pwm \n"
                    "\tleft  rpm: %f  " "\tactual_left_throttle:  %f \n"
                    "\tright rpm: %f  " "\tactual_right_throttle: %f \n"
                    "\tration: %f \terror %f \t m_skip_encoder_count: %d \n",
                    left_latest_rpm, left_latest_actual,
                    right_latest_rpm, right_latest_actual,
                    m_ratio, m_error, m_skip_encoder_status_count
        );
    }

#if 0
    void TwistDrive::run2(const std::function<void(IoBuffer::UPtr iobuptr)>& buffer_callback)
    {
        /**
         *     these variables represent +/- percentage throttle to 3 decimal places
         *     so that 83.012% is represented by 83012
         */
        int delay_counter = 0;
        bool go_straight_on = false;
        int f710_target_throttle_left = 0;
        int f710_target_throttle_right = 0;
        double right_target_throttle_percent = 0.0;
        double left_target_throttle_percent = 0.0;
        double actual_left_throttle{};
        double actual_right_throttle{};
        auto calculate_first_pwm = [&](int fleft, int fright) {
            struct result{double left; double right;};
            double l = (((double)fleft) / (double)INT16_MAX) * 100.0;
            double r = (((double)fright) / (double)INT16_MAX) * 100.0;
            return result {l, r};
        };
        while(true) {
            auto item = m_queue.get_wait();
    //        auto i = item->index();
            if(auto leftright = std::get_if<F710LeftRight::UPtr>(&*item)){
                    auto lr = *(*leftright);
                    int tmp_f710_left = lr.m_left;
                    int tmp_f710_right = lr.m_right;
                    if((f710_target_throttle_left != tmp_f710_left) || (f710_target_throttle_right != tmp_f710_right)) {
                        // the throttle settings have changed
                        if((tmp_f710_right == tmp_f710_left)&& is_max_f710_throttle(tmp_f710_left)) {
                            // start the go-straight process
                            f710_target_throttle_left = tmp_f710_left;
                            f710_target_throttle_right = tmp_f710_right;
                            auto [left, right] = calculate_first_pwm(f710_target_throttle_left, f710_target_throttle_right);
                            auto iobu = make_robot_message(left, right);
                            actual_left_throttle = left;
                            actual_right_throttle = right;
                            RBL_LOG_FMT("left: %6.3f  right: %6.3f", left, right);
                            buffer_callback(std::move(iobu));
                            auto iob2 = make_robot_stream_samples_command(1000);
                            buffer_callback(std::move(iob2));
                        } else {
                            if(go_straight_on) {
                                go_straight_on = false;
                                // other stuff to get out of go-straight smoothly
                            }
                            f710_target_throttle_left = tmp_f710_left;
                            f710_target_throttle_right = tmp_f710_right;
                            auto [left, right] = calculate_first_pwm(f710_target_throttle_left, f710_target_throttle_right);
                            auto iobu = make_robot_message(left, right);
                            actual_left_throttle = left;
                            actual_right_throttle = right;
                            RBL_LOG_FMT("left: %6.3f  right: %6.3f", left, right);
                            delay_counter = 4;
                            buffer_callback(std::move(iobu));
                            auto iob2 = make_robot_stream_samples_off_command();
                            buffer_callback(std::move(iob2));
                        }
                    }
            } else if(auto ec = std::get_if<TwoEncoderStatus::UPtr>(&*item)) {
                printf("XXXXXXXXXXXX got an encoder status message\n");
                if(f710_target_throttle_left == 0 || f710_target_throttle_right == 0 ) {
                    continue;
                }
                auto left_rpm = (*ec)->left.motor_rpm_estimate;
                auto right_rpm = (*ec)->right.motor_rpm_estimate;

                auto [next_left_pwm, next_right_pwm] = calculate_next_pwm(f710_target_throttle_left, f710_target_throttle_right,
                                                        actual_left_throttle, actual_right_throttle,
                                                        left_rpm, right_rpm);
                actual_right_throttle = next_right_pwm;
                actual_left_throttle = next_left_pwm;
                auto iobu = make_robot_message(actual_left_throttle, actual_right_throttle);
                buffer_callback(std::move(iobu));
            } else if(auto fs = std::get_if<FirmwareStartupResponse::UPtr>(&*item)) {

                    RBL_LOG_FMT("motion_control got a firmware startup %s\n", (*fs)->text.c_str());
                    item = nullptr;

            } else {
                throw std::runtime_error("not one of the legal types");
            }
        }
    }
#endif
// bool is_max_f710_throttle(int value)
// {
//     return (value == INT16_MAX) || (value == INT16_MIN);
// }
// std::tuple<double, double> calculate_first_pwm(int f710_left, int f710_right) {
//     double l = (((double)f710_left) / (double)INT16_MAX) * 100.0;
//     double r = (((double)f710_right) / (double)INT16_MAX) * 100.0;
//     return std::tuple<double, double>{l, r};
// };

// std::tuple<double, double> calculate_next_pwm(
//         int left_throttle_target, int right_throttle_target,
//         double left_latest_actual, double right_latest_actual,
//         double left_latest_rpm, double right_latest_rpm
// ) {
//     std::tuple<double, double> result{};

//     if(left_throttle_target == 0 || right_throttle_target == 0 ) {
//         result = {left_latest_actual, right_latest_actual};
//     }
//     auto ratio = left_latest_rpm / right_latest_rpm;
//     if(ratio < 1.0) {
//         // left is the slower wheel
//         auto left_higher = (right_latest_rpm/left_latest_rpm)*left_latest_actual;
//         if(left_higher <= 100.0) {
//             result = {left_higher, right_latest_actual};
//         } else {
//             auto right_lower = (left_latest_rpm/right_latest_rpm) * right_latest_actual;
//             result = {left_latest_actual, right_lower};
//         }
//     } else if (ratio > 1.0) {
//         // right is the slower wheel
//         auto right_higher = (left_latest_rpm/right_latest_rpm)*right_latest_actual;
//         if(right_higher <= 100.0) {
//             result = {left_latest_actual, right_higher};
//         } else {
//             auto left_lower = (right_latest_rpm/left_latest_rpm)*left_latest_actual;
//             result = {left_lower, right_latest_actual};
//         }

//     } else {
//         result = {left_latest_actual, right_latest_actual};
//     }
//     RBL_LOG_FMT("calculate_next_pwm \n\tleft rpm: %f  \n\tright: rpm: %f\n\terror %f"
//                 "\n\tactual_left_throttle: %f \n\tactual_right_throttle: %f "
//                 "\n\tleft/right ratio: %f \n\tnext left_pwm: %f \n\t next right_pwm: %f",
//                 left_latest_rpm, right_latest_rpm, (right_latest_rpm-left_latest_rpm),
//                 left_latest_actual, right_latest_actual,
//                 ratio, std::get<0>(result), std::get<1>(result)
//     );
//     return result;
// };
} //namespace