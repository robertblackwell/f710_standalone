#include <cmath>
#include <cassert>
#define GNU_SOURCE
#include <cstdio>
#include <tuple>
#include "motion_control.h"
double round_to(double value, double precision = 1.0)
{
    return std::round(value / precision) * precision;
}
static IoBuffer::UPtr make_robot_message(double left_percent, double right_percent)
{
//    auto left_percent = (float)round(((float)(left) / (float)INT16_MAX) * 100.0);
//    auto right_percent = (float)round(((float)(right) / (float)INT16_MAX) * 100.0);
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "pwm %f %f \n ", left_percent, right_percent);
    iob->setSize(len);
    return iob;
}
using namespace non_ros_msgs;
using namespace rbl;
struct ThrottleLevel {
    int m_f710_value;
    double m_level;
    ThrottleLevel(): m_f710_value(0), m_level(0.0){}
    void set_value(int f710_value)
    {
        m_f710_value = f710_value;
        m_level = (((double)(f710_value) / (double)INT16_MAX) * 100.0);
    }
    [[nodiscard]] double get_level() const {
        return (double)(round_to(m_level, 1000.0));
    }
    [[nodiscard]] std::string as_string() const
    {
        char* bufptr;
        (void)asprintf(&bufptr,"%5.3f", get_level());
        auto s = std::string(bufptr);
        free(bufptr);
        return s;
    }
};

#ifdef CVQ_USE_HACK
void MotionControl::send_threadsafe(F710LeftRight::UPtr msg)
{
    auto * item = new HackUnion(std::move(msg));
    m_queue.put(item);
}
void MotionControl::send_threadsafe(TwoEncoderStatus::UPtr msg)
{
    auto * item = new HackUnion(std::move(msg));
    m_queue.put(item);
}
void MotionControl::send_threadsafe(FirmwareStartupResponse::UPtr msg)
{
    auto * item = new HackUnion(std::move(msg));
    m_queue.put(item);
}
void MotionControl::run(const std::function<void(IoBuffer::UPtr iobuptr)>& msg_callback)
{
    while(true) {
        auto item = m_queue.get_wait();
        assert(item != nullptr);
        switch (item->type) {
            case 1: {
                int left = item->m_f710_uptr->m_left;
                int right = item->m_f710_uptr->m_right;
                delete item;
                auto iobu = make_robot_message(left, right);
                msg_callback(std::move(iobu));
            }
            break;
            case 2:
                RBL_LOG_FMT("motion_control got a two_encoder_status\n");
                delete item;
                break;
            case 3:
                RBL_LOG_FMT("motion_control got a firmware startup\n");
                delete item;
                break;
        }
    }
}
#else
void MotionControl::send_threadsafe(F710LeftRight::UPtr msg)
{
    auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
    m_queue.put(std::move(item));
}
void MotionControl::send_threadsafe(TwoEncoderStatus::UPtr msg)
{
    auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
    m_queue.put(std::move(item));
}
void MotionControl::send_threadsafe(FirmwareStartupResponse::UPtr msg)
{
    auto item = std::make_unique<QueueItemUPtrVariant>(std::move(msg));
    m_queue.put(std::move(item));
}
void MotionControl::run(const std::function<void(IoBuffer::UPtr iobuptr)>& buffer_callback)
{
    /**
     *     these variables represent +/- percentage throttle to 3 decimal places
     *     so that 83.012% is represented by 83012
     */
     int delay_counter = 0;
    int f710_target_throttle_left = 0;
    int f710_target_throttle_right = 0;
    ThrottleLevel target_left_throttle{};
    ThrottleLevel target_right_throttle{};
    double actual_left_throttle{};
    double actual_right_throttle{};
    auto calculate_next_pwm = [&](int fleft, int fright) {
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
                    f710_target_throttle_left = tmp_f710_left;
                    f710_target_throttle_right = tmp_f710_right;
                    auto [left, right] = calculate_next_pwm(f710_target_throttle_left, f710_target_throttle_left);
                    auto iobu = make_robot_message(left, right);
                    actual_left_throttle = left;
                    actual_right_throttle = right;
                    RBL_LOG_FMT("left: %6.3f  right: %6.3f", left, right);
                    delay_counter = 4;
                    buffer_callback(std::move(iobu));
                }
        } else if(auto ec = std::get_if<TwoEncoderStatus::UPtr>(&*item)) {

            auto left_rpm = (*ec)->left.motor_rpm_estimate;
            auto right_rpm = (*ec)->right.motor_rpm_estimate;
            auto left_pwm = actual_left_throttle;
            auto right_pwm = actual_right_throttle;
            auto error = right_rpm - left_rpm;
            RBL_LOG_FMT("motion_control \nleft ticks: %ld  rpm: %f  \nright: ticks: %ld  rpm: %f\n error %f",
                        (*ec)->left.sample_sum, (*ec)->left.motor_rpm_estimate,
                        (*ec)->right.sample_sum, (*ec)->right.motor_rpm_estimate, error);
            delay_counter--;
            if((actual_left_throttle > 40.0) && (actual_right_throttle > 40.0) && (delay_counter == 0)) {
                // going forward at a reasonable seed
                if (error > 50.0) {
                    delay_counter = 1;
                    auto new_right_pwm = (right_pwm / right_rpm) * left_rpm;
                    actual_left_throttle = left_pwm;
                    actual_right_throttle = new_right_pwm;
                } else if (error < 50.0) {
                    delay_counter = 1;
                    auto new_left_pwm = (left_pwm / left_rpm) * right_rpm;
                    actual_left_throttle = new_left_pwm;
                    actual_right_throttle = right_pwm;
                }
            }
            auto iobuptr = make_robot_message(actual_left_throttle, actual_right_throttle);
            RBL_LOG_FMT("left: %6.3f  right: %6.3f", actual_left_throttle, actual_right_throttle);
            buffer_callback(std::move(iobuptr));
            item = nullptr;

        } else if(auto fs = std::get_if<FirmwareStartupResponse::UPtr>(&*item)) {
        
                RBL_LOG_FMT("motion_control got a firmware startup %s\n", (*fs)->text.c_str());
                item = nullptr;
        
        } else {
            throw std::runtime_error("not one of the legal types");
        }   
    }
}
#endif


#include "motion_control.h"
