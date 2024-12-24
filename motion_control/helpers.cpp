#include <cstdint>
#include <cmath>
#define RBL_LOG_ENABLED
#define RBL_LOG_ALLOW_GLOBAL
#include <rbl/logger.h>
#include <rbl/iobuffer.h>
#include <non_ros_messages/msg_struct.h>
#include <non_ros_messages/msgs.h>
#include "helpers.h"
#define F710_MAX_FORWARD (32767)    // INT16_MAX
#define F710_MAX_REVERSE (-32767)   // -32767 this is not INT16_MIN

using namespace rbl;


double round_to(double value, double precision)
{
    return std::round(value / precision) * precision;
}

IoBuffer::UPtr make_robot_message(double left_percent, double right_percent)
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "pwm %f %f \n ", left_percent, right_percent);
    iob->setSize(len);
    return iob;
}
IoBuffer::UPtr make_robot_stream_samples_command(double interval_ms)
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "es %d \n ", (int)interval_ms);
    iob->setSize(len);
    return iob;
}
IoBuffer::UPtr make_robot_stream_samples_off_command()
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "et \n ");
    iob->setSize(len);
    return iob;
}
IoBuffer::UPtr make_robot_reboot_command()
{
    IoBuffer::UPtr iob = std::make_unique<IoBuffer>(256);
    int len = snprintf(iob->get_first_char_ptr(), 256, "b \n ");
    iob->setSize(len);
    return iob;
}

bool is_max_f710_throttle(int value)
{
    return (value >= F710_MAX_FORWARD) || (value <= F710_MAX_REVERSE);
}
bool is_max_forward(F710LeftRight& lr)
{
    return is_max_forward(lr.m_left, lr.m_right);
}
bool is_max_reverse(F710LeftRight& lr)
{
    return is_max_reverse(lr.m_left, lr.m_right);
}
bool is_max_straight(F710LeftRight& lr)
{
    return is_max_straight(lr.m_left, lr.m_right);
};
bool is_stop(F710LeftRight& lr)
{
    return is_stop(lr.m_left, lr.m_right);
};
bool is_max_straight(int f710_left, int f710_right)
{
    return is_max_forward(f710_left, f710_right) || is_max_reverse(f710_left, f710_right);
}
bool is_side_max_forward(int f710_side)
{
    return (f710_side >= F710_MAX_FORWARD);
}
bool is_side_max_reverse(int f710_side)
{
    return (f710_side <= F710_MAX_REVERSE);
}
bool is_side_stop(int f710_side)
{
    return (f710_side == 0);
}
bool is_max_forward(int f710_left, int f710_right)
{
    return is_side_max_forward(f710_left) && is_side_max_forward(f710_right);
}
bool is_max_reverse(int f710_left, int f710_right)
{
    return is_side_max_reverse(f710_left) && is_side_max_reverse(f710_right);
}
bool is_stop(int f710_left, int f710_right)
{
    return is_side_stop(f710_left) && is_side_stop(f710_right);
}

/**
 * Converts a reading from a f710 control stick -32767 .. 32767 into a pwm percentage reading.
 *
 * Negative stick readings become -ve percentages.
 *
 * Percentages are in the range -100.0 .. -35.00 35.00 .. 100.0
 */
double f7102pwm(int f710_stick_reading)
{
    double value = 0;
    if(f710_stick_reading == 0)
        return value;
    if(f710_stick_reading > 0) {
        value = 35.0 + (
                std::min(1.0, (((double) f710_stick_reading) / (double) F710_MAX_FORWARD)) * (100.0 - 35.0));
        return value;
    }
    return -35.0 - (
                std::min(1.0, (((double) f710_stick_reading) / (double) F710_MAX_REVERSE)) * (100.0 - 35.0));
}

PwmResult calculate_first_pwm(int f710_left, int f710_right)
{
    double l, r;
    if(is_max_forward(f710_left, f710_right)) {
        r = 95.0;
        l = 100.0;
    } else if(is_max_reverse(f710_left, f710_right)) {
        l = -100.0;
        r = -95.0;
    } else {
        l = f7102pwm(f710_left);
        r = f7102pwm(f710_right);
    }
    printf("calculate_first_pwm f710_left: %d f710_right: %d pwm_left: %f pwm_right: %f\n",
            f710_left, f710_right, l, r);
    return {l, r, 0.95, 0};
};

PwmResult calculate_next_pwm(
        int left_throttle_target, int right_throttle_target,
        double left_latest_actual, double right_latest_actual,
        double left_latest_rpm, double right_latest_rpm
) {
    PwmResult result{};
    auto error = right_latest_rpm - left_latest_rpm;
    double ratio = 1.0;
    if(left_throttle_target == 0 || right_throttle_target == 0 ) {
        result = {left_latest_actual, right_latest_actual, 0.0, 0.0};
    } else {
        ratio = left_latest_rpm / right_latest_rpm;
        if(ratio < 1.0) {
            // left is the slower wheel
            auto left_higher = (right_latest_rpm/left_latest_rpm)*left_latest_actual;
            if(left_higher <= 100.0 && left_higher >= -100.00) {
                result = {left_higher, right_latest_actual, ratio, error};
            } else {
                auto right_lower = (left_latest_rpm/right_latest_rpm) * right_latest_actual;
                result = {left_latest_actual, right_lower, ratio, error};
            }
        } else if (ratio > 1.0) {
            // right is the slower wheel
            auto right_higher = (left_latest_rpm/right_latest_rpm)*right_latest_actual;
            if(right_higher <= 100.0 && right_higher >= -100.0) {
                result = {left_latest_actual, right_higher, ratio, error};
            } else {
                auto left_lower = (right_latest_rpm/left_latest_rpm)*left_latest_actual;
                result = {left_lower, right_latest_actual, ratio, error};
            }

        } else {
            result = {left_latest_actual, right_latest_actual, ratio, error};
        }
    }
    RBL_LOG_FMT("calculate_next_pwm \n" 
        "\tleft  rpm: %f " "\tactual_left_throttle : %f " "\tnext left_pwm: %f \n"
        "\tright rpm: %f " "\tactual_right_throttle: %f " "\tnext right_pwm: %f \n"
        "\tleft/right ratio: %f " "\terror %f\n", 
        left_latest_rpm,  left_latest_actual,  result.left,
        right_latest_rpm, right_latest_actual, result.right,
        ratio, (right_latest_rpm-left_latest_rpm)
    );
    return result;
};
