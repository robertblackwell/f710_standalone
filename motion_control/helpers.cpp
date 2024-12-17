#include <cstdint>
#include <cmath>
#define RBL_LOG_ENABLED
#define RBL_LOG_ALLOW_GLOBAL
#include <rbl/logger.h>
#include <rbl/iobuffer.h>
#include "helpers.h"

using namespace rbl;

bool is_max_f710_throttle(int value)
{
    return (value == INT16_MAX) || (value == INT16_MIN);
}

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


std::tuple<double, double> calculate_first_pwm(int f710_left, int f710_right) {
    double l = (((double)f710_left) / (double)INT16_MAX) * 100.0;
    double r = (((double)f710_right) / (double)INT16_MAX) * 100.0;
    return std::tuple<double, double>{l, r};
};

std::tuple<double, double> calculate_next_pwm(
        int left_throttle_target, int right_throttle_target,
        double left_latest_actual, double right_latest_actual,
        double left_latest_rpm, double right_latest_rpm
) {
    std::tuple<double, double> result{};

    if(left_throttle_target == 0 || right_throttle_target == 0 ) {
        result = {left_latest_actual, right_latest_actual};
    }
    auto ratio = left_latest_rpm / right_latest_rpm;
    if(ratio < 1.0) {
        // left is the slower wheel
        auto left_higher = (right_latest_rpm/left_latest_rpm)*left_latest_actual;
        if(left_higher <= 100.0) {
            result = {left_higher, right_latest_actual};
        } else {
            auto right_lower = (left_latest_rpm/right_latest_rpm) * right_latest_actual;
            result = {left_latest_actual, right_lower};
        }
    } else if (ratio > 1.0) {
        // right is the slower wheel
        auto right_higher = (left_latest_rpm/right_latest_rpm)*right_latest_actual;
        if(right_higher <= 100.0) {
            result = {left_latest_actual, right_higher};
        } else {
            auto left_lower = (right_latest_rpm/left_latest_rpm)*left_latest_actual;
            result = {left_lower, right_latest_actual};
        }

    } else {
        result = {left_latest_actual, right_latest_actual};
    }
    RBL_LOG_FMT("calculate_next_pwm \n\tleft rpm: %f  \n\tright: rpm: %f\n\terror %f"
                "\n\tactual_left_throttle: %f \n\tactual_right_throttle: %f "
                "\n\tleft/right ratio: %f \n\tnext left_pwm: %f \n\t next right_pwm: %f",
                left_latest_rpm, right_latest_rpm, (right_latest_rpm-left_latest_rpm),
                left_latest_actual, right_latest_actual,
                ratio, std::get<0>(result), std::get<1>(result)
    );
    return result;
};
