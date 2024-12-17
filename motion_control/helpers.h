#ifndef H_motion_control_helpers_h
#define H_motion_control_helpers_h
#include <tuple>
#include <rbl/iobuffer.h>

bool is_max_f710_throttle(int value);
std::tuple<double, double> calculate_first_pwm(int f710_left, int f710_right);

std::tuple<double, double> calculate_next_pwm(
        int left_throttle_target, int right_throttle_target,
        double left_latest_actual, double right_latest_actual,
        double left_latest_rpm, double right_latest_rpm
);
double round_to(double value, double precision = 1.0);
rbl::IoBuffer::UPtr make_robot_message(double left_percent, double right_percent);
rbl::IoBuffer::UPtr make_robot_stream_samples_command(double interval_ms);
rbl::IoBuffer::UPtr make_robot_stream_samples_off_command();
rbl::IoBuffer::UPtr make_robot_reboot_command();

#endif