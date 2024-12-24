#ifndef H_motion_control_helpers_h
#define H_motion_control_helpers_h
#include <tuple>
#include <rbl/iobuffer.h>
struct PwmResult {
	double left;
	double right;
	double ratio;
    double error;
};
/**
 * The following functions are predicates or tests to be applied to raw readings from an F710 controller.
 *
 * Note the output from the F710 control sticks is a pair of integers [left, right] where each
 * is in the range -32767 .. 32767
 */
bool is_max_f710_throttle(int value);
bool is_max_straight(F710LeftRight& lr);
bool is_max_forward(F710LeftRight& lr);
bool is_max_reverse(F710LeftRight& lr);
bool is_stop(F710LeftRight& lr);
bool is_max_straight(int f710_left, int f710_right);
bool is_max_forward(int f710_left, int f710_right);
bool is_max_reverse(int f710_left, int f710_right);
bool is_stop(int f710_left, int f710_right);

/**
 *  maps a raw f710 reading pair [f710_left, f710_right] to a pwm value in the range
 *  [-100.0 .. -35.0] or [0.0] or [35.0 .. 100.0].
 *
 *  And also applies the initial correction for the fact that the right motor is about 5% faster than the left motor.
 */
PwmResult calculate_first_pwm(int f710_left, int f710_right);

/**
 * Once the robot is up and running this function adjusts the left/right throttle setting based on feedback from
 * the robots encoders.
 */
PwmResult calculate_next_pwm(
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