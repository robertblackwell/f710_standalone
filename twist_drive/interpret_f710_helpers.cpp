#include <cstdint>
#include <cmath>
#define RBL_LOG_ENABLED
#define RBL_LOG_ALLOW_GLOBAL
#include <cstdio>
#include <stdexcept>
#include <rbl/logger.h>
#include "interpret_f710_helpers.h"
#define F710_MAX_FORWARD (32767)    // INT16_MAX
#define F710_MAX_REVERSE (-32767)   // -32767 this is not INT16_MIN

const double axle_length_mm = 220.0;
const double max_rpm = 7000.0;
const double pwm_at_max_rpm = 95.0;
const double min_rpm = 3500.0;
const double pwm_at_zero_rpm = 35.0;
const double wheel_diameter_mm = 70.0;
const double wheel_circumference_mm = (22.0 * wheel_diameter_mm) / 7.0;
const double max_wheel_speed_mm_per_sec = ((max_rpm * wheel_circumference_mm) / 60.0);
const double min_wheel_speed_mm_per_sec = ((min_rpm * wheel_circumference_mm) / 60.0);

const double max_angular_vel_rads_per_sec = (2.0 * max_wheel_speed_mm_per_sec)/axle_length_mm;

const double min_radius = 0.0;
const double radius_scale_factor = 10.0;

namespace twist_drive {

    struct F710Decoded {
        double velocity_ratios;
    };
    double rpm2wheelspeed(double rpm)
    {
        return (rpm * wheel_circumference_mm) / 60.0;
    }
    double wheelspeed2rpm(double ws)
    {
        return (ws * 60) / wheel_circumference_mm;
    }
    double rpm2pwm(double rpm)
    {
        auto max_pwm = 95.0;
        auto min_pwm = 35.0;
        if(rpm > max_rpm) {
            throw std::runtime_error("rpm is too big");
        }
        auto pwm = 35.0 + rpm * (max_pwm - min_pwm)/max_rpm;
        return pwm;
    }
    double pwm2rpm(double pwm)
    {
        if(pwm < 0.0 || pwm > 100.0) {
            throw std::runtime_error("pwm is out of range");
        }
        auto rpm = (pwm - pwm_at_max_rpm) * max_rpm /(pwm_at_max_rpm - pwm_at_zero_rpm);
        return rpm;
    }
    MotionResult interpret_f710(int turn_index, int velocity_index) {
        if (turn_index == 0) {
            return interpret_straight(velocity_index);
        } else if (turn_index < 0) {
            return interpret_turn_left(turn_index, velocity_index);
        } else {
            return interpret_turn_right(turn_index, velocity_index);
        }
    }

    MotionResult interpret_straight(int velocity_index) {
        auto v = (velocity_index == 0)
                ? 0.0
                : (max_wheel_speed_mm_per_sec - min_wheel_speed_mm_per_sec) * velocity_index + min_wheel_speed_mm_per_sec;
        auto left_rpm = 0.0;
        auto right_rpm = 0.0;
        return {
            0,
            velocity_index,
            1.0,
            0.0,
            1.0,
            0.0,
            v,
            left_rpm,
            v,
            right_rpm,
        };
    }

/**
 * Converts the turn_index and velocity_index into
 * the required velocity in mm/sec of the left and right wheel
 * as well as the corresponding rpm of the left and right motors.
 *
 * The conversion takes into account the max and minimum (non-zero) wheel speed that the robot
 * can practically achieve.
 *
 * This function is dependent on two config options:
 *
 * -    the first is sharpest turns or minimum radius of curvature, there are two possible values:
 *
 *      -   0 .. where the robot turns about the center of the drive axle
 *      -   axle_length / 2 .. where the robot turns about the inside wheel
 *
 * -    the second is how velocity is derived from the turn ratio and velocity index:
 *
 *      -   in the simlest case the velocity index is ignored. The outside wheel is given the maximum velocity
 *          and the inside wheels velocity is derived from the turn ratio.
 *
 *      -   the specs for the second method have not been finalized.
 *
 */
    MotionResult interpret_turn_left(int turn_index, int velocity_index) {
        auto radius = (turn_index == F710_MAX_REVERSE)
                      ? min_radius
                      : min_radius + radius_scale_factor * (1.0 / (double) -turn_index);

        auto v_ratio = (1.0 - 2.0 * axle_length_mm / (2.0 * radius + axle_length_mm));
        auto tmp_vel_left = v_ratio * max_wheel_speed_mm_per_sec;
        auto tmp_vel_right = max_wheel_speed_mm_per_sec;
        auto tmp_velocity = 0.5 * (tmp_vel_left + tmp_vel_right);
        auto tmp_angular_velocity = (tmp_vel_right - tmp_vel_left) / axle_length_mm;
        auto tmp_rpm_left = v_ratio * max_rpm;
        auto tmp_rpm_right = max_rpm;
        return {
                turn_index,
                velocity_index,
                tmp_velocity,
                tmp_angular_velocity,
                v_ratio,
                radius,
                tmp_vel_left,
                tmp_rpm_left,
                tmp_vel_right,
                tmp_rpm_right
        };
    }

    MotionResult interpret_turn_right(int turn_index, int velocity_index) {
        auto radius = (turn_index == F710_MAX_REVERSE)
                      ? min_radius
                      : min_radius + radius_scale_factor * (1.0 / (double) turn_index);

        auto v_ratio = (1.0 - 2.0 * axle_length_mm / (2.0 * radius + axle_length_mm));
        auto tmp_vel_right = v_ratio * max_wheel_speed_mm_per_sec;
        auto tmp_vel_left = max_wheel_speed_mm_per_sec;
        auto tmp_velocity = 0.5 * (tmp_vel_left + tmp_vel_right);
        auto tmp_angular_velocity = (tmp_vel_right - tmp_vel_left) / axle_length_mm;
        auto tmp_rpm_left = v_ratio * max_rpm;
        auto tmp_rpm_right = max_rpm;
        return {
                turn_index,
                velocity_index,
                tmp_velocity,
                tmp_angular_velocity,
                v_ratio,
                radius,
                tmp_vel_left,
                tmp_rpm_left,
                tmp_vel_right,
                tmp_rpm_right
        };
    }
} // namespace twist_drive

