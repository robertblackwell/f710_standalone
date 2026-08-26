#ifndef H_twist_control_interpret_f710_helpers_h
#define H_twist_control_interpret_f710_helpers_h
#include <tuple>
namespace twist_drive {
    enum class TwistCategory{
        TC_STOP,
        TC_FORWARD_STRAIGHT,
        TC_REVERSE_STRAIGHT,
        TC_FORWARD_TURNING_RIGHT,
        TC_FORWARD_TURNING_LEFT,
        TC_REVERSE_TURNING_RIGHT,
        TC_REVERSE_TURNING_LEFT,
        TC_TRANSITION_TURNING_TO_GS,
    };
    struct TwistCommand {
        TwistCategory  category;
        long           left;
        long           right;
    };
    struct PwmResult {
        double left;
        double right;
        double ratio;
        double error;
    };
/**
 * An instance of this struct holds the result of converting the pair [turn_index, velocity_index]
 * from an F710 controller into required velocity and rpm for the wheels and motors of a differential
 * robot.
 */
    struct MotionResult {
        int f710_turn_index;
        int f710_velocity_index;
        double robot_linear_velocity;
        double robot_angular_velocity;
        double velocity_ratio;
        double radius_of_curvature;
        double left_velocity;
        double left_rpm;
        double right_velocity;
        double right_rpm;
    };

MotionResult interpret_straight(int velocity_index);
MotionResult interpret_turn_left(int turn_index, int velocity_index);
MotionResult interpret_turn_right(int turn_index, int velocity_index);

} //namespace
#endif