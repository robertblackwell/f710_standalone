#ifndef H_fbcontrol_motor_profile_h
#define H_fbcontrol_motor_profile_h
#include <vector>
#include <optional>
#include <functional>
#include <memory>

enum class MotorSide {left, right};

std::vector<double>& get_rpm();

std::vector<double>& get_pwm(MotorSide side);

struct MotorProfile
{
    using UPtr = std::unique_ptr<MotorProfile>;
    MotorProfile(MotorSide side, std::vector<double>& pwm, std::vector<double>& rpm);
    double guess_pwm(double rpm);
    double slope_at_pwm(double pwm);
    double slope_at_rpm(double rpm);
    std::optional<int> find_pwm_index(double pwm);
    std::optional<int> find_rpm_index(double pwm);


    MotorSide            m_side;
    std::vector<double>& m_pwm;
    std::vector<double>& m_rpm;
    float m_min_pwm;
    float m_max_pwm;
    float m_max_target;
    float m_min_target;
    float m_error_tolerance;
    float m_fixed_step_size;
    float m_proportional_error_tolerance_factor;
    float m_proportional_up_multiplier;
    float m_proportional_down_multiplier;
};


#endif