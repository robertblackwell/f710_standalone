#include "custom.h"

#ifdef __amd64__
#include <stdexcept>
#define FATAL_ERROR_MSG(m) throw std::runtime_error(m);
#else
#include <trace.h>
#endif

CustomController::CustomController(Algorithm algorithm, MotorSide side):
        m_target(ValueWithDirection{}), m_algorithm(algorithm), m_profile(MotorProfile(side, get_pwm(side), get_rpm()))
{
    m_error = 0.0;
    m_side = side;
    m_previous_error = 0.0;
    m_yn_1 = 0.0;
    m_yn = 0.0;
    m_xn_1 = 0.0;
    m_xn = 0.0;
    m_en = 0.0;
    m_en_1 = 0.0;
    m_direction = 1;
    m_count;
}
auto CustomController::set_target(ValueWithDirection new_target) -> ValueWithDirection
{
    m_count = 0;
    m_target = new_target;
    auto new_x = m_profile.guess_pwm(m_target.abs_value());
    m_xn = new_x;
    m_yn = 0.0;
    auto r = new_x < 100.0 ? new_x: 100.0;
    m_count = 1;
    return ValueWithDirection(new_x);

}
auto CustomController::get_target() -> ValueWithDirection
{
    return m_target;
}

auto CustomController::update_linear(ValueWithDirection latest_y) -> ValueWithDirection
{
    FATAL_ERROR_MSG("porportional algorithm not implemented")
//    throw std::runtime_error("porportional algorithm not implemented");
    return 0.0;
    double new_x;
    m_yn = latest_y.abs_value();
    m_error = m_target.abs_value() - latest_y.abs_value();
    double error_percent = m_error / m_target.abs_value();

    if(std::abs(m_error) >= m_profile.m_error_tolerance) {
        double slope1 = (m_yn - m_yn_1) / (m_xn - m_xn_1);
        double slope2 = m_profile.slope_at_rpm(m_target.abs_value());
        double slope3 = m_profile.slope_at_pwm(m_xn);
        double slope = slope2/* one of these*/;
        new_x =  (m_target.abs_value() - m_yn) / slope + m_xn;
        m_xn_1 = m_xn;
        m_yn_1 = m_yn;
        m_xn = new_x;
    } else {
        new_x = m_xn;
    }

}
auto CustomController::update_fixed_step(ValueWithDirection latest_y) -> ValueWithDirection
{
    double new_x;
    m_yn = latest_y.abs_value();
    m_error = m_target.abs_value() - latest_y.abs_value();
    double error_percent = m_error / m_target.abs_value();

    if(std::abs(m_error) < 0.5 * std::abs(m_profile.m_error_tolerance)) {
        printf("HALF m_error %f tolerance: %f\n", m_error, m_profile.m_error_tolerance);
    } else
    if(std::abs(m_error) < std::abs(m_profile.m_error_tolerance)) {
        printf("FULL m_error %f tolerance: %f\n", m_error, m_profile.m_error_tolerance);
    } else
    if(std::abs(m_error) < 2.0 * std::abs(m_profile.m_error_tolerance)) {
        printf("TWICE m_error %f tolerance: %f\n", m_error, m_profile.m_error_tolerance);
    }

    if(m_error > 2 * m_profile.m_error_tolerance) {
        new_x = m_xn + 2 * m_profile.m_fixed_step_size;
    } else if(m_error < -2.0 * m_profile.m_error_tolerance) {
        new_x = m_xn - 2 * m_profile.m_fixed_step_size;

    } else if(m_error > m_profile.m_error_tolerance) {
        new_x = m_xn + m_profile.m_fixed_step_size;
    } else if(m_error < -1.0 * m_profile.m_error_tolerance) {
        new_x = m_xn - m_profile.m_fixed_step_size;

    } else if(m_error > 0.5 * m_profile.m_error_tolerance) {
        new_x = m_xn + 0.5 * m_profile.m_fixed_step_size;
    } else if(m_error < -0.5 * m_profile.m_error_tolerance) {
        new_x = m_xn - 0.5 * m_profile.m_fixed_step_size;

    } else {
        // dont change the x estimate - signal by returning nothing
        new_x = m_xn;
    }
    // the new_x value was changed. Ensure its within range
    // and then return it
    m_xn = new_x;
    m_previous_error = m_error;
    new_x = (new_x > m_profile.m_min_pwm) ? new_x : m_profile.m_min_pwm;
    double r = (new_x <= m_profile.m_max_pwm) ? new_x : m_profile.m_max_pwm;
    if(r == 0.0) {
        printf("Hello");
    }
    ValueWithDirection  v(m_target.sign() * r);
    return ValueWithDirection{v};
}
auto CustomController::update_proportional(ValueWithDirection latest_y) -> ValueWithDirection
{
    FATAL_ERROR_MSG("porportional algorithm not implemented");
//    throw std::runtime_error("porportional algorithm not implemented");
    return 0.0;

    double new_x;
    m_yn = latest_y.abs_value();
    m_error = m_target.abs_value() - latest_y.abs_value();
    double error_percent = m_error / m_target.abs_value();

    if(std::abs(m_error) / m_target.abs_value() > m_profile.m_proportional_error_tolerance_factor) {
        new_x = m_xn * m_profile.m_proportional_up_multiplier;
    } else if(m_error / m_target.abs_value() < -0.1) {
        new_x = m_xn * m_profile.m_proportional_down_multiplier;
    } else {
        new_x = m_xn;
    }
}
auto CustomController::update(ValueWithDirection latest_y) -> ValueWithDirection
{
    double new_x = 0.0;
    if(m_target.abs_value() == 0.0) {
        return ValueWithDirection{};
    }
    if(m_count == 0) {
        m_error = m_target.abs_value() - latest_y.abs_value();
        new_x = m_profile.guess_pwm(m_target.abs_value());
        m_xn = new_x;
        m_yn = 0.0;
        m_count++;
    } else if(1 <= m_count && m_count <= 3) {
        // skips the first few upates to let the motors settle down
        m_count++;
        new_x = m_xn;
    } else {
        switch(m_algorithm) {
            case Algorithm::linear:
                return update_linear(latest_y);
            case Algorithm::fixed_step:
                return update_fixed_step(latest_y);
            case Algorithm::proportional:
                return update_proportional(latest_y);
            default:
                throw std::runtime_error("algorithm switch default");
        }
        m_previous_error = m_error;
        new_x = (new_x > m_profile.m_min_pwm) ? new_x : m_profile.m_min_pwm;
    }
    double r = (new_x <= m_profile.m_max_pwm) ? new_x : m_profile.m_max_pwm;
    ValueWithDirection  v(m_target.sign() * r);
    return v;
}
