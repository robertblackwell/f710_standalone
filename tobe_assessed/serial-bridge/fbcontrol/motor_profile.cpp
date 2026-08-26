#include "motor_profile.h"

#ifdef __amd64__
#include <stdexcept>
#include <format>
#define FATAL_ERROR_MSG(m) throw std::runtime_error(m);
#else
#include <trace.h>
#endif

static std::vector<double> static_rpm_vec   =  {2000.0, 2500.0, 3000.0, 3500.0, 4000.0, 4500.0, 5000.0, 5500.0, 6000.0, 6500.0, 7000.0, 7500.0, 8000.0};
static std::vector<double> static_pwm_left  =  {41.4,   43.70,  47.90,  51.17,  55.35,  62.6,   80.25,  84.25,  87.75,  90.45,   93.15, 99.91,  99.999};
static std::vector<double> static_pwm_right =  {38.9,   43.10,  47.50,  51.05,  54.00,  58.95,  78.25,  83.55,  87.25,  90.20,   93.00, 98.00,  99.999};



std::vector<double>& get_rpm()
{
    return static_rpm_vec;
}

std::vector<double>& get_pwm(MotorSide side)
{
    std::vector<double>& pwm = (side == MotorSide::left) ? static_pwm_left : static_pwm_right;
    return pwm;
}

/**
 * MotorProfile =======================================================================================================
 */
MotorProfile::MotorProfile(MotorSide side, std::vector<double>& pwm, std::vector<double>& rpm)
:m_pwm(pwm), m_rpm(rpm)
{
    m_side = side;
    m_min_pwm = 30.0;
    m_max_pwm = 99.9;
    m_max_target = 7500.0;
    m_min_target = 2000.0;
    m_error_tolerance = 50.0; //+/- 25 rpm either side
    m_fixed_step_size = 0.5;  // remember this is a pwm value and they are in range 0 .. 100.0
    m_proportional_error_tolerance_factor = 0.02; // 2%
    m_proportional_up_multiplier = 1.05;          // this is a steps of say 50% pwm to 52.5% pwm
    m_proportional_down_multiplier = 0.95;        // this is a step of say  50% pwn to 47.5
}
double MotorProfile::guess_pwm(double rpm)
{
    double return_value;
    if(rpm < m_min_target) {
        return 0.0;
    }
    double xvalue =  (rpm >= m_max_target) ? m_max_target: rpm;
    std::vector<double>& pwm = m_pwm;
    std::vector<double>& rpm_vec_local = m_rpm;
    auto neg_flag = (xvalue < 0.0);
    auto abs_xvalue = neg_flag ? -xvalue: xvalue;
    if(std::abs(rpm_vec_local.back() - abs_xvalue) < 0.1) {
        return_value = pwm.back();
    }
    if(abs_xvalue < rpm_vec_local[0] or rpm_vec_local.back() < abs_xvalue) {
        FATAL_ERROR_MSG("guess_pwm function abs_xvalue is out of range ");
    }
    std::optional<int> lower_index = std::nullopt;
    for(int i = 0; i < rpm_vec_local.size() - 1; i++) {
        if(rpm_vec_local[i] <= abs_xvalue && abs_xvalue < rpm_vec_local[i+1]) {
            lower_index = i;
        }
    }
    if(!lower_index) {
        FATAL_ERROR_MSG("guess_pwm failed to find the interval for abs_xvalue ");
    } else {
        auto li = lower_index.value();
        auto x0 = rpm_vec_local[li];
        auto x1 = rpm_vec_local[li + 1];
        auto y0 = pwm[li];
        auto y1 = pwm[li + 1];
        auto y = ((y1 - y0) / (x1 - x0)) * (abs_xvalue - x0) + y0;
        return_value = y;
    }
    return return_value;
}
std::optional<int> MotorProfile::find_pwm_index(double pwm)
{
    if(pwm < 0.0) {
        FATAL_ERROR_MSG("MotorProfile::find_pwm_index  pwm value is negative");
    }
    std::optional<int> lower_index = std::nullopt;
    for(int i = 0; i < m_pwm.size() - 1; i++) {
        if(m_pwm[i] <= pwm && pwm < m_pwm[i+1]) {
            lower_index = i;
        }
    }
    return lower_index;
}
std::optional<int> MotorProfile::find_rpm_index(double rpm)
{
    if(rpm < 0.0) {
        FATAL_ERROR_MSG("MotorProfile::find_rpm_index  rpm value is negative");
    }
    std::optional<int> lower_index = std::nullopt;
    for(int i = 0; i < m_rpm.size() - 1; i++) {
        if(m_rpm[i] <= rpm && rpm < m_rpm[i+1]) {
            lower_index = i;
        }
    }
    // assert 0 <= lower_index < (not equal) m_rpm.size()-1
    return lower_index;
}


double MotorProfile::slope_at_pwm(double pwm)
{
    auto ix0 = find_pwm_index(pwm);
    if(!ix0) {
        FATAL_ERROR_MSG("MotorProfile.slope_at_pwm  failed to find index");
    }
    auto x0 = m_pwm.at(ix0.value());
    auto x1 = m_pwm[ix0.value()+1];
    auto y0 = m_rpm[ix0.value()];
    auto y1 = m_rpm[ix0.value()+1];
    auto slope = (y1 - y0) / (x1 - x0);
    return slope;
}
double MotorProfile::slope_at_rpm(double rpm)
{
    auto ix0 = find_rpm_index(rpm);
    if(!ix0) {
        FATAL_ERROR_MSG("MotorProfile.slope_at_rpm  failed to find index");
    }
    auto x0 = m_pwm[ix0.value()];
    auto x1 = m_pwm[ix0.value()+1];
    auto y0 = m_rpm[ix0.value()];
    auto y1 = m_rpm[ix0.value()+1];
    auto slope = (y1 - y0) / (x1 - x0);
    return slope;
}

