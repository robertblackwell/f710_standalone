#ifndef H_fbcontrol_custom_h
#define H_fbcontrol_custom_h
#include <optional>
#include <functional>
#include <memory>
#include "value_with_direction.h"
#include "motor_profile.h"

struct CustomController
{

    enum class Algorithm {linear, proportional, fixed_step};

    CustomController(Algorithm algorithm, MotorSide side);

    auto set_target(ValueWithDirection new_target) -> ValueWithDirection;
    auto get_target() -> ValueWithDirection;

    auto update(ValueWithDirection latest_y) -> ValueWithDirection;
    auto update_linear(ValueWithDirection latest_y) -> ValueWithDirection ;
    auto update_fixed_step(ValueWithDirection latest_y) -> ValueWithDirection ;
    auto update_proportional(ValueWithDirection latest_y) -> ValueWithDirection ;

    double              m_error;
    MotorSide           m_side;
    MotorProfile        m_profile;
    ValueWithDirection  m_target;
    Algorithm           m_algorithm;
    int                 m_count;
    double              m_previous_error;
    double              m_xn_1;
    double              m_yn_1;
    double              m_xn;
    double              m_yn;
    double              m_en_1;
    double              m_en;
    int                 m_direction;
};

#endif