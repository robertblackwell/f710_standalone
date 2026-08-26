#ifndef H_fbcontrol_value_with_direction_h
#define H_fbcontrol_value_with_direction_h
#include <vector>
#include <optional>
#include <functional>
#include <memory>


struct ValueWithDirection {
    double m_raw_value;

    ValueWithDirection() {
        m_raw_value = 0.0;
    }

    ValueWithDirection(double v) {
        m_raw_value = v;
    }

    [[nodiscard]] double abs_value() const {
        return m_raw_value < 0 ? -m_raw_value : m_raw_value;
    }

    void update_abs_value(double abs_value) {
        if (m_raw_value < 0)
            m_raw_value = -abs_value;
        else
            m_raw_value = abs_value;
    }

    [[nodiscard]] double signed_value() const {
        return m_raw_value;
    }

    double sign() const {
        return (m_raw_value) ? 1.0 : -1.0;
    }
};


#endif