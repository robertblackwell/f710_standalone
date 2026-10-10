#include <rbl/f710_helpers.h>
#include <rbl/f710_exceptions.h>
#include "model.h"
#include "model_defines.h"
#include <format>
#include <chrono>
#include "asio_reader.h"

float scale(const bool high_gear, int value) {
    if (value == 0) {
        return 0.0;
    }
    const auto multiplier = (value < 0) ? -1: 1;
    value = value * multiplier;
    float pwm;
    if (high_gear) {
        pwm = multiplier * (round((static_cast<float>(value) / static_cast<float>(INT16_MAX)) * (85.0 - 50.0)) + 50.0);
    } else {
        pwm = multiplier * (round((static_cast<float>(value) / static_cast<float>(INT16_MAX)) * (60.0 - 30.0)) + 30.0);
    }
    return pwm;
}
std::string format_time_now()
{
    using namespace std::chrono;
    auto now = std::chrono::time_point_cast<milliseconds>(std::chrono::system_clock::now());
    std::string s = std::format("{:%T}", now);
    return s;
}

///
/// This function is called by the Reader instance so that the current state of the controller can be saved for
/// further processing. The cb() function runs on the same thread as the io_context which prevents the
/// f710::ControllerState instance will not change during the call to cb().
///
/// Any state information required after the call to cb() completes must be saved during the cb() cal.
///
void make_pico_velocity_msg(const f710::ControllerState& state) {
    const auto left = -1 * state.m_left.latest_event_value;
    const auto right = -1 * state.m_right.latest_event_value;
    const auto onoff = state.m_button.event_toggle_value;

    const auto pwm_left = scale(onoff, left);
    const auto pwm_right = scale(onoff, right);

    const auto tn = format_time_now();
    printf("from main %s left: %d pwm_left: %f  right: %d pwm_right: %f toggle: %d\n",
        tn.c_str(),
        left, pwm_left, right, pwm_right, static_cast<int>(onoff));
}
int main(int argc, char **argv) {
    try {
        boost::asio::io_context io_context;
        f710::ControllerState controller_state{
            f710::AxisDevice(D_AXIS_LEFT_STICK_FWD_BKWD_NUMBER),
            f710::AxisDevice(D_AXIS_RIGHT_STICK_FWD_BKWD_NUMBER), 
            f710::ToggleButton(D_BUTTON_A)};

        const std::string js_name = "js";
        f710::AsioReader<f710::ControllerState> logitech_f710_reader{io_context, js_name, &controller_state};
        auto handle_f710 = [](const f710::ControllerState& state) {
            make_pico_velocity_msg(state);
        };
        logitech_f710_reader.run(handle_f710);
        io_context.run();
        
    } catch(const f710::F710Exception e) {
        printf("F710Exception %s", e.what());
    } catch(...) {
        printf("got an exception");
    }
    return 0;
}
