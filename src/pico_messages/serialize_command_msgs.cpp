
#include <functional>
#include <memory>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonpath/jsonpath.hpp>
#include <pico_messages/deserialize_response_msgs.h>
#include <rbl/iobuffer.h>
#include "serialize_command_msgs.h"

using IoBuffer = rbl::IoBuffer;
namespace pico_messages {
    /**
     * Serialize functions
    */
    static void serialize(const FirmwareStartupResponse& response, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "1B %s", response.text.c_str());
        buffer.commit(len);

    }
    static void serialize(const FirmwareRebootCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "reset ");
        buffer.commit(len);

    }
    static void serialize(const CmdResponse& response, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "%s %s", (response.ok ? "OK": "FAILED"), response.text.c_str());
        buffer.commit(len);

    }
    static void serialize(const EchoCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        std::string result{};
        for(const auto& s : cmd.text) {
            result +=  " " + s;
        }
        // auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "echo %s\n", result.c_str());
        buffer.append("echo "); buffer.append(result);
        // buffer.commit(len);

    }
    static void serialize(const LoadTestCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "load %d %d %d", cmd.count, cmd.msg_length, cmd.msgs_per_second);
        buffer.commit(len);
    }
    static void serialize(const MotorPwmCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "pwm %f %f", cmd.left_motor_pwm, cmd.right_motor_pwm);
        buffer.commit(len);
    }
    static void serialize(const MotorRpmCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "rpm %f %f", cmd.left_motor_rpm, cmd.right_motor_rpm);
        buffer.commit(len);
    }
    static void serialize(const TextMsg& text_msg, IoBuffer& buffer)
    {
        buffer.clear();
        // auto len = snprintf((char*)buffer.space_ptr(), buffer.space_len(), "%s", cmd_response.text);
        buffer.append(text_msg.text);
    }
    static void serialize(const ReadEncodersCmd& cmd_response, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "e %d", cmd_response.left_or_right);
        buffer.commit(len);
    }
    static void serialize(const PidToggleCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "p");
        buffer.commit(len);

    }
    static void serialize(const WheelVelocityCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "v %f %f", cmd.left_wheel_velocity_ms, cmd.right_wheel_velocity_ms);
        buffer.commit(len);

    }
    static void serialize(const WheelVelocityScaledCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "vs %f %f", cmd.left_wheel_velocity_percent, cmd.right_wheel_velocity_percent);
        buffer.commit(len);

    }
    static void serialize(const StopCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "s");
        buffer.commit(len);

    }
    static void serialize(const StreamEncodersCmd& cmd, IoBuffer& buffer)
    {
        buffer.clear();
        const auto len = snprintf(static_cast<char *>(buffer.space_ptr()), buffer.space_len(), "es %d %d", cmd.left_or_right, cmd.how_long_between_ms);
        buffer.commit(len);
    }

    void serialize(CommandMessageUnion& msg, IoBuffer& buffer)
    {
        switch(msg.tag) {
            case CommandMessageEnum::echo_cmd:
                serialize(msg.echo_cmd, buffer);
                break;
            case CommandMessageEnum::load_test_cmd:
                serialize(msg.load_test_cmd, buffer);
                break;
            case CommandMessageEnum::motor_pwm_cmd:
                serialize(msg.motor_pwm_cmd, buffer);
                break;
            case CommandMessageEnum::motor_rpm_cmd:
                serialize(msg.motor_rpm_cmd, buffer);
                break;
            case CommandMessageEnum::read_encoders_cmd:
                serialize(msg.read_encoders_cmd, buffer);
                break;
            case CommandMessageEnum::firmware_reboot:
                serialize(msg.reboot_cmd, buffer);
                break;
            case CommandMessageEnum::pid_toggle_cmd:
                serialize(msg.pid_toggle_cmd, buffer);
                break;
            case CommandMessageEnum::stop_cmd:
                serialize(msg.stop_cmd, buffer);
                break;
            case CommandMessageEnum::stream_encoders_cmd:
                serialize(msg.stream_encoders_cmd, buffer);
                break;
            case CommandMessageEnum::wheel_velocity_cmd:
                serialize(msg.wheel_velocity_cmd, buffer);
                break;
            case CommandMessageEnum::wheel_velocity_scaled_cmd:
                serialize(msg.wheel_velocity_scaled_cmd, buffer);
                break;
            default:
                break;
        }
    }
} //namespace