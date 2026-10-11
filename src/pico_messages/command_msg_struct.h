#ifndef H_PICO_COMMAND_MESSAGES_H
#define H_PICO_COMMAND_MESSAGES_H
#include <memory>
#include <string>
#include <vector>
namespace pico_messages {
#if 0
    enum class ResponseMessageEnum {
        firmware_startup=0,
        text_msg=1,
        cmd_response=2,
        encoder_status=3,
        two_encoder_status=4
    };
#endif
    enum class CommandMessageEnum {
        firmware_reboot= 21,
        test_msg = 22,
        echo_cmd = 23,
        stop_cmd = 24,
        load_test_cmd = 25,
        motor_pwm_cmd = 26,
        motor_rpm_cmd = 28,
        wheel_velocity_cmd = 29,
        wheel_velocity_scaled_cmd = 31,
        read_encoders_cmd = 32,
        stream_encoders_cmd = 33,
        pid_toggle_cmd = 34,
        pid_args_update_cmd = 35,
};

    struct F710LeftRight {
        using UPtr = std::unique_ptr<F710LeftRight>;
        int m_left;
        int m_right;
        F710LeftRight(int left, int right): m_left(left), m_right(right){};
    };

    /**
     * Command message structures - commands from host to pico
     */
    struct FirmwareRebootCmd {
        // std::string dummy; // just to have some content
        FirmwareRebootCmd() = default;
    };
    struct LoadTestCmd {
        int32_t count;
        int32_t msg_length;
        int32_t msgs_per_second;
        LoadTestCmd() = default;

    };
    struct MotorPwmCmd {
        float left_motor_pwm;
        float right_motor_pwm;
        MotorPwmCmd() = default;
    };
    struct MotorRpmCmd {
        float left_motor_rpm;
        float right_motor_rpm;
        MotorRpmCmd() = default;
    };
    struct EchoCmd {
        std::vector<std::string> text;
        EchoCmd() = default;
    };
    struct WheelVelocityCmd {
        float left_wheel_velocity_ms;
        float right_wheel_velocity_ms;
        WheelVelocityCmd() = default;
    };
    struct WheelVelocityScaledCmd {
        float left_wheel_velocity_percent;
        float right_wheel_velocity_percent;
        WheelVelocityScaledCmd() = default;
    };
    struct ReadEncodersCmd {
        int left_or_right;
        int how_long_between_ms;
        ReadEncodersCmd() = default;
    };
    struct StreamEncodersCmd {
        int left_or_right;
        int how_long_between_ms;
        StreamEncodersCmd() = default;
    };
    struct StopCmd {
        StopCmd() = default;
    };
    struct PidToggleCmd {
        PidToggleCmd() = default;
    };
    struct PidArgsUpdateCmd {
        double kp;
        double ki;
        double kd;
        PidArgsUpdateCmd() = default;
    };
    struct CommandMessageUnion {
        using UPtr = std::unique_ptr<CommandMessageUnion>;
        union {
            FirmwareRebootCmd reboot_cmd;
            LoadTestCmd       load_test_cmd;
            MotorPwmCmd       motor_pwm_cmd;
            MotorRpmCmd       motor_rpm_cmd;
            EchoCmd           echo_cmd;
            StopCmd           stop_cmd;
            ReadEncodersCmd   read_encoders_cmd;
            WheelVelocityCmd  wheel_velocity_cmd;
            WheelVelocityScaledCmd wheel_velocity_scaled_cmd;
            PidToggleCmd      pid_toggle_cmd;
            StreamEncodersCmd stream_encoders_cmd;
            PidArgsUpdateCmd  pid_args_update_cmd;
            union {} dummy; // to stop static analyser incorrectly complaining about not init all legs of the union
        };
        CommandMessageEnum tag;
        CommandMessageUnion(): dummy(), tag(CommandMessageEnum::firmware_reboot)  {}

        explicit CommandMessageUnion(CommandMessageEnum tag):  dummy(), tag(tag) {
            change_to(tag);
        }
        void change_to(CommandMessageEnum new_tag)
        {
            reset();
            tag = new_tag;
            switch (tag) {
                case pico_messages::CommandMessageEnum::firmware_reboot:
                    new (&reboot_cmd) FirmwareRebootCmd();
                    break;
                case pico_messages::CommandMessageEnum::load_test_cmd:
                    new (&load_test_cmd) LoadTestCmd();
                    break;
                case pico_messages::CommandMessageEnum::motor_pwm_cmd:
                    new (&motor_pwm_cmd) MotorPwmCmd();
                    break;
                case pico_messages::CommandMessageEnum::motor_rpm_cmd:
                    new (&motor_rpm_cmd) MotorRpmCmd();
                    break;
                case pico_messages::CommandMessageEnum::echo_cmd:
                    new (&echo_cmd) EchoCmd();
                    break;
                case pico_messages::CommandMessageEnum::stop_cmd:
                    new (&stop_cmd) StopCmd();
                    break;
                case pico_messages::CommandMessageEnum::read_encoders_cmd:
                    new (&read_encoders_cmd) ReadEncodersCmd();
                    break;
                case pico_messages::CommandMessageEnum::wheel_velocity_cmd:
                    new (&wheel_velocity_cmd) WheelVelocityCmd();
                    break;
                case pico_messages::CommandMessageEnum::wheel_velocity_scaled_cmd:
                    new (&wheel_velocity_scaled_cmd) WheelVelocityScaledCmd();
                    break;
                case pico_messages::CommandMessageEnum::pid_toggle_cmd:
                    new (&pid_toggle_cmd) PidToggleCmd();
                    break;
                case pico_messages::CommandMessageEnum::stream_encoders_cmd:
                    new (&stream_encoders_cmd) StreamEncodersCmd();
                    break;
                case pico_messages::CommandMessageEnum::pid_args_update_cmd:
                    new (&pid_args_update_cmd) PidArgsUpdateCmd();
                    break;
                default:
                    break;
            }

        }
        ~CommandMessageUnion() {
            reset();
        }
        void reset()
        {
            switch (tag) {
                case pico_messages::CommandMessageEnum::firmware_reboot:
                    reboot_cmd.~FirmwareRebootCmd();
                    break;
                case pico_messages::CommandMessageEnum::load_test_cmd:
                    load_test_cmd.~LoadTestCmd();
                    break;
                case pico_messages::CommandMessageEnum::motor_pwm_cmd:
                    motor_pwm_cmd.~MotorPwmCmd();
                    break;
                case pico_messages::CommandMessageEnum::motor_rpm_cmd:
                    motor_rpm_cmd.~MotorRpmCmd();
                    break;
                case pico_messages::CommandMessageEnum::echo_cmd:
                    echo_cmd.~EchoCmd();
                    break;
                case pico_messages::CommandMessageEnum::stop_cmd:
                    stop_cmd.~StopCmd();
                    break;
                case pico_messages::CommandMessageEnum::read_encoders_cmd:
                    read_encoders_cmd.~ReadEncodersCmd();
                    break;
                 case pico_messages::CommandMessageEnum::wheel_velocity_cmd:
                    wheel_velocity_cmd.~WheelVelocityCmd();
                    break;
                case pico_messages::CommandMessageEnum::wheel_velocity_scaled_cmd:
                    wheel_velocity_scaled_cmd.~WheelVelocityScaledCmd();
                    break;
                case pico_messages::CommandMessageEnum::pid_toggle_cmd:
                    pid_toggle_cmd.~PidToggleCmd();
                    break;
                case pico_messages::CommandMessageEnum::stream_encoders_cmd:
                    stream_encoders_cmd.~StreamEncodersCmd();
                    break;
                case pico_messages::CommandMessageEnum::pid_args_update_cmd:
                    pid_args_update_cmd.~PidArgsUpdateCmd();
                    break;
                default:
                    break;
            }
        }
    };
#if 0
    /**
     * Responses from pico to host
     */
    struct FirmwareStartupResponse {
        bool ok;
        std::string text; // dummy just to have some content
        FirmwareStartupResponse(): ok(true), text("") {}
    };
    struct CmdResponse {
        bool ok;
        std::string text;
        CmdResponse(): ok(true), text("") {}
    };
    struct EncoderStatus {
        int64_t sample_sum;
        int64_t sample_timestamp_usecs;
        float   wheel_speed_mps;
        float   motor_rpm_estimate;
        bool    direction_pin_state;
        EncoderStatus(): sample_sum(0), sample_timestamp_usecs(0), wheel_speed_mps(0), motor_rpm_estimate(0), direction_pin_state(false) {}
    };
    struct TextMsg {
        std::string text;
        TextMsg(): text("default") {}
    };
    struct TwoEncoderStatus {
        EncoderStatus left;
        EncoderStatus right;
        TwoEncoderStatus(): left(), right() {}
    };


    struct ResponseMessageUnion {
        using UPtr = std::unique_ptr<ResponseMessageUnion>;
        union {
            FirmwareStartupResponse firmware_startup_response;
            TextMsg             text_msg;
            CmdResponse         cmd_response;
            EncoderStatus       encoder_status;
            TwoEncoderStatus    two_encoder_status;
            union {} dummy;;
        };
        ResponseMessageEnum tag;
        ResponseMessageUnion(): dummy(), tag(ResponseMessageEnum::firmware_startup){}
        explicit ResponseMessageUnion(ResponseMessageEnum tag): dummy(), tag(tag)
        {
            change_to(tag);
        }
        void change_to(ResponseMessageEnum new_tag)
        {
            reset();
            tag = new_tag;
            switch (tag) {
                case ResponseMessageEnum::firmware_startup:
                    new (&firmware_startup_response) FirmwareStartupResponse();
                    break;
                case ResponseMessageEnum::text_msg:
                    new (&text_msg) TextMsg();
                    break;
                case ResponseMessageEnum::two_encoder_status:
                    new (&encoder_status) EncoderStatus();
                    break;
                case ResponseMessageEnum::cmd_response:
                    new (&cmd_response) CmdResponse();
                    break;
                default:
                    break;
            }
        }
        ~ResponseMessageUnion() {
            reset();
        }
        void reset() {
            switch (tag) {
                case ResponseMessageEnum::firmware_startup:
                    firmware_startup_response.~FirmwareStartupResponse();
                    break;
                case ResponseMessageEnum::text_msg:
                    text_msg.~TextMsg();
                    break;
                case ResponseMessageEnum::two_encoder_status:
                    encoder_status.~EncoderStatus();
                    break;
                case ResponseMessageEnum::cmd_response:
                    cmd_response.~CmdResponse();
                    break;
                default:
                    break;
            }
        }

    };
    // Compile-time layout guarantees required for pointer arithmetic
    static_assert(std::is_standard_layout_v<ResponseMessageUnion>, "InputMessageUnion must be Standard Layout!");
#endif
    static_assert(std::is_standard_layout_v<CommandMessageUnion>, "OutputMessageUnion must be Standard Layout!");

}
#endif //H_PICO_MESSAGES_H
