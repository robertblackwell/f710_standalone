#include <string>
#include <vector>
#ifndef F710_CMD_RESPONSE_H
#define F710_CMD_RESPONSE_H
namespace non_ros_msgs {
    struct F710LeftRight {
        using UPtr = std::unique_ptr<F710LeftRight>;
        int m_left;
        int m_right;
        F710LeftRight(int left, int right): m_left(left), m_right(right){};
    };
    struct FirmwareRebootCmd {
        using UPtr = std::unique_ptr<FirmwareRebootCmd>;
        std::string dummy; // just to have some content

    };
    struct FirmwareStartupResponse {
        using UPtr = std::unique_ptr<FirmwareStartupResponse>;
        bool ok;
        std::string text; // dummy just to have some content
    };
    struct MotorPwmCmd {
        using UPtr = std::unique_ptr<MotorPwmCmd>;
        float left_motor_pwm;
        float right_motor_pwm;
    };
    struct MotorRpmCmd {
        using UPtr = std::unique_ptr<MotorRpmCmd>;
        float left_motor_rpm;
        float right_motor_rpm;
    };
    struct CmdResponse {
        using UPtr = std::unique_ptr<CmdResponse>;
        bool ok;
        std::string text;
    };
    struct EncoderStatus {
        using UPtr = std::unique_ptr<EncoderStatus>;
        int64_t sample_sum;
        int64_t sample_timestamp_usecs;
        float   motor_rpm_estimate;
        bool    direction_pin_state;
    };
    struct LoadTestCmd {
        using UPtr = std::unique_ptr<LoadTestCmd>;
        int32_t count;
        int32_t msg_length;
        int32_t msgs_per_second;
    };
    struct TextMsg {
        using UPtr = std::unique_ptr<TextMsg>;
        std::string text;
    };
    struct EchoCmd {
        using UPtr = std::unique_ptr<EchoCmd>;
        std::vector<std::string> text;
    };
    struct ReadEncodersCmd {
        using UPtr = std::unique_ptr<ReadEncodersCmd>;
        int left_or_right;
        int how_long_between_ms;
    };
    struct TwoEncoderStatus {
        using UPtr = std::unique_ptr<TwoEncoderStatus>;
        EncoderStatus left;
        EncoderStatus right;
    };
}
#endif //F710_CMD_RESPONSE_H
