#ifndef H_ros2_bridge_msgs_h
#define H_ros2_bridge_msgs_h
#include <string>
#include <variant>
#include <optional>
#include <rbl/iobuffer.h>
#include "msg_struct.h"

#ifdef ROS2_MSGS
using namespace sample_interfaces::msg;
#else
using namespace non_ros_msgs;
#endif
using namespace rbl;
/**
 * See the companion package of ROS2 messages called 'sample_interfaces'
 * to understand what message types we area dealing with.
 * 
 * Serialize and Deserialize Functions
 * ===================================
 * These functions are only intended for use by ros2 nodes running on a Linux host.
 *  
 * Hence, Deserialize functions are only required for those messages that originate in the micro-controller
 * and are received by the host. They are:
 * 
 * -    CmdResponse
 * -    TwoEncoderStatus
 * -    TextMsg
 * -    FirmwareStartupResponse
 * 
 * Deserialize functions convert a packet of bytes into a std::variant 
 * that holds one of a selection of ROS2 messasge structures.
 * 
 * This variant structure is called an InputMessage
 */
enum class InputMessageVariantIndex {firmware_startup=0, text_msg=1, cmd_response=2, encoder_status=3, two_encoder_status=4};

using InputMessageVariant = std::variant<FirmwareStartupResponse, TextMsg, CmdResponse, EncoderStatus, TwoEncoderStatus>;

using InputMessageVariantUPtr = std::unique_ptr<InputMessageVariant>;

enum class InputMessageUPtrVariantIndex {
        firmaware_startup=0,
        text_msg = 1,
        cmd_response=2,
        encoder_status=3,
        two_encoder_status=4
};
using InputMessageUPtrVariant = std::variant<FirmwareStartupResponse::UPtr,
                    TextMsg::UPtr,
                    CmdResponse::UPtr,
                    EncoderStatus::UPtr,
                    TwoEncoderStatus::UPtr> ;

/** 
 * The following 4 type specific deserialize functions are only exposed for testing and experimenting
 * and should not be avoided. 
*/
bool deserialize(IoBuffer& buffer, FirmwareStartupResponse& response);
bool deserialize(IoBuffer& buffer, CmdResponse& response);
bool deserialize(IoBuffer& buffer, EncoderStatus& cmd_response);
bool deserialize(IoBuffer& buffer, TwoEncoderStatus& cmd_response);
bool deserialize(IoBuffer& buffer, TextMsg& cmd_response);

/**
 * Use this general deserialize function 
*/
bool deserialize(IoBuffer& buffer, InputMessageVariant& inmsg);
bool deserialize(IoBuffer& buffer, InputMessageUPtrVariant& inmsg);
std::unique_ptr<InputMessageUPtrVariant> deserialize(IoBuffer& buffer);
/**
 *  * Similarly serialize functions are only required for those messages sent from the Host to the 
 * Micro-controller. They are:
 * 
 * -    EchoCmd
 * -    LoadTestCmd
 * -    MotorPwmCmd
 * -    MotorRpmCmd
 * -    TextMsgs - I anticipate using these for logging 
 * 
 * Serialize functions convert these ROS2 messages into a packet of bytes ready
 * for transmission to the microcontroller.
 * 
 * The input to the serialize process is again a std::variant which holds instances
 * of one of the message types listed above.
 * 
 * The std::variant is called an OutputMessageVariant
*/
enum class OutputMessageVariantIndex {firmware_reboot, 
        test_msg, 
        echo_cmd, 
        load_test_cmd, 
        motor_pwm_cmd,
        motor_rpm_cmd,
        read_encoders_cmd};
typedef std::variant< FirmwareRebootCmd,
        TextMsg, EchoCmd, LoadTestCmd, MotorPwmCmd, MotorRpmCmd, ReadEncodersCmd> OutputMessageVariant;

enum class OutputMessageUPtrVariantIndex {
        firmware_reboot_cmd,
        test_msg,
        echo_cmd,
        load_test_cmd,
        motor_pwm_cmd,
        read_encoders_cmd
};
typedef std::variant<
        FirmwareRebootCmd::UPtr,
        TextMsg::UPtr,
        EchoCmd::UPtr,
        LoadTestCmd::UPtr,
        MotorPwmCmd::UPtr,
        MotorRpmCmd::UPtr,
        ReadEncodersCmd::UPtr> OutputMessageUPtrVariant;

typedef std::unique_ptr<OutputMessageVariant> OutputMessageVariant_Uptr;

/** 
 * The following 4 type specific serialize functions are only exposed for testing and experimenting
 * and should not be avoided. 
*/
void serialize(FirmwareRebootCmd& cmd, IoBuffer& buffer);
void serialize(EchoCmd& cmd, IoBuffer& buffer);
void serialize(LoadTestCmd& cmd, IoBuffer& buffer);
void serialize(MotorPwmCmd& cmd, IoBuffer& buffer);
void serialize(MotorRpmCmd& cmd, IoBuffer& buffer);
void serialize(ReadEncodersCmd& cmd_response, IoBuffer& buffer);

/**
 * Use this general serialize function 
*/
void serialize(OutputMessageVariant& out_msg, IoBuffer& buffer);


#endif