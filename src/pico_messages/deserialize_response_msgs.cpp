
#include <functional>
#include <memory>
#include <tuple>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonpath/jsonpath.hpp>
#include <pico_messages/deserialize_response_msgs.h>
#include <rbl/iobuffer.h>

using IoBuffer = rbl::IoBuffer;
namespace pico_messages {
    static bool test_buffer_prefix(IoBuffer& buffer, std::string prefix)
    {
        std::string test = buffer.substr(0, 2);
        // printf("%s  %s\n", test.c_str(), prefix.c_str());
        bool x = (test  == prefix);
        return x;
    }
    static bool deserialize(IoBuffer& buffer, FirmwareStartupResponse& cmd_response)
    {
        if(! test_buffer_prefix(buffer, "1B")) {
            return false;
        }
        cmd_response.text = std::string(buffer.c_str());
        return true;
    }
    static bool deserialize(IoBuffer& buffer, CmdResponse& cmd_response)
    {
        if(! test_buffer_prefix(buffer, "1P")) {
            return false;
        }
        std::string s = buffer.to_string();
        cmd_response.ok = s.find("OK") != std::string::npos;
        cmd_response.text = s;
        return true;
    }

    static bool deserialize(IoBuffer& buffer, EncoderStatus& status)
    {
        if(! test_buffer_prefix(buffer, "1K")) {
            return false;
        }try {
            std::string json_string = buffer.to_string().substr(2, std::string::npos);
            jsoncons::json j = jsoncons::json::parse(json_string);
            status.motor_rpm_estimate = j["mr"].as<float>();
            status.wheel_speed_mps = j["wsms"].as<float>();
            return true;
        } catch(std::exception& e) {
            return false;
        }
        // status.direction_pin_state = j["ps"].as<uint8_t>();
    }
    static bool deserialize(IoBuffer& buffer, TwoEncoderStatus& status)
    {
        if(! test_buffer_prefix(buffer, "1J")) {
            return false;
        }
        //    printf("deserialize TwoEncoderStatus %s\n", buffer.to_string().c_str());
        std::string json_string = buffer.to_string().substr(2,std::string::npos);
        jsoncons::json j = jsoncons::json::parse(json_string);
        status.left.motor_rpm_estimate = j[0]["mr"].as<float>();
        status.left.wheel_speed_mps = j[0]["wsms"].as<float>();

        status.right.motor_rpm_estimate = j[1]["mr"].as<float>();
        status.right.wheel_speed_mps = j[1]["wsms"].as<float>();
        return true;
    }

    static bool deserialize(IoBuffer& buffer, TextMsg& text_msg)
    {
        text_msg.text = buffer.to_string();
        return true;
    }
    bool deserialize(IoBuffer& buffer, ResponseMessageUnion& inmsg)
    {
        if (deserialize(buffer, inmsg.firmware_startup_response)) {
            inmsg.tag = ResponseMessageEnum::firmware_startup;
        } else if (deserialize(buffer, inmsg.cmd_response)) {
            inmsg.tag = ResponseMessageEnum::cmd_response;
        } else if (deserialize(buffer, inmsg.encoder_status)) {
            inmsg.tag = ResponseMessageEnum::encoder_status;
        } else if (deserialize(buffer, inmsg.two_encoder_status)) {
            inmsg.tag = ResponseMessageEnum::two_encoder_status;
        } else if (deserialize(buffer, inmsg.text_msg)) {
            inmsg.tag = ResponseMessageEnum::text_msg;
        } else {
            return  false;
        }
        return true;
    }
    std::unique_ptr<ResponseMessageUnion> deserialize_2(IoBuffer& buffer)
    {
        std::unique_ptr<ResponseMessageUnion> msg_uptr = std::make_unique<ResponseMessageUnion>();
        if(const auto flag = deserialize(buffer, *msg_uptr); !flag) {
            return nullptr;
        }
        return msg_uptr;
    }
}