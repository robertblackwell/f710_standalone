#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <optional>
#include <format>
#include "jsoncons/json.hpp"
#include "jsoncons_ext/jsonpath/jsonpath.hpp"

#include "ros2-serial-bridge/src/non_ros_messages/msgs.h"
#include "bridge_lib/iobuffer.h"
void test_01()
{
    printf("Hello this is test_msgs\n");
	ros2_bridge::IoBuffer buffer{};
	buffer.append(std::string{R"({"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"})"});
	printf("json string is %s\n", buffer.to_string().c_str());
    std::string json_string = buffer.to_string();
    jsoncons::json j = jsoncons::json::parse(json_string);
    sample_interfaces::msg::EncoderStatus status;
    status.sample_sum = j["ss"].as<int64_t>();
    status.sample_time_stamp_usecs = j["ts"].as<int64_t>();
    status.motor_rpm_estimate = j["mr"].as<float>();

    std::string x = j["ps"].as<std::string>();

}
void test_02()
{
    printf("Hello this is test_msgs\n");
	ros2_bridge::IoBuffer buffer{};
	buffer.append(std::string{R"([{"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"}, {"ss":77777777, "ts":129283746, "mr":5767.43, "ps":120}])"});
	printf("json string is %s\n", buffer.to_string().c_str());
    std::string json_string = buffer.to_string();
    jsoncons::json j = jsoncons::json::parse(json_string);
    sample_interfaces::msg::TwoEncoderStatus status;
    status.left.sample_sum              = j[0]["ss"].as<int64_t>();
    status.left.sample_time_stamp_usecs = j[0]["ts"].as<int64_t>();
    status.left.motor_rpm_estimate      = j[0]["mr"].as<float>();
    status.right.sample_sum              = j[1]["ss"].as<int64_t>();
    status.right.sample_time_stamp_usecs = j[1]["ts"].as<int64_t>();
    status.right.motor_rpm_estimate      = j[1]["mr"].as<float>();

    std::string x0 = j[0]["ps"].as<std::string>();
    std::string x1 = j[1]["ps"].as<std::string>();

}
using namespace sample_interfaces::msg;
using namespace ros2_bridge;

void f(CmdResponse& resp)
{
    printf("Do something with a cmd response\n");
}
void f(EncoderStatus& status)
{
    printf("Do something with a encoder response\n");
}
void f(TwoEncoderStatus& status)
{
    printf("Do something with a 222 Two encoder response\n");
}
void f(TextMsg& resp)
{
    printf("Do something with a text msg\n");
}
struct overloadf
{
    void operator()(CmdResponse& inmsg){f(inmsg);}
    void operator()(EncoderStatus& inmsg){f(inmsg);}
    void operator()(TwoEncoderStatus& inmsg){f(inmsg);}
    void operator()(TextMsg& inmsg){f(inmsg);}
};
void test_03()
{
	ros2_bridge::IoBuffer buffer{};
	buffer.append(std::string{R"({"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"})"});
    EncoderStatus status{};
    deserialize(buffer, status);
    InputMessage vv{status};
    std::visit(overloadf{}, vv);
    printf("we are done");
}
void test_04()
{
	ros2_bridge::IoBuffer buffer{};
	buffer.append(std::string{R"(1K[{"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"}, {"ss":77777777, "ts":129283746, "mr":5767.43, "ps":120}])"});
    EncoderStatus status{};
    // deserialize(buffer, status);
    // InputMessage vv{status};
    InputMessage vv{};
    deserialize(buffer, vv);
    if(std::holds_alternative<TwoEncoderStatus>(vv)) {
        printf("222TwoEncoderStatus \n");
    } else if(std::holds_alternative<EncoderStatus>(vv)) {
        printf("EncoderStatus \n");
    } else if(std::holds_alternative<TextMsg>(vv)) {
        printf("Text Msg \n");
    }
    if(auto x = std::get_if<TwoEncoderStatus>(&vv)) {
        printf("222TwoEncoderStatus \n");
    } else if(auto x = std::get_if<EncoderStatus>(&vv)) {
        printf("EncoderStatus \n");
    } else if(auto x = std::get_if<TextMsg>(&vv)) {
        printf("Text Msg \n");
    }
    printf("we are done");
}
void test_05()
{
    auto make_iobuffer = [](OutputMessage msg) {
        IoBuffer::UPtr buf_uptr = std::make_unique<IoBuffer>();
        serialize(msg, *buf_uptr);
        return buf_uptr;
    };

    auto echo = EchoCmd();
    echo.data = std::vector<std::string>({std::string("1111"),std::string("2222")});
    for(auto &s : echo.data) {
        const char* tmp = s.c_str();
        printf("%s\n", tmp);
    }
    auto echo2 = make_iobuffer(sample_interfaces::build<EchoCmd>().data({std::vector<std::string>{"AAAAAA","BBBBBB"}}));
    printf("We r done");
}
int main(int argc, char * argv[])
{
    printf("Hello this is test_msgs\n");
    test_05();
    test_04();
    test_03();
    test_01();
    test_02();

	return 0;
}
