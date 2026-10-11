#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <vector>
#include <optional>
#include <format>
#include <jsoncons/json.hpp>
#include <jsoncons_ext/jsonpath/jsonpath.hpp>

#include <pico_messages/deserialize_response_msgs.h>
#include <rbl/iobuffer.h>

static void test_01()
{
    printf("Hello this is test_msgs\n");
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"({"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"})"});
	printf("json string is %s\n", buffer.to_string().c_str());
    std::string json_string = buffer.to_string();
    jsoncons::json j = jsoncons::json::parse(json_string);
    pico_messages::EncoderStatus status{};
    status.sample_sum = j["ss"].as<int64_t>();
    status.motor_rpm_estimate = j["mr"].as<float>();
    auto x = j["ps"].as<std::string>();
}

static void test_02()
{
    printf("Hello this is test_msgs\n");
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"([{"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"}, {"ss":77777777, "ts":129283746, "mr":5767.43, "ps":120}])"});
	printf("json string is %s\n", buffer.to_string().c_str());
    std::string json_string = buffer.to_string();
    jsoncons::json j = jsoncons::json::parse(json_string);
    pico_messages::TwoEncoderStatus status{};
    status.left.sample_sum              = j[0]["ss"].as<int64_t>();
    status.left.motor_rpm_estimate      = j[0]["mr"].as<float>();
    status.right.sample_sum              = j[1]["ss"].as<int64_t>();
    status.right.motor_rpm_estimate      = j[1]["mr"].as<float>();

    std::string x0 = j[0]["ps"].as<std::string>();
    std::string x1 = j[1]["ps"].as<std::string>();

}

static void test_03()
{
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"({"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"})"});
    pico_messages::ResponseMessageUnion status{};
    deserialize(buffer, status);
    printf("Hello");
}

static void test_04()
{
	rbl::IoBuffer buffer{};
	buffer.append(std::string{R"(1K[{"ss":19283746, "ts":987654321, "mr":6708.43, "ps":"F"}, {"ss":77777777, "ts":129283746, "mr":5767.43, "ps":120}])"});
    pico_messages::ResponseMessageUnion status{};
    deserialize(buffer, status);
    printf("we are done");
}

static void test_05()
{

    pico_messages::CommandMessageUnion echo{};
	echo.change_to(pico_messages::CommandMessageEnum::echo_cmd);
    echo.tag = pico_messages::CommandMessageEnum::echo_cmd;
    echo.echo_cmd.text = std::vector<std::string>({std::string("1111"),std::string("2222")});
    for(auto &s : echo.echo_cmd.text) {
        const char* tmp = s.c_str();
        printf("%s\n", tmp);
    }
    // auto echo2 = make_iobuffer(sample_interfaces::build<EchoCmd>().data({std::vector<std::string>{"AAAAAA","BBBBBB"}}));
    printf("We r done\n");
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
