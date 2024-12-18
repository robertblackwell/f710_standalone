#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2-serial-bridge/bridge_lib/iobuffer.h"
#include "ros2-serial-bridge/bridge_lib/serial_settings.h"
#include "ros2-serial-bridge/non_ros_messages/msgs.h"
#include "ros2-serial-bridge/bridge_lib/serial_link.h"

using namespace std::chrono_literals;
using namespace std::chrono;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class Bridge : public rclcpp::Node
{
public:
	Bridge()
  	: Node("bridge")
	{
		
		/**
		 * Create the serial link that the does the communication on the communication thread.
		*/
		auto port_path_list = list_serial_devices();
		if(port_path_list.size() != 1) {
			throw std::runtime_error("could not find exactly one suitable serial port path");
		}
		auto port = port_path_list[0];
		int fd = open_serial(port);
		apply_default_settings(fd);
		m_serial_link_uptr = std::make_unique<ros2_bridge::SerialLink>(fd, (rclcpp::Node*)this, std::bind(&Bridge::on_recv_message, this, std::placeholders::_1));

		/**
		 * One publisher for demo purposes but when being used in a real app the will 
		 * probably be 
		 * -	a publisher for each different message type comming from
		 * 		the communication thread
		 * 
		 * -	a subscriber for each message type the communication thread needs to send to
		 * 		the micro controller 
		*/
		m_byte_count = 0;
		m_msg_count = 0;
		m_count = 0;

		m_response_publisher_sptr = this->create_publisher<sample_interfaces::msg::CmdResponse>("cmd_response", 10);
		m_encoder_publisher_sptr  = this->create_publisher<sample_interfaces::msg::TwoEncoderStatus>("two_encoders_status", 10);
		m_text_in_publisher_sptr     = this->create_publisher<sample_interfaces::msg::TextMsg>("text_in", 10);

		m_echo_subscriber_sptr          = this->create_subscription<sample_interfaces::msg::EchoCmd>(        "echo_cmd",10, std::bind(&Bridge::echo_handler,          this, std::placeholders::_1));
		m_rpm_subscriber_sptr           = this->create_subscription<sample_interfaces::msg::MotorRpmCmd>(    "rpm_cmd",10, std::bind(&Bridge::rpm_handler,           this, std::placeholders::_1));
		m_pwm_subscriber_sptr           = this->create_subscription<sample_interfaces::msg::MotorPwmCmd>(    "pwm_cmd",10, std::bind(&Bridge::pwm_handler,           this, std::placeholders::_1));
		// m_load_test_subscriber_sptr     = this->create_subscription<sample_interfaces::msg::LoadTestCmd>(    "x",10, std::bind(&Bridge::load_test_handler,     this, std::placeholders::_1));
		m_read_encoders_subscriber_sptr = this->create_subscription<sample_interfaces::msg::ReadEncodersCmd>("read_encoders_cmd",10, std::bind(&Bridge::read_encoders_handler, this, std::placeholders::_1));
		m_text_out_subscriber_sptr          = this->create_subscription<sample_interfaces::msg::TextMsg>(        "text_out",10, std::bind(&Bridge::text_handler,          this, std::placeholders::_1));

		m_timer = this->create_wall_timer(200000ms, std::bind(&Bridge::timer_callback, this));
		/**
		 * Start the serial link thread
		*/
		// m_serial_link_uptr->start_thread();
		// printf("after starting serial_link thread\n");
	}
	/**
	 * All subscribed messages are serialized and sent to the serial link
	*/

	void echo_handler         (const sample_interfaces::msg::EchoCmd& msg) const        { OutputMessage omsg{msg}; serialize_send(omsg); }
	void rpm_handler          (const sample_interfaces::msg::MotorRpmCmd& msg) const    { OutputMessage omsg{msg}; serialize_send(omsg); }
	void pwm_handler          (const sample_interfaces::msg::MotorPwmCmd& msg) const    { 
		printf("bridge::pwm_handler \n");
		OutputMessage omsg{msg}; 
		serialize_send(omsg); 
	}
	void load_test_handler    (const sample_interfaces::msg::LoadTestCmd& msg) const    { OutputMessage omsg{msg}; serialize_send(omsg); }
	void read_encoders_handler(const sample_interfaces::msg::ReadEncodersCmd& msg)const {
		printf("bridge::read_encoders_handler \n");
		OutputMessage omsg{msg}; 
		serialize_send(omsg); 
	}
	void text_handler         (const sample_interfaces::msg::TextMsg& msg)const         { OutputMessage omsg{msg}; serialize_send(omsg); }
	void serialize_send(OutputMessage& msg) const
	{
		ros2_bridge::IoBuffer::UPtr buf_uptr = std::make_unique<ros2_bridge::IoBuffer>();
		serialize(msg, *buf_uptr);
		printf("bridge::serialize_send buffer: [%s]\n", buf_uptr->to_string().c_str());
		m_serial_link_uptr->send_threadsafe(std::move(buf_uptr));
		buf_uptr = nullptr;
		std::this_thread::yield();
	}
	void on_recv_message(ros2_bridge::IoBuffer::UPtr buffer)
	{
		printf("bridge on_recv_message: [%s]\n", buffer->to_string().c_str());
		InputMessage inmsg;
		if(!deserialize(*buffer, inmsg)) {
			buffer = nullptr;
			throw std::runtime_error("deserialization failed");
		} 
		buffer = nullptr;
		if(auto response_ptr = std::get_if<CmdResponse>(&inmsg)) {
			m_response_publisher_sptr->publish(*response_ptr);
		} else if(auto two_encoders_ptr = std::get_if<TwoEncoderStatus>(&inmsg)) {
			printf("bridge just got a twoencoder message\n");
			m_encoder_publisher_sptr->publish(*two_encoders_ptr);
		} else if(auto text_msg_ptr = std::get_if<TextMsg>(&inmsg)) {
			std_msgs::msg::String msg;
			m_text_in_publisher_sptr->publish(*text_msg_ptr);
		} else {
			printf("bridge::on_recv_message ELSE CLAUSE\n");
		}
		std::this_thread::yield();
	}
	/**
	 * Periodically send a message to the micro controller
	*/
  	void timer_callback()
	{
		return;
		m_count++;
		EchoCmd echo{};
		echo.data = std::vector{std::format("ABCDEFG{}", m_count)};
		ros2_bridge::IoBuffer::UPtr buf_uptr = std::make_unique<ros2_bridge::IoBuffer>();
		serialize(echo, *buf_uptr);
		m_serial_link_uptr->send_threadsafe(std::move(buf_uptr));
	}

	/**
	 * Declare members
	*/

	int m_byte_count;
	int m_msg_count;
	steady_clock::time_point m_start_time;
	rclcpp::GuardCondition::SharedPtr m_guard_condition_sptr;
	rclcpp::TimerBase::SharedPtr m_timer;

	rclcpp::Publisher<sample_interfaces::msg::CmdResponse>::SharedPtr      	  m_response_publisher_sptr;
	rclcpp::Publisher<sample_interfaces::msg::TwoEncoderStatus>::SharedPtr 	  m_encoder_publisher_sptr;
	rclcpp::Publisher<sample_interfaces::msg::TextMsg>::SharedPtr             m_text_in_publisher_sptr;
	
	rclcpp::Subscription<sample_interfaces::msg::EchoCmd>::SharedPtr       	  m_echo_subscriber_sptr;
	rclcpp::Subscription<sample_interfaces::msg::MotorRpmCmd>::SharedPtr   	  m_rpm_subscriber_sptr;
	rclcpp::Subscription<sample_interfaces::msg::MotorPwmCmd>::SharedPtr   	  m_pwm_subscriber_sptr;
	rclcpp::Subscription<sample_interfaces::msg::TextMsg>::SharedPtr       	  m_text_out_subscriber_sptr;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       		  m_twist_subscriber_sptr;
	rclcpp::Subscription<sample_interfaces::msg::LoadTestCmd>::SharedPtr      m_load_test_subscriber_sptr;
	rclcpp::Subscription<sample_interfaces::msg::ReadEncodersCmd>::SharedPtr m_read_encoders_subscriber_sptr;

	ros2_bridge::SerialLink::UPtr m_serial_link_uptr;
	size_t m_count;
};


int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node_sptr = std::make_shared<Bridge>();
	std::thread t{[&](){
		node_sptr->m_serial_link_uptr->run();
	}};
	printf("after serial thread start about to spin\n");
	rclcpp::spin(node_sptr);
	rclcpp::shutdown();
	return 0;
}
