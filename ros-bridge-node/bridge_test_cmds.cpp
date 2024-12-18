#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2-serial-bridge/bridge_lib/iobuffer.h"
#include "ros2-serial-bridge/bridge_lib/serial_settings.h"
#include "ros2-serial-bridge/non_ros_messages/msgs.h"
#include "ros2-serial-bridge/bridge_lib/serial_link.h"

using namespace std::chrono_literals;
using namespace std::chrono;
using namespace ros2_bridge;

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
		m_cmd_index = 0;
		m_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
		m_timer = this->create_wall_timer(5000ms, std::bind(&Bridge::timer_callback, this));
		/**
		 * Start the serial link thread
		*/
		// m_serial_link_uptr->start_thread();
		// printf("after starting serial_link thread\n");
	}
	void on_recv_message(IoBuffer::UPtr buf_uptr)
	{
		printf("ON RECV MESSAGE callback msg: [%s]\n", buf_uptr->to_string().c_str());
		InputMessage inmsg;
		if(!deserialize(*buf_uptr, inmsg)) {
			throw std::runtime_error("deserialization failed");
		} 
		if(auto response_ptr = std::get_if<CmdResponse>(&inmsg)) {
			printf("command response message\n");
			// m_response_publisher_sptr->publish(*response_ptr);
		} else if(auto two_encoders_ptr = std::get_if<TwoEncoderStatus>(&inmsg)) {
			printf("bridge just got a twoencoder message\n");
			// m_encoder_publisher_sptr->publish(*two_encoders_ptr);
		} else if(auto text_msg_ptr = std::get_if<TextMsg>(&inmsg)) {
			std_msgs::msg::String msg;
			// m_text_publisher_sptr->publish(*text_msg_ptr);
		} else {
			printf("bridge::on_recv_message ELSE CLAUSE\n");
		}
		buf_uptr = nullptr;
	}
	/**
	 * Periodically send a message to the micro controller
	*/
  	void timer_callback()
	{
		using namespace sample_interfaces;
		printf("Timer callback 1  cmd_index: %ld\n", m_cmd_index);

		auto make_iobuffer_uptr = [](OutputMessage& msg) {
			IoBuffer::UPtr buf_uptr = std::make_unique<IoBuffer>();
			serialize(msg, *buf_uptr);
			return buf_uptr;
		};
		// std::vector<OutputMessage> output_messages;
		auto add_to_vector = [&](const OutputMessage& om) {
			m_output_messages.emplace_back(om);
		};
		if(m_output_messages.empty()) {
			add_to_vector(build<EchoCmd>().data({std::vector<std::string>{"AAAAAA","BBBBBB"}}));
			add_to_vector(build<MotorPwmCmd>().left_motor_pwm(80.0).right_motor_pwm(80.0));
			add_to_vector(build<ReadEncodersCmd>().n(1));
			add_to_vector(build<ReadEncodersCmd>().n(1));
			add_to_vector(build<ReadEncodersCmd>().n(1));
			add_to_vector(build<ReadEncodersCmd>().n(1));
			add_to_vector(build<ReadEncodersCmd>().n(1));
			add_to_vector(build<ReadEncodersCmd>().n(1));
			add_to_vector(build<ReadEncodersCmd>().n(1));
			add_to_vector(build<MotorPwmCmd>().left_motor_pwm(0.0).right_motor_pwm(0.0));
		}
		printf("output_messages size: %ld\n", m_output_messages.size());

		std::size_t cmd_list_size = m_output_messages.size();
		// printf("Timer callback 3 cmd_list.size(): %ld\n", cmd_list_size);
		IoBuffer::UPtr buf_uptr = make_iobuffer_uptr(m_output_messages[m_cmd_index]);
		// printf("timer_callback m_count: %ld buffer: %s\n", m_count, buf_uptr->to_string().c_str());
		m_serial_link_uptr->send_threadsafe(std::move(buf_uptr));
		m_cmd_index = (m_cmd_index + 1) % cmd_list_size;
	}
	int m_byte_count;
	int m_msg_count;
	steady_clock::time_point m_start_time;
	rclcpp::GuardCondition::SharedPtr m_guard_condition_sptr;
	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
	ros2_bridge::SerialLink::UPtr m_serial_link_uptr;
	size_t m_count;

	size_t m_cmd_index;
	std::vector<OutputMessage> m_output_messages;
};

#include "ros2-serial-bridge/bridge_lib/iobuffer.h"
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
