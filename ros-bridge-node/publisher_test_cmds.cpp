#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2-serial-bridge/bridge_lib/iobuffer.h"
#include "ros2-serial-bridge/bridge_lib/serial_settings.h"
#include "ros2-serial-bridge/non_ros_messages/msgs.h"

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
		m_count =  0;
		m_cmd_index = 0;

		m_echo_publisher_sptr          = this->create_publisher<sample_interfaces::msg::EchoCmd>("echo_cmd", 10);
		m_rpm_publisher_sptr           = this->create_publisher<sample_interfaces::msg::MotorRpmCmd>("rpm_cmd", 10);
		m_pwm_publisher_sptr           = this->create_publisher<sample_interfaces::msg::MotorPwmCmd>("pwm_cmd", 10);
		m_text_publisher_sptr          = this->create_publisher<sample_interfaces::msg::TextMsg>("text_out", 10);
		m_load_test_publisher_sptr     = this->create_publisher<sample_interfaces::msg::LoadTestCmd>("loadtest", 10);
		m_read_encoders_publisher_sptr = this->create_publisher<sample_interfaces::msg::ReadEncodersCmd>("read_encoders_cmd", 10);
		
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
		buf_uptr = nullptr;
	}
	/**
	 * Periodically send a message to the micro controller
	*/
  	void timer_callback()
	{
		using namespace sample_interfaces;
		printf("Timer callback 1  cmd_index: %ld\n", m_cmd_index);

		auto f_echo = [&]() {auto msg = build<EchoCmd>().data({std::vector<std::string>{"AAAAAA","BBBBBB"}});	m_echo_publisher_sptr->publish(msg);	};
		auto f_pwm  = [&]() {auto msg = build<MotorPwmCmd>().left_motor_pwm(80.0).right_motor_pwm(80.0);	    m_pwm_publisher_sptr->publish(msg);	};
		auto f_read = [&]() {auto msg = build<ReadEncodersCmd>().n(1);	m_read_encoders_publisher_sptr->publish(msg);	};
		auto f_stop = [&]() {auto msg = build<MotorPwmCmd>().left_motor_pwm(0.0).right_motor_pwm(0.0);	m_pwm_publisher_sptr->publish(msg);	};

		printf("output_messages size: %ld\n", m_output_messages.size());
		switch(m_cmd_index) {
			case 0: f_echo();break;
			case 1: f_pwm(); break;
			case 2: f_read(); break;
			case 3: f_read(); break;
			case 4: f_read(); break;
			case 5: f_read(); break;
			case 6: f_read(); break;
			case 7: f_read(); break;
			case 8: f_read(); break;
			case 9: f_stop(); break;
		}
		m_cmd_index = (m_cmd_index + 1) % 10;
	}
	int m_byte_count;
	int m_msg_count;
	steady_clock::time_point m_start_time;
	rclcpp::GuardCondition::SharedPtr m_guard_condition_sptr;
	rclcpp::TimerBase::SharedPtr m_timer;

	rclcpp::Publisher<sample_interfaces::msg::EchoCmd>::SharedPtr       	  m_echo_publisher_sptr;
	rclcpp::Publisher<sample_interfaces::msg::MotorRpmCmd>::SharedPtr   	  m_rpm_publisher_sptr;
	rclcpp::Publisher<sample_interfaces::msg::MotorPwmCmd>::SharedPtr   	  m_pwm_publisher_sptr;
	rclcpp::Publisher<sample_interfaces::msg::TextMsg>::SharedPtr       	  m_text_publisher_sptr;
	rclcpp::Publisher<sample_interfaces::msg::LoadTestCmd>::SharedPtr         m_load_test_publisher_sptr;
	rclcpp::Publisher<sample_interfaces::msg::ReadEncodersCmd>::SharedPtr     m_read_encoders_publisher_sptr;


	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
	size_t m_count;

	size_t m_cmd_index;
	std::vector<OutputMessage> m_output_messages;
};

#include "ros2-serial-bridge/bridge_lib/iobuffer.h"
int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	auto node_sptr = std::make_shared<Bridge>();
	rclcpp::spin(node_sptr);
	rclcpp::shutdown();
	return 0;
}
