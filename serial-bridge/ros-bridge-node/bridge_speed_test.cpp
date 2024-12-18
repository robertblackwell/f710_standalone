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
		m_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
		m_timer = this->create_wall_timer(2000000ms, std::bind(&Bridge::timer_callback, this));
		/**
		 * Start the serial link thread
		*/
		// m_serial_link_uptr->start_thread();
		// printf("after starting serial_link thread\n");
	}
	void on_recv_message(IoBuffer::UPtr buf_uptr)
	{
		if(m_msg_count == 0) {
			m_msg_count = 0;
			m_byte_count = 0;
			m_start_time = steady_clock::now();
		}
		m_byte_count += buf_uptr->size();
		m_msg_count += 1;
		#ifndef BRIDGE_SPEED_TEST_MESSAGE_COUNT
			#define BRIDGE_SPEED_TEST_MESSAGE_COUNT 10000
		#endif
		if(m_msg_count >= BRIDGE_SPEED_TEST_MESSAGE_COUNT) {
			steady_clock::time_point end = steady_clock::now();
			auto elapsed = duration_cast<milliseconds>(end - m_start_time).count();
			double bytes_per_second = ((double)m_byte_count) / (((double)elapsed)/(1000.0));
			printf("\n%d msgs byte_count: %ld  elapsed time %ld millisecs bytes/sec: %f\n", BRIDGE_SPEED_TEST_MESSAGE_COUNT, m_byte_count, elapsed, bytes_per_second);
			m_msg_count = 0;
			throw std::runtime_error("Test over time to stop - throwing an exception to end the program is a bit of a hack");
		} else {
			#if (BRIDGE_SPEED_TEST_VERBOSE==1)
				printf("ON RECV MESSAGE callback msg: [%s]\n", buf_uptr->to_string().c_str());
			#else
				printf(".");
				if(m_msg_count % 100 == 0){
					printf("\n");
				}
			#endif
		}
		// printf("ON RECV MESSAGE callback msg: [%s]\n", muptr->c_str());
	}
	/**
	 * Periodically send a message to the micro controller
	*/
  	void timer_callback()
	{

		m_count++;
		EchoCmd echo_cmd{};
		echo_cmd.data = std::vector{std::format("ABCDEFG{}\n", m_count)};
		IoBuffer::UPtr buf_uptr = std::make_unique<IoBuffer>();
		serialize(echo_cmd, *buf_uptr);
		// RCLCPP_INFO(this->get_logger(), "send_threadsafe : '%s'", echo_cmd.c_str());
		m_serial_link_uptr->send_threadsafe(std::move(buf_uptr));
	}
	std::size_t m_byte_count;
	int m_msg_count;
	steady_clock::time_point m_start_time;
	rclcpp::GuardCondition::SharedPtr m_guard_condition_sptr;
	rclcpp::TimerBase::SharedPtr m_timer;
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
	ros2_bridge::SerialLink::UPtr m_serial_link_uptr;
	size_t m_count;
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
