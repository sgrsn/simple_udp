#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "simple_udp/simple_udp.h"
#include "simple_udp/simple_msgs.h"
#include "user_msgs/msg/motor_state.hpp"

template <class sub_type>
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(simple_udp::msgs topic_type, std::string topic_name)
  : Node(topic_name), udp0("192.168.0.20 ", 4001), topic_type_(topic_type), topic_name_(topic_name)
  {
    subscription_ = this->create_subscription<sub_type>(topic_name, 10, std::bind(&MinimalSubscriber::callback, this, std::placeholders::_1));
    udp0.udp_bind();
  }
private:
  void callback(typename sub_type::SharedPtr msg)
  {
    udp0.udp_send<simple_udp::msgs>(topic_type_);
    udp0.udp_send<std::string>(topic_name_);
    send_raw_data(msg);
  }
  void send_raw_data(typename sub_type::SharedPtr msg)
  {
    udp0.udp_send<sub_type>(msg->data);
  }
  typename rclcpp::Subscription<sub_type>::SharedPtr subscription_;
  SimpleUdp udp0;
  simple_udp::msgs topic_type_;
  std::string topic_name_;
};

template<>
void MinimalSubscriber<std_msgs::msg::String>::send_raw_data(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  udp0.udp_send<std::string>(msg->data);
}

template<>
void MinimalSubscriber<std_msgs::msg::Float64>::send_raw_data(const std_msgs::msg::Float64::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
  udp0.udp_send<double>(msg->data);
}
template<>
void MinimalSubscriber<user_msgs::msg::MotorState>::send_raw_data(const user_msgs::msg::MotorState::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "I heard: MotorState %f, %f", msg->angle, msg->velocity);
  udp0.udp_send<double>(msg->angle);
  udp0.udp_send<double>(msg->velocity);
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{ 
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node1 = std::make_shared<MinimalSubscriber<user_msgs::msg::MotorState>>(simple_udp::msgs::motor_state, "motor_state");
  exec.add_node(node1);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}