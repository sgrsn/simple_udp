#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "simple_udp/simple_udp.h"

template <class sub_type>
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber(std::string topic_type, std::string topic_name)
  : Node(topic_name), udp0("127.0.0.1", 4001), topic_type_(topic_type), topic_name_(topic_name)
  {
    subscription_ = this->create_subscription<sub_type>(topic_name, 10, std::bind(&MinimalSubscriber::callback, this, std::placeholders::_1));
    udp0.udp_bind();
  }
private:
  void callback(typename sub_type::SharedPtr msg)
  {
    udp0.udp_send<std::string>(topic_type_);
    udp0.udp_send<std::string>(topic_name_);
    send_raw_data(msg);
  }
  void send_raw_data(typename sub_type::SharedPtr msg)
  {
    udp0.udp_send<sub_type>(msg->data);
  }
  typename rclcpp::Subscription<sub_type>::SharedPtr subscription_;
  SimpleUdp udp0;
  std::string topic_type_;
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

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{ 
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node1 = std::make_shared<MinimalSubscriber<std_msgs::msg::String>>("std_msgs::msg::String", "simple_udp_string");
  exec.add_node(node1);
  auto node2 = std::make_shared<MinimalSubscriber<std_msgs::msg::Float64>>("std_msgs::msg::Float64", "simple_udp_double");
  exec.add_node(node2);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}