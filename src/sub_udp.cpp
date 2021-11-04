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

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(std::string topic_name="bridge_node")
  : Node(topic_name), udp0("127.0.0.1", 4001), topic_name_(topic_name), close_(false)
  {
    string_publisher_ = this->create_publisher<std_msgs::msg::String>("string", 10);
    float64_publisher_ = this->create_publisher<std_msgs::msg::Float64>("double", 10);
    udp0.udp_bind();
    udp_subscribe_thread_ = std::thread(std::bind(&MinimalPublisher::subUdpThread, this));
  }
  ~MinimalPublisher()
  {
    close_ = true;
  }
private:
  simple_udp::msgs subscribe_udp()
  {
    simple_udp::msgs topic_type;
    udp0.udp_recv((char*)&topic_type, sizeof(simple_udp::msgs));
    std::string topic_name = udp0.udp_recv();
    return topic_type;
  }
     
  void subUdpThread()
  {
    while(!close_)
    {
      simple_udp::msgs topic_type = subscribe_udp();
      switch(topic_type)
      {
        case simple_udp::msgs::string:
        {
          std_msgs::msg::String msg;
          msg.data = udp0.udp_recv();
          string_publisher_->publish(msg);
        }
        break;
        case simple_udp::msgs::float64:
        {
          std_msgs::msg::Float64 msg;
          udp0.udp_recv((char*)&msg.data, sizeof(double));
          float64_publisher_->publish(msg);
        }
      }
    }
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr float64_publisher_;
  SimpleUdp udp0;
  std::thread udp_subscribe_thread_;
  std::string topic_name_;
  bool close_;
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv){

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node1 = std::make_shared<MinimalPublisher>();
  exec.add_node(node1);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}