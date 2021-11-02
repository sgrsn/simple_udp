#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "simple_udp/simple_udp.h"

template <class sub_type>
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher(std::string topic_type, std::string topic_name)
  : Node(topic_name), udp0("127.0.0.1", 4001), topic_type_(topic_type), topic_name_(topic_name), close_(false)
  {
    publisher_ = this->create_publisher<sub_type>(topic_name_, 10);
    udp0.udp_bind();
    udp_subscribe_thread_ = std::thread(std::bind(&MinimalPublisher::subUdpThread, this));
  }
  ~MinimalPublisher()
  {
    close_ = true;
  }
private:
  void subUdpThread()
  {
    while(!close_)
    {
      std::string topic_type = udp0.udp_recv();
      std::string topic_name = udp0.udp_recv();
      //double dval = 0;
      if(topic_type == "std_msgs::msg::String")
      {
        std::string data = udp0.udp_recv();
        printf("recv:%s, %s, %s\n", topic_type.c_str(), topic_name.c_str(), data.c_str());
        std_msgs::msg::String pub_msg;
        pub_msg.data = data;
        publisher_->publish(pub_msg);
      }
      /*else if(topic_type == "std_msgs::msg::Float64")
      {
        udp0.udp_recv((char*)&dval, sizeof(double));
        printf("recv:%s, %s, %f\n", topic_type.c_str(), topic_name.c_str(), dval);
      }*/
    }
  }
  typename rclcpp::Publisher<sub_type>::SharedPtr publisher_;
  SimpleUdp udp0;
  std::thread udp_subscribe_thread_;
  std::string topic_type_;
  std::string topic_name_;
  bool close_;
};

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv){

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node1 = std::make_shared<MinimalPublisher<std_msgs::msg::String>>("std_msgs::msg::String", "string");
  exec.add_node(node1);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}