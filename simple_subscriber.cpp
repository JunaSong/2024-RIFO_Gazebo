#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using std::placeholders::_1;

class CppSubscriber : public rclcpp::Node
{
  public:
    CppSubscriber()
    : Node("simple_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Int32>("pubsub_topic", 10, std::bind(&CppSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Hello, Robotory: %d", msg->data);
    }
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppSubscriber>());
  rclcpp::shutdown();
  return 0;
}