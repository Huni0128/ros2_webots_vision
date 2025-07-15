#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// GUI 명령 수신 및 상태 퍼블리시 노드
class CommandSub : public rclcpp::Node
{
public:
  CommandSub()
  : Node("command_sub")
  {
    // /pick_box 구독 (객체 픽 명령 수신)
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pick_box", 10,
      std::bind(&CommandSub::on_command, this, std::placeholders::_1)
    );

    // /pick_box_status 퍼블리셔 (상태 전달)
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/pick_box_status", 10
    );

    RCLCPP_INFO(this->get_logger(), "CommandSub node started.");
  }

private:
  // 명령 수신 콜백
  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());

    // 상태 메시지 퍼블리시
    auto status = std_msgs::msg::String();
    status.data = "SCANNING";
    status_pub_->publish(status);

    RCLCPP_INFO(this->get_logger(), "Status: '%s'", status.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandSub>());
  rclcpp::shutdown();
  return 0;
}
