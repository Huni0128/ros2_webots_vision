#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

class CommandSub : public rclcpp::Node
{
public:
  CommandSub()
  : Node("command_sub"), has_command_(false)
  {
    // 픽 명령 수신 (/pick_box)
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pick_box", 10,
      std::bind(&CommandSub::on_command, this, std::placeholders::_1)
    );

    // 객체 감지 결과 수신 (/object_detector/detections)
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/object_detector/detections", 10,
      std::bind(&CommandSub::on_detection, this, std::placeholders::_1)
    );

    // 상태 메시지 퍼블리시 (/pick_box_status)
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/pick_box_status", 10
    );

    RCLCPP_INFO(this->get_logger(), "CommandSub node started.");
  }

private:
  std::string current_command_;
  bool has_command_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr       command_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr         status_pub_;

  // 명령 수신 콜백
  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    current_command_ = msg->data;
    has_command_ = true;
    RCLCPP_INFO(this->get_logger(), "Received pick command: '%s'", current_command_.c_str());

    // 명령 수신 후 즉시 SCANNING 상태 전송
    auto status = std_msgs::msg::String();
    status.data = "SCANNING";
    status_pub_->publish(status);
    RCLCPP_INFO(this->get_logger(), "Published status: '%s'", status.data.c_str());
  }

  // 감지 결과 수신 콜백
  void on_detection(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (!has_command_) return;

    // 명령 색상 → 클래스 ID
    int target_id = color_to_id(current_command_);
    if (target_id < 0) {
      RCLCPP_WARN(this->get_logger(), "Unknown command color: '%s'", current_command_.c_str());
      has_command_ = false;
      return;
    }

    // 감지 결과 중 타겟 ID가 있는지 확인
    bool matched = false;
    for (const auto &det : msg->detections) {
      for (const auto &hypo : det.results) {
        if (std::stoi(hypo.hypothesis.class_id) == target_id) {
          matched = true;
          break;
        }
      }
      if (matched) break;
    }

    // 상태 전송 (MATCHED 또는 NOT_FOUND)
    auto status = std_msgs::msg::String();
    if (matched) {
      status.data = "MATCHED";
      has_command_ = false;  // 매칭 성공 시 명령 소멸
    } else {
      status.data = "NOT_FOUND";  // 실패 시 상태만 갱신, 명령 유지
    }

    status_pub_->publish(status);
    RCLCPP_INFO(this->get_logger(), "Published status: '%s'", status.data.c_str());
  }

  // 색상 이름 → 클래스 ID 변환
  int color_to_id(const std::string &color) const
  {
    if (color == "red_box")   return 0;
    if (color == "green_box") return 1;
    if (color == "blue_box")  return 2;
    return -1;
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandSub>());
  rclcpp::shutdown();
  return 0;
}
