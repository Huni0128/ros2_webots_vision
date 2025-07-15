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
    // 픽 명령 수신 구독자
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pick_box", 10,
      std::bind(&CommandSub::on_command, this, std::placeholders::_1)
    );

    // 박스 감지 결과 구독자
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/object_detector/detections", 10,
      std::bind(&CommandSub::on_detection, this, std::placeholders::_1)
    );

    // 상태 메시지 퍼블리셔
    status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/pick_box_status", 10
    );

    RCLCPP_INFO(this->get_logger(), "CommandSub node started.");
  }

private:
  // 현재 명령된 색상 및 처리 여부
  std::string current_command_;
  bool has_command_;

  // ROS 통신 객체
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // 명령 수신 콜백
  void on_command(const std_msgs::msg::String::SharedPtr msg)
  {
    current_command_ = msg->data;
    has_command_ = true;
    RCLCPP_INFO(this->get_logger(), "Received pick command: '%s'", current_command_.c_str());

    auto status = std_msgs::msg::String();
    status.data = "SCANNING";
    status_pub_->publish(status);
    RCLCPP_INFO(this->get_logger(), "Published status: '%s'", status.data.c_str());
  }

  // 감지 결과 수신 콜백
  void on_detection(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (!has_command_) {
      return;  // 명령 없으면 무시
    }

    // 색상 문자열 → 클래스 ID 변환
    int target_id = color_to_id(current_command_);
    if (target_id < 0) {
      RCLCPP_WARN(this->get_logger(), "Unknown command color: '%s'", current_command_.c_str());
      has_command_ = false;
      return;
    }

    // 감지된 객체 중 해당 ID 존재 여부 확인
    bool matched = false;
    for (const auto &det : msg->detections) {
      for (const auto &hypo : det.results) {
        int detected_id = std::stoi(hypo.hypothesis.class_id);
        if (detected_id == target_id) {
          matched = true;
          break;
        }
      }
      if (matched) break;
    }

    // 상태 퍼블리시 (매칭 여부에 따라)
    auto status = std_msgs::msg::String();
    status.data = matched ? "MATCHED" : "NOT_FOUND";
    status_pub_->publish(status);
    RCLCPP_INFO(this->get_logger(), "Published status: '%s'", status.data.c_str());

    has_command_ = false;  // 명령 완료 처리
  }

  // 색상 이름 → 클래스 ID 매핑
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
