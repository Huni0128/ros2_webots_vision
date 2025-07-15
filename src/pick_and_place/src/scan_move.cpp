#include <memory>
#include <vector>
#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace std::chrono_literals;

class ScanMove : public rclcpp::Node
{
public:
  ScanMove()
  : Node("scan_move"),
    scanning_active_(false),
    direction_(+1),
    target_color_id_(-1)
  {
    // 파라미터 선언 및 읽기
    this->declare_parameter<double>("scan.limit_deg", 90.0);
    this->declare_parameter<double>("scan.step_deg",   2.0);
    this->declare_parameter<int>("scan.joint_index",  2);

    this->get_parameter("scan.limit_deg",   scan_limit_deg_);
    this->get_parameter("scan.step_deg",    scan_step_deg_);
    this->get_parameter("scan.joint_index", joint_index_);

    scan_limit_rad_ = scan_limit_deg_ * M_PI / 180.0;
    scan_step_rad_  = scan_step_deg_  * M_PI / 180.0;

    current_joint_positions_.assign(7, 0.0);  // 초기 관절값

    // 픽 명령 수신 (/pick_box)
    pick_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pick_box", 10,
      std::bind(&ScanMove::onPickBox, this, std::placeholders::_1)
    );

    // 상태 수신 (/pick_box_status)
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pick_box_status", 10,
      std::bind(&ScanMove::onStatus, this, std::placeholders::_1)
    );

    // 감지 결과 수신 (/object_detector/detections)
    detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
      "/object_detector/detections", 10,
      std::bind(&ScanMove::onDetection, this, std::placeholders::_1)
    );

    // 관절 상태 수신 (/panda_joint_states)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/panda_joint_states", 10,
      std::bind(&ScanMove::jointStateCallback, this, std::placeholders::_1)
    );

    // 관절 명령 퍼블리셔 (/panda_joint_command)
    joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/panda_joint_command", 10
    );

    // 주기적 타이머 (50Hz)
    timer_ = this->create_wall_timer(
      20ms,
      std::bind(&ScanMove::onTimer, this)
    );

    RCLCPP_INFO(get_logger(),
      "ScanMove ready: joint[%d], limit=±%.1f°, step=%.1f°",
      joint_index_, scan_limit_deg_, scan_step_deg_);
  }

private:
  // 스캔 상태 및 방향
  bool scanning_active_;
  int  direction_;        // +1: 우회전, -1: 좌회전
  int  target_color_id_;  // 목표 색상 ID (0~2), 없으면 -1
  int  joint_index_;

  // 스캔 파라미터
  double scan_limit_deg_, scan_step_deg_;
  double scan_limit_rad_, scan_step_rad_;

  // 현재 관절 위치
  std::vector<double> current_joint_positions_;

  // ROS 통신 객체
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pick_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 픽 명령 콜백
  void onPickBox(const std_msgs::msg::String::SharedPtr msg)
  {
    auto c = msg->data;
    if      (c == "red_box")   target_color_id_ = 0;
    else if (c == "green_box") target_color_id_ = 1;
    else if (c == "blue_box")  target_color_id_ = 2;
    else {
      RCLCPP_WARN(get_logger(), "Unknown pick_box '%s'", c.c_str());
      target_color_id_ = -1;
      return;
    }

    RCLCPP_INFO(get_logger(),
      "pick_box command: target_color='%s' id=%d",
      c.c_str(), target_color_id_);
  }

  // 상태 수신 콜백
  void onStatus(const std_msgs::msg::String::SharedPtr msg)
  {
    if (target_color_id_ < 0) return;

    auto s = msg->data;
    if (s == "SCANNING" || s == "NOT_FOUND") {
      if (!scanning_active_) {
        scanning_active_ = true;
        direction_ = +1;
        RCLCPP_INFO(get_logger(), "Status '%s' → resume scanning", s.c_str());
      }
    } else if (s == "MATCHED") {
      if (scanning_active_) {
        scanning_active_ = false;
        double deg = current_joint_positions_[joint_index_] * 180.0 / M_PI;
        RCLCPP_INFO(get_logger(), "Status 'MATCHED' → hold at %.2f°", deg);
      }
    }
  }

  // 감지 결과 수신 콜백
  void onDetection(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (!scanning_active_ || msg->detections.empty()) return;

    int det_id = std::stoi(msg->detections[0].results[0].hypothesis.class_id);

    switch (det_id) {
      case 1: direction_ = +1; break;  // green → 오른쪽
      case 2: direction_ = -1; break;  // blue  → 왼쪽
      case 0:                        // red: context에 따라 방향 결정
        if (target_color_id_ == 1)      direction_ = -1;
        else if (target_color_id_ == 2) direction_ = +1;
        break;
      default: return;
    }

    RCLCPP_DEBUG(get_logger(),
      "Detected id=%d → direction=%s",
      det_id, (direction_ > 0 ? "RIGHT" : "LEFT"));
  }

  // 관절 상태 수신 콜백
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() >= current_joint_positions_.size()) {
      current_joint_positions_ = msg->position;
    }
  }

  // 타이머 콜백: 주기적 스캔 동작
  void onTimer()
  {
    if (!scanning_active_) return;

    double &cur = current_joint_positions_[joint_index_];
    double next = cur + direction_ * scan_step_rad_;

    if (next > scan_limit_rad_) {
      next = scan_limit_rad_;
      direction_ = -1;
    } else if (next < -scan_limit_rad_) {
      next = -scan_limit_rad_;
      direction_ = +1;
    }

    cur = next;

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = current_joint_positions_;
    joint_cmd_pub_->publish(cmd);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMove>());
  rclcpp::shutdown();
  return 0;
}
