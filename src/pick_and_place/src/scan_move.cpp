// scan_move.cpp

#include <memory>
#include <vector>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class ScanMove : public rclcpp::Node
{
public:
  ScanMove()
  : Node("scan_move"),
    scanning_active_(false),
    direction_(+1)
  {
    // 파라미터 선언 및 가져오기
    this->declare_parameter<double>("scan.limit_deg", 90.0);
    this->declare_parameter<double>("scan.step_deg", 2.0);
    this->declare_parameter<int>("scan.joint_index", 2);  // 기본: 3번 관절 (0-based)

    this->get_parameter("scan.limit_deg", scan_limit_deg_);
    this->get_parameter("scan.step_deg", scan_step_deg_);
    this->get_parameter("scan.joint_index", joint_index_);

    scan_limit_rad_ = scan_limit_deg_ * M_PI / 180.0;
    scan_step_rad_  = scan_step_deg_  * M_PI / 180.0;

    // 초기 관절값 (7자유도)
    current_joint_positions_.assign(7, 0.0);

    // 상태 수신 (/pick_box_status)
    status_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/pick_box_status", 10,
      std::bind(&ScanMove::statusCallback, this, std::placeholders::_1)
    );

    // 관절 상태 수신 (/panda_joint_states)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/panda_joint_states", 10,
      std::bind(&ScanMove::jointStateCallback, this, std::placeholders::_1)
    );

    // 관절 제어 퍼블리셔 (/panda_joint_command)
    joint_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/panda_joint_command", 10
    );

    // 주기적 타이머 (50Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ScanMove::onTimer, this)
    );

    RCLCPP_INFO(get_logger(),
      "ScanMove initialized: joint[%d], limit=±%.1f°, step=%.1f°",
      joint_index_, scan_limit_deg_, scan_step_deg_
    );
  }

private:
  // 노드 상태 변수
  bool scanning_active_;
  int  direction_;
  int  joint_index_;

  // 파라미터 값
  double scan_limit_deg_, scan_step_deg_;
  double scan_limit_rad_, scan_step_rad_;

  // 관절 위치 저장
  std::vector<double> current_joint_positions_;

  // ROS 통신 객체
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr        status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                  timer_;

  // 상태 메시지 수신 콜백
  void statusCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto s = msg->data;
    if ((s == "SCANNING" || s == "NOT_FOUND") && !scanning_active_) {
      scanning_active_ = true;
      direction_ = +1;
      RCLCPP_INFO(get_logger(), "Start scanning joint[%d]", joint_index_);
    }
    else if (s == "MATCHED" && scanning_active_) {
      scanning_active_ = false;
      double deg = current_joint_positions_[joint_index_] * 180.0 / M_PI;
      RCLCPP_INFO(get_logger(), "Matched! Holding joint[%d] at %.2f°", joint_index_, deg);
      publishCurrentPosition();
    }
  }

  // 관절 상태 수신 콜백
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() >= current_joint_positions_.size()) {
      current_joint_positions_ = msg->position;
    }
  }

  // 주기적 스캔 동작 수행
  void onTimer()
  {
    if (!scanning_active_) return;

    double next = current_joint_positions_[joint_index_] + direction_ * scan_step_rad_;
    if (next >  scan_limit_rad_) { next =  scan_limit_rad_; direction_ = -1; }
    if (next < -scan_limit_rad_) { next = -scan_limit_rad_; direction_ = +1; }

    current_joint_positions_[joint_index_] = next;
    publishCurrentPosition();
  }

  // 현재 관절 위치 명령 퍼블리시
  void publishCurrentPosition()
  {
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
