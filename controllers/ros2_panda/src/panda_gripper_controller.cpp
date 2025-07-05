#include "panda_gripper_controller.hpp"

PandaGripperController::PandaGripperController(webots::Robot* robot, rclcpp::Node::SharedPtr node, int timestep)
  : node_(node), timestep_(timestep) {
  // Webots의 모터 및 센서 초기화 (이름은 PROTO 내 LinearMotor 및 PositionSensor에 정의된 이름 사용)
  left_finger_ = robot->getMotor("panda_finger::left");
  right_finger_ = robot->getMotor("panda_finger::right");

  left_sensor_ = robot->getPositionSensor("panda_finger::left_sensor");
  right_sensor_ = robot->getPositionSensor("panda_finger::right_sensor");

  if (left_sensor_) left_sensor_->enable(timestep_);
  if (right_sensor_) right_sensor_->enable(timestep_);

  // ROS2 구독자: 그리퍼 명령 수신
  command_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
    "/panda_gripper_command", 10,
    std::bind(&PandaGripperController::commandCallback, this, std::placeholders::_1));

  // ROS2 퍼블리셔: 그리퍼 상태 퍼블리시
  state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/panda_gripper_state", 10);
}

// 매 루프마다 호출되어 상태 퍼블리시
void PandaGripperController::update() {
  rclcpp::spin_some(node_);
  publishState();
}

// ROS2 명령 수신 시 호출되는 콜백: 위치 명령을 모터에 전달
void PandaGripperController::commandCallback(const std_msgs::msg::Float64::SharedPtr msg) {
  double position = std::clamp(msg->data, 0.0, 0.04);  // 최대 개방 범위 제한
  if (left_finger_) left_finger_->setPosition(position);
  if (right_finger_) right_finger_->setPosition(position);
}

// 현재 그리퍼 센서 값을 JointState 메시지로 퍼블리시
void PandaGripperController::publishState() {
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->now();
  msg.name = {"panda_finger_joint1", "panda_finger_joint2"};

  if (left_sensor_ && right_sensor_) {
    msg.position = {left_sensor_->getValue(), right_sensor_->getValue()};
  }

  state_pub_->publish(msg);
}
