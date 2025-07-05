#ifndef PANDA_GRIPPER_CONTROLLER_HPP
#define PANDA_GRIPPER_CONTROLLER_HPP

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Panda 그리퍼 제어 클래스
class PandaGripperController {
public:
  // 생성자
  PandaGripperController(webots::Robot* robot, rclcpp::Node::SharedPtr node, int timestep);

  // 주기적으로 호출되어 상태 업데이트 수행
  void update();

private:
  // Webots 모터 및 센서
  webots::Motor* left_finger_;
  webots::Motor* right_finger_;
  webots::PositionSensor* left_sensor_;
  webots::PositionSensor* right_sensor_;

  int timestep_;  // Webots 타임스텝

  // ROS2 노드 및 통신 객체
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;

  // 명령 콜백: 그리퍼 위치 설정
  void commandCallback(const std_msgs::msg::Float64::SharedPtr msg);

  // 현재 그리퍼 상태 퍼블리시
  void publishState();
};

#endif  // PANDA_GRIPPER_CONTROLLER_HPP
