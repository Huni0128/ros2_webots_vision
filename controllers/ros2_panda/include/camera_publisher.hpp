// Webots 및 ROS2 관련 헤더
#include <webots/Camera.hpp>
#include <webots/Robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// Webots 카메라 영상을 ROS2 토픽으로 퍼블리시하는 클래스
class CameraPublisher {
public:
  // 생성자: Webots 로봇, ROS2 노드, 타임스텝 초기화
  CameraPublisher(webots::Robot* robot, rclcpp::Node::SharedPtr node, int timestep);

  // 카메라 이미지를 ROS2로 퍼블리시
  void publish();

private:
  webots::Camera* camera_;  // Webots 카메라 객체
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;  // ROS2 퍼블리셔
  rclcpp::Node::SharedPtr node_;  // ROS2 노드 핸들
  int timestep_;  // Webots 시뮬레이션 타임스텝 (ms)
};
