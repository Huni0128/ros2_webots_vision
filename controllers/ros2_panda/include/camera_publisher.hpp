#ifndef CAMERA_PUBLISHER_HPP
#define CAMERA_PUBLISHER_HPP

// Webots
#include <webots/Robot.hpp>        // ← add this
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraPublisher {
public:
  // 생성자: Webots 로봇, ROS2 노드, 타임스텝(ms) 초기화
  CameraPublisher(webots::Robot* robot,
                  rclcpp::Node::SharedPtr node,
                  int timestep);

  // RGB & Depth 이미지를 ROS2로 퍼블리시
  void publish();

private:
  // Webots 센서
  webots::Camera*      camera_;
  webots::RangeFinder* range_finder_;

  // ROS2 퍼블리셔
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;

  // ROS2 노드 핸들
  rclcpp::Node::SharedPtr node_;

  // Webots 타임스텝 (ms)
  int timestep_;
};

#endif  // CAMERA_PUBLISHER_HPP
