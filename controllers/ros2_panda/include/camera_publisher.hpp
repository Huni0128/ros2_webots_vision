#pragma once

#include <webots/Camera.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <webots/Robot.hpp>

class CameraPublisher {
public:
  CameraPublisher(webots::Robot* robot, rclcpp::Node::SharedPtr node, int timestep);
  void publish();

private:
  webots::Camera* camera_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;
  int timestep_;
};
