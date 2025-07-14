#ifndef CAMERA_PUBLISHER_HPP
#define CAMERA_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>

class CameraPublisher
{
public:
    CameraPublisher(webots::Robot* robot, rclcpp::Node::SharedPtr node, int timestep);
    void publish();

private:
    rclcpp::Node::SharedPtr node_;
    int timestep_;

    webots::Camera* camera_;
    webots::RangeFinder* range_finder_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    sensor_msgs::msg::CameraInfo camera_info_msg_;
};

#endif // CAMERA_PUBLISHER_HPP
