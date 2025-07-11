#ifndef CAMERA_PUBLISHER_HPP
#define CAMERA_PUBLISHER_HPP

// Webots
#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Publishes camera RGB, depth, camera_info, and static TF.
class CameraPublisher {
public:
    // Constructor
    CameraPublisher(webots::Robot* robot,
                    rclcpp::Node::SharedPtr node,
                    int timestep);

    // Publish RGB, depth, CameraInfo, and static TF
    void publish();

private:
    // Webots sensors
    webots::Camera*      camera_;        ///< RGB camera
    webots::RangeFinder* range_finder_;  ///< Depth sensor

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      rgb_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // Static TF broadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    // Camera intrinsics template
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    // ROS2 node handle
    rclcpp::Node::SharedPtr node_;

    // Simulation timestep (ms)
    int timestep_;
};

#endif  // CAMERA_PUBLISHER_HPP
