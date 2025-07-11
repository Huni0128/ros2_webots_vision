#include "camera_publisher.hpp"
#include <cstring>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

CameraPublisher::CameraPublisher(webots::Robot* robot,
                                 rclcpp::Node::SharedPtr node,
                                 int timestep)
    : node_(node),
      timestep_(timestep)
{
    // RGB camera
    camera_ = robot->getCamera("camera");
    camera_->enable(timestep_);
    rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS());

    // Depth sensor
    range_finder_ = robot->getRangeFinder("range_finder");
    range_finder_->enable(timestep_);
    depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "/range_finder/image_raw", rclcpp::SensorDataQoS());

    // Camera info publisher
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/camera_info", rclcpp::SensorDataQoS());

    // Static TF broadcaster
    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);

    // Compute intrinsics from FOV & resolution
    double fov    = camera_->getFov();
    int    w      = camera_->getWidth();
    int    h      = camera_->getHeight();
    double fx     = w / (2.0 * std::tan(fov / 2.0));
    double fy     = fx;
    double cx     = w / 2.0;
    double cy     = h / 2.0;

    // Fill CameraInfo template
    camera_info_msg_.header.frame_id  = "camera_color_frame";
    camera_info_msg_.width            = w;
    camera_info_msg_.height           = h;
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.d = {0, 0, 0, 0, 0};
    camera_info_msg_.k = {fx, 0.0, cx,
                          0.0, fy, cy,
                          0.0, 0.0, 1.0};
    camera_info_msg_.r = {1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0};
    camera_info_msg_.p = {fx, 0.0, cx, 0.0,
                          0.0, fy, cy, 0.0,
                          0.0, 0.0, 1.0, 0.0};

    // Publish static transform from robot base to camera frame
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = node_->get_clock()->now();
    tf_msg.header.frame_id         = "panda_link8";
    tf_msg.child_frame_id          = "camera_color_frame";
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.06;
    tf_msg.transform.rotation.x    = 0.0;
    tf_msg.transform.rotation.y    = -1.0;
    tf_msg.transform.rotation.z    = 0.0;
    tf_msg.transform.rotation.w    = 1.5708;
    tf_broadcaster_->sendTransform(tf_msg);
}

void CameraPublisher::publish()
{
    // 1) Publish RGB image
    const unsigned char* img = camera_->getImage();
    if (img) {
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp    = node_->get_clock()->now();
        msg.header.frame_id = "camera_color_frame";
        msg.height          = camera_->getHeight();
        msg.width           = camera_->getWidth();
        msg.encoding        = "rgb8";
        msg.is_bigendian    = 0;
        msg.step            = msg.width * 3;
        msg.data.resize(msg.height * msg.step);
        for (int y = 0; y < msg.height; ++y) {
            for (int x = 0; x < msg.width; ++x) {
                int src = 4 * (y * msg.width + x);
                int dst = 3 * (y * msg.width + x);
                msg.data[dst + 0] = img[src + 0];
                msg.data[dst + 1] = img[src + 1];
                msg.data[dst + 2] = img[src + 2];
            }
        }
        rgb_pub_->publish(std::move(msg));
    }

    // 2) Publish depth image
    const float* range = range_finder_->getRangeImage();
    if (range) {
        int w = range_finder_->getWidth();
        int h = range_finder_->getHeight();
        auto msg = sensor_msgs::msg::Image();
        msg.header.stamp    = node_->get_clock()->now();
        msg.header.frame_id = "camera_depth_frame";
        msg.height          = h;
        msg.width           = w;
        msg.encoding        = "32FC1";
        msg.is_bigendian    = 0;
        msg.step            = w * sizeof(float);
        msg.data.resize(h * msg.step);
        std::memcpy(msg.data.data(),
                    reinterpret_cast<const uint8_t*>(range),
                    msg.data.size());
        depth_pub_->publish(std::move(msg));
    }

    // 3) Publish camera info
    camera_info_msg_.header.stamp = node_->get_clock()->now();
    camera_info_pub_->publish(camera_info_msg_);
}
