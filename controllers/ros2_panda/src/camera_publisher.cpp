#include "camera_publisher.hpp"
#include <cstring>  // for memcpy

CameraPublisher::CameraPublisher(webots::Robot* robot,
                                 rclcpp::Node::SharedPtr node,
                                 int timestep)
  : node_(node),
    timestep_(timestep) {
  // --- RGB 카메라 셋업 ---
  camera_ = robot->getCamera("camera");
  camera_->enable(timestep_);
  rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS());

  // --- RangeFinder 셋업 ---
  range_finder_ = robot->getRangeFinder("range_finder");
  range_finder_->enable(timestep_);
  depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/range_finder/image_raw", rclcpp::SensorDataQoS());
}

void CameraPublisher::publish() {
  // ==== 1) RGB 이미지 퍼블리시 ====
  const unsigned char* image_data = camera_->getImage();
  if (image_data) {
    auto rgb_msg = sensor_msgs::msg::Image();
    rgb_msg.header.stamp = node_->get_clock()->now();
    rgb_msg.header.frame_id = "camera_color_frame";
    rgb_msg.height = camera_->getHeight();
    rgb_msg.width  = camera_->getWidth();
    rgb_msg.encoding = "rgb8";
    rgb_msg.is_bigendian = 0;
    rgb_msg.step = rgb_msg.width * 3;
    rgb_msg.data.resize(rgb_msg.height * rgb_msg.step);

    // Webots RGBA → ROS RGB
    for (int y = 0; y < rgb_msg.height; ++y) {
      for (int x = 0; x < rgb_msg.width; ++x) {
        int src = 4 * (y * rgb_msg.width + x);
        int dst = 3 * (y * rgb_msg.width + x);
        rgb_msg.data[dst + 0] = image_data[src + 0];
        rgb_msg.data[dst + 1] = image_data[src + 1];
        rgb_msg.data[dst + 2] = image_data[src + 2];
      }
    }
    rgb_pub_->publish(std::move(rgb_msg));
  }

  // ==== 2) 깊이(depth) 이미지 퍼블리시 ====
  const float* range_image = range_finder_->getRangeImage();
  if (range_image) {
    int w = range_finder_->getWidth();
    int h = range_finder_->getHeight();

    auto depth_msg = sensor_msgs::msg::Image();
    depth_msg.header.stamp = node_->get_clock()->now();
    depth_msg.header.frame_id = "camera_depth_frame";
    depth_msg.height = h;
    depth_msg.width  = w;
    depth_msg.encoding = "32FC1";            // 32-bit float 단일 채널
    depth_msg.is_bigendian = 0;
    depth_msg.step = w * sizeof(float);      // 한 줄당 바이트 수
    depth_msg.data.resize(h * depth_msg.step);

    // 메모리 복사: Webots float 배열 → ROS 메시지 버퍼
    std::memcpy(depth_msg.data.data(),
                reinterpret_cast<const uint8_t*>(range_image),
                depth_msg.data.size());

    depth_pub_->publish(std::move(depth_msg));
  }
}
