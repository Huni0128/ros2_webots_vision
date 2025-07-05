#include "camera_publisher.hpp"

CameraPublisher::CameraPublisher(webots::Robot* robot, rclcpp::Node::SharedPtr node, int timestep)
  : node_(node), timestep_(timestep) {
  camera_ = robot->getCamera("camera");
  camera_->enable(timestep_);
  publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS());
}

void CameraPublisher::publish() {
  const unsigned char* image_data = camera_->getImage();
  if (!image_data) return;

  auto msg = sensor_msgs::msg::Image();
  msg.header.stamp = node_->get_clock()->now();
  msg.height = camera_->getHeight();
  msg.width = camera_->getWidth();
  msg.encoding = "rgb8";  // 또는 bgr8도 가능
  msg.is_bigendian = 0;
  msg.step = msg.width * 3;
  msg.data.resize(msg.height * msg.step);

  for (int y = 0; y < msg.height; ++y) {
    for (int x = 0; x < msg.width; ++x) {
      int src_index = 4 * (y * msg.width + x);   // Webots: RGBA
      int dst_index = 3 * (y * msg.width + x);   // ROS: RGB

      msg.data[dst_index + 0] = image_data[src_index + 0];  // R
      msg.data[dst_index + 1] = image_data[src_index + 1];  // G
      msg.data[dst_index + 2] = image_data[src_index + 2];  // B
    }
  }

  publisher_->publish(msg);
}

