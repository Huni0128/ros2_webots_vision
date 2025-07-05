#include "camera_publisher.hpp"

// 생성자: Webots 카메라 초기화 및 ROS 퍼블리셔 설정
CameraPublisher::CameraPublisher(webots::Robot* robot, rclcpp::Node::SharedPtr node, int timestep)
  : node_(node), timestep_(timestep) {
  camera_ = robot->getCamera("camera");              // Webots 카메라 객체 가져오기
  camera_->enable(timestep_);                        // 카메라 센서 활성화
  publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS()); // ROS2 퍼블리셔 생성
}

// 카메라 이미지 데이터를 ROS2 메시지로 변환해 퍼블리시
void CameraPublisher::publish() {
  const unsigned char* image_data = camera_->getImage(); // Webots에서 이미지 가져오기
  if (!image_data) return;

  auto msg = sensor_msgs::msg::Image();           // ROS2 이미지 메시지 생성
  msg.header.stamp = node_->get_clock()->now();   // 현재 시간 기록
  msg.height = camera_->getHeight();              // 이미지 높이
  msg.width = camera_->getWidth();                // 이미지 너비
  msg.encoding = "rgb8";                          // RGB 형식 지정
  msg.is_bigendian = 0;
  msg.step = msg.width * 3;                       // 한 줄당 바이트 수 (RGB)
  msg.data.resize(msg.height * msg.step);         // 전체 데이터 공간 확보

  // Webots 이미지(RGBA)를 ROS 이미지(RGB)로 변환
  for (int y = 0; y < msg.height; ++y) {
    for (int x = 0; x < msg.width; ++x) {
      int src_index = 4 * (y * msg.width + x);    // Webots RGBA 포맷 인덱스
      int dst_index = 3 * (y * msg.width + x);    // ROS RGB 포맷 인덱스

      msg.data[dst_index + 0] = image_data[src_index + 0];  // R
      msg.data[dst_index + 1] = image_data[src_index + 1];  // G
      msg.data[dst_index + 2] = image_data[src_index + 2];  // B
    }
  }

  publisher_->publish(msg);  // 토픽 퍼블리시
}
