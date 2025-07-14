#include "camera_publisher.hpp"
#include <cstring>

// 생성자: Webots 로봇, ROS 노드, 타임스텝을 받아 카메라 및 퍼블리셔 초기화
CameraPublisher::CameraPublisher(webots::Robot* robot,
                                 rclcpp::Node::SharedPtr node,
                                 int timestep)
  : node_(node),
    timestep_(timestep)
{
  // RGB 카메라 장치 가져와 활성화
  camera_ = robot->getCamera("camera");
  camera_->enable(timestep_);
  rgb_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS());

  // 깊이 센서(RangeFinder) 가져와 활성화
  range_finder_ = robot->getRangeFinder("range_finder");
  range_finder_->enable(timestep_);
  depth_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(
    "/range_finder/image_raw", rclcpp::SensorDataQoS());

  // 카메라 정보 퍼블리셔 생성
  camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", rclcpp::SensorDataQoS());

  // 카메라 FOV와 해상도 기반으로 내부 파라미터 계산
  double fov = camera_->getFov();
  int w = camera_->getWidth();
  int h = camera_->getHeight();
  double fx = w / (2.0 * std::tan(fov / 2.0));
  double fy = fx;
  double cx = w / 2.0;
  double cy = h / 2.0;

  // CameraInfo 메시지 템플릿 구성
  camera_info_msg_.header.frame_id = "camera_color_frame";
  camera_info_msg_.width  = w;
  camera_info_msg_.height = h;
  camera_info_msg_.distortion_model = "plumb_bob";
  camera_info_msg_.d = {0, 0, 0, 0, 0};
  camera_info_msg_.k = {
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0
  };
  camera_info_msg_.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };
  camera_info_msg_.p = {
    fx, 0.0, cx, 0.0,
    0.0, fy, cy, 0.0,
    0.0, 0.0, 1.0, 0.0
  };
}

// publish(): 주기마다 RGB 이미지, 깊이 이미지, 카메라 정보를 퍼블리시
void CameraPublisher::publish()
{
  // 1) RGB 이미지 퍼블리시
  const unsigned char* img = camera_->getImage();
  if (img) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = "camera_color_frame";
    msg.height = camera_->getHeight();
    msg.width  = camera_->getWidth();
    msg.encoding    = "rgb8";
    msg.is_bigendian = 0;
    msg.step        = msg.width * 3;
    msg.data.resize(msg.height * msg.step);

    // Webots RGBA → ROS RGB 변환
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

  // 2) 깊이 이미지 퍼블리시
  const float* range = range_finder_->getRangeImage();
  if (range) {
    int w = range_finder_->getWidth();
    int h = range_finder_->getHeight();
    sensor_msgs::msg::Image msg;
    msg.header.stamp = node_->get_clock()->now();
    msg.header.frame_id = "camera_depth_frame";
    msg.height   = h;
    msg.width    = w;
    msg.encoding = "32FC1";
    msg.is_bigendian = 0;
    msg.step = w * sizeof(float);
    msg.data.resize(h * msg.step);

    // 메모리 블록 복사로 깊이 값 전달
    std::memcpy(msg.data.data(),
                reinterpret_cast<const uint8_t*>(range),
                msg.data.size());
    depth_pub_->publish(std::move(msg));
  }

  // 3) 카메라 정보 퍼블리시
  camera_info_msg_.header.stamp = node_->get_clock()->now();
  camera_info_pub_->publish(camera_info_msg_);
}
