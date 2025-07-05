#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

// 카메라 이미지 토픽을 구독하고 화면에 출력하는 노드 클래스
class ImageViewer : public rclcpp::Node {
public:
  ImageViewer() : Node("image_viewer") {
    // "/camera/image_raw" 토픽 구독 (센서용 QoS)
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ImageViewer::callback, this, std::placeholders::_1));
  }

private:
  // 이미지 수신 콜백
  void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      // ROS2 이미지 메시지를 OpenCV 형식으로 변환
      cv::Mat img = cv_bridge::toCvShare(msg, "rgb8")->image;

      // 이미지 화면에 표시
      cv::imshow("Camera View", img);
      cv::waitKey(1);  // GUI 이벤트 유지
    } catch (cv_bridge::Exception& e) {
      // 변환 실패 시 로그 출력
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  // 이미지 구독자
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

// 노드 실행 진입점
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageViewer>());
  rclcpp::shutdown();
  return 0;
}
