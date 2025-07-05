#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class ImageViewer : public rclcpp::Node {
public:
  ImageViewer() : Node("image_viewer") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&ImageViewer::callback, this, std::placeholders::_1));
  }

private:
  void callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv::Mat img = cv_bridge::toCvShare(msg, "rgb8")->image;
      cv::imshow("Camera View", img);
      cv::waitKey(1);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageViewer>());
  rclcpp::shutdown();
  return 0;
}
