#include <webots/Robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include "camera_publisher.hpp"
#include <memory>

class PandaController {

public:
  PandaController(const rclcpp::Node::SharedPtr& node)
      : node_(node) {
    robot_ = new webots::Robot();
    timestep_ = robot_->getBasicTimeStep();
  }

  void setup() {
    camera_pub_ = std::make_shared<CameraPublisher>(robot_, node_, timestep_);
  }

  void run() {
    while (robot_->step(timestep_) != -1 && rclcpp::ok()) {
      rclcpp::spin_some(node_);
      camera_pub_->publish();
    }
  }

private:
  webots::Robot* robot_;
  int timestep_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<CameraPublisher> camera_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("panda_controller");
  PandaController controller(node);
  controller.setup();
  controller.run();

  rclcpp::shutdown();
  return 0;
}
