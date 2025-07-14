#include <webots/Robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include "camera_publisher.hpp"
#include "panda_joint_controller.hpp"
#include "panda_gripper_controller.hpp"
#include "panda_tf_publisher.hpp"
#include <memory>

// Webots + ROS2 통합 제어 클래스 정의
class PandaController {
public:
  PandaController(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
    robot_ = new webots::Robot();
    timestep_ = robot_->getBasicTimeStep();
  }

  void setup() {
    camera_pub_   = std::make_shared<CameraPublisher>(robot_, node_, timestep_);
    joint_ctrl_   = std::make_shared<PandaJointController>(robot_, node_, timestep_);
    gripper_ctrl_ = std::make_shared<PandaGripperController>(robot_, node_, timestep_);
    tf_pub_       = std::make_shared<PandaTFPublisher>(node_);

    tf_pub_->publish_static_tf(); // 정적 TF 최초 1회 발행
  }

  void run() {
    while (robot_->step(timestep_) != -1 && rclcpp::ok()) {
      rclcpp::spin_some(node_);
      camera_pub_->publish();
      joint_ctrl_->update();
      gripper_ctrl_->update();
      // 필요시 여기에 동적 TF 발행 추가 (예: joint states 기반)
    }
  }

  ~PandaController() {
    delete robot_;
  }

private:
  webots::Robot* robot_;
  int timestep_;
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<CameraPublisher> camera_pub_;
  std::shared_ptr<PandaJointController> joint_ctrl_;
  std::shared_ptr<PandaGripperController> gripper_ctrl_;
  std::shared_ptr<PandaTFPublisher> tf_pub_;
};

// 메인 함수: ROS2 초기화 및 컨트롤러 실행
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("panda_controller");

  PandaController controller(node);
  controller.setup();
  controller.run();

  rclcpp::shutdown();
  return 0;
}
