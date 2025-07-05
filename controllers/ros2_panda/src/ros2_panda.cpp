#include <webots/Robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include "camera_publisher.hpp"
#include "panda_joint_controller.hpp"
#include "panda_gripper_controller.hpp"  // 그리퍼 제어기 포함
#include <memory>

// Webots + ROS2 통합 제어 클래스 정의
class PandaController {
public:
  // 생성자: Webots 로봇 객체 및 ROS2 노드 초기화
  PandaController(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
    robot_ = new webots::Robot();            // Webots 로봇 인스턴스 생성
    timestep_ = robot_->getBasicTimeStep();  // Webots 시뮬레이션 타임스텝(ms) 획득
  }

  // 각 구성 요소 초기화 (카메라, 관절, 그리퍼)
  void setup() {
    camera_pub_ = std::make_shared<CameraPublisher>(robot_, node_, timestep_);
    joint_ctrl_ = std::make_shared<PandaJointController>(robot_, node_, timestep_);
    gripper_ctrl_ = std::make_shared<PandaGripperController>(robot_, node_, timestep_);
  }

  // Webots 메인 루프 내 ROS2 콜백 처리 및 센서/모터 업데이트
  void run() {
    while (robot_->step(timestep_) != -1 && rclcpp::ok()) {
      rclcpp::spin_some(node_);     // ROS2 콜백 실행
      camera_pub_->publish();       // 카메라 이미지 퍼블리시
      joint_ctrl_->update();        // 관절 제어기 업데이트
      gripper_ctrl_->update();      // 그리퍼 제어기 업데이트
    }
  }

  // 소멸자: Webots 객체 메모리 해제
  ~PandaController() {
    delete robot_;
  }

private:
  webots::Robot* robot_;                                        // Webots 로봇 인스턴스
  int timestep_;                                                // 시뮬레이션 타임스텝(ms)
  rclcpp::Node::SharedPtr node_;                                // ROS2 노드 핸들
  std::shared_ptr<CameraPublisher> camera_pub_;                 // 카메라 퍼블리셔
  std::shared_ptr<PandaJointController> joint_ctrl_;            // 관절 제어기
  std::shared_ptr<PandaGripperController> gripper_ctrl_;        // 그리퍼 제어기
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
