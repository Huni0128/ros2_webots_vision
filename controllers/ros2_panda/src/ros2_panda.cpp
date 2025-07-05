#include <webots/Robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include "camera_publisher.hpp"
#include "panda_joint_controller.hpp"
#include <memory>

// Webots + ROS2 통합 컨트롤러 클래스 정의
class PandaController {
public:
  // 생성자: Webots 로봇 및 ROS2 노드 초기화
  PandaController(const rclcpp::Node::SharedPtr& node)
    : node_(node) {
    robot_ = new webots::Robot();                   // Webots 로봇 객체 생성
    timestep_ = robot_->getBasicTimeStep();         // Webots 시뮬레이션 타임스텝(ms) 획득
  }

  // 카메라 퍼블리셔 및 관절 제어기 초기화
  void setup() {
    camera_pub_ = std::make_shared<CameraPublisher>(robot_, node_, timestep_);
    joint_ctrl_ = std::make_shared<PandaJointController>(robot_, node_, timestep_);
  }

  // Webots 루프 내에서 ROS 콜백 및 센서/모터 업데이트
  void run() {
    while (robot_->step(timestep_) != -1 && rclcpp::ok()) {
      rclcpp::spin_some(node_);      // ROS2 콜백 처리
      camera_pub_->publish();        // 카메라 이미지 퍼블리시
      joint_ctrl_->update();         // 관절 상태 퍼블리시 및 명령 처리
    }
  }

  // 소멸자: Webots 로봇 객체 메모리 해제
  ~PandaController() {
    delete robot_;
  }

private:
  webots::Robot* robot_;                         // Webots 로봇 인스턴스
  int timestep_;                                 // Webots 시뮬레이션 타임스텝
  rclcpp::Node::SharedPtr node_;                 // ROS2 노드
  std::shared_ptr<CameraPublisher> camera_pub_;  // 카메라 퍼블리셔
  std::shared_ptr<PandaJointController> joint_ctrl_; // 관절 제어기
};

// 메인 함수: ROS2 노드 실행 및 컨트롤러 시작
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);                                      // ROS2 초기화
  auto node = std::make_shared<rclcpp::Node>("panda_controller"); // 노드 생성

  PandaController controller(node);  // 컨트롤러 인스턴스 생성
  controller.setup();                // 구성 요소 초기화
  controller.run();                  // 메인 루프 실행

  rclcpp::shutdown();                // 종료 처리
  return 0;
}
