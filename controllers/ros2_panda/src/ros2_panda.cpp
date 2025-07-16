#include <webots/Robot.hpp>
#include <rclcpp/rclcpp.hpp>
#include "camera_publisher.hpp"
#include "panda_joint_controller.hpp"
#include "panda_gripper_controller.hpp"
#include "panda_tf_publisher.hpp"
#include <memory>

// PandaController 클래스: Webots 로봇과 ROS2 노드를 연동하여 제어
class PandaController {
public:
    // 생성자: ROS 노드 저장, Webots Robot 생성 및 시뮬레이션 타임스텝 설정
    PandaController(const rclcpp::Node::SharedPtr& node)
    : node_(node)
    {
        robot_ = new webots::Robot();
        timestep_ = robot_->getBasicTimeStep();
    }

    // setup: 카메라 퍼블리셔, 관절 컨트롤러, 그리퍼 컨트롤러, TF 퍼블리셔 초기화
    //        정적 TF를 한 번 발행
    void setup() {
        camera_pub_   = std::make_shared<CameraPublisher>(robot_, node_, timestep_);
        joint_ctrl_   = std::make_shared<PandaJointController>(robot_, node_, timestep_);
        gripper_ctrl_ = std::make_shared<PandaGripperController>(robot_, node_, timestep_);
        tf_pub_       = std::make_shared<PandaTFPublisher>(node_);

        tf_pub_->publish_static_tf();
    }

    // run: 메인 루프 실행
    //      Webots 시뮬레이션 스텝 진행, ROS 콜백 처리, 각 서브시스템 업데이트
    void run() {
        while (robot_->step(timestep_) != -1 && rclcpp::ok()) {
            rclcpp::spin_some(node_);
            camera_pub_->publish();
            joint_ctrl_->update();
            gripper_ctrl_->update();
            // 필요시 동적 TF 발행 로직 추가 가능
        }
    }

    // 소멸자: Webots Robot 객체 해제
    ~PandaController() {
        delete robot_;
    }

private:
    // Webots 및 ROS 관련 멤버 변수
    webots::Robot* robot_;
    int            timestep_;
    rclcpp::Node::SharedPtr node_;

    // 서브시스템 객체들
    std::shared_ptr<CameraPublisher>       camera_pub_;
    std::shared_ptr<PandaJointController>  joint_ctrl_;
    std::shared_ptr<PandaGripperController> gripper_ctrl_;
    std::shared_ptr<PandaTFPublisher>      tf_pub_;
};

// main 함수: ROS2 초기화 → PandaController 생성 → setup → run → 종료
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("panda_controller");

    PandaController controller(node);
    controller.setup();
    controller.run();

    rclcpp::shutdown();
    return 0;
}
