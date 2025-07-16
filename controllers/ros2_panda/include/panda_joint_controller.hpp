#ifndef PANDA_JOINT_CONTROLLER_HPP
#define PANDA_JOINT_CONTROLLER_HPP

// Webots 및 ROS2 관련 헤더
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>

// ROS2를 통해 Webots Panda 로봇의 관절을 제어하는 클래스
class PandaJointController {
public:
    // 생성자: 로봇 객체, ROS 노드, 타임스텝 초기화
    PandaJointController(
        webots::Robot* robot,
        const rclcpp::Node::SharedPtr& node,
        int timestep);

    // 관절 상태 퍼블리시 및 콜백 실행 (매 시뮬레이션 루프마다 호출)
    void update();

private:
    webots::Robot*                                   robot_;        // Webots 로봇 객체
    rclcpp::Node::SharedPtr                          node_;         // ROS2 노드
    int                                              timestep_;     // 시뮬레이션 타임스텝 (ms)

    std::vector<std::string>                         jointNames_;   // 관절 이름 목록
    std::vector<webots::Motor*>                      joints_;       // 관절 모터 포인터 목록
    std::vector<webots::PositionSensor*>             sensors_;      // 관절 센서 포인터 목록

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr            joint_pub_; // 관절 상태 퍼블리셔
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr    joint_sub_; // 관절 명령 서브스크라이버

    // 관절 제어 명령 수신 콜백
    void jointCommandCallback(
        const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // 현재 관절 상태를 퍼블리시
    void publishJointStates();
};

#endif  // PANDA_JOINT_CONTROLLER_HPP
