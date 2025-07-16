#include "panda_joint_controller.hpp"

// 생성자: 관절 이름 초기화, 모터 및 센서 설정, ROS 퍼블리셔/서브스크라이버 생성
PandaJointController::PandaJointController(
    webots::Robot* robot,
    const rclcpp::Node::SharedPtr& node,
    int timestep)
  : robot_(robot),
    node_(node),
    timestep_(timestep)
{
    // 제어할 Panda 로봇의 관절 이름 정의
    jointNames_ = {
        "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
        "panda_joint5", "panda_joint6", "panda_joint7"
    };

    // 각 관절에 대해 모터 및 센서 등록
    for (const auto& name : jointNames_) {
        auto motor = robot_->getMotor(name);
        if (!motor) {
            RCLCPP_ERROR(node_->get_logger(), "Motor not found: %s", name.c_str());
            continue;
        }

        auto sensor = motor->getPositionSensor();
        if (!sensor) {
            RCLCPP_ERROR(node_->get_logger(), "Sensor not found for: %s", name.c_str());
            continue;
        }

        sensor->enable(timestep_);
        joints_.push_back(motor);
        sensors_.push_back(sensor);
    }

    RCLCPP_INFO(
        node_->get_logger(),
        "PandaJointController initialized with %zu joints.",
        joints_.size());

    // 관절 상태 퍼블리셔 설정
    joint_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
        "/panda_joint_states", 10);

    // 관절 제어 명령 서브스크라이버 설정
    joint_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/panda_joint_command", 10,
        std::bind(
            &PandaJointController::jointCommandCallback,
            this,
            std::placeholders::_1));
}

// 주기적으로 호출됨: 콜백 처리 및 현재 상태 퍼블리시
void PandaJointController::update()
{
    rclcpp::spin_some(node_);
    publishJointStates();
}

// 수신한 관절 위치 명령을 각 모터에 적용
void PandaJointController::jointCommandCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() != joints_.size()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Invalid joint command size: %zu (expected %zu)",
            msg->data.size(),
            joints_.size());
        return;
    }

    for (size_t i = 0; i < joints_.size(); ++i) {
        joints_[i]->setPosition(msg->data[i]);
    }
}

// 현재 관절 상태를 JointState 메시지로 구성하여 퍼블리시
void PandaJointController::publishJointStates()
{
    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = node_->now();

    for (size_t i = 0; i < joints_.size(); ++i) {
        msg.name.push_back(jointNames_[i]);
        msg.position.push_back(sensors_[i]->getValue());
    }

    joint_pub_->publish(msg);
}
