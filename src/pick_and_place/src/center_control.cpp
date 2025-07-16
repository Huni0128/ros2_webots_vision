#include <memory>
#include <vector>
#include <cstdint>       // int64_t 타입 사용
#include <cmath>
#include <string>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"             // 상태 메시지 구독용
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class CenterControl : public rclcpp::Node
{
public:
    CenterControl()
    : Node("center_control"),
      target_visible_(false),
      start_centering_(false)
    {
        // 1) 제어 파라미터 선언 및 읽기
        this->declare_parameter<std::vector<int64_t>>(
            "control.x_joint_indices", std::vector<int64_t>{0,2});  // X축 관절 인덱스
        this->declare_parameter<std::vector<int64_t>>(
            "control.y_joint_indices", std::vector<int64_t>{1,3});  // Y축 관절 인덱스
        this->declare_parameter<double>("control.kp_x", 0.5);       // X축 비례계수
        this->declare_parameter<double>("control.kp_y", 0.5);       // Y축 비례계수
        this->declare_parameter<double>("control.threshold", 0.01); // 허용 오차 [m]

        this->get_parameter("control.x_joint_indices", x_joints_);
        this->get_parameter("control.y_joint_indices", y_joints_);
        this->get_parameter("control.kp_x",             Kp_x_);
        this->get_parameter("control.kp_y",             Kp_y_);
        this->get_parameter("control.threshold",        thresh_);

        // 2) 구독자 설정
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "/object_depth/pose_array", 10,
            std::bind(&CenterControl::poseCallback, this, std::placeholders::_1));

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/panda_joint_states", 10,
            std::bind(&CenterControl::jointCallback, this, std::placeholders::_1));

        status_sub_ = create_subscription<std_msgs::msg::String>(
            "/pick_box_status", 10,
            std::bind(&CenterControl::statusCallback, this, std::placeholders::_1));

        // 3) 퍼블리셔 설정
        cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/panda_joint_command", 10);

        // 4) 제어 주기 타이머 (20Hz)
        timer_ = create_wall_timer(
            50ms, std::bind(&CenterControl::controlLoop, this));

        RCLCPP_INFO(get_logger(),
            "CenterControl ready: X joints=[%ld,%ld], Y joints=[%ld,%ld], "
            "Kp_x=%.2f Kp_y=%.2f thr=%.3f",
            static_cast<long>(x_joints_[0]), static_cast<long>(x_joints_[1]),
            static_cast<long>(y_joints_[0]), static_cast<long>(y_joints_[1]),
            Kp_x_, Kp_y_, thresh_);
    }

private:
    // 상태 메시지 콜백: "MATCHED" 수신 시 센터링 시작
    void statusCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "MATCHED") {
            start_centering_ = true;
            init_positions_ = joint_positions_;  // 기준 위치 저장
            RCLCPP_INFO(get_logger(),
                "MATCHED → centering start. init_positions_[%ld]=%.3f rad",
                static_cast<long>(x_joints_[0]),
                init_positions_[x_joints_[0]]);
        } else {
            start_centering_ = false;
        }
    }

    // 객체 위치 콜백: 3D 위치 받아오기
    void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (!msg->poses.empty()) {
            target_x_ = msg->poses[0].position.x;
            target_y_ = msg->poses[0].position.y;
            target_visible_ = true;
        } else {
            target_visible_ = false;
        }
    }

    // 관절 상태 콜백: 현재 관절 위치 저장
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        size_t max_idx = static_cast<size_t>(
            std::max(x_joints_.back(), y_joints_.back()));  // 최대 인덱스 확인

        if (msg->position.size() > max_idx) {
            joint_positions_ = msg->position;
        }
    }

    // 주기 제어 루프
    void controlLoop()
    {
        if (!start_centering_ || !target_visible_ || joint_positions_.empty()) {
            return;
        }

        double err_x = target_x_;  // X축 오차
        double err_y = target_y_;  // Y축 오차

        // 오차가 임계값 이내면 완료
        bool done_x = std::abs(err_x) < thresh_;
        bool done_y = std::abs(err_y) < thresh_;

        if (done_x && done_y) {
            RCLCPP_INFO(get_logger(),
                "Centered: err_x=%.3f m, err_y=%.3f m (both < %.3f)",
                err_x, err_y, thresh_);
            start_centering_ = false;
            return;
        }

        // 디버그 출력 (0.5초 주기)
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[DBG] err_x=%.3f (ok=%d), err_y=%.3f (ok=%d), thresh=%.3f",
            err_x, done_x, err_y, done_y, thresh_);

        // 제어 입력 계산 (비례 제어)
        double delta_x = done_x ? 0.0 : -Kp_x_ * err_x;
        double delta_y = done_y ? 0.0 : -Kp_y_ * err_y;

        // 제어 명령 준비
        auto cmd = std_msgs::msg::Float64MultiArray();
        cmd.data = joint_positions_;  // 현재 관절 상태 복사

        // X축 관절에 제어 분배
        if (!done_x) {
            double share_x = delta_x / static_cast<double>(x_joints_.size());
            for (int64_t idx : x_joints_) {
                cmd.data[idx] += share_x;
            }
        }

        // Y축 관절에 제어 분배
        if (!done_y) {
            double share_y = delta_y / static_cast<double>(y_joints_.size());
            for (int64_t idx : y_joints_) {
                cmd.data[idx] += share_y;
            }
        }

        // 명령 퍼블리시
        cmd_pub_->publish(cmd);

        // 제어값 로그 출력 (0.5초 주기)
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Control: Δx=%.3f rad, Δy=%.3f rad", delta_x, delta_y);
    }

    // 멤버 변수
    std::vector<int64_t>    x_joints_, y_joints_;       // 제어할 관절 인덱스
    double                  Kp_x_, Kp_y_, thresh_;       // 제어 파라미터
    bool                    target_visible_, start_centering_;  // 상태 플래그
    double                  target_x_, target_y_;        // 목표 위치 (x, y)
    std::vector<double>     joint_positions_, init_positions_;  // 관절 상태

    // ROS 통신 객체
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr    pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr    joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          status_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);                           // ROS2 초기화
    rclcpp::spin(std::make_shared<CenterControl>());    // 노드 실행
    rclcpp::shutdown();                                 // 종료 처리
    return 0;
}
