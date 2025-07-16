#include <memory>
#include <vector>
#include <cmath>
#include <string>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"  // TF2로 물체 위치 확인

// TF2 관련 헤더
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

class ScanMove : public rclcpp::Node
{
public:
  ScanMove()
  : Node("scan_move"),
    scanning_active_(false),
    direction_(+1),
    target_color_id_(-1),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    // 파라미터 선언
    this->declare_parameter<double>("scan.limit_deg",      90.0);
    this->declare_parameter<double>("scan.step_deg",       2.0);
    this->declare_parameter<int>("scan.joint_index",       2);
    this->declare_parameter<std::string>("tf.camera_frame", "camera_link");
    this->declare_parameter<std::string>("tf.object_frame", "object_frame");

    // 파라미터 값 읽어오기
    this->get_parameter("scan.limit_deg",   scan_limit_deg_);
    this->get_parameter("scan.step_deg",    scan_step_deg_);
    this->get_parameter("scan.joint_index", joint_index_);
    this->get_parameter("tf.camera_frame",  camera_frame_);
    this->get_parameter("tf.object_frame",  object_frame_);

    // 각도(rad) 변환
    scan_limit_rad_ = scan_limit_deg_ * M_PI / 180.0;
    scan_step_rad_  = scan_step_deg_  * M_PI / 180.0;

    // 초기 관절 위치 설정
    current_joint_positions_.assign(7, 0.0);

    // 토픽 구독 및 퍼블리시 설정
    pick_sub_      = create_subscription<std_msgs::msg::String>(
      "/pick_box", 10,
      std::bind(&ScanMove::onPickBox, this, std::placeholders::_1)
    );
    status_sub_    = create_subscription<std_msgs::msg::String>(
      "/pick_box_status", 10,
      std::bind(&ScanMove::onStatus, this, std::placeholders::_1)
    );
    detection_sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
      "/object_detector/detections", 10,
      std::bind(&ScanMove::onDetection, this, std::placeholders::_1)
    );
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/panda_joint_states", 10,
      std::bind(&ScanMove::jointStateCallback, this, std::placeholders::_1)
    );
    joint_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/panda_joint_command", 10
    );

    // 주기 타이머 설정 (50Hz)
    timer_ = create_wall_timer(
      20ms,
      std::bind(&ScanMove::onTimer, this)
    );

    // 초기 상태 출력
    RCLCPP_INFO(get_logger(),
      "ScanMove ready: joint[%d], limit=±%.1f°, step=%.1f°, "
      "camera_frame='%s', object_frame='%s'",
      joint_index_, scan_limit_deg_, scan_step_deg_,
      camera_frame_.c_str(), object_frame_.c_str());
  }

private:
  // ─── 멤버 변수 ─────────────────────────────────────────
  bool   scanning_active_;                  // 스캔 활성화 상태
  int    direction_;                        // 스캔 방향 (+1: 왼쪽, -1: 오른쪽)
  int    target_color_id_;                  // 목표 색상 ID
  int    joint_index_;                      // 제어할 관절 인덱스
  double scan_limit_deg_, scan_step_deg_;   // 스캔 한계·스텝 (deg)
  double scan_limit_rad_, scan_step_rad_;   // 스캔 한계·스텝 (rad)
  std::string camera_frame_, object_frame_; // TF 프레임 이름
  std::vector<double> current_joint_positions_; // 현재 관절 위치

  // 통신 객체
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          pick_sub_, status_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr   joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr  joint_cmd_pub_;
  rclcpp::TimerBase::SharedPtr                                   timer_;

  // TF2 객체
  tf2_ros::Buffer        tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ─── 콜백 함수 ─────────────────────────────────────────

  // 1) pick_box 명령 처리
  void onPickBox(const std_msgs::msg::String::SharedPtr msg)
  {
    auto c = msg->data;
    if      (c=="red_box")   target_color_id_ = 0;
    else if (c=="green_box") target_color_id_ = 1;
    else if (c=="blue_box")  target_color_id_ = 2;
    else {
      RCLCPP_WARN(get_logger(), "Unknown pick_box '%s'", c.c_str());
      target_color_id_ = -1; 
      return;
    }
    scanning_active_ = true;
    RCLCPP_INFO(get_logger(),"pick_box='%s' → start scanning", c.c_str());
  }

  // 2) pick_box_status 처리
  void onStatus(const std_msgs::msg::String::SharedPtr msg)
  {
    if (target_color_id_ < 0) return;
    auto s = msg->data;
    if ((s=="SCANNING" || s=="NOT_FOUND") && !scanning_active_) {
      scanning_active_ = true;
      RCLCPP_INFO(get_logger(),"Status '%s' → resume scanning", s.c_str());
    }
    else if (s=="MATCHED" && scanning_active_) {
      scanning_active_ = false;
      double deg = current_joint_positions_[joint_index_] * 180.0 / M_PI;
      RCLCPP_INFO(get_logger(),"Status 'MATCHED' → hold at %.2f°", deg);
    }
  }

  // 3) 감지 결과 기반 스캔 방향 우선순위 결정
  void onDetection(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
  {
    if (!scanning_active_ || msg->detections.empty()) return;
    int det_id = std::stoi(msg->detections[0].results[0].hypothesis.class_id);
    switch (det_id) {
      case 1:  // green
        direction_ = +1;
        RCLCPP_INFO(get_logger(),"Detected GREEN → LEFT first");
        break;
      case 2:  // blue
        direction_ = -1;
        RCLCPP_INFO(get_logger(),"Detected BLUE → RIGHT first");
        break;
      case 0:  // red
        if (target_color_id_ == 1) {
          direction_ = -1;
          RCLCPP_INFO(get_logger(),"Detected RED, target GREEN → RIGHT first");
        } else if (target_color_id_ == 2) {
          direction_ = +1;
          RCLCPP_INFO(get_logger(),"Detected RED, target BLUE  → LEFT first");
        }
        break;
      default:
        return;
    }
  }

  // 4) 관절 상태 업데이트
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->position.size() >= current_joint_positions_.size())
      current_joint_positions_ = msg->position;
  }

  // 5) 주기 콜백: TF 검사 후 스캔 또는 정지
  void onTimer()
  {
    if (!scanning_active_) return;

    bool real_object = false;
    geometry_msgs::msg::TransformStamped tfst;
    try {
      tfst = tf_buffer_.lookupTransform(
        camera_frame_, object_frame_, tf2::TimePointZero);
      if (tfst.header.stamp.sec > 0) real_object = true;
    } catch (...) {
      real_object = false;
    }

    if (real_object) {
      // 실제 객체 검출 시 정지
      double y = tfst.transform.translation.y;
      direction_ = (y >= 0.0 ? +1 : -1);
      scanning_active_ = false;
      double deg = current_joint_positions_[joint_index_] * 180.0 / M_PI;
      RCLCPP_INFO(get_logger(),
        "TF found (real): y=%.3fm → %s scan, holding at %.2f°",
        y, (direction_>0?"RIGHT":"LEFT"), deg);
      return;
    }

    // 스캔 범위 내에서 좌우 반복
    double &cur = current_joint_positions_[joint_index_];
    double next = cur + direction_ * scan_step_rad_;
    if      (next >  scan_limit_rad_) { next =  scan_limit_rad_; direction_ = -1; }
    else if (next < -scan_limit_rad_) { next = -scan_limit_rad_; direction_ = +1; }
    cur = next;

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = current_joint_positions_;
    joint_cmd_pub_->publish(cmd);
  }
};

int main(int argc,char**argv)
{
  rclcpp::init(argc,argv);                             // ROS2 초기화
  rclcpp::spin(std::make_shared<ScanMove>());          // 노드 실행 및 스핀
  rclcpp::shutdown();                                  // 종료 처리
  return 0;
}
