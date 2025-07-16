#include "panda_tf_publisher.hpp"

// 생성자: 노드 포인터 저장 및 StaticTransformBroadcaster 초기화
PandaTFPublisher::PandaTFPublisher(rclcpp::Node::SharedPtr node)
: node_(node)
{
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

// Static TF를 한 번만 발행하는 메서드
void PandaTFPublisher::publish_static_tf()
{
    geometry_msgs::msg::TransformStamped tf_msg;

    // 타임스탬프와 프레임 아이디 설정
    tf_msg.header.stamp = node_->get_clock()->now();
    tf_msg.header.frame_id = "panda_link8";
    tf_msg.child_frame_id = "camera_color_frame";

    // 평행 이동(미터 단위)
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.06;

    // 회전(쿼터니언)
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = -1.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.5708;

    // StaticTransformBroadcaster를 통해 TF 발행
    tf_broadcaster_->sendTransform(tf_msg);
}
