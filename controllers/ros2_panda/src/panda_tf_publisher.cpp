#include "panda_tf_publisher.hpp"

PandaTFPublisher::PandaTFPublisher(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
}

void PandaTFPublisher::publish_static_tf()
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp            = node_->get_clock()->now();
    tf_msg.header.frame_id         = "panda_link8";
    tf_msg.child_frame_id          = "camera_color_frame";
    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.06;
    tf_msg.transform.rotation.x    = 0.0;
    tf_msg.transform.rotation.y    = -1.0;
    tf_msg.transform.rotation.z    = 0.0;
    tf_msg.transform.rotation.w    = 1.5708;

    tf_broadcaster_->sendTransform(tf_msg);
}
