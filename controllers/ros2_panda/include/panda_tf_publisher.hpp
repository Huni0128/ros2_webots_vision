#ifndef PANDA_TF_PUBLISHER_HPP
#define PANDA_TF_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class PandaTFPublisher
{
public:
    PandaTFPublisher(rclcpp::Node::SharedPtr node);
    void publish_static_tf();

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

#endif // PANDA_TF_PUBLISHER_HPP
