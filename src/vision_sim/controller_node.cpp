#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class JointCommander : public rclcpp::Node
{
public:
    JointCommander() : Node("joint_commander") {
        pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/panda/joint1_position_controller/command", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&JointCommander::timer_callback, this));
    }

private:
    void timer_callback() {
        std_msgs::msg::Float64 msg;
        msg.data = 1.57;  // 90도
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg.data);
        pub_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointCommander>());
    rclcpp::shutdown();
    return 0;
}
