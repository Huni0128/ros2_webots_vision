#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class ObjectDepthEstimator : public rclcpp::Node {
public:
    ObjectDepthEstimator()
    : Node("object_depth_estimator")
    {
        // 2D 검출, 깊이, 카메라 정보 토픽 구독
        det_sub_.subscribe(this, "/object_detector/detections", rmw_qos_profile_sensor_data);
        depth_sub_.subscribe(this, "/range_finder/image_raw",    rmw_qos_profile_sensor_data);
        info_sub_.subscribe(this,  "/camera/camera_info",       rmw_qos_profile_sensor_data);

        // ApproximateTime 동기화 설정
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), det_sub_, depth_sub_, info_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
        sync_->registerCallback(&ObjectDepthEstimator::callback, this);

        // 3D 포즈 배열 퍼블리셔
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/object_depth/pose_array", 10);

        RCLCPP_INFO(get_logger(), "ObjectDepthEstimator ready");
    }

private:
    void callback(
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr dets,
        const sensor_msgs::msg::Image::ConstSharedPtr         depth_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr    info)
    {
        // 카메라 내부 파라미터 읽기
        const auto& K = info->k;
        double fx = K[0], fy = K[4], cx = K[2], cy = K[5];

        // 깊이 이미지를 OpenCV 형식으로 변환
        cv::Mat depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding)->image;

        geometry_msgs::msg::PoseArray poses;
        poses.header = depth_msg->header;

        for (const auto &det : dets->detections) {
            // 바운딩 박스 중심 픽셀 좌표
            int u = static_cast<int>(det.bbox.center.x);
            int v = static_cast<int>(det.bbox.center.y);
            if (u < 0 || u >= depth.cols || v < 0 || v >= depth.rows) {
                continue;
            }

            float z = depth.at<float>(v, u);
            if (z <= 0.0f) {
                continue;
            }

            // 픽셀 좌표를 3D 포인트로 역투영
            float X = (u - cx) * z / fx;
            float Y = (v - cy) * z / fy;

            geometry_msgs::msg::Pose p;
            p.position.x    = X;
            p.position.y    = Y;
            p.position.z    = z;
            p.orientation.w = 1.0;
            poses.poses.push_back(p);

            // 로그 출력
            RCLCPP_INFO(
                get_logger(),
                "Det [%s] → (x,y,z) = (%.3f, %.3f, %.3f) m",
                det.results.empty() ? "?" : det.results[0].hypothesis.class_id.c_str(),
                X, Y, z);
        }

        // 전체 포즈 배열 퍼블리시
        pose_pub_->publish(poses);
    }

    // 토픽 동기화를 위한 서브스크라이버
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image>             depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo>        info_sub_;

    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<
        vision_msgs::msg::Detection2DArray,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    // 3D 포즈 퍼블리셔
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDepthEstimator>());
    rclcpp::shutdown();
    return 0;
}
