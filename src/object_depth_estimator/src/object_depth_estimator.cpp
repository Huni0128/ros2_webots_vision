// ROS2 및 메시지 타입 관련 헤더
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <unordered_map>
#include <cmath>

class ObjectDepthEstimator : public rclcpp::Node {
public:
    ObjectDepthEstimator()
    : Node("object_depth_estimator")
    {
        // 입력 토픽 구독 설정
        det_sub_.subscribe(this, "/object_detector/detections", rmw_qos_profile_sensor_data);
        depth_sub_.subscribe(this, "/range_finder/image_raw",    rmw_qos_profile_sensor_data);
        info_sub_.subscribe(this,  "/camera/camera_info",       rmw_qos_profile_sensor_data);

        // 메시지 동기화 설정
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), det_sub_, depth_sub_, info_sub_);
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
        sync_->registerCallback(&ObjectDepthEstimator::callback, this);

        // 결과 퍼블리셔 생성
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
        // 카메라 내부 파라미터 추출
        const auto& K = info->k;
        double fx = K[0], fy = K[4], cx = K[2], cy = K[5];
        int color_w = info->width, color_h = info->height;

        // 깊이 이미지 변환
        const std::string enc = depth_msg->encoding;
        RCLCPP_INFO(get_logger(), "Depth encoding: %s", enc.c_str());

        namespace encs = sensor_msgs::image_encodings;
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat depth_f;
        if (enc == encs::TYPE_16UC1) {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, encs::TYPE_16UC1);
            cv_ptr->image.convertTo(depth_f, CV_32FC1, 0.001f);
        }
        else if (enc == encs::TYPE_32FC1) {
            cv_ptr = cv_bridge::toCvCopy(depth_msg, encs::TYPE_32FC1);
            depth_f = cv_ptr->image;
        }
        else {
            RCLCPP_ERROR(get_logger(), "Unsupported depth encoding '%s'", enc.c_str());
            return;
        }

        // 해상도 및 스케일 계산
        int depth_w = depth_f.cols, depth_h = depth_f.rows;
        RCLCPP_INFO(get_logger(), "Color res=(%d,%d), Depth res=(%d,%d)", color_w, color_h, depth_w, depth_h);
        double scale_x = double(depth_w) / color_w;
        double scale_y = double(depth_h) / color_h;
        RCLCPP_INFO(get_logger(), "Scale (x,y) = (%.3f, %.3f)", scale_x, scale_y);

        // 클래스 ID → 컬러 이름 매핑
        static const std::unordered_map<std::string, std::string> id_to_color = {
            {"0", "red_box"}, {"1", "green_box"}, {"2", "blue_box"},
        };

        // PoseArray 초기화
        geometry_msgs::msg::PoseArray poses;
        poses.header = depth_msg->header;

        // 각 객체 탐지에 대해 3D 위치 추정
        for (size_t i = 0; i < dets->detections.size(); ++i) {
            const auto &det = dets->detections[i];
            double u_color = det.bbox.center.position.x;
            double v_color = det.bbox.center.position.y;

            int u_depth = int(u_color * scale_x + 0.5);
            int v_depth = int(v_color * scale_y + 0.5);

            RCLCPP_INFO(get_logger(), "Det[%zu] color (%.1f,%.1f) → depth (u,v)=(%d,%d)", i, u_color, v_color, u_depth, v_depth);

            // 이미지 경계 체크
            if (u_depth < 0 || u_depth >= depth_w || v_depth < 0 || v_depth >= depth_h) {
                RCLCPP_WARN(get_logger(), "  → mapped pixel out of bounds, skip");
                continue;
            }

            // 깊이값 확인
            float z = depth_f.at<float>(v_depth, u_depth);
            RCLCPP_INFO(get_logger(), "  raw z @(%d,%d) = %.3f m", u_depth, v_depth, z);
            if (z <= 0.0f || std::isnan(z)) {
                RCLCPP_WARN(get_logger(), "  → invalid z, skip");
                continue;
            }

            // 3D 좌표 계산
            float X = (u_color - cx) * z / fx;
            float Y = (v_color - cy) * z / fy;

            // 클래스 ID 및 색상 이름 확인
            std::string class_id = det.results.empty() ? "?" : det.results[0].hypothesis.class_id;
            auto it = id_to_color.find(class_id);
            std::string color_name = (it != id_to_color.end()) ? it->second : "unknown";

            RCLCPP_INFO(get_logger(), "  Det[%zu] class_id=%s  color=%s  → (X,Y,Z) = (%.3f, %.3f, %.3f) m",
                        i, class_id.c_str(), color_name.c_str(), X, Y, z);

            // Pose 메시지 생성
            geometry_msgs::msg::Pose p;
            p.position.x    = X;
            p.position.y    = Y;
            p.position.z    = z;
            p.orientation.w = 1.0;
            poses.poses.push_back(p);
        }

        // 결과 퍼블리시
        pose_pub_->publish(poses);
    }

    // 메시지 필터 구독자
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> det_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image>            depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo>       info_sub_;

    // 동기화 정책 정의
    using MySyncPolicy = message_filters::sync_policies::ApproximateTime<
        vision_msgs::msg::Detection2DArray,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;
    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    // 퍼블리셔
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObjectDepthEstimator>());
    rclcpp::shutdown();
    return 0;
}
