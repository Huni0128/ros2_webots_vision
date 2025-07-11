#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import os
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
    ObjectHypothesis,
    Pose2D,
)
from geometry_msgs.msg import PoseWithCovariance

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.bridge = CvBridge()

        # 구독: 카메라 원본
        self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        # 퍼블리시: 주석 이미지
        self.pub_annot = self.create_publisher(
            Image, '/camera/detected_objects',
            qos_profile_sensor_data)

        # 퍼블리시: 검출 결과
        self.pub_det = self.create_publisher(
            Detection2DArray, '/object_detector/detections', 10)

        # 모델 로드
        pkg_dir = get_package_share_directory('object_detector')
        model_path = os.path.join(pkg_dir, 'models', 'box_detect.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model: {model_path}")

        # 색상→ID 매핑
        self.color_ids = {'red_box': 0,
                          'green_box': 1,
                          'blue_box': 2}

    def image_callback(self, msg: Image):
        # 이미지 변환: ROS→OpenCV
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        # BGR→RGB for YOLO
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        # 검출 배열 초기화
        det_array = Detection2DArray()
        det_array.header = msg.header

        # 추론 및 메시지 작성
        try:
            results = self.model.predict(
                source=frame_rgb,
                conf=0.15, imgsz=1280,
                iou=0.45, stream=False,
                verbose=False)

            # 로그: 박스 개수
            count = len(results[0].boxes) if results else 0
            self.get_logger().info(f"Detected {count} boxes")

            for box in (results[0].boxes if results else []):
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf_score = float(box.conf[0])
                self.get_logger().info(
                    f"  box: ({x1},{y1})→({x2},{y2}), conf={conf_score:.2f}")

                # Detection2D 생성
                det = Detection2D()
                det.header = msg.header

                # 바운딩박스 설정
                cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                det.bbox.center = Pose2D(x=cx, y=cy, theta=0.0)
                det.bbox.size_x = float(x2 - x1)
                det.bbox.size_y = float(y2 - y1)

                # 색상 판정
                roi = frame_bgr[y1:y2, x1:x2]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                def ratio(img, lo, hi):
                    m = cv2.inRange(img, lo, hi)
                    return cv2.countNonZero(m) / (m.size + 1e-6)
                red   = ratio(hsv, (0,100,100), (10,255,255)) + \
                        ratio(hsv, (160,100,100), (179,255,255))
                green = ratio(hsv, (35,100,100), (85,255,255))
                blue  = ratio(hsv, (100,100,100), (130,255,255))
                color = max([("red_box", red),
                             ("green_box", green),
                             ("blue_box", blue)],
                            key=lambda x: x[1])[0]

                # 결과 설정
                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = str(self.color_ids[color])
                hypo.hypothesis.score    = conf_score
                hypo.pose                = PoseWithCovariance()

                det.results.append(hypo)
                det_array.detections.append(det)

                # 화면 표시용 박스
                cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(frame_bgr, color, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255,255,255), 2)

        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')

        # 퍼블리시
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame_bgr, 'bgr8')
            out_msg.header = msg.header
            self.pub_annot.publish(out_msg)
            self.pub_det.publish(det_array)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

        # 윈도우 표시
        cv2.imshow("Detected Boxes", frame_bgr)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = BoxDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
