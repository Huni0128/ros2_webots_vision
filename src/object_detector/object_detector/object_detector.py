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
    Point2D,
    Pose2D,
)
from geometry_msgs.msg import PoseWithCovariance

class BoxDetector(Node):
    def __init__(self):
        super().__init__('box_detector')
        self.bridge = CvBridge()

        # 카메라 원본 이미지 구독
        self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data)

        # 주석 이미지 퍼블리시
        self.pub_annot = self.create_publisher(
            Image, '/camera/detected_objects',
            qos_profile_sensor_data)

        # 박스 검출 결과 퍼블리시
        self.pub_det = self.create_publisher(
            Detection2DArray, '/object_detector/detections', 10)

        # YOLO 모델 로드
        pkg_dir = get_package_share_directory('object_detector')
        model_path = os.path.join(pkg_dir, 'models', 'box_detect.pt')
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model: {model_path}")

        # 색상 → 클래스 ID 매핑
        self.color_ids = {
            'red_box': 0,
            'green_box': 1,
            'blue_box': 2,
        }

        # HSV 색상 범위 설정
        self.hsv_ranges = {
            'red_box': [
                ((0,   50,  50), (10, 255, 255)),
                ((170, 50,  50), (179, 255, 255)),
            ],
            'green_box': [
                ((35,  50,  50), (85, 255, 255)),
            ],
            'blue_box': [
                ((90,  50,  50), (130,255,255)),
            ],
        }

    def image_callback(self, msg: Image):
        # 1) ROS 이미지 → OpenCV BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        h, w = frame.shape[:2]
        img_cx, img_cy = w / 2.0, h / 2.0

        # Detection2DArray 초기화
        det_array = Detection2DArray()
        det_array.header = msg.header

        # 2) YOLO 추론 수행
        try:
            results = self.model.predict(
                source=frame,
                conf=0.15,
                imgsz=1280,
                iou=0.45,
                stream=False,
                verbose=False)
        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            results = []

        # 3) 모든 박스 중 중앙에 가장 가까운 박스 하나만 선택
        best_box = None
        best_dist = float('inf')
        if results:
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                dist2 = (cx - img_cx)**2 + (cy - img_cy)**2
                if dist2 < best_dist:
                    best_dist = dist2
                    best_box = box

        # 4) 선택된 박스가 있을 때만 HSV 분석 및 퍼블리시
        if best_box is not None:
            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            conf_score = float(best_box.conf[0])

            det = Detection2D()
            det.header = msg.header

            # 박스 중심 좌표 및 크기 설정
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w_box = float(x2 - x1)
            h_box = float(y2 - y1)
            center = Pose2D()
            center.position = Point2D(x=cx, y=cy)
            center.theta = 0.0
            det.bbox.center = center
            det.bbox.size_x = w_box
            det.bbox.size_y = h_box

            # ROI 영역 HSV 변환 후 색상 판별 (원본 코드 그대로)
            roi = frame[y1:y2, x1:x2]
            hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
            def ratio(hsv_img, lo, hi):
                mask = cv2.inRange(hsv_img, lo, hi)
                return cv2.countNonZero(mask) / (mask.size + 1e-6)
            scores = {
                cname: sum(ratio(hsv, lo, hi) for lo, hi in ranges)
                for cname, ranges in self.hsv_ranges.items()
            }
            color = max(scores, key=scores.get)

            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = str(self.color_ids[color])
            hypo.hypothesis.score    = conf_score
            hypo.pose = PoseWithCovariance()
            det.results.append(hypo)
            det_array.detections.append(det)

            # 원본 코드대로 주석 표시
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.putText(frame, color, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # 5) 주석 이미지 및 검출 결과 퍼블리시 (원본 변환 그대로)
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
            out_msg.header = msg.header
            self.pub_annot.publish(out_msg)
            self.pub_det.publish(det_array)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

        # 6) 디버그용 이미지 디스플레이 (원본 코드대로)
        disp = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.imshow("Detected Boxes", disp)
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
