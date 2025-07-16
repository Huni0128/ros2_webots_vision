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

        # 카메라 이미지 구독
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
            'red_box':   0,
            'green_box': 1,
            'blue_box':  2,
            'none':      3,
        }

        # HSV 범위 설정
        self.hsv_ranges = {
            'red_box': [
                ((  0, 220, 220), ( 10, 255, 255)),
                ((170, 220, 220), (180, 255, 255)),
            ],
            'green_box': [
                (( 50, 150, 150), ( 70, 255, 255)),
            ],
            'blue_box': [
                ((100, 180, 180), (125, 255, 255)),
            ],
        }

        # 색상 판별 조건
        self.min_color_ratio = 0.3
        self.min_color_pixels = 100

    def image_callback(self, msg: Image):
        # ROS → OpenCV 변환
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        h, w = frame.shape[:2]
        img_cx, img_cy = w / 2.0, h / 2.0

        # 결과 메시지 초기화
        det_array = Detection2DArray()
        det_array.header = msg.header

        # YOLO 추론
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

        # 가장 중앙에 가까운 박스 선택
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

        # 박스 있을 경우 HSV 판별 및 메시지 작성
        if best_box is not None:
            x1, y1, x2, y2 = map(int, best_box.xyxy[0])
            conf_score = float(best_box.conf[0])

            det = Detection2D()
            det.header = msg.header

            # 중심 위치 및 크기
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

            # ROI → HSV 변환
            roi = frame[y1:y2, x1:x2]
            hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

            # 색상 판별
            scores = {}
            pixels = {}
            for cname, ranges in self.hsv_ranges.items():
                total_ratio = 0.0
                total_pix = 0
                for lo, hi in ranges:
                    mask = cv2.inRange(hsv, lo, hi)
                    cnt = cv2.countNonZero(mask)
                    total_ratio += cnt / (mask.size + 1e-6)
                    total_pix += cnt
                scores[cname] = total_ratio
                pixels[cname] = total_pix

            # 조건 만족 여부
            best_color, best_ratio = max(scores.items(), key=lambda x: x[1])
            if best_ratio < self.min_color_ratio or pixels[best_color] < self.min_color_pixels:
                class_id = str(self.color_ids['none'])
            else:
                class_id = str(self.color_ids[best_color])

            # 결과 작성
            hypo = ObjectHypothesisWithPose()
            hypo.hypothesis.class_id = class_id
            hypo.hypothesis.score = conf_score
            hypo.pose = PoseWithCovariance()
            det.results.append(hypo)
            det_array.detections.append(det)

            # 주석 표시
            box_color = (0,255,0) if class_id != str(self.color_ids['none']) else (128,128,128)
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            label = list(self.color_ids.keys())[list(self.color_ids.values()).index(int(class_id))]
            cv2.putText(frame, label, (x1, y1-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        # 결과 퍼블리시
        try:
            out_msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
            out_msg.header = msg.header
            self.pub_annot.publish(out_msg)
            self.pub_det.publish(det_array)
        except Exception as e:
            self.get_logger().error(f'Publish error: {e}')

        # 디버그 표시
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
