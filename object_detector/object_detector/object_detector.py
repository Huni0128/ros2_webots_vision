import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class YOLOv8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')
        self.bridge = CvBridge()

        # 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # 탐지 결과 이미지 퍼블리셔
        self.publisher = self.create_publisher(
            Image,
            '/camera/detected_objects',
            qos_profile_sensor_data
        )

        # YOLO 모델 로드 (box 클래스만 포함된 커스텀 모델)
        model_path = 'yolo_models/yolov8n.pt'
        self.model = YOLO(model_path)

        self.get_logger().info(f"YOLOv8 Detector initialized with model: {model_path}")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        try:
            results = self.model.predict(source=frame, verbose=False, stream=True)
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    roi = frame[y1:y2, x1:x2]
                    hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

                    # 색상 비율 계산 함수
                    def color_ratio(hsv_img, lower, upper):
                        mask = cv2.inRange(hsv_img, lower, upper)
                        return cv2.countNonZero(mask) / (mask.size + 1e-6)

                    red = (
                        color_ratio(hsv, (0, 100, 100), (10, 255, 255)) +
                        color_ratio(hsv, (160, 100, 100), (179, 255, 255))
                    )
                    green = color_ratio(hsv, (35, 100, 100), (85, 255, 255))
                    blue = color_ratio(hsv, (100, 100, 100), (130, 255, 255))

                    color = max(
                        [("red", red), ("green", green), ("blue", blue)],
                        key=lambda x: x[1]
                    )[0]

                    # 주석 추가
                    label = f"{color}_box"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                # 결과 표시 및 퍼블리시
                annotated = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                cv2.imshow("YOLOv8 Color Detection", annotated)
                cv2.waitKey(1)

                out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                self.publisher.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f'YOLO inference error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
