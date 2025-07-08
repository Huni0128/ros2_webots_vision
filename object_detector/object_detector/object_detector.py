import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from rclpy.qos import qos_profile_sensor_data

class YOLOv8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')
        self.bridge = CvBridge()

        # QoS 설정: 센서 데이터용
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # 모델 로드 (경량 YOLOv8 + 명시적 경로)
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
                annotated = r.plot()
                cv2.imshow("YOLOv8 Detection", annotated)
                cv2.waitKey(1)
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
