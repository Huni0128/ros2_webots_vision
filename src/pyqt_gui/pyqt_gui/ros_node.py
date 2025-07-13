# pyqt_gui/ros_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, Float64MultiArray, String   # ← String 추가
from cv_bridge import CvBridge

from PyQt5.QtGui import QImage


class RosQtNode(Node):
    def __init__(self):
        super().__init__('rosqt_gui_node')
        self.bridge = CvBridge()

        # 카메라 QoS
        camera_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)

        # 1) 카메라 원본 영상 구독
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            qos_profile=camera_qos
        )

        # 2) 객체 탐지 결과 영상 구독
        self.create_subscription(
            Image,
            '/camera/detected_objects',
            self.detected_cb,
            qos_profile=camera_qos
        )

        # 3) 로봇 조인트 상태 구독
        self.create_subscription(
            JointState,
            '/panda_joint_states',
            self.joint_cb,
            10
        )

        # 4) 그리퍼 상태 구독
        self.create_subscription(
            JointState,
            '/panda_gripper_state',
            self.gripper_cb,
            10
        )

        # 5) 수동 제어 퍼블리셔
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/panda_joint_command',
            10
        )
        self.gripper_cmd_pub = self.create_publisher(
            Float64,
            '/panda_gripper_command',
            10
        )

        # ▶ 추가: 박스 픽 명령 퍼블리셔
        self.pick_cmd_pub = self.create_publisher(
            String,
            '/pick_box',
            10
        )
        # ▶ 추가: 픽 상태(sub) 구독
        self.create_subscription(
            String,
            '/pick_box_status',
            self._on_pick_status,
            10
        )

        # GUI 콜백 핸들러들
        self._img_callback = None
        self._detected_img_callback = None
        self._joint_callback = None
        self._gripper_callback = None
        # ▶ 픽 상태 콜백
        self._pick_status_callback = None

    # … 기존 콜백들 그대로 …

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        h, w, _ = cv_img.shape
        rgb = cv_img[:, :, ::-1].copy()
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        if self._img_callback:
            self._img_callback(qimg)

    def detected_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_img.shape
        rgb = cv_img[:, :, ::-1].copy()
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        if self._detected_img_callback:
            self._detected_img_callback(qimg)

    def joint_cb(self, msg: JointState):
        if self._joint_callback:
            self._joint_callback(msg.name, msg.position)

    def gripper_cb(self, msg: JointState):
        if self._gripper_callback:
            self._gripper_callback(msg.name, msg.position)

    # ▶ 박스 픽 명령을 퍼블리시
    def send_pick_box(self, color_id: str):
        msg = String()
        msg.data = color_id
        self.pick_cmd_pub.publish(msg)
        self.get_logger().info(f"Sent pick command: {color_id}")

    # ▶ '/pick_box_status' 콜백
    def _on_pick_status(self, msg: String):
        if self._pick_status_callback:
            self._pick_status_callback(msg.data)
        else:
            self.get_logger().warn(f"Status received but no callback set: {msg.data}")

    # ▶ GUI에서 콜백 등록용
    def set_pick_status_callback(self, func):
        self._pick_status_callback = func

    # … 기존 send_joint_commands, send_gripper_command 그대로 …

    def send_joint_commands(self, joint_positions: list):
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.joint_cmd_pub.publish(msg)
        self.get_logger().debug(f"Sent joint commands: {joint_positions}")

    def send_gripper_command(self, value: float):
        msg = Float64()
        msg.data = value
        self.gripper_cmd_pub.publish(msg)
        self.get_logger().debug(f"Sent gripper command: {value:.3f}")
