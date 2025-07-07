# pyqt_gui/ros_node.py

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

class RosQtNode(Node):
    def __init__(self):
        super().__init__('rosqt_gui_node')
        self.bridge = CvBridge()

        # 1) 카메라: Best‐Effort QoS
        camera_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            qos_profile=camera_qos)

        # 2) 로봇 조인트 상태
        self.create_subscription(
            JointState,
            '/panda_joint_states',
            self.joint_cb,
            10)

        # 3) 그리퍼 상태 (JointState)
        self.create_subscription(
            JointState,
            '/panda_gripper_state',
            self.gripper_cb,
            10)

        # GUI 콜백 슬롯
        self._img_callback = None
        self._joint_callback = None
        self._gripper_callback = None

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_img.shape
        rgb = cv_img[:, :, ::-1].copy()
        from PyQt5.QtGui import QImage
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        if self._img_callback:
            self._img_callback(qimg)

    def joint_cb(self, msg: JointState):
        if self._joint_callback:
            self._joint_callback(msg.name, msg.position)

    def gripper_cb(self, msg: JointState):
        if self._gripper_callback:
            self._gripper_callback(msg.name, msg.position)
