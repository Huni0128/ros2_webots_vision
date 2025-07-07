import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64, Float64MultiArray
from cv_bridge import CvBridge

class RosQtNode(Node):
    # ROS2 노드: Webots→ROS 구독 및 GUI↔ROS 퍼블리시 담당
    def __init__(self):
        super().__init__('rosqt_gui_node')
        self.bridge = CvBridge()

        # 1) 카메라 영상 구독 (Best‐Effort QoS)
        camera_qos = QoSProfile(depth=10,
                                reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            qos_profile=camera_qos
        )

        # 2) 로봇 조인트 상태 구독
        self.create_subscription(
            JointState,
            '/panda_joint_states',
            self.joint_cb,
            10
        )

        # 3) 그리퍼 상태 구독 (JointState)
        self.create_subscription(
            JointState,
            '/panda_gripper_state',
            self.gripper_cb,
            10
        )

        # 4) 수동 제어 퍼블리셔
        #    - joint_cmd_pub: 7개 관절 일괄 제어용 Float64MultiArray
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/panda_joint_command',
            10
        )
        #    - gripper_cmd_pub: 그리퍼 개별 제어용 Float64
        self.gripper_cmd_pub = self.create_publisher(
            Float64,
            '/panda_gripper_command',
            10
        )

        # GUI 콜백 슬롯 초기화
        self._img_callback = None
        self._joint_callback = None
        self._gripper_callback = None

    def image_cb(self, msg: Image):
        # 수신된 ROS Image 메시지를 QImage로 변환 후 콜백 호출
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_img.shape
        rgb = cv_img[:, :, ::-1].copy()
        from PyQt5.QtGui import QImage
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        if self._img_callback:
            self._img_callback(qimg)

    def joint_cb(self, msg: JointState):
        # 수신된 JointState 메시지 전달
        if self._joint_callback:
            self._joint_callback(msg.name, msg.position)

    def gripper_cb(self, msg: JointState):
        # 수신된 그리퍼 JointState 메시지 전달
        if self._gripper_callback:
            self._gripper_callback(msg.name, msg.position)

    def send_joint_commands(self, joint_positions: list):
        # 모든 관절 위치를 Float64MultiArray로 퍼블리시
        msg = Float64MultiArray()
        msg.data = joint_positions
        self.joint_cmd_pub.publish(msg)
        self.get_logger().debug(f"Sent joint commands: {joint_positions}")

    def send_gripper_command(self, value: float):
        # 그리퍼 개방 폭을 Float64로 퍼블리시
        msg = Float64()
        msg.data = value
        self.gripper_cmd_pub.publish(msg)
        self.get_logger().debug(f"Sent gripper command: {value:.3f}")
