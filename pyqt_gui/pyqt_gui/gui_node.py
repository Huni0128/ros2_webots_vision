# pyqt_gui/gui_node.py

import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QLabel, QVBoxLayout, QTableWidget,
    QTableWidgetItem, QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap

# 그리퍼을 Open/Closed 로 구분할 임계값 (m 단위)
GRIPPER_OPEN_THRESHOLD = 0.015


class RosQtNode(Node):
    def __init__(self):
        super().__init__('rosqt_gui_node')
        self.bridge = CvBridge()

        # 1) 카메라: Best‐Effort QoS (Webots 발행 정책과 매칭)
        camera_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            qos_profile=camera_qos
        )

        # 2) 로봇 조인트 상태
        self.create_subscription(
            JointState,
            '/panda_joint_states',
            self.joint_cb,
            10
        )

        # 3) 그리퍼 상태 (JointState)
        self.create_subscription(
            JointState,
            '/panda_gripper_state',
            self.gripper_cb,
            10
        )

        # GUI 업데이트용 콜백 레퍼런스
        self._img_callback = None
        self._joint_callback = None
        self._gripper_callback = None

    def image_cb(self, msg: Image):
        # ROS Image → OpenCV → QImage
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_img.shape
        rgb = cv_img[:, :, ::-1].copy()
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        if self._img_callback:
            self._img_callback(qimg)

    def joint_cb(self, msg: JointState):
        if self._joint_callback:
            self._joint_callback(msg.name, msg.position)

    def gripper_cb(self, msg: JointState):
        # 두 핑거 위치 값 리스트를 전달
        if self._gripper_callback:
            self._gripper_callback(msg.name, msg.position)


class MainWindow(QMainWindow):
    def __init__(self, ros_node: RosQtNode):
        super().__init__()
        self.setWindowTitle('ROS2 PyQt GUI')

        # 중앙 레이아웃
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        # ── 카메라 영상 ───────────────────────────
        self.image_label = QLabel('Waiting for image...')
        self.image_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.image_label)

        # ── 조인트 + 그리퍼 상태 ───────────────────
        hbox = QHBoxLayout()
        layout.addLayout(hbox)

        # 1) 로봇 조인트 테이블
        self.joint_table = QTableWidget(0, 2)
        self.joint_table.setHorizontalHeaderLabels(['Joint', 'Position (rad)'])
        hbox.addWidget(self.joint_table)

        # 2) 그리퍼 상태 라벨
        self.gripper_label = QLabel('Gripper: --')
        self.gripper_label.setAlignment(Qt.AlignCenter)
        hbox.addWidget(self.gripper_label)

        # ROS 콜백 바인딩
        ros_node._img_callback     = self.update_image
        ros_node._joint_callback   = self.update_joints
        ros_node._gripper_callback = self.update_gripper

    def update_image(self, qimg: QImage):
        pix = QPixmap.fromImage(qimg).scaled(640, 480, Qt.KeepAspectRatio)
        self.image_label.setPixmap(pix)

    def update_joints(self, names, positions):
        self.joint_table.setRowCount(len(names))
        for i, (n, p) in enumerate(zip(names, positions)):
            self.joint_table.setItem(i, 0, QTableWidgetItem(n))
            self.joint_table.setItem(i, 1, QTableWidgetItem(f'{p:.3f}'))

    def update_gripper(self, names, positions):
        """
        names: ['panda_finger_joint1','panda_finger_joint2']
        positions: [left_val, right_val]
        """
        if not positions:
            return
        # 두 값 평균 사용
        avg = sum(positions) / len(positions)
        status = "Open" if avg > GRIPPER_OPEN_THRESHOLD else "Closed"
        self.gripper_label.setText(f'Gripper: {status} ({avg:.3f} m)')


def main(args=None):
    # 1) ROS2 초기화
    rclpy.init(args=args)
    ros_node = RosQtNode()

    # 2) Qt 애플리케이션
    app = QApplication(sys.argv)
    win = MainWindow(ros_node)
    win.show()

    # 3) Qt 타이머로 ROS 스핀 호출 (10 ms 간격)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    # 4) Qt 이벤트 루프
    ret = app.exec_()

    # 5) 정리
    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
