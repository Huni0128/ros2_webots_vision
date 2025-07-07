# pyqt_gui/main.py

import sys
import rclpy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout
)
from PyQt5.QtCore import QTimer

from .ros_node import RosQtNode
from .widgets.image_view import ImageView
from .widgets.joint_state import JointStateView  # <-- 여전히 이 위젯 하나만 사용
from .utils import GRIPPER_OPEN_THRESHOLD

class MainWindow(QMainWindow):
    def __init__(self, ros_node: RosQtNode):
        super().__init__()
        self.setWindowTitle('ROS2 PyQt GUI')

        # 지난 joint / gripper 값 저장
        self.joint_names = []
        self.joint_positions = []
        self.gripper_avg = None

        # 중앙 레이아웃
        central = QWidget()
        self.setCentralWidget(central)
        vlay = QVBoxLayout(central)

        # 1) 카메라 영상
        self.image_view = ImageView()
        vlay.addWidget(self.image_view)

        # 2) 단일 테이블: joints + gripper
        self.state_table = JointStateView()
        vlay.addWidget(self.state_table)

        # ROS 콜백 바인딩
        ros_node._img_callback     = self.image_view.update_image
        ros_node._joint_callback   = self.on_joint_state
        ros_node._gripper_callback = self.on_gripper_state

        # ── 윈도우 초기 사이즈 지정 ─────────────────
        # 이미지(640x480) + 테이블 높이 + 여유분을 고려
        self.resize(800, 650)

    def on_joint_state(self, names, positions):
        self.joint_names = names
        self.joint_positions = positions
        self._refresh_table()

    def on_gripper_state(self, names, positions):
        # gripper positions: 두 finger 센서 값 평균 사용
        if positions:
            self.gripper_avg = sum(positions) / len(positions)
        else:
            self.gripper_avg = 0.0
        self._refresh_table()

    def _refresh_table(self):
        # joint data 없으면 아직 초기화 안 된 상태이므로 무시
        if not self.joint_names:
            return

        # 테이블에 들어갈 이름/값 리스트 준비
        names = list(self.joint_names)
        values = list(self.joint_positions)

        # 그리퍼 추가
        if self.gripper_avg is not None:
            names.append('gripper')
            values.append(self.gripper_avg)

        self.state_table.update_states(names, values)

def main(args=None):
    rclpy.init(args=args)
    ros_node = RosQtNode()

    app = QApplication(sys.argv)
    win = MainWindow(ros_node)
    win.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    ret = app.exec_()

    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
