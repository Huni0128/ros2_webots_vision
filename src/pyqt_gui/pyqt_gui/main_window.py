from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QRadioButton, QButtonGroup, QLabel, QMessageBox
)
from .widgets.image_view import ImageView
from .widgets.joint_state import JointStateView
from .widgets.control_panel import ControlPanel
from .ros_node import RosQtNode


class MainWindow(QMainWindow):
    def __init__(self, ros_node: RosQtNode):
        super().__init__()
        self.setWindowTitle('ROS2 PyQt GUI')

        self.ros_node = ros_node
        self.joint_names = []
        self.joint_positions = []
        self.gripper_avg = None
        self.joints_initialized = False
        self.initial_joint_positions = None
        self.initial_gripper = None
        self.latest_normal_img = None
        self.latest_detected_img = None

        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)

        # 왼쪽 패널
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)

        # 라디오 버튼 UI 추가 (일반 화면 / 객체 탐지)
        self.radio_normal = QRadioButton("일반 화면")
        self.radio_detected = QRadioButton("객체 탐지")
        self.radio_normal.setChecked(True)

        radio_group = QButtonGroup()
        radio_group.addButton(self.radio_normal)
        radio_group.addButton(self.radio_detected)

        radio_layout = QHBoxLayout()
        radio_layout.addWidget(QLabel("화면 선택:"))
        radio_layout.addWidget(self.radio_normal)
        radio_layout.addWidget(self.radio_detected)
        left_layout.addLayout(radio_layout)

        # 이미지 출력 뷰
        self.image_view = ImageView()
        left_layout.addWidget(self.image_view)

        # 상태 테이블
        self.state_table = JointStateView()
        left_layout.addWidget(self.state_table)
        root_layout.addWidget(left_widget, stretch=3)

        # 오른쪽 패널 (조작)
        joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        self.control_panel = ControlPanel(joint_names)
        root_layout.addWidget(self.control_panel, stretch=1)

        # ROS 콜백 연결
        ros_node._img_callback = self.on_normal_image
        ros_node._detected_img_callback = self.on_detected_image
        ros_node._joint_callback = self.on_joint_state
        ros_node._gripper_callback = self.on_gripper_state

        # 제어 이벤트 연결
        self.control_panel.connect_joint_callback(ros_node.send_joint_commands)
        self.control_panel.connect_gripper_callback(ros_node.send_gripper_command)
        self.control_panel.connect_gripper_open_close(
            lambda: ros_node.send_gripper_command(0.04),
            lambda: ros_node.send_gripper_command(0.0)
        )
        self.control_panel.connect_reset(self.reset_to_initial)

        # 라디오 버튼 동작 연결
        self.radio_normal.toggled.connect(self.on_radio_toggle)
        self.radio_detected.toggled.connect(self.on_radio_toggle)

        self.resize(1024, 768)

    def on_normal_image(self, qimg):
        self.latest_normal_img = qimg
        if self.radio_normal.isChecked():
            self.image_view.update_image(qimg)

    def on_detected_image(self, qimg):
        self.latest_detected_img = qimg
        if self.radio_detected.isChecked():
            self.image_view.update_image(qimg)

    def on_radio_toggle(self):
        if self.radio_normal.isChecked() and self.latest_normal_img:
            self.image_view.update_image(self.latest_normal_img)
        elif self.radio_detected.isChecked() and self.latest_detected_img:
            self.image_view.update_image(self.latest_detected_img)

    def on_joint_state(self, names, positions):
        self.joint_names = names
        self.joint_positions = positions
        self._refresh_table()

        if not self.joints_initialized:
            self.control_panel.update_joint_values(names, positions)
            self.joints_initialized = True

        if self.initial_joint_positions is None:
            self.initial_joint_positions = list(positions)

    def on_gripper_state(self, names, positions):
        self.gripper_avg = sum(positions) / len(positions) if positions else 0.0
        self._refresh_table()

        if self.initial_gripper is None:
            self.initial_gripper = self.gripper_avg

    def _refresh_table(self):
        if not self.joint_names:
            return
        names = list(self.joint_names)
        values = list(self.joint_positions)
        if self.gripper_avg is not None:
            names.append('gripper')
            values.append(self.gripper_avg)
        self.state_table.update_states(names, values)

    def reset_to_initial(self):
        resp = QMessageBox.question(
            self, 'Confirm Reset', '초기화 하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No,
        )
        if resp != QMessageBox.Yes:
            return

        if self.initial_joint_positions is not None:
            self.ros_node.send_joint_commands(self.initial_joint_positions)
            self.control_panel.update_joint_values(self.joint_names, self.initial_joint_positions)

        if self.initial_gripper is not None:
            self.ros_node.send_gripper_command(self.initial_gripper)
            self.control_panel.gripper_sb.setValue(self.initial_gripper)
