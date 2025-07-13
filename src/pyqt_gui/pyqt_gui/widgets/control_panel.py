import math
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QHBoxLayout,
    QLabel, QDoubleSpinBox, QPushButton,
    QRadioButton, QButtonGroup               # ← 추가
)
from PyQt5.QtCore import Qt, pyqtSignal     # ← pyqtSignal 추가
from ..utils import JOINT_LIMITS_DEG

class ControlPanel(QWidget):
    # Panda 로봇 관절 및 그리퍼 수동 제어 패널

    pick_box = pyqtSignal(str)                # ← 추가: 'red_box' 등 문자열 emit

    def __init__(self, joint_names):
        super().__init__()
        self.joint_names = joint_names

        # 전체 레이아웃 (수직)
        layout = QVBoxLayout(self)

        # 제목 라벨
        title = QLabel("Manual Control")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # ◼ 관절 제어 섹션
        self.joint_spinboxes = {}
        for name in joint_names:
            group = QGroupBox(name)
            group_layout = QHBoxLayout(group)

            sb = QDoubleSpinBox()
            sb.setDecimals(2)
            lo_deg, hi_deg = JOINT_LIMITS_DEG[name]
            sb.setRange(lo_deg, hi_deg)
            sb.setSingleStep(1.0)

            limit_lbl = QLabel(f"{lo_deg:.1f}° ~ {hi_deg:.1f}°")
            limit_lbl.setAlignment(Qt.AlignCenter)

            group_layout.addWidget(sb)
            group_layout.addWidget(limit_lbl)
            layout.addWidget(group)

            self.joint_spinboxes[name] = sb

        # 관절 일괄 적용 버튼
        self.apply_all_btn = QPushButton("Apply All Joints")
        layout.addWidget(self.apply_all_btn)
        # 초기값 복원 버튼
        self.reset_btn = QPushButton("Reset to Initial")
        layout.addWidget(self.reset_btn)

        layout.addStretch()

        # ◼ 그리퍼 제어 섹션
        ggroup = QGroupBox("Gripper")
        glayout = QVBoxLayout(ggroup)

        spin_layout = QHBoxLayout()
        self.gripper_sb = QDoubleSpinBox()
        self.gripper_sb.setDecimals(3)
        self.gripper_sb.setRange(0.000, 0.040)
        self.gripper_sb.setSingleStep(0.001)
        self.gripper_btn = QPushButton("Set Gripper")
        spin_layout.addWidget(QLabel("Width [m]:"))
        spin_layout.addWidget(self.gripper_sb)
        spin_layout.addWidget(self.gripper_btn)
        glayout.addLayout(spin_layout)

        btn_layout = QHBoxLayout()
        self.open_btn = QPushButton("Open")
        self.close_btn = QPushButton("Close")
        btn_layout.addWidget(self.open_btn)
        btn_layout.addWidget(self.close_btn)
        glayout.addLayout(btn_layout)

        layout.addWidget(ggroup)

        # ◼ 픽 박스 제어 섹션 추가                  # ← 새로 추가된 섹션 시작
        pgroup = QGroupBox("Pick Box")
        playout = QHBoxLayout(pgroup)
        playout.addWidget(QLabel("Color:"))

        self.rb_red   = QRadioButton("Red")
        self.rb_green = QRadioButton("Green")
        self.rb_blue  = QRadioButton("Blue")
        self.rb_red.setChecked(True)

        color_group = QButtonGroup(self)
        for rb in (self.rb_red, self.rb_green, self.rb_blue):
            color_group.addButton(rb)
            playout.addWidget(rb)

        self.pick_btn = QPushButton("Pick")
        playout.addWidget(self.pick_btn)
        layout.addWidget(pgroup)

        # 클릭 시 내부 핸들러로 연결
        self.pick_btn.clicked.connect(self._on_pick_clicked)  # ← 추가된 연결
        # ◼ 픽 박스 섹션 끝

    def _on_pick_clicked(self):                   # ← 추가: 시그널 emit 핸들러
        if self.rb_red.isChecked():
            color = 'red_box'
        elif self.rb_green.isChecked():
            color = 'green_box'
        else:
            color = 'blue_box'
        self.pick_box.emit(color)

    def connect_joint_callback(self, fn):
        # Apply All 클릭 시 SpinBox(°)를 라디안으로 변환 후 퍼블리시
        self.apply_all_btn.clicked.connect(
            lambda: fn([
                self.joint_spinboxes[name].value() * math.pi / 180.0
                for name in self.joint_names
            ])
        )

    def connect_reset(self, fn):
        # Reset 클릭 시 초기 상태 복원 콜백 호출
        self.reset_btn.clicked.connect(fn)

    def connect_gripper_callback(self, fn):
        # Set Gripper 클릭 시 SpinBox(m) 값 그대로 퍼블리시
        self.gripper_btn.clicked.connect(
            lambda: fn(self.gripper_sb.value())
        )

    def connect_gripper_open_close(self, fn_open, fn_close):
        # Open 클릭 시 fn_open, Close 클릭 시 fn_close 호출
        self.open_btn.clicked.connect(fn_open)
        self.close_btn.clicked.connect(fn_close)

    def connect_pick_box(self, fn):               # ← 추가: 외부 연결 메서드
        """
        GUI 외부에서 pick 명령을 콜백(fn)으로 연결할 때 사용합니다.
        ex) control_panel.connect_pick_box(ros_node.send_pick_box)
        """
        self.pick_box.connect(fn)

    def update_joint_values(self, names, positions):
        # 첫 수신된 라디안 값 → SpinBox(°)로 초기값 설정
        mapping = dict(zip(names, positions))
        for name, sb in self.joint_spinboxes.items():
            if name in mapping:
                sb.setValue(mapping[name] * 180.0 / math.pi)
