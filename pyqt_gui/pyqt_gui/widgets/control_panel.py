# pyqt_gui/widgets/control_panel.py

import math
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QHBoxLayout,
    QLabel, QDoubleSpinBox, QPushButton
)
from PyQt5.QtCore import Qt

class ControlPanel(QWidget):
    # Panda 로봇 조인트와 그리퍼를 수동으로 제어하기 위한 패널

    def __init__(self, joint_names):
        # joint_names: ['panda_joint1', …, 'panda_joint7']
        super().__init__()
        self.joint_names = joint_names

        # 전체 레이아웃 (수직)
        layout = QVBoxLayout(self)

        # 제목
        label = QLabel("Manual Control")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        # 각 조인트별 SpinBox (단위: 도)
        self.joint_spinboxes = {}
        for name in joint_names:
            group = QGroupBox(name)
            group_layout = QHBoxLayout(group)

            sb = QDoubleSpinBox()
            sb.setDecimals(2)            # 소수점 2자리
            sb.setRange(-180.0, 180.0)    # 도 단위 범위
            sb.setSingleStep(1.0)        # 1° 단위 이동

            group_layout.addWidget(sb)
            layout.addWidget(group)
            self.joint_spinboxes[name] = sb

        # 모든 조인트 일괄 적용 버튼
        self.apply_all_btn = QPushButton("Apply All Joints")
        layout.addWidget(self.apply_all_btn)

        # 아래 여백
        layout.addStretch()

        # 그리퍼 제어 그룹 (단위: m)
        ggroup = QGroupBox("gripper")
        glayout = QHBoxLayout(ggroup)

        self.gripper_sb = QDoubleSpinBox()
        self.gripper_sb.setDecimals(3)
        self.gripper_sb.setRange(0.000, 0.040)
        self.gripper_sb.setSingleStep(0.001)

        self.gripper_btn = QPushButton("Set Gripper")
        glayout.addWidget(self.gripper_sb)
        glayout.addWidget(self.gripper_btn)
        layout.addWidget(ggroup)

    def connect_joint_callback(self, fn):
        # Apply All 클릭 시 spinbox(도) → 라디안 변환 후 전달
        self.apply_all_btn.clicked.connect(
            lambda: fn([
                self.joint_spinboxes[name].value() * math.pi / 180.0
                for name in self.joint_names
            ])
        )

    def connect_gripper_callback(self, fn):
        # Set Gripper 클릭 시 값 전달
        self.gripper_btn.clicked.connect(
            lambda: fn(self.gripper_sb.value())
        )

    def update_joint_values(self, names, positions):
        # JointState(라디안) → SpinBox(도) 초기값 설정
        mapping = dict(zip(names, positions))
        for name, sb in self.joint_spinboxes.items():
            if name in mapping:
                sb.setValue(mapping[name] * 180.0 / math.pi)
