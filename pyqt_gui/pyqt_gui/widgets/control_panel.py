import math
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QHBoxLayout,
    QLabel, QDoubleSpinBox, QPushButton
)
from PyQt5.QtCore import Qt
from ..utils import JOINT_LIMITS_DEG

class ControlPanel(QWidget):
    # Panda 로봇의 관절 및 그리퍼를 수동 제어하는 패널

    def __init__(self, joint_names):
        # joint_names: ['panda_joint1', …, 'panda_joint7']
        super().__init__()
        self.joint_names = joint_names

        # 전체 레이아웃 (수직 배치)
        layout = QVBoxLayout(self)

        # 제목 라벨
        title = QLabel("Manual Control")
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # 관절 제어 위젯 생성
        self.joint_spinboxes = {}
        for name in joint_names:
            # 각 관절마다 그룹 박스로 이름 표시
            group = QGroupBox(name)
            group_layout = QHBoxLayout(group)

            # 각도 입력용 SpinBox (단위: 도)
            sb = QDoubleSpinBox()
            sb.setDecimals(2)              # 소수점 2자리
            lo_deg, hi_deg = JOINT_LIMITS_DEG[name]
            sb.setRange(lo_deg, hi_deg)    # 관절별 허용 범위
            sb.setSingleStep(1.0)          # 1도 단위 조절

            # 범위 표시 라벨
            limit_lbl = QLabel(f"{lo_deg:.1f}° ~ {hi_deg:.1f}°")
            limit_lbl.setAlignment(Qt.AlignCenter)

            # 그룹 레이아웃에 추가
            group_layout.addWidget(sb)
            group_layout.addWidget(limit_lbl)
            layout.addWidget(group)

            self.joint_spinboxes[name] = sb

        # 관절 일괄 적용 버튼
        self.apply_all_btn = QPushButton("Apply All Joints")
        layout.addWidget(self.apply_all_btn)

        # 아래쪽 여백 추가
        layout.addStretch()

        # 그리퍼 제어 위젯 생성
        ggroup = QGroupBox("Gripper")
        glayout = QVBoxLayout(ggroup)

        # 그리퍼 정확 제어용 SpinBox + 적용 버튼
        spin_layout = QHBoxLayout()
        self.gripper_sb = QDoubleSpinBox()
        self.gripper_sb.setDecimals(3)     # 소수점 3자리
        self.gripper_sb.setRange(0.000, 0.040)  # 개방 폭 범위 (m)
        self.gripper_sb.setSingleStep(0.001)    # 0.001m 단위
        self.gripper_btn = QPushButton("Set Gripper")
        spin_layout.addWidget(QLabel("Width [m]:"))
        spin_layout.addWidget(self.gripper_sb)
        spin_layout.addWidget(self.gripper_btn)
        glayout.addLayout(spin_layout)

        # 그리퍼 Open/Close 버튼
        btn_layout = QHBoxLayout()
        self.open_btn = QPushButton("Open")
        self.close_btn = QPushButton("Close")
        btn_layout.addWidget(self.open_btn)
        btn_layout.addWidget(self.close_btn)
        glayout.addLayout(btn_layout)

        # 그리퍼 그룹을 메인 레이아웃에 추가
        layout.addWidget(ggroup)

    def connect_joint_callback(self, fn):
        # Apply All 클릭 시 SpinBox(°) → 라디안 변환 후 fn 호출
        self.apply_all_btn.clicked.connect(
            lambda: fn([
                self.joint_spinboxes[name].value() * math.pi / 180.0
                for name in self.joint_names
            ])
        )

    def connect_gripper_callback(self, fn):
        # Set Gripper 클릭 시 SpinBox(m) 값 그대로 fn 호출
        self.gripper_btn.clicked.connect(
            lambda: fn(self.gripper_sb.value())
        )

    def connect_gripper_open_close(self, fn_open, fn_close):
        # Open/Close 버튼 클릭 시 각각 fn_open, fn_close 호출
        self.open_btn.clicked.connect(fn_open)
        self.close_btn.clicked.connect(fn_close)

    def update_joint_values(self, names, positions):
        # 수신된 라디안 값 → 도 단위로 변환 후 SpinBox 초기화
        mapping = dict(zip(names, positions))
        for name, sb in self.joint_spinboxes.items():
            if name in mapping:
                sb.setValue(mapping[name] * 180.0 / math.pi)
