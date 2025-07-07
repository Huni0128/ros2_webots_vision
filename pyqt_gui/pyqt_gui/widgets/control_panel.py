from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QHBoxLayout,
    QLabel, QDoubleSpinBox, QPushButton
)
from PyQt5.QtCore import Qt

class ControlPanel(QWidget):
    # Panda 로봇 조인트와 그리퍼를 수동으로 제어하기 위한 패널

    def __init__(self, joint_names):
        # joint_names: ['panda_joint1', …, 'panda_joint7'] 형태의 리스트
        super().__init__()
        self.joint_names = joint_names

        # 전체 레이아웃 설정 (수직)
        layout = QVBoxLayout(self)

        # 제목 라벨
        label = QLabel("Manual Control")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        # 각 조인트별 SpinBox 생성
        self.joint_spinboxes = {}
        for name in joint_names:
            # 그룹 박스로 이름 표시
            group = QGroupBox(name)
            group_layout = QHBoxLayout(group)

            sb = QDoubleSpinBox()
            sb.setDecimals(2)          # 소수점 2자리 표시
            sb.setRange(-3.14, 3.14)   # 관절 허용 범위 (rad)
            sb.setSingleStep(0.01)     # 스텝 크기

            group_layout.addWidget(sb)
            layout.addWidget(group)
            self.joint_spinboxes[name] = sb

        # 모든 조인트 값을 한 번에 퍼블리시하는 버튼
        self.apply_all_btn = QPushButton("Apply All Joints")
        layout.addWidget(self.apply_all_btn)

        # 아래쪽 여백 확보
        layout.addStretch()

        # 그리퍼 제어용 그룹 박스
        ggroup = QGroupBox("gripper")
        glayout = QHBoxLayout(ggroup)

        self.gripper_sb = QDoubleSpinBox()
        self.gripper_sb.setDecimals(3)      # 소수점 3자리 표시
        self.gripper_sb.setRange(0.000, 0.040)  # 그리퍼 개방 범위 (m)
        self.gripper_sb.setSingleStep(0.001)     # 스텝 크기 (m)

        self.gripper_btn = QPushButton("Set Gripper")
        glayout.addWidget(self.gripper_sb)
        glayout.addWidget(self.gripper_btn)
        layout.addWidget(ggroup)

    def connect_joint_callback(self, fn):
        # "Apply All Joints" 버튼 클릭 시 fn(joint_values_list) 호출
        self.apply_all_btn.clicked.connect(
            lambda: fn([
                self.joint_spinboxes[name].value()
                for name in self.joint_names
            ])
        )

    def connect_gripper_callback(self, fn):
        # "Set Gripper" 버튼 클릭 시 fn(gripper_value) 호출
        self.gripper_btn.clicked.connect(
            lambda: fn(self.gripper_sb.value())
        )

    def update_joint_values(self, names, positions):
        # 받은 이름/값 쌍을 SpinBox 값으로 초기화
        mapping = dict(zip(names, positions))
        for name, sb in self.joint_spinboxes.items():
            if name in mapping:
                sb.setValue(mapping[name])
