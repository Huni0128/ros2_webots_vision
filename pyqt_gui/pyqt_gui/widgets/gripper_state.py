from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt
from ..utils import GRIPPER_OPEN_THRESHOLD

class GripperStateView(QLabel):
    # 그리퍼 상태(Open/Closed)와 현재 개방 폭을 표시하는 QLabel

    def __init__(self):
        # 초기 텍스트 설정 및 가운데 정렬
        super().__init__('Gripper: --')
        self.setAlignment(Qt.AlignCenter)

    def update_state(self, names, positions):
        # 그리퍼 위치 센서 값 리스트를 받아 상태 갱신
        # names: ['panda_finger_joint1', 'panda_finger_joint2']
        # positions: [left_value, right_value]
        if not positions:
            return

        # 두 센서 값의 평균을 사용
        avg = sum(positions) / len(positions)

        # 임계값 초과 시 Open, 이하면 Closed
        status = 'Open' if avg > GRIPPER_OPEN_THRESHOLD else 'Closed'

        # 최종 텍스트: 상태 및 평균 개방 폭 (소수점 3자리)
        self.setText(f'Gripper: {status} ({avg:.3f} m)')
