# pyqt_gui/widgets/joint_state.py

import math
from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem

class JointStateView(QTableWidget):
    # 조인트 및 그리퍼 값을 도(°) 단위로 표시하는 테이블 위젯

    def __init__(self):
        # 0행 2열로 초기화, 헤더는 'Name'과 'Position (°)'
        super().__init__(0, 2)
        self.setHorizontalHeaderLabels(['Name', 'Position (°)'])

    def update_states(self, names, positions):
        # 전달된 이름(names)과 값(positions: 라디안)으로 테이블 갱신
        # names: ['panda_joint1', ..., 'gripper']
        # positions: [rad1, ..., radN or m for gripper]
        row_count = len(names)
        self.setRowCount(row_count)

        for row, (name, pos) in enumerate(zip(names, positions)):
            # 첫 열: 이름
            self.setItem(row, 0, QTableWidgetItem(name))
            # 둘째 열: 라디안 → 도(°) 변환, 그리퍼면 그대로 미터 단위 표시
            if name.startswith('panda_joint'):
                deg = pos * 180.0 / math.pi
                text = f'{deg:.2f}°'
            else:
                # gripper 행: 원래 값(m)을 소수점 3자리로
                text = f'{pos:.3f} m'
            self.setItem(row, 1, QTableWidgetItem(text))
