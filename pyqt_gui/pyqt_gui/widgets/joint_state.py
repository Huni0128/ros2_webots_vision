# pyqt_gui/widgets/joint_state.py

from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem

class JointStateView(QTableWidget):
    def __init__(self):
        super().__init__(0, 2)
        # 범용 컬럼명: Name / Value
        self.setHorizontalHeaderLabels(['Name', 'Value'])

    def update_states(self, names, positions):
        """
        names: list of joint names (e.g. ['panda_joint1', …, 'panda_finger_joint1'])
        positions: list of corresponding values
        """
        count = len(names)
        self.setRowCount(count)
        for i, (n, p) in enumerate(zip(names, positions)):
            self.setItem(i, 0, QTableWidgetItem(n))
            # 단위 혼합(rad vs m)도 같이 표시하고 싶으면 여기서 포맷 변경 가능
            self.setItem(i, 1, QTableWidgetItem(f'{p:.3f}'))
