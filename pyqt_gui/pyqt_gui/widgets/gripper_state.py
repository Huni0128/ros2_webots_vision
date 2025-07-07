# pyqt_gui/widgets/gripper_state.py

from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import Qt
from ..utils import GRIPPER_OPEN_THRESHOLD

class GripperStateView(QLabel):
    def __init__(self):
        super().__init__('Gripper: --')
        self.setAlignment(Qt.AlignCenter)

    def update_state(self, names, positions):
        if not positions:
            return
        avg = sum(positions) / len(positions)
        status = 'Open' if avg > GRIPPER_OPEN_THRESHOLD else 'Closed'
        self.setText(f'Gripper: {status} ({avg:.3f} m)')
