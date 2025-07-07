from PyQt5.QtWidgets import QTableWidget, QTableWidgetItem

class JointStateView(QTableWidget):
    # 조인트 및 그리퍼 값을 표시하는 테이블 위젯

    def __init__(self):
        # 0행 2열로 초기화, 헤더는 'Name'과 'Value'
        super().__init__(0, 2)
        self.setHorizontalHeaderLabels(['Name', 'Value'])

    def update_states(self, names, positions):
        # 전달된 이름(names)과 값(positions)으로 테이블 갱신
        # names: ['panda_joint1', ..., 'gripper']
        # positions: [value1, ..., valueN]
        row_count = len(names)
        self.setRowCount(row_count)

        for row, (name, pos) in enumerate(zip(names, positions)):
            # 첫 열: 이름, 둘째 열: 소수점 3자리 값
            self.setItem(row, 0, QTableWidgetItem(name))
            self.setItem(row, 1, QTableWidgetItem(f'{pos:.3f}'))
