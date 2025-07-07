import sys
import rclpy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget,
    QVBoxLayout, QHBoxLayout, QMessageBox
)
from PyQt5.QtCore import QTimer

from .ros_node import RosQtNode
from .widgets.image_view import ImageView
from .widgets.joint_state import JointStateView
from .widgets.control_panel import ControlPanel


class MainWindow(QMainWindow):
    # 메인 윈도우: 카메라 영상, 상태 테이블, 수동 제어 패널 포함
    def __init__(self, ros_node: RosQtNode):
        super().__init__()
        self.setWindowTitle('ROS2 PyQt GUI')

        # ROS 노드 참조 저장 (Reset 시 사용)
        self.ros_node = ros_node

        # 내부 상태 저장 변수
        self.joint_names = []              # 최신 조인트 이름 리스트
        self.joint_positions = []          # 최신 조인트 위치 리스트
        self.gripper_avg = None            # 최신 그리퍼 평균 값
        self.joints_initialized = False    # SpinBox 초기화 여부 플래그

        # 최초 수신된 초기값 저장용
        self.initial_joint_positions = None
        self.initial_gripper = None

        # 중앙 위젯 및 레이아웃 설정
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)

        # 왼쪽 패널: 카메라 영상 + 조인트·그리퍼 상태 테이블
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        # 1) 카메라 영상
        self.image_view = ImageView()
        left_layout.addWidget(self.image_view)
        # 2) 상태 테이블
        self.state_table = JointStateView()
        left_layout.addWidget(self.state_table)
        root_layout.addWidget(left_widget, stretch=3)

        # 오른쪽 패널: 수동 제어 패널
        joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        self.control_panel = ControlPanel(joint_names)
        root_layout.addWidget(self.control_panel, stretch=1)

        # ROS 콜백 바인딩
        ros_node._img_callback     = self.image_view.update_image
        ros_node._joint_callback   = self.on_joint_state
        ros_node._gripper_callback = self.on_gripper_state

        # 제어 패널 버튼 → ROS 퍼블리시 함수 연결
        # 1) 관절 일괄 적용
        self.control_panel.connect_joint_callback(ros_node.send_joint_commands)
        # 2) 그리퍼 정확 제어
        self.control_panel.connect_gripper_callback(ros_node.send_gripper_command)
        # 3) 그리퍼 Open/Close
        self.control_panel.connect_gripper_open_close(
            lambda: ros_node.send_gripper_command(0.04),   # fully open
            lambda: ros_node.send_gripper_command(0.0)     # fully closed
        )
        # 4) 초기값 복원 (Reset)
        self.control_panel.connect_reset(self.reset_to_initial)

        # 초기 창 크기 지정
        self.resize(1024, 768)

    def on_joint_state(self, names, positions):
        # JointState 메시지 수신 시 호출
        #  - 테이블 갱신
        #  - 최초 수신 시 SpinBox 초기값 설정 및 초기값 저장
        self.joint_names = names
        self.joint_positions = positions
        self._refresh_table()

        if not self.joints_initialized:
            self.control_panel.update_joint_values(names, positions)
            self.joints_initialized = True

        if self.initial_joint_positions is None:
            self.initial_joint_positions = list(positions)

    def on_gripper_state(self, names, positions):
        # Gripper JointState 메시지 수신 시 호출
        #  - 평균 계산 → 테이블 갱신
        #  - 최초 수신 시 초기값 저장
        if positions:
            self.gripper_avg = sum(positions) / len(positions)
        else:
            self.gripper_avg = 0.0
        self._refresh_table()

        if self.initial_gripper is None:
            self.initial_gripper = self.gripper_avg

    def _refresh_table(self):
        # 테이블에 조인트 + 그리퍼 상태 모두 표시
        if not self.joint_names:
            return
        names = list(self.joint_names)
        values = list(self.joint_positions)
        if self.gripper_avg is not None:
            names.append('gripper')
            values.append(self.gripper_avg)
        self.state_table.update_states(names, values)

    def reset_to_initial(self):
        # Reset 버튼 클릭 시 최초 수신된 값으로 복원
        # 1) 관절 복원
        resp = QMessageBox.question(
            self,
            'Confirm Reset',
            '초기화 하시겠습니까?',
            QMessageBox.Yes | QMessageBox.No,
        )
        
        if resp != QMessageBox.Yes:
            return

        if self.initial_joint_positions is not None:
            self.ros_node.send_joint_commands(self.initial_joint_positions)
            self.control_panel.update_joint_values(
                self.joint_names,
                self.initial_joint_positions
            )
        # 2) 그리퍼 복원
        if self.initial_gripper is not None:
            self.ros_node.send_gripper_command(self.initial_gripper)
            self.control_panel.gripper_sb.setValue(self.initial_gripper)


def main(args=None):
    # 1) ROS2 초기화
    rclpy.init(args=args)
    ros_node = RosQtNode()

    # 2) Qt 애플리케이션 및 메인 윈도우 생성
    app = QApplication(sys.argv)
    win = MainWindow(ros_node)
    win.show()

    # 3) QTimer로 주기적 ROS 스핀 (10ms 간격)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    # 4) Qt 이벤트 루프 실행
    ret = app.exec_()

    # 5) 정리: 스핀 중지, 노드·ROS 종료
    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
