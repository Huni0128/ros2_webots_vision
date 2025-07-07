import sys
import rclpy
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout
from PyQt5.QtCore import QTimer

from .ros_node import RosQtNode
from .widgets.image_view import ImageView
from .widgets.joint_state import JointStateView
from .widgets.control_panel import ControlPanel
from .utils import GRIPPER_OPEN_THRESHOLD

class MainWindow(QMainWindow):
    # 메인 윈도우: 카메라 영상, 상태 테이블, 수동 제어 패널 포함
    def __init__(self, ros_node: RosQtNode):
        super().__init__()
        self.setWindowTitle('ROS2 PyQt GUI')

        # 내부 상태 저장 변수
        self.joint_names = []
        self.joint_positions = []
        self.gripper_avg = None
        self.joints_initialized = False  # 스핀박스 초기화 플래그

        # 중앙 위젯 및 레이아웃 설정
        central = QWidget()
        self.setCentralWidget(central)
        root_layout = QHBoxLayout(central)

        # 왼쪽: 카메라 영상 + 상태 테이블
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        self.image_view = ImageView()         # 카메라 영상 표시
        left_layout.addWidget(self.image_view)
        self.state_table = JointStateView()   # 조인트+그리퍼 상태 테이블
        left_layout.addWidget(self.state_table)
        root_layout.addWidget(left_widget, stretch=3)

        # 오른쪽: 수동 제어 패널
        joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        self.control_panel = ControlPanel(joint_names)
        root_layout.addWidget(self.control_panel, stretch=1)

        # ROS 콜백 연결
        ros_node._img_callback     = self.image_view.update_image
        ros_node._joint_callback   = self.on_joint_state
        ros_node._gripper_callback = self.on_gripper_state

        # ControlPanel 버튼 → ROS 퍼블리시 함수 연결
        self.control_panel.connect_joint_callback(ros_node.send_joint_commands)
        self.control_panel.connect_gripper_callback(ros_node.send_gripper_command)

        # 초기 창 크기 지정
        self.resize(1024, 768)

    def on_joint_state(self, names, positions):
        # JointState 수신 시 호출, 테이블 갱신 및 스핀박스 초기화
        self.joint_names = names
        self.joint_positions = positions
        self._refresh_table()
        if not self.joints_initialized:
            self.control_panel.update_joint_values(names, positions)
            self.joints_initialized = True

    def on_gripper_state(self, names, positions):
        # Gripper JointState 수신 시 호출, 평균 계산 후 테이블 갱신
        if positions:
            self.gripper_avg = sum(positions) / len(positions)
        else:
            self.gripper_avg = 0.0
        self._refresh_table()

    def _refresh_table(self):
        # 테이블에 Joint + Gripper 상태를 모두 갱신
        if not self.joint_names:
            return
        names = list(self.joint_names)
        values = list(self.joint_positions)
        if self.gripper_avg is not None:
            names.append('gripper')
            values.append(self.gripper_avg)
        self.state_table.update_states(names, values)

def main(args=None):
    # ROS2 초기화
    rclpy.init(args=args)
    ros_node = RosQtNode()

    # Qt 애플리케이션 및 메인 윈도우 생성
    app = QApplication(sys.argv)
    win = MainWindow(ros_node)
    win.show()

    # QTimer로 주기적 ROS 스핀 (10ms 간격)
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    # Qt 이벤트 루프 실행
    ret = app.exec_()

    # 종료 정리
    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)

if __name__ == '__main__':
    main()
