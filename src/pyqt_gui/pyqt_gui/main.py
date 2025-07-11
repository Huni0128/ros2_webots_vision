import sys
import rclpy
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from .ros_node import RosQtNode
from .main_window import MainWindow


def main(args=None):
    rclpy.init(args=args)
    ros_node = RosQtNode()

    app = QApplication(sys.argv)
    win = MainWindow(ros_node)
    win.show()

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)

    ret = app.exec_()

    timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
