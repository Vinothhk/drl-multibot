#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import sys
from std_msgs.msg import Float32MultiArray
from PyQt5.QtWidgets import *
from PyQt5.QtCore import Qt, QThread, QWaitCondition, QMutex, pyqtSignal, QTimer
from PyQt5 import QtCore

class ROS2Thread(QThread):
    def __init__(self, node):
        QThread.__init__(self)
        self.node = node

    def run(self):
        rclpy.spin(self.node)

class ROS2Node(Node):
    def __init__(self):
        super().__init__('progress3')
        self.create_subscription(Float32MultiArray, "bot_3/action", self.get_array, 10)
        self.data = None

    def get_array(self, array):
        self.data = array

class GUI(QMainWindow):
    change_value1 = pyqtSignal(int)
    change_value2 = pyqtSignal(int)
    change_value3 = pyqtSignal(int)
    change_value4 = pyqtSignal(int)
    change_value5 = pyqtSignal(int)
    change_total_reward = pyqtSignal(str)
    change_reward = pyqtSignal(str)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.initUI()
        self.initThread()

    def initUI(self):
        self.setWindowTitle("Action State for BOT 3")
        self.setGeometry(100, 100, 300, 400)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QGridLayout()

        self.pgsb1 = QProgressBar()
        self.pgsb1.setOrientation(Qt.Vertical)
        self.pgsb1.setRange(0, 100)
        layout.addWidget(self.pgsb1, 0, 1)

        self.pgsb2 = QProgressBar()
        self.pgsb2.setOrientation(Qt.Vertical)
        self.pgsb2.setRange(0, 100)
        layout.addWidget(self.pgsb2, 0, 2)

        self.pgsb3 = QProgressBar()
        self.pgsb3.setOrientation(Qt.Vertical)
        self.pgsb3.setRange(0, 100)
        layout.addWidget(self.pgsb3, 0, 3)

        self.pgsb4 = QProgressBar()
        self.pgsb4.setOrientation(Qt.Vertical)
        self.pgsb4.setRange(0, 100)
        layout.addWidget(self.pgsb4, 0, 4)

        self.pgsb5 = QProgressBar()
        self.pgsb5.setOrientation(Qt.Vertical)
        self.pgsb5.setRange(0, 100)
        layout.addWidget(self.pgsb5, 0, 5)

        self.total_reward_Label = QLabel("Total reward")
        layout.addWidget(self.total_reward_Label, 1, 0)
        self.total_reward = QLineEdit()
        self.total_reward.setDisabled(True)
        layout.addWidget(self.total_reward, 1, 1, 1, 2)

        self.reward_Label = QLabel("Reward")
        layout.addWidget(self.reward_Label, 2, 0)
        self.reward = QLineEdit()
        self.reward.setDisabled(True)
        layout.addWidget(self.reward, 2, 1, 1, 2)

        central_widget.setLayout(layout)

        self.change_value1.connect(self.pgsb1.setValue)
        self.change_value2.connect(self.pgsb2.setValue)
        self.change_value3.connect(self.pgsb3.setValue)
        self.change_value4.connect(self.pgsb4.setValue)
        self.change_value5.connect(self.pgsb5.setValue)
        self.change_total_reward.connect(self.total_reward.setText)
        self.change_reward.connect(self.reward.setText)

    def initThread(self):
        self.ros2_thread = ROS2Thread(self.node)
        self.ros2_thread.start()
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)

    def update_ui(self):
        if self.node.data is None:
            return

        data = self.node.data.data
        self.change_value1.emit(0)
        self.change_value2.emit(0)
        self.change_value3.emit(0)
        self.change_value4.emit(0)
        self.change_value5.emit(0)

        if data[0] == 0:
            self.change_value1.emit(100)
        elif data[0] == 1:
            self.change_value2.emit(100)
        elif data[0] == 2:
            self.change_value3.emit(100)
        elif data[0] == 3:
            self.change_value4.emit(100)
        elif data[0] == 4:
            self.change_value5.emit(100)

        self.change_total_reward.emit(str(round(data[-2], 2)))
        self.change_reward.emit(str(round(data[-1], 2)))

def main(args=None):
    rclpy.init(args=args)
    node = ROS2Node()
    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()