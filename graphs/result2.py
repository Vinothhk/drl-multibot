#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyqtgraph as pg
import sys
import pickle
from PyQt5.QtWidgets import QApplication
from std_msgs.msg import Float32MultiArray, Float32
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QMainWindow
class GraphNode(Node):
    def __init__(self):
        super().__init__('graph')
        self.graph_sub = self.create_subscription(Float32MultiArray, 'bot_2/result', self.data_callback, 10)
        self.ep = []
        self.data = []
        self.rewards = []
        self.count = 1
        self.size_ep = 0
        load_data = False

        if load_data:
            self.ep, self.data = self.load_data()
            self.size_ep = len(self.ep)

    def data_callback(self, msg):
        self.data.append(msg.data[0])
        self.ep.append(self.size_ep + self.count)
        self.count += 1
        self.rewards.append(msg.data[1])

    def load_data(self):
        try:
            with open("graph2.txt", "rb") as f:
                x, y = pickle.load(f)
        except:
            x, y = [], []
        return x, y

    def save_data(self, data):
        with open("graph2.txt", "wb") as f:
            pickle.dump(data, f)
            
class Window(QMainWindow):
    def __init__(self,node):
        super(Window, self).__init__()
        self.node = node
        self.setWindowTitle("Result for BOT 2")
        self.setGeometry(50, 50, 600, 650)
        self.x = []
        self.plot()

    def plot(self):
        self.qValuePlt = pg.PlotWidget(self, title="Average max Q-value")
        self.qValuePlt.move(0, 320)
        self.qValuePlt.resize(600, 300)
        self.timer1 = pg.QtCore.QTimer()
        self.timer1.timeout.connect(self.update)
        self.timer1.start(200)

        self.rewardsPlt = pg.PlotWidget(self, title="Total reward")
        self.rewardsPlt.move(0, 10)
        self.rewardsPlt.resize(600, 300)

        self.timer2 = pg.QtCore.QTimer()
        self.timer2.timeout.connect(self.update)
        self.timer2.start(100)

        self.show()

    def update(self):
        self.rewardsPlt.showGrid(x=True, y=True)
        self.qValuePlt.showGrid(x=True, y=True)
        self.rewardsPlt.plot(self.node.ep, self.node.data, pen=(255, 0, 0))
        self.node.save_data([self.node.ep, self.node.data])
        self.qValuePlt.plot(self.node.ep, self.node.rewards, pen=(0, 255, 0))


# def run():
#         rospy.init_node('graph')
#         app = QApplication(sys.argv)
#         GUI = Window()
#         sys.exit(app.exec_())

# run()

def main(args=None):
    rclpy.init(args=args)
    node = GraphNode()
    app = QApplication(sys.argv)
    gui = Window(node)

    def ros_spin():
        rclpy.spin(node)

    thread = QThread()
    thread.run = ros_spin
    thread.start()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()