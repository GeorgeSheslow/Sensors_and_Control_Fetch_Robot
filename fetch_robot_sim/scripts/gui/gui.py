#!/usr/bin/env python
from cmath import pi
import os
import sys

import rospy
import actionlib

from PyQt5 import uic, QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import (
    QApplication,
    QDesktopWidget,
    QMainWindow,
    QWidget,
    QFileDialog,
)

from PyQt5 import uic

from teleop import TeleOp
from camera_viz import Cameras
from fetch import Robot

from sensor_msgs.msg import JointState

import math

#from scripts.sim.fetch import Robot

class GUI(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.currentJointStates = 0

        rospy.init_node("gui", anonymous=True)

        rospy.Subscriber("joint_states", JointState, self.jointCallback)
        uic.loadUi("ui/control.ui", self)

        self.fetch = Robot("Fetchy")

        #self.cameras = Cameras()
        self.teleop = TeleOp(self, self.fetch)
        self.show()

        #self.layout.addWidget(self.cameras)
        self.layout.addWidget(self.teleop)


    def jointCallback(self, data):
        self.currentJointStates = data.position
        self.J1_label.setText(self.returnDegreesString(1,data.position[0]))
        self.J2_label.setText(self.returnDegreesString(2,data.position[1]))
        self.J3_label.setText(self.returnDegreesString(3,data.position[2]))
        self.J4_label.setText(self.returnDegreesString(4,data.position[3]))
        self.J5_label.setText(self.returnDegreesString(5,data.position[4]))
        self.J6_label.setText(self.returnDegreesString(6,data.position[5]))
        self.J7_label.setText(self.returnDegreesString(7,data.position[6]))


    def returnDegreesString(self,joint_num, rads):
        #"{:.1f}".format(data)
        temp = "J" + str(joint_num) + ": "
        temp += str("{:.1f}".format(rads * (180/math.pi)))
        return temp

def main(args=None):

    app = QApplication(sys.argv)
    window = GUI()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
