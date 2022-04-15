#!/usr/bin/env python

import os
import sys

import rospy
import time
from PyQt5 import uic, QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import (
    QApplication,
    QDesktopWidget,
    QMainWindow,
    QWidget,
    QFileDialog,
)

from teleop import TeleOp
from camera_viz import Cameras
from fetch import Robot


class GUI(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        rospy.init_node("gui", anonymous=True)

        uic.loadUi("ui/control.ui", self)

        self.fetch = Robot("Fetchy")

        self.cameras = Cameras()
        self.teleop = TeleOp(self, self.fetch)

        self.layout.addWidget(self.cameras)
        self.layout.addWidget(self.teleop)

        self.Arm_Reset_Button.clicked.connect(self.reset_robo_arm)

        self.show()

    def reset_robo_arm(self):
        self.fetch.reset_arm()


def main(args=None):

    app = QApplication(sys.argv)
    window = GUI()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
