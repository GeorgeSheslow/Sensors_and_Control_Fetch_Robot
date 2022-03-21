#!/usr/bin/env python
import os
import sys

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
from cameras import Cameras
from sim.fetch import Robot

#from scripts.sim.fetch import Robot

class GUI(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        uic.loadUi("ui/control.ui", self)

        self.cameras = Cameras()
        self.teleop = TeleOp()

        self.fetch = Robot()

        self.layout.addWidget(self.cameras)
        self.layout.addWidget(self.teleop)

        self.show()

        self.teleop.J1_Slider.valueChanged.connect(self.robo_J1_update)
        self.teleop.J2_Slider.valueChanged.connect(self.robo_J2_update)
        self.teleop.J3_Slider.valueChanged.connect(self.robo_J3_update)
        self.teleop.J4_Slider.valueChanged.connect(self.robo_J4_update)
        self.teleop.J5_Slider.valueChanged.connect(self.robo_J5_update)
        self.teleop.J6_Slider.valueChanged.connect(self.robo_J6_update)
        self.teleop.J7_Slider.valueChanged.connect(self.robo_J7_update)

        self.teleop.Head_Slider.valueChanged.connect(self.robo_head_update)
        self.teleop.Bellow_Slider.valueChanged.connect(self.robo_bellow_update)
        self.teleop.Gripper_Slider.valueChanged.connect(self.robo_gripper_update)

        self.teleop.Forward_Button.clicked.connect(self.robo_move_forward)
        self.teleop.Back_Button.clicked.connect(self.robo_move_back)
        # Add rotation buttons and commands


    # values from slider goes from int 0 -> 99
    def robo_J1_update(self):
        print("updating joint")
        print(self.teleop.J1_Slider.value())
        self.fetch.update_arm_joints(0,self.teleop.J1_Slider.value())

    def robo_J2_update(self):
        self.fetch.update_arm_joints(0,self.teleop.J2_Slider.value())

    def robo_J3_update(self):
        self.fetch.update_arm_joints(0,self.teleop.J3_Slider.value())

    def robo_J4_update(self):
        self.fetch.update_arm_joints(0,self.teleop.J4_Slider.value())

    def robo_J5_update(self):
        self.fetch.update_arm_joints(0,self.teleop.J5_Slider.value())

    def robo_J6_update(self):
        self.fetch.update_arm_joints(0,self.teleop.J6_Slider.value())

    def robo_J7_update(self):
        self.fetch.update_arm_joints(0,self.teleop.J7_Slider.value())

    def robo_head_update(self):
        pass

    def robo_bellow_update(self):
        pass

    def robo_gripper_update(self):
        pass

    
    def robo_move_forward(self):
        pass

    def robo_move_back(self):
        pass

def main(args=None):

    app = QApplication(sys.argv)
    window = GUI()
    app.exec_()


if __name__ == "__main__":
    main()
