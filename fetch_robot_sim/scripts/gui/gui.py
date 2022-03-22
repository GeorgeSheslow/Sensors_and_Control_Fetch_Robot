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
from sim.teleop_twist_keyboard import PublishThread
#from scripts.sim.fetch import Robot

class GUI(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        uic.loadUi("ui/control.ui", self)

        self.cameras = Cameras()
        self.teleop = TeleOp()

        self.fetch = Robot("Fetchy")

        self.robot_move_control = PublishThread(0)

        self.layout.addWidget(self.cameras)
        self.layout.addWidget(self.teleop)

        self.show()
        
        self.Arm_Reset_Button.clicked.connect(self.robo_arm_reset)
        self.teleop.J1_Slider.sliderReleased.connect(self.robo_J1_update)
        self.teleop.J2_Slider.sliderReleased.connect(self.robo_J2_update)
        self.teleop.J3_Slider.sliderReleased.connect(self.robo_J3_update)
        self.teleop.J4_Slider.sliderReleased.connect(self.robo_J4_update)
        self.teleop.J5_Slider.sliderReleased.connect(self.robo_J5_update)
        self.teleop.J6_Slider.sliderReleased.connect(self.robo_J6_update)
        self.teleop.J7_Slider.sliderReleased.connect(self.robo_J7_update)

        self.teleop.Head_Slider.sliderReleased.connect(self.robo_head_update)
        self.teleop.Bellow_Slider.sliderReleased.connect(self.robo_bellow_update)
        self.teleop.Gripper_Slider.sliderReleased.connect(self.robo_gripper_update)

        self.teleop.Forward_Button.clicked.connect(self.robo_move_forward)
        self.teleop.Back_Button.clicked.connect(self.robo_move_back)
        # Add rotation buttons and commands

    def robo_arm_reset(self):
        self.fetch.reset_arm()

    # values from slider goes from int 0 -> 99
    def robo_J1_update(self):
        print("updating joint")
        print(self.teleop.J1_Slider.value())
        self.fetch.update_arm_joints(0,self.teleop.J1_Slider.value())

    def robo_J2_update(self):
        self.fetch.update_arm_joints(1,self.teleop.J2_Slider.value())

    def robo_J3_update(self):
        self.fetch.update_arm_joints(2,self.teleop.J3_Slider.value())

    def robo_J4_update(self):
        self.fetch.update_arm_joints(3,self.teleop.J4_Slider.value())

    def robo_J5_update(self):
        self.fetch.update_arm_joints(4,self.teleop.J5_Slider.value())

    def robo_J6_update(self):
        self.fetch.update_arm_joints(5,self.teleop.J6_Slider.value())

    def robo_J7_update(self):
        self.fetch.update_arm_joints(6,self.teleop.J7_Slider.value())

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
