import os
import sys

import rospy
import actionlib

from sensor_msgs.msg import JointState

from PyQt5 import uic, QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import (
    QApplication,
    QDesktopWidget,
    QMainWindow,
    QWidget,
    QFileDialog,
)

from PyQt5 import uic

class TeleOp(QWidget):

    def __init__(self,gui, fetch, parent=None) -> None:
        super().__init__(parent)
        self.gui = gui
        uic.loadUi("ui/teleop.ui", self)
        self.show()

        self.fetch = fetch
        #update sliders

        #self.Arm_Reset_Button.clicked.connect(self.robo_arm_reset)
        self.J1_Slider.sliderReleased.connect(self.robo_J1_update)
        self.J2_Slider.sliderReleased.connect(self.robo_J2_update)
        self.J3_Slider.sliderReleased.connect(self.robo_J3_update)
        self.J4_Slider.sliderReleased.connect(self.robo_J4_update)
        self.J5_Slider.sliderReleased.connect(self.robo_J5_update)
        self.J6_Slider.sliderReleased.connect(self.robo_J6_update)
        self.J7_Slider.sliderReleased.connect(self.robo_J7_update)

        self.Head_Slider.sliderReleased.connect(self.robo_head_update)
        self.Bellow_Slider.sliderReleased.connect(self.robo_torso_update)
        self.Gripper_Slider.sliderReleased.connect(self.robo_gripper_update)

        #Update Sliders
        
        self.J1_Slider.setValue(self.jointMath(0,self.gui.currentJointStates[6]))
        self.J2_Slider.setValue(self.jointMath(1,self.gui.currentJointStates[7]))
        self.J3_Slider.setValue(self.jointMath(2,self.gui.currentJointStates[8]))
        self.J4_Slider.setValue(self.jointMath(3,self.gui.currentJointStates[9]))
        self.J5_Slider.setValue(self.jointMath(4,self.gui.currentJointStates[10]))
        self.J6_Slider.setValue(self.jointMath(5,self.gui.currentJointStates[11]))
        self.J7_Slider.setValue(self.jointMath(6,self.gui.currentJointStates[12]))

        self.Head_Slider.setValue(self.gui.currentJointStates[5])


        self.Bellow_Slider.setValue(100* (0.4/self.gui.currentJointStates[2]))
        
        # Set gripper to open
        self.fetch.update_gripper(100)
        self.Gripper_Slider.setValue(100)


        self.Forward_Button.clicked.connect(self.moveForward)
        self.Back_Button.clicked.connect(self.moveBack)
        self.CCW_Button.clicked.connect(self.moveCCW)
        self.CW_Button.clicked.connect(self.moveCW)
    
    def jointMath(self, jointNum, val):
        m = 1 / (self.fetch.jointMax[jointNum] - self.fetch.jointMin[jointNum])
        b = -m * self.fetch.jointMin[jointNum]
        x = m * val + b
        percent = x * 100.0

        return percent


    def robo_arm_reset(self):
        pass

    # values from slider goes from int 0 -> 99
    def robo_J1_update(self):
        print(self.J1_Slider.value())
        self.fetch.update_arm_joints(0,self.J1_Slider.value())

    def robo_J2_update(self):
        self.fetch.update_arm_joints(1,self.J2_Slider.value())

    def robo_J3_update(self):
        self.fetch.update_arm_joints(2,self.J3_Slider.value())

    def robo_J4_update(self):
        self.fetch.update_arm_joints(3,self.J4_Slider.value())

    def robo_J5_update(self):
        self.fetch.update_arm_joints(4,self.J5_Slider.value())

    def robo_J6_update(self):
        self.fetch.update_arm_joints(5,self.J6_Slider.value())

    def robo_J7_update(self):
        self.fetch.update_arm_joints(6,self.J7_Slider.value())

    def robo_head_update(self):
        self.fetch.update_head(self.Head_Slider.value())

    def robo_torso_update(self):
        self.fetch.update_torso(self.Bellow_Slider.value())

    def robo_gripper_update(self):
        self.fetch.update_gripper(self.Gripper_Slider.value())

    
    def moveForward(self):
        print("forward")
        self.fetch.execute_twist(1,0)

    def moveBack(self):
        print("Back")
        self.fetch.execute_twist(-1,0)

    def moveCCW(self):
        pass

    def moveCW(self):
        pass