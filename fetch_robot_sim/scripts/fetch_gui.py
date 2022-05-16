#!/usr/bin/env python3

import os
import sys
from python_qt_binding import loadUi
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
from fetch_messages.msg import Obj_Detect
from gui.teleop import TeleOp
from gui.camera_viz import Cameras
from gui.fetch import Robot


import rospkg

class GUI(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        rospy.init_node("gui", anonymous=True)
        rp = rospkg.RosPack()
        package_path = rp.get_path('fetch_robot_sim')
        ui_file = os.path.join(package_path, "ui", "control.ui")
        loadUi(ui_file, self)

        # Subs
        self.obj_info = rospy.Subscriber(
            "/obj_info", Obj_Detect, self.obj_info_callback
        )
        self.fetch = Robot("Fetchy")

        self.cameras = Cameras()
        self.teleop = TeleOp(self, self.fetch)

        self.layout.addWidget(self.cameras)
        self.layout.addWidget(self.teleop)

        self.Grasp_Prep_Button.clicked.connect(self.grasp_prep)
        self.Arm_Reset_Button.clicked.connect(self.reset_robo_arm)
        self.Find_Obj_Button.clicked.connect(self.obj_finder)
        self.Kinematics_Button.clicked.connect(self.kinematics_finder)
        self.Grasp_Button.clicked.connect(self.grasp_obj)

        self.show()

    def grasp_prep(self):
        self.fetch.reset_arm()
        for i in range(12):
            self.fetch.execute_twist(0.1, 0)
            time.sleep(0.3)

        self.fetch.update_torso(80)
        self.fetch.update_head(50)

    def reset_robo_arm(self):
        self.fetch.reset_arm()

    def obj_finder(self):
        pass

    def kinematics_finder(self):
        pass

    def grasp_obj(self):
        x = self.grasp_x.text()
        y = self.grasp_y.text()
        z = self.grasp_z.text()
        print("sending: " + x + ", " + y + ", " + z)
        self.fetch.grasp(x,y,z)

    def obj_info_callback(self, data):
        self.obj_detect_label.setText("Obj Detected:  " + str(data.obj_name))
        self.obj_camera_label.setText(
            str("{:.1f}".format(data.x))
            + ", "
            + str("{:.1f}".format(data.y))
            + ", "
            + str("{:.2f}".format(data.z))
        )


def main(args=None):

    app = QApplication(sys.argv)
    window = GUI()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
