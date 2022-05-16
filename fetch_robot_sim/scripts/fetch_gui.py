#!/usr/bin/env python3

import os
import sys

from python_qt_binding import loadUi
import rospy
import rospkg

import time
from PyQt5.QtWidgets import (
    QApplication,
    QDesktopWidget,
    QMainWindow,
    QWidget,
    QFileDialog,
)
from PyQt5.QtCore import QTimer

from fetch_robot_sim.msg import RGB_Image_Info
from std_msgs.msg import String
from gui.teleop import TeleOp
from gui.camera_viz import Cameras
from gui.fetch import Robot


class GUI(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        rospy.init_node("gui", anonymous=True)
        rp = rospkg.RosPack()
        package_path = rp.get_path('fetch_robot_sim')
        ui_file = os.path.join(package_path, "ui", "control.ui")
        loadUi(ui_file, self)

        # Subs
        self.detect_object_request = rospy.Publisher("/object_detect_request", String, queue_size=10)
        self.detect_object_feedback =rospy.Subscriber("/object_info", RGB_Image_Info, self.obj_info_callback)
        

        self.fetch = Robot("Fetchy")

        self.cameras = Cameras()
        self.teleop = TeleOp(self, self.fetch)

        self.layout.addWidget(self.cameras)
        self.layout.addWidget(self.teleop)

        self.Grasp_Prep_Button.clicked.connect(self.grasp_prep)
        self.Arm_Reset_Button.clicked.connect(self.reset_robo_arm)
        # self.Find_Obj_Button.clicked.connect(self.obj_finder)
        self.Kinematics_Button.clicked.connect(self.kinematics_finder)
        self.Grasp_Button.clicked.connect(self.grasp_obj)

        self.show()

        self.timer = QTimer()
        self.timer.setInterval(1000) # 1 second
        self.timer.timeout.connect(self.check_obj_request)
        self.timer.start()

    def check_obj_request(self):
        self.detect_object_request.publish(self.comboBox.currentText())


    def grasp_prep(self):
        self.fetch.reset_arm()
        for i in range(12):
            self.fetch.execute_twist(0.1, 0)
            time.sleep(0.3)

        # self.fetch.update_torso(80)
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
        self.obj_detect_label.setText("Obj Detected:  " + str(data.objectName))
        self.obj_detect_info_label.setText(
            str("Image Frame (x,y): ") +
            str("{:.1f}".format(data.x))
            + ", "
            + str("{:.1f}".format(data.y))
        )


def main(args=None):

    app = QApplication(sys.argv)
    window = GUI()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
