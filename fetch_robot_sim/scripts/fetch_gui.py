#!/usr/bin/env python3

import os
import sys
from urllib import response

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

from gui.teleop import TeleOp
from gui.camera_viz import Cameras
from gui.fetch import Robot

from fetch_robot_sim.msg import RGB_Image_Info
from fetch_robot_sim.msg import Location_3D
from std_msgs.msg import String
from iksolver.srv import calcTraj
from moveit_msgs.msg import RobotTrajectory
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
        self.distance_to_object =rospy.Subscriber("/distance", Location_3D, self.obj_distance)
        self.obj_distance = Location_3D()

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
    def obj_distance(self,data):
        self.obj_distance = data
        self.obj_detect_info_2_label.setText(str("Robot Frame(x,y,z): ") + str(data.x) + str(", ")+ str(data.y) + str(", ")+ str(data.z) + str(", "))

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
        print("IK service")
        rospy.wait_for_service('calc_traj')
        try:
            trajectory = rospy.ServiceProxy('calc_traj', calcTraj)
            self.obj_distance.z = 0.5
            self.obj_distance.x = 0.7
            self.obj_distance.y = 0
            response = trajectory(self.obj_distance)
            
            print("success")
            # joints = RobotTrajectory()
            # joints = response.joint_trajectory.points
            print(str(response.traj))
            print(str(response.traj.joint_trajectory.points[-1].positions))
            self.gripper_pose_info.setText(str(response.traj.joint_trajectory.points[-1].positions))
            self.fetch.updateJoints(response.traj.joint_trajectory.points[-1].positions)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def grasp_obj(self):
        self.fetch.execute_arm_update()

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
