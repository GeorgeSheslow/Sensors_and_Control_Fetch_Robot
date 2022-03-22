from ast import Pass
import sys

import rospy
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import math


class Robot():
    def __init__(self, robot_name):
        rospy.init_node(str(robot_name))

        #
        # Arm Joints Info
        #
        # Joint Names
        self.arm_joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
              "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        # Joint Inital Positions
        self.joint_positions = [0.0, -0.62, 0, 0, 0.0, 0.62, 0.0]
        # Saved Joint Positions
        self.joint_zero_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_pre_grasp_position = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]

        #
        # Head Joints Info
        #
        # Head Names
        self.head_joint_names = ["head_pan_joint", "head_tilt_joint"]
        #Head Inital Positions
        self.head_joint_positions = [0.0, 0.0]

        #
        # Gripper Infor
        #
        # Gripper Inital Position
        self.gripper_position = 0


        rospy.loginfo("Waiting for head_controller...")
        self.head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.head_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for arm_controller...")
        self.arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("...connected.")


        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for torso_controller...")
        self.arm_client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("...connected.")

    def setup_torso(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.head_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.head_joint_positions
        trajectory.points[0].velocities = [0.0] * len(self.head_joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(self.head_joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(5.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        rospy.loginfo("Setting positions...")
        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result(rospy.Duration(1.0))        

    def setup_head(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.head_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.head_joint_positions
        trajectory.points[0].velocities = [0.0] * len(self.head_joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(self.head_joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(5.0)

        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.0)

        rospy.loginfo("Setting positions...")
        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result(rospy.Duration(1.0))


    def setup_arm(self):
        pass

    def reset_arm(self):
        self.joint_positions = self.joint_pre_grasp_position
        self.execute_arm_update()
    
    def update_arm_joints(self, arm_joint_num, pos):
        # map slider pos from 0 -> 9 to joint min and max
        pos = pos / 100
        if (arm_joint_num == 0):
            value = pos * math.pi /2
        if (arm_joint_num == 1):
            value = pos * math.pi /2
        if (arm_joint_num == 2):
            value = pos * math.pi /2
        if (arm_joint_num == 3):
            value = pos * math.pi /2
        if (arm_joint_num == 4):
            value = pos * math.pi /2
        if (arm_joint_num == 5):
            value = pos * math.pi /2
        if (arm_joint_num == 6):
            value = pos * math.pi /2
        if (arm_joint_num == 7):
            value = pos * math.pi /2

        self.joint_positions[arm_joint_num] = value
        self.execute_arm_update()


    def update_head(self, pos):
        # map 0 -> 99 to required range
        pass
    
    def update_bellows(self, pos):
        # map 0 -> 99 to required range
        pass

    def update_gripper(self, pos):
        # map 0 -> 99 to required range
        pass




    def execute_arm_update(self):

        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.joint_positions
        trajectory.points[0].velocities =  [0.0] * len(self.joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(self.joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(1.0)

        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory = trajectory
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)


        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result(rospy.Duration(1.0))

    def execute_head_update(self):
        pass

    def execute_bellow_update(self):
        pass

    def execute_gripper_update(self):
        pass



    def get_remapped_arm_joints(self):
        pass
        