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

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty



class Robot():
    def __init__(self, robot_name):

        self.twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        #rospy.init_node(str(robot_name))

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

        self.jointMax = [math.pi/2, math.pi/2, math.pi/2, math.pi/2, math.pi,math.pi/2, math.pi/2]
        self.jointMin = [-math.pi/2, -math.pi/2, -math.pi/2, -math.pi/2, -math.pi, -math.pi/2, -math.pi/2]
        #
        # Head Joints Info
        #
        # Head Names
        self.head_joint_names = ["head_pan_joint", "head_tilt_joint"]
        #Head Inital Positions
        self.head_joint_positions = [0.0, 0.0]

        self.gripper_joint_names = ["l_gripper_finger_joint"]
        #
        # Gripper Info
        #
        # Gripper Inital Position
        self.gripper_position = 0

        #
        # Twist Info
        #

        # speed = rospy.get_param("~speed", 0.5)
        # turn = rospy.get_param("~turn", 1.0)
        # repeat = rospy.get_param("~repeat_rate", 0.0)
        # self.pub_thread = PublishThread(repeat)


        #self.pub_thread.update(self.x, self.y, self.z, self.th, speed, turn)

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
        self.torso_client = actionlib.SimpleActionClient("torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.torso_client.wait_for_server()
        rospy.loginfo("...connected.")

        self.torso_joint_names = ["torso_lift_joint"]
        self.torso_joint_positions = [0]

        self.reset_arm()

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
        percent = pos / 100.0

        m = 1 / (self.jointMax[arm_joint_num] - self.jointMin[arm_joint_num])
        b = -m * self.jointMin[arm_joint_num]
        value = (percent - b) / m

        self.joint_positions[arm_joint_num] = value
        self.execute_arm_update()


    def update_head(self, pos):
        # map 0 -> 99 to required range
        self.head_joint_positions[1] = pos/100
        self.execute_head_update()
    
    def update_torso(self, pos):
        self.torso_joint_positions[0] = (pos/100 * 0.4)
        self.execute_torso_update()

    def update_gripper(self, pos):
        # map 0 -> 99 to required range
        self.gripper_position = pos/100 * 0.05
        self.execute_gripper_update()



    def getArmPose(self):
        return self.joint_positions

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
        self.arm_client.wait_for_result(rospy.Duration(5.0))

    def execute_head_update(self):
        print("head upate")
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

        self.head_client.send_goal(head_goal)
        self.head_client.wait_for_result(rospy.Duration(5.0))


    def execute_torso_update(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.torso_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = self.torso_joint_positions
        trajectory.points[0].velocities = [0.0] * len(self.torso_joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(self.torso_joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(5.0)

        print(self.torso_joint_positions[0])
        torso_goal = FollowJointTrajectoryGoal()
        torso_goal.trajectory = trajectory
        torso_goal.goal_time_tolerance = rospy.Duration(0.0)

        self.torso_client.send_goal(torso_goal)
        self.torso_client.wait_for_result(rospy.Duration(5.0))


    def execute_gripper_update(self):
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.max_effort = 10.0
        gripper_goal.command.position = self.gripper_position
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result(rospy.Duration(5.0))

        

    def execute_twist(self,x,y):

        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        self.twist_pub.publish(twist)




class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

