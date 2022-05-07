import posix
from tkinter import Y
import cv2
import sys
import os

from matplotlib.backend_bases import LocationEvent
import roslib
import time
import rospy
import numpy as np
import math

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped #, PoseWithConvariance, TwistWithConvariance
from fetch_robot_sim.msg import Object_Info
from fetch_robot_sim.msg import Location
from fetch_robot_sim.msg import Location_3D

# Create an object to read camera video
class RGBD_Detection:
    def __init__(self):
        self.bridge_object = CvBridge()
        
        #Subscribers
        self.image_sub = rospy.Subscriber(
            "/head_camera/rgb/image_raw", Image, self.cameraRGBCallBack
        )        
        self.depth_sub = rospy.Subscriber("/head_camera/depth_registered/image_raw",Image,self.cameraDepthCallBack)
        self.odom_sub = rospy.Subscriber("/odom",PoseStamped,self.calculationsCallBack)
        #Publishers
        self.detect_object = rospy.Publisher("/object_info", Object_Info, queue_size=10)
        self.location_3D = rospy.Publisher("/distance",Location_3D, queue_size=10)
        self.bounding_image = rospy.Publisher("/bounding_image",Image, queue_size=10)
        self.bounding_image1 = rospy.Publisher("/bounding_image1",Image, queue_size=10)
        
        #Globals for the class
        self.midPoints = Location(0, 0)
        self.locationPos = Location_3D()
        self.posLocalGlobal = Location_3D()
        self.x = 0
        self.y = 0
        self.z = 0
        self.sync = 0 #when sync = 0 it runs RGB, sync = 1 Depth
        
    def cameraRGBCallBack(self, data):
        cap = self.bridge_object.imgmsg_to_cv2(data, "bgr8")

        if self.sync == 0:
                        
            # Convert BGR to HSV
            hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
            # define blue colour range
            light_blue = np.array([100, 150, 0], np.uint8)
            dark_blue = np.array([140, 255, 255], np.uint8)

            # Threshold the HSV image to get only blue colours
            blue_mask = cv2.inRange(hsv, light_blue, dark_blue)

            # define red colour range
            light_red = np.array([0, 100, 20], np.uint8)
            dark_red = np.array([10, 255, 255], np.uint8)

            # Threshold the HSV image to get only red colours
            red_mask = cv2.inRange(hsv, light_red, dark_red)

            # define green colour range
            light_green = np.array([0, 100, 0], np.uint8)
            dark_green = np.array([105, 255, 105], np.uint8)

            # Threshold the HSV image to get only green colours
            green_mask = cv2.inRange(cap, light_green, dark_green)

            kernal = np.ones((5, 5), "uint8")
            blue_mask = cv2.dilate(blue_mask, kernal)
            red_mask = cv2.dilate(red_mask, kernal)
            green_mask = cv2.dilate(green_mask, kernal)

            # Bitwise-AND mask and original image
            output_blue = cv2.bitwise_and(cap, cap, mask=blue_mask)
            output_red = cv2.bitwise_and(cap, cap, mask=red_mask)
            output_green = cv2.bitwise_and(cap, cap, mask=green_mask)

            # Creating contour to track red colour
            contours, hierarchy = cv2.findContours(
                red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            #Drawing the rectangle
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (0, 0, 255), 2)


                    # Preparing center point
                    self.midPoints.x = x + (w / 2)
                    self.midPoints.y = y + (h / 2)
                    
                    #Text on the rectangle
                    cv2.putText(
                        cap,
                        "Red Cylinder",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                    )

            # Creating contour to track green colour
            contours, hierarchy = cv2.findContours(
                green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            #Drawing the rectangle
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    # Preparing center point
                    self.midPoints.x = x + (w / 2)
                    self.midPoints.y = y + (h / 2)
                    #Text on the rectangle
                    cv2.putText(
                        cap,
                        "Green Large Cube",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                    )

            # Creating contour to track blue colour
            contours, hierarchy = cv2.findContours(
                blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )
            #Drawing the rectangle
            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (255, 0, 0), 2)

                    # Preparing center point
                    midPoints_B = Location()
                    
                    midPoints_B.x = x + (w / 2)
                    midPoints_B.y = y + (h / 2)
                    
                    #Text on the rectangle
                    cv2.putText(
                        cap,
                        "Blue Small Cube",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 0, 0),
                    )
            self.sync = 1
        self.bounding_image.publish(self.bridge_object.cv2_to_imgmsg(cap, "bgr8"))
            
            
    def cameraDepthCallBack(self,data):

        if self.sync == 1:
            cv_cap2 = self.bridge_object.imgmsg_to_cv2(data,"passthrough")
            
            #to ensure that there is a center point
            if self.midPoints.x > 0:
                if self.midPoints.y > 0:
                    #midpoints from RGB
                    x = int(self.midPoints.x)
                    y = int(self.midPoints.y)
                    
                    #depth when given x,y from camer's point of view
                    self.z = cv_cap2[x,y]

                    #publishing result for IKsolver
                    
                    self.locationPos.x = x
                    self.locationPos.y = y
                    self.locationPos.z = self.z
                    #self.location_3D.publish(locationPos)
            self.sync = 2

    def calculationsCallBack(self,data):
        if self.sync == 2:
            print(self.sync)
            robotPos = Location_3D()
            robotPos.x = data.pose.position.x
            robotPos.y = data.pose.position.y
            robotPos.z = data.pose.position.z  #position of the starting arm location?
            rotation = data.pose.rotation    #in Rad
            
            print(robotPos)
            print(rotation)
            
            self.posLocalGlobal.x = self.locationPos.z * math.acos(rotation) + robotPos.x #z shows the depth
            self.posLocalGlobal.y = self.locationPos.x * math.asin(rotation) + robotPos.y #x is width
            self.posLocalGlobal.z = self.locationPos.y + robotPos.z #y is the hight
            self.location_3D.publish(self.posLocalGlobal)
            #self.sync = 0
        
        

if __name__ == "__main__":
    rospy.init_node("Paul_Python_Sensor")
    cv_thing = RGBD_Detection()

    while not rospy.is_shutdown():
        rospy.spin()
