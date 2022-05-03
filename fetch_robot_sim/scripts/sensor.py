from tkinter import Y
import cv2
import sys
import os

from matplotlib.backend_bases import LocationEvent
import roslib
import time
import rospy
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError

from fetch_robot_sim.msg import Object_Info
from fetch_robot_sim.msg import Location
from fetch_robot_sim.msg import Location_3D

# Create an object to read camera video
class RGBD_Detection:
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/head_camera/rgb/image_raw", Image, self.cameraRGBCallBack
        )

        self.pointCentre = rospy.Publisher("point_center", Location, queue_size=10)
        
        
        self.depth_sub =rospy.Subscriber("/head_camera/depth_registered/image_raw",Image,self.cameraDepthCallBack)
        #self.location_sub =rospy.Subscriber("point_center",Location,self.cameraDepthCallBack)
        
        self.detect_object = rospy.Publisher("object_info", Object_Info, queue_size=10)
        self.location_3D = rospy.Publisher("distance",Location_3D, queue_size=10)
        self.bounding_image = rospy.Publisher("/bounding_image",Image, queue_size=10)
        self.bounding_image1 = rospy.Publisher("/bounding_image1",Image, queue_size=10)
        
        self.midPoints = Location(0, 0)
        self.x = 0
        self.y = 0
        self.z = 0
        self.sync = 0
        
    def cameraRGBCallBack(self, data):

        if self.sync == 0:
            print("RGB")
            cap = self.bridge_object.imgmsg_to_cv2(data, "bgr8")
            # Convert BGR to HSV
            hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
            # define blue colour range
            light_blue = np.array([86, 31, 4], np.uint8)
            dark_blue = np.array([255, 0, 0], np.uint8)

            # Threshold the HSV image to get only blue colours
            blue_mask = cv2.inRange(hsv, light_blue, dark_blue)

            # define red colour range
            light_red = np.array([94, 80, 2], np.uint8)
            dark_red = np.array([0, 0, 255], np.uint8)

            # Threshold the HSV image to get only red colours
            red_mask = cv2.inRange(hsv, light_red, dark_red)

            # define green colour range
            light_green = np.array([50, 90, 50], np.uint8)
            dark_green = np.array([0, 255, 0], np.uint8)

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

            # array of vector with x,y location

            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (0, 0, 255), 2)

                    # Prep ROS message and publish
                    #midPoints_R = Location()
                    
                    self.midPoints.x = x + (w / 2)
                    self.midPoints.y = y + (h / 2)
                    #self.pointCentre.publish(midPoints)

                    cv2.putText(
                        cap,
                        "Red",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                    )

            # Creating contour to track green colour
            contours, hierarchy = cv2.findContours(
                green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (0, 255, 0), 2)

                    # Prep ROS message and publish
                    #midPoints_G = Location()
                    
                    self.midPoints.x = x + (w / 2)
                    self.midPoints.y = y + (h / 2)
                    #self.pointCentre.publish(midPoints)

                    cv2.putText(
                        cap,
                        "Green",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                    )

            # Creating contour to track blue colour
            contours, hierarchy = cv2.findContours(
                blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
            )

            for pic, contour in enumerate(contours):
                area = cv2.contourArea(contour)
                if area > 300:
                    x, y, w, h = cv2.boundingRect(contour)
                    cap = cv2.rectangle(cap, (x, y), (x + w, y + h), (255, 0, 0), 2)

                    # Prep ROS message and publish
                    midPoints_B = Location()
                    
                    midPoints_B.x = x + (w / 2)
                    midPoints_B.y = y + (h / 2)
                    #self.pointCentre.publish(midPoints)

                    cv2.putText(
                        cap,
                        "Blue",
                        (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (255, 0, 0),
                    )

            # Program Termination
            
            
            self.bounding_image.publish(self.bridge_object.cv2_to_imgmsg(cap, "bgr8"))
            self.sync = 1
            
            


    def cameraDepthCallBack(self,data):

        if self.sync == 1:
            print("depth")
            cv_cap2 = self.bridge_object.imgmsg_to_cv2(data,"passthrough")
            print(self.midPoints)
            if self.midPoints.x > 0:
                if self.midPoints.y > 0:
                    x = int(self.midPoints.x)
                    y = int(self.midPoints.y)
                    self.z = cv_cap2[x,y]
                    print(self.z)

                    locationPos = Location()
                    locationPos.x = x
                    locationPos.y = y
                    locationPos.z = self.z
                    self.location_3D.publish(locationPos)
            self.sync = 0


if __name__ == "__main__":
    rospy.init_node("Paul_Python_Sensor")
    cv_thing = RGBD_Detection()

    while not rospy.is_shutdown():
        rospy.spin()
