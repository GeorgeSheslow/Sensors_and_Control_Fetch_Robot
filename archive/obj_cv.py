#!/usr/bin/env python3
import sys
import os
import time
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32

import cv2
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

from fetch_robot_sim.msg import Obj_Detect

# Input RGB and Depth
# Output Bounding Image and coordinates


class CubeDetect:
    def __init__(self):
        rospy.init_node("CubeDetect")

        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 1
        params.maxThreshold = 100

        # Filter by Area.
        params.filterByArea = True
        params.minArea = 100

        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1

        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.05

        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        # Create a detector with the parameters

        self.detector = cv2.SimpleBlobDetector_create(params)

        # Subs
        self.rgb_sub = rospy.Subscriber(
            "/head_camera/rgb/image_raw", Image, self.rgb_callback
        )
        self.depth_sub = rospy.Subscriber(
            "/head_camera/depth_registered/image_raw", Image, self.depth_callback
        )

        # Pubs
        self.img_pub = rospy.Publisher("bounding_image", Image, queue_size=10)
        self.obj_info = rospy.Publisher("obj_info", Obj_Detect, queue_size=10)

        # Im varaibles
        self.rgb = Image
        self.depth = Image

        self.bridge = CvBridge()

        # In pixels
        self.x = 0
        self.y = 0
        self.z = 0

    def image_processing(self):
        pass

    def rgb_callback(self, data):
        # self.rgb = data
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        im = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect blobs.
        keypoints = self.detector.detect(im)

        # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
        im_with_keypoints = cv2.drawKeypoints(
            cv_image,
            keypoints,
            np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )

        im_point = cv2.KeyPoint_convert(keypoints)
        pts = [p.pt for p in keypoints]
        if len(pts) > 0:
            pt = pts[0]
            self.x = pt[0]
            self.y = pt[1]
            # print(pt[0])

        # Show keypoints
        ros_image = self.bridge.cv2_to_imgmsg(im_with_keypoints, "bgr8")
        self.img_pub.publish(ros_image)

        # sift = cv2.SIFT_create()
        # kp = sift.detect(im,None)
        # print(kp)
        # img=cv2.drawKeypoints(im,kp,cv_image)

    def depth_callback(self, data):

        dep = self.bridge.imgmsg_to_cv2(data, "passthrough")

        x = int(self.x)
        y = int(self.y)
        self.z = dep[x, y]

        message = Obj_Detect()
        message.x = float(self.x)
        message.y = float(self.y)
        message.z = float(self.z)
        message.obj_name = "Small_Cube"
        self.obj_info.publish(message)
        # print(self.x,self.y,self.z)
        # cv2.imshow("Keypoints",dep)
        # cv2.waitKey(3)


if __name__ == "__main__":
    cupDetector = CubeDetect()

    while not rospy.is_shutdown():
        rospy.spin()
