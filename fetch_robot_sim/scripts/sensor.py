import cv2
import sys
import os
import roslib
import time
import rospy
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64MultiArray, Float32
from cv_bridge import CvBridge, CvBridgeError
#from msg import Location

# Create an object to read camera video 
class RGB_Detection:

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub =rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.cameraRGBCallBack)
        
    #    self.pointCentre = rospy.Publisher("point_center", Location,queue_size = 10)
        

    def cameraRGBCallBack(self,data):
        try:
            cap = self.bridge_object.imgmsg_to_cv2(data,"bgr8")
            #ret, frame = cap.read()
            # width = int(cap.get(3))
            # height = int(cap.get(4))
            # Convert BGR to HSV
            hsv = cv2.cvtColor(cap, cv2.COLOR_BGR2HSV)
            # define blue colour range
            light_blue = np.array([94,80,2], np.uint8) 
            dark_blue = np.array([130,255,255], np.uint8)

            # Threshold the HSV image to get only blue colours
            blue_mask = cv2.inRange(hsv, light_blue, dark_blue)

            # define red colour range
            light_red = np.array([136,87, 11], np.uint8)
            dark_red = np.array([180,255,255], np.uint8)
            
            # Threshold the HSV image to get only red colours
            red_mask = cv2.inRange(hsv, light_red, dark_red)
            
            # define green colour range
            light_green = np.array([25, 52, 72], np.uint8) 
            dark_green = np.array([102, 255, 255], np.uint8) 
            
            # Threshold the HSV image to get only green colours
            green_mask = cv2.inRange(cap, light_green, dark_green) 


            kernal = np.ones((5,5), "uint8")
            blue_mask= cv2.dilate(blue_mask, kernal)
            red_mask= cv2.dilate(red_mask, kernal)
            green_mask= cv2.dilate(green_mask, kernal)
            
            # Bitwise-AND mask and original image
            output_blue = cv2.bitwise_and(cap,cap, mask= blue_mask)
            output_red = cv2.bitwise_and(cap,cap, mask= red_mask)
            output_green = cv2.bitwise_and(cap,cap, mask= green_mask)
            
            # Creating contour to track red colour 
            contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
            
            #array of vector with x,y location
            
            
            for pic, contour in enumerate(contours): 
                area = cv2.contourArea(contour) 
                if(area > 300): 
                    x, y, w, h = cv2.boundingRect(contour) 
                    cap = cv2.rectangle(cap, (x, y),  (x + w, y + h), (0, 0, 255), 2) 
                    
                    midPoints = [x+(w/2),y+(h/2)]
    #                self.pointCentre.publish(midPoints)
                    
                    cv2.putText(cap, "Red Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))	 

            # Creating contour to track green colour 
            contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
            
            for pic, contour in enumerate(contours): 
                area = cv2.contourArea(contour) 
                if(area > 300): 
                    x, y, w, h = cv2.boundingRect(contour) 
                    cap = cv2.rectangle(cap, (x, y), (x + w, y + h),  (0, 255, 0), 2) 
                    
                    midPoints = [x+(w/2),y+(h/2)]
        #            self.pointCentre.publish(midPoints)
                
                    cv2.putText(cap, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0)) 

            # Creating contour to track blue colour 
            contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE,  cv2.CHAIN_APPROX_SIMPLE) 
            
            for pic, contour in enumerate(contours): 
                area = cv2.contourArea(contour) 
                if(area > 300): 
                    x, y, w, h = cv2.boundingRect(contour) 
                    cap = cv2.rectangle(cap, (x, y),  (x + w, y + h),  (255, 0, 0), 2) 
                    
                    midPoints = [x+(w/2),y+(h/2)]
        #            self.pointCentre.publish(midPoints)
                
                    cv2.putText(cap, "Blue Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0)) 
                
            # Program Termination 
            cv2.imshow("Multiple Color Detection", cap)                         

            # Press Q on keyboard to stop recording
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cap)
        cv2.waitKey(3)
        
        

# release video capture
# and video write objects

#video_output.release()

# Closes all the frames

#print("The video was successfully saved")   

if __name__ == '__main__':
    rospy.init_node("Paul Obj Detect")
    cv_thing = RGB_Detection()

    while not rospy.is_shutdown():
        rospy.spin()