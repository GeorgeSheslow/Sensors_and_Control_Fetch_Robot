import cv2
import sys
import roslib
import rospy
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
#from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
# Create an object to read camera video 
# class RGB_Detection(object):
  
#   def __init__(self):
#     self.bridge_object = CvBridge()
#     self.image_sub =rospy.Subscriber("/camera/rgb/image_raw",Image,cameraCallBack)
#   def cameraCallBack(self,data):
#     try:
#       cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8") 
#     except CvBridgeError as 0:
#       print(e)
    
#     cv2.imshow("Image window", cv_image)
#     cv2.waitKey(1)
    
# cap = cv2.VideoCapture(0)
cap = rospy.Subscriber("/head_camera/rgb/image_raw",Image,1)
#video_cod = cv2.VideoWriter_fourcc(*'XVID')
#video_output= cv2.VideoWriter('captured_video.avi', video_cod,10,(640,480))
def main ():
  rospy.spin()
  #while(True):
  ret, frame = cap.read()
  width = int(cap.get(3))
  height = int(cap.get(4))
   # Convert BGR to HSV
  hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
  green_mask = cv2.inRange(frame, light_green, dark_green) 


  kernal = np.ones((5,5), "uint8")
  blue_mask= cv2.dilate(blue_mask, kernal)
  red_mask= cv2.dilate(red_mask, kernal)
  green_mask= cv2.dilate(green_mask, kernal)
      
      # Bitwise-AND mask and original image
  output_blue = cv2.bitwise_and(frame,frame, mask= blue_mask)
  output_red = cv2.bitwise_and(frame,frame, mask= red_mask)
  output_green = cv2.bitwise_and(frame,frame, mask= green_mask)
      
      # Creating contour to track red color 
  contours, hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
      
  for pic, contour in enumerate(contours): 
    area = cv2.contourArea(contour) 
    if(area > 300): 
      x, y, w, h = cv2.boundingRect(contour) 
      frame = cv2.rectangle(frame, (x, y),  (x + w, y + h), (0, 0, 255), 2) 
          
      cv2.putText(frame, "Red Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255))	 

      # Creating contour to track green color 
  contours, hierarchy = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
      
  for pic, contour in enumerate(contours): 
    area = cv2.contourArea(contour) 
    if(area > 300): 
      x, y, w, h = cv2.boundingRect(contour) 
      frame = cv2.rectangle(frame, (x, y), (x + w, y + h),  (0, 255, 0), 2) 
          
      cv2.putText(frame, "Green Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0)) 

      # Creating contour to track blue color 
  contours, hierarchy = cv2.findContours(blue_mask, cv2.RETR_TREE,  cv2.CHAIN_APPROX_SIMPLE) 
  for pic, contour in enumerate(contours): 
    area = cv2.contourArea(contour) 
    if(area > 300): 
      x, y, w, h = cv2.boundingRect(contour) 
      frame = cv2.rectangle(frame, (x, y),  (x + w, y + h),  (255, 0, 0), 2) 
          
      cv2.putText(frame, "Blue Colour", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0)) 
          
      # Program Termination 
  cv2.imshow("Multiple Color Detection in Real-TIme", frame) 
      
      
      
      # Write the frame into the file 'captured_video.avi'
      #video_output.write(output)

      # Display the frame, saved in the file   
      #cv2.imshow('output',output)

      # Press Q on keyboard to stop recording
  if cv2.waitKey(1) & 0xFF == ord('Q'):
    

# release video capture
# and video write objects
    cap.release()
#video_output.release()

# Closes all the frames
    cv2.destroyAllWindows() 

#print("The video was successfully saved")   