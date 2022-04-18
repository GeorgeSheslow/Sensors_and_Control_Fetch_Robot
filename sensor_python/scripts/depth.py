import cv2
import sys
import roslib
import rospy
import numpy as np

from std_msgs.msg import String, Float64MultiArray, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from geometry_msgs.msg import Twist
from msg import Object_Info
from msg import Location


class Depth_Detection:

    def __init__(self):
        self.bridge_object = CvBridge()
        
        
        self.depth_sub =rospy.Subscriber("/head_camera/depth_registered/image_raw",Image,self.cameraDepthCallBack)
        self.depth_sub =rospy.Subscriber("Point_Center",Location,self.cameraDepthCallBack)
        
        self.detect_object = rospy.Publisher("object_info", Object_Info, queue_size=10)
        self.depth_dist = rospy.Publisher("distance",float, queue_size=10)

    def cameraDepthCallBack(self,data):
        try:
            cv_cap = self.bridge_object.imgmsg_to_cv2(data,"passthrough") 
            
            x = float(self.x)
            y = float(self.y)
            self.z = cv_cap[x,y] 
            
            object = Object_Info()
            object.x = float(self.x)
            object.y = float(self.y)
            object.z = float(self.z)
            object.obj_name = "object"
            self.detect_object.publish(object)       
            
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Depth Image window", cv_cap)
        cv2.waitKey(3)
        

        



if __name__ == '__main__':
    rospy.init_node("Depth_Detect")
    cv_thing = Depth_Detection()

    while not rospy.is_shutdown():
        rospy.spin()
    