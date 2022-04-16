import cv2
import sys
import roslib
import rospy
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
#from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


cap = rospy.Subscriber("/head_camera/depth_downsample/image_raw",Image,1)






#dep main():
    