import os
import sys
import rospy
from PyQt5 import uic, QtCore, QtWidgets, QtGui

import sys
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
import cv2
from sensor_msgs.msg import Image # Image is the message type
from PyQt5.QtWidgets import (
    QApplication,
    QDesktopWidget,
    QMainWindow,
    QWidget,
    QFileDialog,
)

from PyQt5 import uic

from cv_bridge import CvBridge, CvBridgeError
import cv2
import roslib
class Cameras(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        uic.loadUi("ui/cameras.ui", self)
        self.show()

        self.thread = QThread()
        self.worker = image_converter()
        self.worker.moveToThread(self.thread)
        rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.worker.run)
        self.thread.start()
        self.worker.ImageUpdate.connect(self.ImageUpdateSlot)

    def ImageUpdateSlot(self, Image):
        self.FeedLabel.setPixmap(QPixmap.fromImage(Image))


class image_converter(QObject):
    ImageUpdate = pyqtSignal(QImage)
    bridge = CvBridge()

    def run(self,data):
        self.ThreadActive = True
        
        while self.ThreadActive:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            FlippedImage = cv2.flip(image, 1)
            ConvertToQtFormat = QImage(FlippedImage.data, FlippedImage.shape[1], FlippedImage.shape[0], QImage.Format_RGB888)
            Pic = ConvertToQtFormat.scaled(640, 480, Qt.KeepAspectRatio)
            self.ImageUpdate.emit(Pic)
            self.ThreadActive = False



#############Testing Code Below, not used############################
class MainWindow(QWidget):

    def __init__(self):
        super(MainWindow, self).__init__()
        rospy.init_node("gui_cameras")

        self.VBL = QVBoxLayout()

        self.FeedLabel = QLabel()
        self.VBL.addWidget(self.FeedLabel)

        self.CancelBTN = QPushButton("Cancel")
        self.CancelBTN.clicked.connect(self.CancelFeed)
        self.VBL.addWidget(self.CancelBTN)

        self.thread = QThread()
        self.worker = image_converter()
        rospy.Subscriber("/head_camera/rgb/image_raw",Image,self.worker.run)

        self.worker.moveToThread(self.thread)
        self.thread.start()

        self.worker.ImageUpdate.connect(self.ImageUpdateSlot)

        self.setLayout(self.VBL)

    def ImageUpdateSlot(self, Image):
        self.FeedLabel.setPixmap(QPixmap.fromImage(Image))

    def CancelFeed(self):
        self.thread.finish()

if __name__ == "__main__":
    App = QApplication(sys.argv)
    Root = MainWindow()
    Root.show()
    sys.exit(App.exec())
