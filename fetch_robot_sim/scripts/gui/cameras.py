import os
import sys

from PyQt5 import uic, QtCore, QtWidgets, QtGui
from PyQt5.QtWidgets import (
    QApplication,
    QDesktopWidget,
    QMainWindow,
    QWidget,
    QFileDialog,
)

from PyQt5 import uic

class Cameras(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)

        uic.loadUi("ui/cameras.ui", self)
        self.show()


