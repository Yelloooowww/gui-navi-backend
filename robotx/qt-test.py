#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import cv2
import json
import requests

from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *

# exit when ctrl+C
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

class ImageLabel(QLabel):
    scale = 1.0
    def showImage(self, img):
        height, width, channel = img.shape
        bytesPerline = 3 * width
        self.qImg = QImage(img.data, width, height, bytesPerline, QImage.Format_RGB888).rgbSwapped()
        self.setPixmap(QPixmap.fromImage(self.qImg))

    def mouseDoubleClickEvent(self,event):
        self.x = event.x()
        self.y = event.y()
        print('mouseDoubleClickEvent '+str(self.x) + ' ' + str(self.y))

    def mousePressEvent(self,event):
        self.moveFlag = True
        self.movePosition = event.globalPos() - self.pos()
        self.setCursor(QCursor(Qt.OpenHandCursor))
        event.accept()

    def mouseMoveEvent(self, event):
        self.x = event.x()
        self.y = event.y()
        # print('mouseMoveEvent '+str(self.x) + ' ' + str(self.y))
        self.move(event.globalPos() - self.movePosition)
        event.accept()

    def mouseReleaseEvent(self, event):
        self.moveFlag = False
        self.setCursor(Qt.CrossCursor)

    def wheelEvent(self, event):
        numDegrees = event.angleDelta() / 8
        numSteps = numDegrees / 15
        # print(numSteps.y())
        height, width, _ = self.img.shape
        if numSteps.y() == -1:
            if (self.scale > 1):
                self.scale -= 0.02
        else:
            if (self.scale <= 20.0):
                self.scale += 0.02
        print(self.scale)
        height2 = int(height * self.scale)
        width2 = int(width * self.scale)
        img2 = cv2.resize(self.img, (width2, height2), interpolation=cv2.INTER_AREA)
        # self.setAlignment(Qt.AlignCenter)
        self.setFixedSize(width2, height2)
        self.showImage(img2)



class MyDialog(QDialog):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.label = ImageLabel()
        layout = QGridLayout(self)
        layout.addWidget(self.label, 0, 0, 4, 4)
        self.label.img = cv2.imread('cave_map_3.png', -1)
        self.label.showImage(self.label.img)
        height, width, _ = self.label.img.shape
        self.resize(width,height)
        self.label.setFixedSize(width,height)
        self.setFixedSize(self.size())





if __name__ == '__main__':
    a = QApplication(sys.argv)
    dialog = MyDialog()
    dialog.show()
    sys.exit(a.exec_())
