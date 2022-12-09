
# Python Imports
from concurrent.futures import thread
from symtable import Symbol
import sys
from tkinter import W
from typing import Callable
from urllib.request import Request

# Data manipulation
import matplotlib as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import numpy as np
import threading
import time
from enum import Enum

# PYQT Imports
from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6 import uic
from PyQt6.QtWidgets import QMainWindow, QWidget, QPushButton, QHBoxLayout, QVBoxLayout, QLabel, QApplication
from PyQt6.QtCore     import QSize, Qt
from PyQt6.QtGui      import QIcon, QAction, QPixmap
from requests import request

# ROS Imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.action import ActionClient
from std_msgs.msg import Bool


from dashboard_node import GUINode
from dashboard_node import EnableButton

# Very useful tutorial on image conversion from OpenCV to PyQt
# https://www.imagetracking.org.uk/2020/12/displaying-opencv-images-in-pyqt/



   
class UI(QMainWindow):
    def __init__(self):
        super(UI, self).__init__()
        uic.loadUi("install/autonomy_hmi/share/ament_index/resource_index/packages/window.ui", self)
        self.show()

        self.ros_node = GUINode(parentGUI=self)
        self.ros_thread = threading.Thread(target=self.rosSpin)
        self.ros_thread.start()

        self.enableBtn = EnableButton(self, self.enable_btn)

    # ============================= ROS STARTUP =============================

    def rosSpin(self):
        self.ros_node.get_logger().info('Spinning up ros')
        rclpy.spin(self.ros_node)
    
    def close_node(self):
        self.ros_node.get_logger().info('Shutting down ros')
        self.ros_node.destroy_node()

    
    # ============================= Switchable views =============================
    def setSwitchableImage(self, frame):
        scaled_frame = frame.scaled(self.dashDisplay.width(), self.dashDisplay.height(), Qt.AspectRatioMode.KeepAspectRatio)
        self.dashDisplay.setPixmap(QPixmap.fromImage(scaled_frame))
        pass


    # ============================= MISC =============================

    def updateHealth(self, new_health_status: bool):
        if new_health_status:
            self.enable_check_status.setText("HEALTHY")
            self.enable_check_status.setStyleSheet('background-color: green')

        else:
            self.enable_check_status.setText("UNHEALTHY")
            self.enable_check_status.setStyleSheet('background-color: red')

    @QtCore.pyqtSlot()
    def on_actionNew_triggered(self):
        print("New")
        
    @QtCore.pyqtSlot()
    def on_actionOpen_triggered(self):
        print("Open")
        
    @QtCore.pyqtSlot()
    def on_actionExit_triggered(self):
        print("Exit")
        
    # Handles switching tabs using list on left-hand side of GUI
    def on_sidebar_list_itemClicked(self, item: QtWidgets.QListWidgetItem):
        ab = QtWidgets.QStackedWidget()
        if item.data(0) == "Dashboard":  
            self.stackedWidget.setCurrentIndex(0)
        elif item.data(0) == "Camera Tuning":
            self.stackedWidget.setCurrentIndex(1)
        elif item.data(0) == "WIP":
            self.stackedWidget.setCurrentIndex(2)
        elif item.data(0) == "WIP":
            self.stackedWidget.setCurrentIndex(3)

        self.ros_node.get_logger().warn("SIDEBAR CLICKED: " + str(item.data(0)))


    # ============================= AUTONOMY ENABLE BUTTON =============================

    @QtCore.pyqtSlot()
    def on_enable_btn_clicked(self):
        self.ros_node.get_logger().warn("ENABLE BTN PRESSED")
        self.enableBtn.pressed()
        # self.indicator_gear_label.setText(str(result))

        # if result:
        #     self.enable_btn.setStyleSheet('background-color: green')
        # else:
        #     pass



def main():
    rclpy.init()

    app = QtWidgets.QApplication(sys.argv)
    window = UI()

    with open('install/autonomy_hmi/share/ament_index/resource_index/packages/style.qss', "r") as f:
        _style = f.read()
        window.setStyleSheet(_style)

    app.exec()

    window.close_node()

    rclpy.shutdown()
    
        
if __name__ == "__main__":
    main()