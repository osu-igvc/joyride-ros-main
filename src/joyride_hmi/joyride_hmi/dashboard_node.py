
# ROS Imports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from sensor_msgs.msg import Image

# OpenCV
import cv2

# PyQt
from PyQt6 import QtGui
from PyQt6.QtWidgets import QPushButton

# Python
from typing import Callable
from enum import Enum

# Custom ROS
from joyride_interfaces.action import RequestEnableDisable

# Custom GUI
from dashboard_gui import UI

class GUINode(Node):

    def __init__(self, parentGUI: UI = None):

        super().__init__('gui_node')
        self.parentGUI = parentGUI
        self.numCount = 0
        self.subscription = self.create_subscription(Image, '/cameras/front_lane_markings', self.imageCallback, 10)
        self.bridge = CvBridge()

        self.enable_server_client = ActionClient(self, RequestEnableDisable, 'SystemEnable')

        # Health subs
        self.health_server_sub = self.create_subscription(Bool, '/health/system_health', self.systemHealthCallback, 10)
        self.enable_status_sub = self.create_subscription(Bool, '/health/enable_status', self.systemEnableCallback, 10)
        self._systemHealthStatus = False

    def systemHealthCallback(self, msg):
        self._systemHealthStatus = msg.data

        if self.parentGUI is not None:
            self.parentGUI.enableBtn.healthStateUpdated(self._systemHealthStatus)
            self.parentGUI.updateHealth(self._systemHealthStatus)

    def systemEnableCallback(self, msg):
        pass

    def imageCallback(self, msg):
        #self.get_logger().info('Received image')
        self.numCount += 1

        cv_img = self.bridge.imgmsg_to_cv2(msg)
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        bytes_per_line = ch * w

        qt_img = QtGui.QImage(rgb_img.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        
        if self.parentGUI is not None:
            #self.parentGUI.newImage('image #' + str(self.numCount))
            self.parentGUI.setSwitchableImage(qt_img)

    
    def updateEnableStatus(self, new_status, feedback_callback: Callable, goal_response_callback: Callable):
        enable_msg = RequestEnableDisable.Goal()
        enable_msg.set_enabled = new_status

        self.enable_server_client.wait_for_server()

        # Not sure if correct
        self.__send_goal_future = self.enable_server_client.send_goal_async(enable_msg, feedback_callback)
        self.__send_goal_future.add_done_callback(goal_response_callback)
    

class EnableButton():

    class EnableButtonState(Enum):
        disabled_not_healthy = 1,
        disabled_healthy = 2,
        waiting = 3,
        enabled = 4

    def __init__(self, parent: UI, button: QPushButton):
        self.btn = button
        self.healthState = False
        self.buttonState = self.EnableButtonState.disabled_not_healthy
        self.nextButtonState = self.EnableButtonState.disabled_not_healthy
        self.parent = parent

        self.updateState()

    def getState(self):
        return self.buttonState

    def updateState(self):
        if self.buttonState == self.EnableButtonState.disabled_not_healthy:
            self.btn.setText('DISABLED')
            self.btn.setDisabled(True)
            self.btn.setStyleSheet('background-color: grey')

        elif self.buttonState == self.EnableButtonState.disabled_healthy:
            self.btn.setText('DISABLED')
            self.btn.setEnabled(True)
            self.btn.setStyleSheet('background-color: yellow')

        elif self.buttonState == self.EnableButtonState.waiting:
            self.btn.setText('WAITING')
            self.btn.setStyleSheet('background-color: orange')
        
        elif self.buttonState == self.EnableButtonState.enabled:
            self.btn.setText('ENABLED')
            self.btn.setStyleSheet('background-color: green')


    def healthStateUpdated(self, newState: bool):
        self.healthState = newState

        if self.healthState:
            if self.buttonState == self.EnableButtonState.disabled_not_healthy:
                self.buttonState = self.EnableButtonState.disabled_healthy
        else:
            self.buttonState == self.EnableButtonState.waiting
            self.requestDisable()

        self.updateState()

    def pressed(self):
        if self.buttonState == self.EnableButtonState.disabled_not_healthy:
            self.parent.ros_node.get_logger().info('DISABLED - NOT HEALTHY')

        elif self.buttonState == self.EnableButtonState.disabled_healthy:
            self.parent.ros_node.get_logger().info('REQUESTING ENABLE')
            self.requestEnable()

        elif self.buttonState == self.EnableButtonState.waiting:
            self.parent.ros_node.get_logger().info('WAITING ON ENABLE RESPONSE')

        elif self.buttonState == self.EnableButtonState.enabled:
            self.parent.ros_node.get_logger().info('REQUESTING DISABLE')
            self.requestDisable()

        else: # If not in known state, set to one.
            self.buttonState = self.EnableButtonState.disabled_not_healthy

    def requestEnable(self):
        #pass
        # Ask: Are you sure you want to enable autonomy?
        self.nextButtonState = self.EnableButtonState.enabled
        self.parent.ros_node.updateEnableStatus(True, self.goalFeedback, self.goalResponse)

    def requestDisable(self):
        #pass
        self.nextButtonState = self.EnableButtonState.disabled_healthy
        self.parent.ros_node.updateEnableStatus(False, self.goalFeedback, self.goalResponse)
        # Ask: are you sure you want to disable autonomy?

    def goalFeedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.parent.ros_node.get_logger().info('FEEDBACK RECEIVED')

    def goalResponse(self, response_msg):
        rs = response_msg.result()

        if not rs.accepted:
            self.nextButtonState = self.buttonState
            self.parent.ros_node.get_logger().info('RESULT NOT ACCEPTED')
            return
        
        self.parent.ros_node.get_logger().info('RESULT ACCEPTED')
        self.future_result = rs.get_result_async()
        self.future_result.add_done_callback(self.goalResultCallback)

    def goalResultCallback(self, result_msg):
        result = result_msg.result().result
        self.parent.ros_node.get_logger().info('RESULT: {0}'.format(result.new_enable_status))
        self.buttonState = self.nextButtonState
        self.updateState()


