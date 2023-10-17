# Publish images from video file or live camera feed.
# Author: Max DeSantis
# Project: IGVC Self Drive FA22 Autonomy Team
# Reference: https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os.path

from joyride_interfaces.msg import Heartbeat
#from joyride_interfaces.heartbeat_manager import HeartbeatManager

class ImagePublisher(Node):
    def __init__(self) -> None:
        super().__init__('raw_image_publisher')

        self.declare_parameter('image_source', 'video0')
        self.declare_parameter('image_topic', '/cameras/unknown_raw')
        self.topic_name = self.get_parameter('image_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(Image, self.topic_name, 10)
        #self.heartbeat_pub = self.create_publisher(Heartbeat, '/health/cameras/front', 10)
        #self.heartbeat_manager = HeartbeatManager(self, HeartbeatManager.Type.publisher, None)

        self.count = 0

        # Publishes every 0.1 seconds - increase in future
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.cap = self.getImageCaptureMethod()

        # Need to check for error here

        self.bridge = CvBridge()
    
    def getImageCaptureMethod(self):
        
        img_src_param = self.get_parameter('image_source').get_parameter_value().string_value
        
        #self.get_logger().info("Image source: " + img_src_param)


        if img_src_param == 'video0':
            return cv2.VideoCapture(0)
        elif os.path.isfile(img_src_param):
            return cv2.VideoCapture(img_src_param)
        else:
            self.get_logger().error('Unable to find image source for ' + self.topic_name + ' at ' + img_src_param)
            return -1

    
    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

            #self.heartbeat_manager.beatOnce()

            self.count += 1

        #self.get_logger().info("Publishing image on: " + self.topic_name)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()

    rclpy.spin(image_publisher)
    image_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()