import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

import cv2
from cv_bridge import CvBridge
import torch, requests

# Object classifier that uses YOLO to detect traffic signs and other objects
# Runs as fast as new images are received, as YOLO is faster than our cameras. Runs on GPU.

# - Outputs an image showing classifications
# - Outputs a 2D detection array with bounding boxes, classification IDs, confidence, etc.

# Much of the code is sourced from:
# https://github.com/open-shade/yolov5/blob/main/yolov5/interface.py

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')

        # ROS Parameters
        self.image_source_topic = self.declare_parameter('image_source', '/sensors/cameras/lane/image_raw').get_parameter_value().string_value
        self.image_output_topic = self.declare_parameter('image_output', '/perception/signs_detected').get_parameter_value().string_value
        self.detections_output_topic = self.declare_parameter('detection_output','/perception/sign_predictions').get_parameter_value().string_value


        self.weights = self.declare_parameter('weights_path', 'yolov5s.pt')
        
        # YOLO Model using PyTorch Hub

        # - absolute path to yolov5
        self.path_hubconfig = self.declare_parameter('model_repo_path', '/home/joyride-obc/joyride-ros-main/src/yolov5').get_parameter_value().string_value

        # - absolute path to best.pt
        self.path_trained_model = self.declare_parameter('model_weights_path', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/config/yolov5s.pt').get_parameter_value().string_value
        self.model = torch.hub.load(self.path_hubconfig, 'custom', path=self.path_trained_model, source='local', force_reload=False)

        # ROS pubs
        self.detection_pub = self.create_publisher(Detection2DArray, self.detections_output_topic, 10)
        self.classification_pub = self.create_publisher(Image, self.image_output_topic, 10)

        self.bridge = CvBridge()
        # ROS subs
        self.image_sub = self.create_subscription(Image, self.image_source_topic, self.new_raw_image_cb, 1)
        self.get_logger().info('Traffic Detector loaded')

    
    def new_raw_image_cb(self, msg:Image):

        cvImg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        results = self.model(cvImg)
        print(results)
        results.render()
        processed_img = self.bridge.cv2_to_imgmsg(results.ims[0], encoding='rgb8')
        #print(processed_img)
        self.classification_pub.publish(processed_img)


        #results.print()
        # Run classifier
        # Annotate image
        # publish annotated
        # Build detection array
        # publish detection array


def main():
    rclpy.init()
    tDetector = TrafficSignDetector()
    rclpy.spin(tDetector)

    tDetector.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()