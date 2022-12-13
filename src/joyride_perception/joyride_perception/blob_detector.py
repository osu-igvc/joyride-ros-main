
# ROS Imports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# OpenCV Imports
import cv2

# Other imports
import numpy as np

# IGVC Autonomy Imports
#from joyride_interfaces.heartbeat_manager import HeartbeatManager
import joyride_perception.submodules.vision_utility as vis



class BlobDetector(Node):
    
    def __init__(self):
        # Boilerplate setup
        super().__init__('blob_detector')
        #self.heartbeat = HeartbeatManager(self, HeartbeatManager.Type.publisher)
        self.bridge = CvBridge()

        # Setup heartbeat timer

        # ROS Interconnection
        self.declare_parameter('image_source_topic', '/cameras/unknown_raw')
        self.declare_parameter('image_output_topic', '/perception/blob_detected')
        self.declare_parameter('hue_upper', 40)
        self.declare_parameter('hue_lower', 30)
        self.declare_parameter('sat_upper', 255)
        self.declare_parameter('sat_lower', 130)
        self.declare_parameter('val_upper', 255)
        self.declare_parameter('val_lower', 130)

        # --- Get parameters
        self.topic_name = self.get_parameter('image_source_topic').get_parameter_value().string_value
        self.output_topic_name = self.get_parameter('image_output_topic').get_parameter_value().string_value
        self.MASK_HUE_LOWER = self.get_parameter('hue_lower').get_parameter_value().integer_value
        self.MASK_HUE_UPPER = self.get_parameter('hue_upper').get_parameter_value().integer_value
        
        self.MASK_SAT_LOWER = self.get_parameter('sat_lower').get_parameter_value().integer_value
        self.MASK_SAT_UPPER = self.get_parameter('sat_upper').get_parameter_value().integer_value

        self.MASK_VAL_LOWER = self.get_parameter('val_lower').get_parameter_value().integer_value
        self.MASK_VAL_UPPER = self.get_parameter('val_upper').get_parameter_value().integer_value

        # --- Setup pub/sub
        self.image_sub = self.create_subscription(Image, self.topic_name, self.imageCallback, 10)
        self.contour_pub = self.create_publisher(Image, self.output_topic_name, 10)

        

        self.MASK_ERODE_ITERATIONS = 1
        self.MASK_DILATE_ITERATIONS = 1

        self.MASK_BLUR_KERNEL_SIZE = 9

    def imageCallback(self, img_msg):
        #self.get_logger().info('Blob detector received image')
        frame = self.bridge.imgmsg_to_cv2(img_msg)
        self.locateBlob(frame)

    def colorMask(self, im_hsv):



        desired_pixels = np.where( (im_hsv[:, :, 0] <= self.MASK_HUE_UPPER) &
            (im_hsv[:, :, 0] >= self.MASK_HUE_LOWER) &
            (im_hsv[:, :, 1] >= self.MASK_SAT_LOWER) &
            (im_hsv[:, :, 2] >= self.MASK_VAL_LOWER))

        # Filter noise
        rows, cols = desired_pixels
        #self.get_logger().warn('row size:' + str(len(rows)) + 'col size: ' + str(len(cols)))
        kernel = np.ones((self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), np.uint8) # Parameterize

        im_hsv_masked = im_hsv
        im_hsv_masked[:, :] = (0, 0, 0)
        im_hsv_masked[rows, cols] = (50, 255, 255)
        
        #return cv2.cvtColor(im_hsv_masked, cv2.COLOR_HSV2BGR)

        im_bgr_intermediate = cv2.cvtColor(im_hsv_masked, cv2.COLOR_HSV2BGR)
        im_mono = cv2.cvtColor(im_bgr_intermediate, cv2.COLOR_BGR2GRAY)

        im_mono = cv2.erode(im_mono, kernel, iterations = self.MASK_ERODE_ITERATIONS)
        im_mono = cv2.dilate(im_mono, kernel, iterations = self.MASK_DILATE_ITERATIONS)

        return im_mono


    def locateBlob(self, frame):
        
        im_blurred = cv2.GaussianBlur(frame, (self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), 0) # Parameterize

        im_hsv = cv2.cvtColor(im_blurred, cv2.COLOR_BGR2HSV)
        masked_frame = self.colorMask(im_hsv)
    
        
        # Finds contours in frame
        
        contours, hierarchy = cv2.findContours(masked_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if(len(contours) > 0):
            cntsSrted = sorted(contours, key=lambda x: cv2.contourArea(x))
            cnt = cntsSrted[len(cntsSrted) - 1]
            if cv2.contourArea(cnt) > 1200:
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                

                cv2.drawContours(frame, [cnt], -1, (0, 255, 0), 3)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(frame, "Pedestrian", (cX - 20, cY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        self.contour_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

        
        # How to define obstacles in this package? What amount of separation to define?
        # - Should this just determine blob location in camera's optical frame? I think so.
        # - Blob location then projected into 3D space. How? Know: camera position, parameters.


def main():
    rclpy.init()

    detector = BlobDetector()

    rclpy.spin(detector)

    detector.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()