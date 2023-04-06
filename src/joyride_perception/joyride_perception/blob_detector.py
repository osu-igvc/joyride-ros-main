
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
        self.declare_parameter('hue_upper', 84)
        self.declare_parameter('hue_lower', 8)
        self.declare_parameter('sat_upper', 255)
        self.declare_parameter('sat_lower', 105)
        self.declare_parameter('val_upper', 255)
        self.declare_parameter('val_lower', 192)

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

        self.MASK_BLUR_KERNEL_SIZE = 5

    def imageCallback(self, img_msg):
        #self.get_logger().info('Blob detector received image')
        frame = self.bridge.imgmsg_to_cv2(img_msg)
        self.locateBlob(frame)

    # Maybe modify to filter noise from color extractor
    # def colorMask(self, im_hsv):

    #     desired_pixels = np.where( (im_hsv[:, :, 0] <= self.MASK_HUE_UPPER) &
    #         (im_hsv[:, :, 0] >= self.MASK_HUE_LOWER) &
    #         (im_hsv[:, :, 1] >= self.MASK_SAT_LOWER) &
    #         (im_hsv[:, :, 2] >= self.MASK_VAL_LOWER))

    #     # Filter noise
    #     rows, cols = desired_pixels
    #     #self.get_logger().warn('row size:' + str(len(rows)) + 'col size: ' + str(len(cols)))
    #     kernel = np.ones((self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), np.uint8) # Parameterize

    #     im_hsv_masked = im_hsv
    #     im_hsv_masked[:, :] = (0, 0, 0)
    #     im_hsv_masked[rows, cols] = (50, 255, 255)
        
    #     #return cv2.cvtColor(im_hsv_masked, cv2.COLOR_HSV2BGR)

    #     im_bgr_intermediate = cv2.cvtColor(im_hsv_masked, cv2.COLOR_HSV2BGR)
    #     im_mono = cv2.cvtColor(im_bgr_intermediate, cv2.COLOR_BGR2GRAY)

    #     im_mono = cv2.erode(im_mono, kernel, iterations = self.MASK_ERODE_ITERATIONS)
    #     im_mono = cv2.dilate(im_mono, kernel, iterations = self.MASK_DILATE_ITERATIONS)

    #     return im_mono



    # Kalman Filter
    # Find the center point of a bounding box around a detected pedestrian
    def center(points):
        x = np.float32(
               (points[0][0] +
                points[1][0] +
                points[2][0] +
                points[3][0]) / 4.0)
        y = np.float32(
               (points[0][1] +
                points[1][1] +
                points[2][1] +
                points[3][1]) / 4.0)
        return np.array([np.float32(x), np.float32(y)], np.float32)

    def locateBlob(self, frame):

        #hog = cv2.HOGDescriptor()
        #hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        hog = cv2.HOGDescriptor((48,96),(16,16),(8,8),(8,8),9)
        hog.setSVMDetector(cv2.HOGDescriptor_getDaimlerPeopleDetector())

        frame_resize = cv2.resize(frame, (600, 400))
        
        im_blurred = cv2.GaussianBlur(frame_resize, (self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), 0) # Parameterize

        im_hsv = cv2.cvtColor(im_blurred, cv2.COLOR_BGR2HSV)

        gray_frame = cv2.cvtColor(im_blurred, cv2.COLOR_BGR2GRAY)

        # HOG Pedestrian Detector 
        rois, weights = hog.detectMultiScale(gray_frame, hitThreshold=1.3, winStride=(4,4), padding=(4.4), scale=1.05)
            # ROIS Coordinates where person is detected inside
            # Weights - confidence values [0, ~3]

        rois = np.array([[[[x,y],[x,y+h],[x+w,y+h],[x+w,y]]] for (x,y,w,h) in rois])
        background_mask = np.zeros_like(frame_resize)
        for roi in rois:
            cv2.fillPoly(background_mask, roi, (255,255,255)) 
        ped_extract = cv2.bitwise_and(frame_resize, background_mask)

        orange_lower = np.array([self.MASK_HUE_LOWER, self.MASK_SAT_LOWER, self.MASK_VAL_LOWER])
        orange_upper = np.array([self.MASK_HUE_UPPER, self.MASK_SAT_UPPER, self.MASK_VAL_UPPER])
        orange_mask = cv2.inRange(im_hsv, orange_lower, orange_upper)

        blob = cv2.bitwise_and(ped_extract, ped_extract, orange_mask)
        


        self.contour_pub.publish(self.bridge.cv2_to_imgmsg(blob, 'bgr8'))


def main():
    rclpy.init()

    detector = BlobDetector()

    rclpy.spin(detector)

    cv2.destroyAllWindows()

    detector.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()