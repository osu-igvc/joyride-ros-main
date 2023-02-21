
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
        self.declare_parameter('edge_threshold1', 255)
        self.declare_parameter('edge_threshold2', 25)
        self.declare_parameter('object_area_lower', 5000)
        self.declare_parameter('object_area_upper', 41000)

        # --- Get parameters
        self.topic_name = self.get_parameter('image_source_topic').get_parameter_value().string_value
        self.output_topic_name = self.get_parameter('image_output_topic').get_parameter_value().string_value
        self.MASK_HUE_LOWER = self.get_parameter('hue_lower').get_parameter_value().integer_value
        self.MASK_HUE_UPPER = self.get_parameter('hue_upper').get_parameter_value().integer_value
        
        self.MASK_SAT_LOWER = self.get_parameter('sat_lower').get_parameter_value().integer_value
        self.MASK_SAT_UPPER = self.get_parameter('sat_upper').get_parameter_value().integer_value

        self.MASK_VAL_LOWER = self.get_parameter('val_lower').get_parameter_value().integer_value
        self.MASK_VAL_UPPER = self.get_parameter('val_upper').get_parameter_value().integer_value

        self.MASK_EDGE_THRESHOLD1 = self.get_parameter('edge_threshold1').get_parameter_value().integer_value
        self.MASK_EDGE_THRESHOLD2 = self.get_parameter('edge_threshold2').get_parameter_value().integer_value

        self.MASK_AREA_LOWER = self.get_parameter('object_area_lower').get_parameter_value().integer_value
        self.MASK_AREA_UPPER = self.get_parameter('object_area_upper').get_parameter_value().integer_value

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

    def detectContours(self, frame, mask):
        cont, hierarchy = cv2.findContours(frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        count = 0
        for cnts in cont:
            area = cv2.contourArea(cnts)
            perimeter = cv2.arcLength(cnts, True)
            numSides = len(cv2.approxPolyDP(cnts, 0.02*perimeter, True))
            if area > self.MASK_AREA_LOWER and area < self.MASK_AREA_UPPER and numSides < 10:
                count += 1
                cv2.drawContours(mask, [cnts], -1, (255,255,255), -1)   #Extracts object to blank canvas
        return count


    def locateBlob(self, frame):

        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        cv2.startWindowThread()

        frame_resize = cv2.resize(frame, (640, 480))
        
        im_blurred = cv2.GaussianBlur(frame_resize, (self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), 0) # Parameterize

        im_hsv = cv2.cvtColor(im_blurred, cv2.COLOR_BGR2HSV)
        masked_frame = self.colorMask(im_hsv)

        gray_frame = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2GRAY)
        im_gray = cv2.cvtColor(im_blurred, cv2.COLOR_BGR2GRAY)
    

        # ROI mask for Pedestrian
        roi_ped_frame = frame_resize.copy()
        roi_ped_mask = np.array([[[0,0], [0,480], [160,240], [480,240], [640, 480], [640, 0]]], dtype=np.int32)
        cv2.fillPoly(roi_ped_frame, roi_ped_mask, (0,0,0))
        

        # HOG People Detector 
        rois, weights = hog.detectMultiScale(gray_frame, hitThreshold=0.3, winStride=(8,8))
            # Rectangle Coordinates where person is detected inside
            # Weights - confidence values [0, ~3]

        rois = np.array([[[[x,y],[x,y+h],[x+w,y+h],[x+w,y]]] for (x,y,w,h) in rois])
        back_masker = np.zeros_like(frame_resize)
        for roi in rois:
            cv2.fillPoly(back_masker, roi, (255,255,255)) 
        ped_extract = cv2.bitwise_and(frame_resize, back_masker)

        orange_lower = np.array([self.MASK_HUE_LOWER, self.MASK_SAT_LOWER, self.MASK_VAL_LOWER])
        orange_upper = np.array([self.MASK_HUE_UPPER, self.MASK_SAT_UPPER, self.MASK_VAL_UPPER])
        orange_mask = cv2.inRange(im_hsv, orange_lower, orange_upper)

        ped_blob = cv2.bitwise_and(ped_extract, ped_extract, orange_mask)
        


        # Object Detector
        im_Canny = cv2.Canny(im_gray, self.MASK_EDGE_THRESHOLD1, self.MASK_EDGE_THRESHOLD2)

            # Crops Video Feed using Trapezoid method to detect objects/contours in lane of traffic
        lane_crop = np.array([[[0,0], [0,480], [5,480], [165,245], [475,245], [635, 480], [640, 480], [640, 0]]], dtype=np.int32) 
        cv2.fillPoly(im_Canny, lane_crop, (255,255,255))

        kernel = np.ones((self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), np.uint8)
        dilation = cv2.dilate(im_Canny, kernel=kernel, iterations=self.MASK_DILATE_ITERATIONS)

        ground_masker = np.zeros_like(frame_resize)
        num_objects = self.detectContours(dilation, ground_masker)
        obj_extract = cv2.bitwise_and(frame_resize, ground_masker)


        # Combining Color and Shape Mask Video Feed
        finalFrame = cv2.bitwise_or(obj_extract, ped_blob)

       
        cv2.imshow("init", ped_blob)
        cv2.imshow("FRAME", finalFrame)


        self.contour_pub.publish(self.bridge.cv2_to_imgmsg(finalFrame, 'bgr8'))


def main():
    rclpy.init()

    detector = BlobDetector()

    rclpy.spin(detector)

    cv2.destroyAllWindows()

    detector.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()