
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


class ObstacleDetector(Node):
    """
    Node for detecting obstacles in camera images and publishing the result.

    Attributes:
        bridge (CvBridge): ROS OpenCV bridge for image conversion.
        topic_name (str): Input topic for subscribing to camera images.
        output_topic_name (str): Output topic for publishing obstacle detections.
        MASK_EDGE_THRESHOLD1 (int): Edge detection threshold 1.
        MASK_EDGE_THRESHOLD2 (int): Edge detection threshold 2.
        MASK_AREA_LOWER (int): Lower threshold for object area.
        MASK_AREA_UPPER (int): Upper threshold for object area.
        MASK_ERODE_ITERATIONS (int): Number of iterations for erosion.
        MASK_DILATE_ITERATIONS (int): Number of iterations for dilation.
        MASK_BLUR_KERNEL_SIZE (int): Size of the Gaussian blur kernel.
        image_sub (Subscription): ROS subscription to camera images.
        contour_pub (Publisher): ROS publisher for publishing obstacle detections.
    """
    def __init__(self):
        """
        Initializes the ObstacleDetector node.
        """
        # Boilerplate setup
        super().__init__('obstacle_detector')
        #self.heartbeat = HeartbeatManager(self, HeartbeatManager.Type.publisher)
        self.bridge = CvBridge()

        # Setup heartbeat timer

        # ROS Interconnection
        self.declare_parameter('image_source_topic', '/sensors/cameras/center/image')
        self.declare_parameter('image_output_topic', '/perception/blob_detected')
        self.declare_parameter('edge_threshold1', 230)
        self.declare_parameter('edge_threshold2', 255)
        self.declare_parameter('object_area_lower', 3000)
        self.declare_parameter('object_area_upper', 40000)

        # --- Get parameters
        self.topic_name = self.get_parameter('image_source_topic').get_parameter_value().string_value
        self.output_topic_name = self.get_parameter('image_output_topic').get_parameter_value().string_value
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
        """
        Callback for processing camera images.

        Args:
            img_msg (sensor_msgs.msg.Image): The received image message.

        Returns:
            None
        """
        #self.get_logger().info('Blob detector received image')
        frame = self.bridge.imgmsg_to_cv2(img_msg)
        self.locateBlob(frame)


    def detectContours(self, frame, mask):
        """
        Detects contours in the frame and extracts objects based on area thresholds.

        Args:
            frame (numpy.ndarray): The input image frame.
            mask (numpy.ndarray): The mask to apply to the frame.

        Returns:
            int: The number of detected objects.
        """
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


    # For future Kalman Filter integration
    # Find the center point of a bounding box around a detected obstacle
    def center(points):
        """
        Calculate the center point of a bounding box.

        Args:
            points (numpy.ndarray): Array of shape (4, 2) representing the four corners
                                    of the bounding box.

        Returns:
            numpy.ndarray: Array of shape (2,) representing the (x, y) coordinates of
                           the center point.
        """
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
        """
        Locates blobs in the input frame and publishes the result.

        Args:
            frame (numpy.ndarray): The input image frame.

        Returns:
            None
        """
        frame_resize = cv2.resize(frame, (600, 400))
        
        im_blurred = cv2.GaussianBlur(frame_resize, (self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), 0) # Parameterize

        gray_frame = cv2.cvtColor(im_blurred, cv2.COLOR_BGR2GRAY)
    

        # Region of Interest
        roi_objFrame = frame_resize.copy()
        roi_objMask = np.array([[[0,0], [0,480], [160,240], [480,240], [640, 480], [640, 0]]], dtype=np.int32)
        cv2.fillPoly(roi_objFrame, roi_objMask, (0,0,0))


        # Edge Detection
        imgCanny = cv2.Canny(gray_frame, self.MASK_EDGE_THRESHOLD1, self.MASK_EDGE_THRESHOLD2)

        # Crops Video Feed using Trapezoid method to detect objects/contours in lane of traffic
        roi_crop = np.array([[[0,0], [0,449], [5,449], [250,145], [350,145], [595, 449], [600, 449], [600, 0]]], dtype=np.int32) 
        cv2.fillPoly(imgCanny, roi_crop, (255,255,255))

        kernel = np.ones((self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), np.uint8)
        imgCanny = cv2.dilate(imgCanny, kernel=kernel, iterations=self.MASK_DILATE_ITERATIONS)

        # Object Extraction
        ground_masker = np.zeros_like(frame_resize)
        num_objects = self.detectContours(imgCanny, ground_masker)
        obj_extract = cv2.bitwise_and(frame_resize, ground_masker)


        self.contour_pub.publish(self.bridge.cv2_to_imgmsg(obj_extract, 'bgr8'))


def main():
    """
    Main function to initialize and run the ObstacleDetector node.
    """
    rclpy.init()

    detector = ObstacleDetector()

    rclpy.spin(detector)

    detector.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()