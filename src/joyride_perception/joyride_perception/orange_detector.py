
# ROS Imports
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Header
from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField, CompressedImage

# OpenCV Imports
import cv2

# Other imports
import numpy as np

# Lazer Scan Correlation
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

class PointCorresponder():
    """
    Class for handling point clustering and correlation in polar coordinates.
    """
    def __init__(self, θ_err:float, n_clusters: float, R: float, clustering_threshold: float = 0.1):
        """
        Initialize the PointCorresponder.

        Parameters:
            θ_err (float): Angular error.
            n_clusters (float): Number of clusters.
            R (float): Radius.
            clustering_threshold (float): Clustering threshold.
        """
        self.km = KMeans(n_clusters = n_clusters, n_init = 10, random_state = 170)
        self.θ_err = np.pi/6
        self.R = R
        self.clustering_threshold = clustering_threshold
        self.angular_tol = 0.1

    def window_data(self, data:np.ndarray, window: np.ndarray) -> np.ndarray:
        """
        Extract windowed data based on angular tolerance and clustering window.

        Parameters:
            data (np.ndarray): Input data.
            window (np.ndarray): Window specifications.

        Returns:
            np.ndarray: Windowed data.
        """
        try:
            x, y = data[:,0], data[:,1]
            bounds = [np.argwhere(np.isclose(x, w, atol = self.angular_tol))[0][0] for w in window[0]]
            temp = np.array([x[bounds[0]:bounds[1]], y[bounds[0]:bounds[1]]]).T
            windowed= [row for row in temp if (window[1][0]<=row[1]<=window[1][1])]
            return np.array(windowed)
        except:
            return None

    def points_polar(self, θ_obs: float, scan: np.ndarray) -> np.array:
        """
        Convert points to polar coordinates.

        Parameters:
            θ_obs (float): Observed angle.
            scan (np.ndarray): Input scan data.

        Returns:
            np.array: Points in polar coordinates.
        """
        obs = []
        for i, θ in enumerate(θ_obs):
            θ_min = max(θ - self.θ_err, np.deg2rad(-60))
            θ_max = min(θ + self.θ_err, np.deg2rad(60))
            windowed = self.window_data(scan, np.array([[θ_min ,θ_max],[0,self.R]]))
            kmeans = self.km.fit(windowed)
            clusters = kmeans.cluster_centers_
            if len(obs) == 0:
                obs.extend(clusters)
            else:
                new_clusters = [c for c in clusters if (np.any(np.abs(obs -c) >= self.clustering_threshold)) ]
                obs.extend(new_clusters)
        return np.array(obs)
    
    def polar_cartesian_convert(self, polar:np.array) -> np.array:
        """
        Convert polar coordinates to cartesian coordinates.

        Parameters:
            polar (np.array): Points in polar coordinates.

        Returns:
            np.array: Points in cartesian coordinates.
        """
        if polar is None or len(polar) == 0:
            return None
        return np.array([polar[:,1]*np.cos(polar[:,0]), polar[:,1]*np.sin(polar[:,0]), 0*polar[:,1]]).T
    
    def points_cart(self, θ_obs: float, scan: np.ndarray) -> np.array:
        """
        Convert points to cartesian coordinates.

        Parameters:
            θ_obs (float): Observed angle.
            scan (np.ndarray): Input scan data.

        Returns:
            np.array: Points in cartesian coordinates.
        """
        obs = self.points_polar(θ_obs, scan)
        return self.polar_cartesian_convert(obs)

class BlobDetector(Node):
    """
    Class for detecting blobs and publishing results.
    """
    def __init__(self):
        """
        Initialize the BlobDetector.
        """
        # Boilerplate setup
        super().__init__('blob_detector')
        self.bridge = CvBridge()

        # --- Get parameters
        image_sub_topic = self.declare_parameter('image_source_topic', 'sensors/cameras/center/image/compressed').get_parameter_value().string_value
        image_pub_name = self.declare_parameter('image_output_topic', 'perception/blob_detected').get_parameter_value().string_value
        laser_scan_topic = self.declare_parameter("laser_scan_topics",'sensors/lidar/front_lidar/scan').get_parameter_value().string_value
        obstacles_pub_name = self.declare_parameter('obstacles_output_topic', 'perception/obstacles').get_parameter_value().string_value


        # --- Setup pub/sub
        self.image_sub = self.create_subscription(CompressedImage, "/"+image_sub_topic, self.imageCallback, 10)
        self.laser_sub = self.create_subscription(LaserScan, "/"+laser_scan_topic, self.laser_scan_cb,10)
        self.contour_pub = self.create_publisher(Image, "/"+image_pub_name, 10)
        self.obs_pub = self.create_publisher(PointCloud2, "/"+obstacles_pub_name, 10)

        self.MASK_BLUR_KERNEL_SIZE = 5
        
        ### Lazer Scan Correlation
        self.scan = None

        R = self.declare_parameter("max_detection_range", 25.0).get_parameter_value().double_value
        L = self.declare_parameter("lidar_camera_baseline", 0.5).get_parameter_value().double_value
        self.FOV =self.declare_parameter("FOV", 0.929).get_parameter_value().double_value 
        self.clustering_threshold = self.declare_parameter("cluster_threshold", 0.1).get_parameter_value().double_value
        n_clusters = self.declare_parameter("clusters",5).get_parameter_value().integer_value

        self.km = KMeans(n_clusters = n_clusters, n_init = 10, random_state = 170)
        θ_tilde = np.arcsin(L/R * np.sin(self.FOV))
        self.point_corresponder = PointCorresponder(θ_err = θ_tilde, n_clusters = n_clusters, R = R)

    def imageCallback(self, img_msg: CompressedImage) -> None:
        """
        Callback function for image messages.

        Parameters:
            img_msg (CompressedImage): Input image message.
        """
        #frame = self.bridge.imgmsg_to_cv2(img_msg)
        frame = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        θ_obs, image_obs = self.detect_obstacles(frame)

        if (self.scan is not None) and (len(θ_obs) > 0) and (len(self.scan) > 0):
            pos_obs = self.point_corresponder.points_cart(θ_obs,self.scan)
            if pos_obs is not None:
                obs_msg = PointCloud2()
                obs_msg.header.stamp = self.get_clock().now().to_msg()
                obs_msg.header.frame_id = 'hokuyo_front'
                obs_msg.height = 1
                obs_msg.width = len(pos_obs)
                obs_msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
                obs_msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
                obs_msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
                obs_msg.is_bigendian = False
                obs_msg.point_step = 12
                obs_msg.row_step = obs_msg.point_step *obs_msg.width
                obs_msg.is_dense = True
                obs_msg.data = np.asarray(pos_obs, dtype=np.float32).tobytes()
                self.obs_pub.publish(obs_msg)
        #self.contour_pub.publish(self.bridge.cv2_to_imgmsg(image_obs, 'bgr8')) #bgr8 8UC1

    def laser_scan_cb(self, msg: LaserScan) -> None:
        """
        Callback function for laser scan messages.

        Parameters:
            msg (LaserScan): Input laser scan message.
        """
        θ_bounds = np.array([msg.angle_min, msg.angle_max])
        ranges = np.array(msg.ranges)
        self.scan = np.array([np.linspace(θ_bounds[0], θ_bounds[1],len(ranges),endpoint=True), ranges]).T
        self.point_corresponder.angular_tol = msg.angle_increment

    
    def detect_obstacles(self, frame) -> np.ndarray:
        """
        Detect obstacles in the input frame.

        Parameters:
            frame: Input frame.

        Returns:
            Tuple: Tuple containing detected angles and processed frame.
        """
        hog = cv2.HOGDescriptor((48,96),(16,16),(8,8),(8,8),9) # God what the hell are these values coming from
        hog.setSVMDetector(cv2.HOGDescriptor_getDaimlerPeopleDetector())
        
        im_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        im_blurred = cv2.GaussianBlur(im_hsv, (self.MASK_BLUR_KERNEL_SIZE, self.MASK_BLUR_KERNEL_SIZE), 0) # Parameterize

        orange_lower = np.array([4, 50, 50])
        orange_upper = np.array([6, 255, 255])
        orange_mask = cv2.inRange(im_blurred, orange_lower, orange_upper)

        masked_img = cv2.bitwise_and(frame, frame, mask=orange_mask)
        
        gray_masked_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
        
        # Contour Detection
        contours, hierarchy = cv2.findContours(gray_masked_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cont = [c for c in contours if len(c) > 150]
        cv2.drawContours(frame, cont, -1, (0,255,0), cv2.LINE_AA)
        θ = [ self.heading(np.average(c, axis =0)[0], frame.shape[1]) for c in cont]

        # If you want to know why this is commented out please contact me as I want to know too.
        # for i, c in enumerate(contours):
            
        #     C = np.average(c, axis = 0)[0]
        #     θ.extend(self.heading(np.array([Cx, Cy]), frame.shape[1]))

        #     print(i, C)
        #         #M = cv2.moments(c)
        #         #Cx = int(np.average)#int(M['m10'] / M['m00'])
        #         #Cy = #int(M['m01'] / M['m00'])

        #         # Get Angle of Blobs 
        #         #θ.extend(self.heading(np.array([Cx, Cy]), frame.shape[1]))
        #     #except:
        #     #    pass
            
        # # Blob Detection
        # # keypoints = self.detector.detect(gray_masked_img)
        # # blobs_img = cv2.drawKeypoints(masked_img, keypoints, np.array([]), (0,255,0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        # # θ = [self.heading(K,frame.shape[1]) for K in keypoints]
        # θ = []
        return θ, frame

    def heading(self,keypoint:np.ndarray, width: int) -> float
        """
        Calculate heading angle based on keypoint and image width.

        Parameters:
            keypoint (np.ndarray): Keypoint coordinates.
            width (int): Image width.

        Returns:
            float: Heading angle.
        """    
        # At least this methods simple 
        # Distance between centroid and origin
        distance_u = (width / 2) - keypoint[0]
        # Get theta
        theta = distance_u / width * self.FOV
        return theta

def main():
    """
    Main function to initiate and run the BlobDetector.
    """
    rclpy.init()
    detector = BlobDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()