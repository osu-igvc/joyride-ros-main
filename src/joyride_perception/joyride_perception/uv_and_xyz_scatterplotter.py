import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import scipy.linalg as LA
from scipy.optimize import lsq_linear
import matplotlib.pyplot as plt  # Import matplotlib

class PointCloudPublisher(Node):
    """
    ROS node for publishing PointCloud2 messages from UV points.

    Attributes:
    - camera_frame (str): ROS parameter for the camera frame.
    - trans_info (str): ROS parameter for the transformation information.
    - calibration_type (str): ROS parameter for the type of calibration.
    - calibration_file (str): ROS parameter for the calibration file.
    - image_sub_topic (str): ROS parameter for the image subscriber topic.
    - compressed (bool): Flag indicating whether the image is compressed.
    - hsv_bounds (np.ndarray): HSV color bounds for image processing.
    - Θ (np.ndarray): Calibration matrix.
    - publisher (Publisher): ROS publisher for PointCloud2 messages.
    - bridge (CvBridge): ROS-CV bridge for image conversion.
    - compressed_image_sub (Subscription): ROS subscription for compressed image.
    - image_sub (Subscription): ROS subscription for image.
    - uv_points (list): List to store UV points.
    - xyz_points (list): List to store XYZ points.
    """
    def __init__(self):
        """
        Initialize the PointCloudPublisher node.
        """
        super().__init__('point_cloud_publisher')

        # Get ROS parameters
        self.camera_frame = self.declare_parameter('camera_frame', 'center').get_parameter_value().string_value
        self.trans_info = self.declare_parameter('data_transformed', 'lanes').get_parameter_value().string_value
        self.calibration_type = self.declare_parameter("calibration_type", "params").get_parameter_value().string_value
        self.calibration_file = self.declare_parameter('calibration_file',
                                                      '/home/joyride-obc/joyride-ros-main/CurveFitParams_center.csv').get_parameter_value().string_value
        self.image_sub_topic = self.declare_parameter("subscriber_topic", "/perception/lane/white").get_parameter_value().string_value

        self.compressed = True if self.image_sub_topic.split("/")[-1] == "compressed" else False

        self.hsv_bounds = np.array([[41, 38, 143], [80, 255, 255]], dtype=np.uint8)

        # Create Calibration
        if not os.path.exists(self.calibration_file):
            raise FileExistsError("Calibration file not found")
        self.Θ = self.calibrate_uv_xyz_transform(self.calibration_type, self.calibration_file)

        # Create a PointCloud2 publisher
        self.publisher = self.create_publisher(PointCloud2, f'perception/{self.camera_frame}/{self.trans_info}_point_cloud', 10)

        # Create an OpenCV bridge
        self.bridge = CvBridge()

        # Create a subscriber for the binary image
        if self.compressed:
            self.compressed_image_sub = self.create_subscription(
                CompressedImage,
                self.image_sub_topic,
                self.compressed_image_callback,
                10
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.image_sub_topic,
                self.image_callback,
                10
            )

        # Lists to store UV and XYZ points
        self.uv_points = []
        self.xyz_points = []

    def compressed_image_callback(self, msg: CompressedImage): # why no flip if we flip the other?????
        """
        Callback function for processing compressed image messages.

        Parameters:
            msg (CompressedImage): Input compressed image message.
        """
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        uv = self.extract_uv_points(cv_image)
        point_cloud_msg = self.project_image(uv)
        self.publisher.publish(point_cloud_msg)

    def image_callback(self, msg: Image): #UwW whats this. ANOTHER DAMN IMAGE CALLBACK
        """
        Callback function for processing image messages.

        Parameters:
            msg (Image): Input image message.
        """
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        cv_image = cv2.flip(cv_image, 0)
        cv_image = cv2.flip(cv_image, 1)
        uv_points = self.extract_uv_points(cv_image)
        point_cloud_msg = self.project_image(uv_points)
        self.publisher.publish(point_cloud_msg)

    def calibrate_uv_xyz_transform(self, calibration_type: str, calibration_file: str) -> np.ndarray:
        """
        Calibrate UV to XYZ transformation.

        Parameters:
        - calibration_type (str): Type of calibration.
        - calibration_file (str): Path to the calibration file.

        Returns:
        - np.ndarray: Calibration matrix.
        """
        if calibration_type == "correlated":
            U, V, X, Y, Z = np.loadtxt(calibration_file, skiprows=2, unpack=True, delimiter=",")
            R = self.create_regressor(np.array([U, V]).T)
            sol = lsq_linear(LA.block_diag(R, R, R), np.append(X, [Y, Z]))
            Θ = sol.x.reshape(10, 3, order="F")
            return Θ
        elif calibration_type == "params":
            Θ = np.loadtxt(calibration_file, skiprows=1, delimiter=",")
            return Θ

    def extract_uv_points(self, image: np.ndarray) -> np.ndarray:
        """
        Extract UV points from the image.

        Parameters:
        - image (np.ndarray): Input image.

        Returns:
        - np.ndarray: UV points.
        """
        cv2.imshow("Image", image)
        cv2.waitKey(1)
        uv = np.argwhere(image != 0)
        return uv

    def project_image(self, uv: np.ndarray) -> PointCloud2:
        """
        Project UV points to PointCloud2.

        Parameters:
        - uv (np.ndarray): UV points.

        Returns:
        - PointCloud2: Output PointCloud2 message.
        """
        #NOTE: Looking at this makes me violently ill. May God help your soul after deciphering this 
        Φ = np.transpose([np.ones_like(uv[:,0]), uv[:,0], uv[:,0]**2, uv[:,0]**3, uv[:,1], uv[:,1]**2, uv[:,1]**3, uv[:,0]*uv[:,1], uv[:,0]*uv[:,1]**2, uv[:,0]**2 *uv[:,1]])
        P = Φ @ self.Θ

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'bfly_center'
        msg.height = 1
        msg.width = len(uv)
        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step = len(msg.fields) * 4
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.asarray(P, dtype=np.float32).tobytes()

        # Append UV and XYZ points to their respective lists
        self.uv_points.extend(uv.tolist())
        self.xyz_points.extend(P.tolist())

        # Scatterplot the UV points
        uv_points = np.array(self.uv_points)
        plt.figure(1)
        plt.scatter(uv_points[:, 1], uv_points[:, 0], s=1, color='blue')
        plt.title("UV Points")
        plt.xlabel("V")
        plt.ylabel("U")

        # Scatterplot the XYZ points
        xyz_points = np.array(self.xyz_points)
        plt.figure(2)
        plt.scatter(xyz_points[:, 0], xyz_points[:, 1], c=xyz_points[:, 2], cmap='viridis', s=1)
        plt.title("XYZ Points")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.colorbar()

        plt.pause(0.01)  # Update the plots in real-time

        return msg

    def create_regressor(self, uv: np.ndarray) -> np.ndarray:
        """
        Create a regressor for calibration.

        Parameters:
            uv (np.ndarray): UV points.

        Returns:
            np.ndarray: Regressor matrix.
        """
        # This was supposed to replace having somelines but I think we gave up half way through?
        U, V = uv[:, 0], uv[:, 1]
        Φ = np.transpose([np.ones_like(U), U, U**2, U**3, V, V**2, V**3, U*V, U*V**2, U**2*V])
        return Φ

def main(args=None):
    """
    Main function to initiate and run the PointCloudPublisher node.
    """
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()

    # A try statment in main to control shutdown.
    # I like who wrote this portion
    # ples len fom dis.
    try:
        while rclpy.ok():
            rclpy.spin_once(point_cloud_publisher, timeout_sec=1.0)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()