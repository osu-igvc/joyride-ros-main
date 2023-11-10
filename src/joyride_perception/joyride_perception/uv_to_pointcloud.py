import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import scipy.linalg as LA
from scipy.optimize import lsq_linear


import matplotlib.pyplot as plt #Imported for troubleshooting



class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')

        # Get ROS parameters
        self.camera_frame = self.declare_parameter('camera_frame','center').get_parameter_value().string_value
        self.trans_info = self.declare_parameter('data_transformed','lanes').get_parameter_value().string_value
        self.calibration_type = self.declare_parameter("calibration_type","correlated").get_parameter_value().string_value
        self.calibration_file = self.declare_parameter('calibration_file', '/home/joyride-obc/joyride-ros-main/src/joyride_perception/joyride_perception/calibration_center_tm.csv').get_parameter_value().string_value
        self.image_sub_topic = self.declare_parameter("subscriber_topic", "/perception/lane/white").get_parameter_value().string_value
        #self.image_sub_topic = self.declare_parameter("subscriber_topic", "/perception/object_to_point_cloud").get_parameter_value().string_value


        self.compressed = True if self.image_sub_topic.split("/")[-1] == "compressed" else False

        self.hsv_bounds = np.array([[41, 38, 143],[80,255,255]], dtype=np.uint8)
        #print('filepath: ', os.getcwd())
        # Create Calibration
        if not os.path.exists(self.calibration_file):
            raise FileExistsError("Calibration file not found")
        self.Θ = self.calibrate_uv_xyz_transform(self.calibration_type, self.calibration_file)

        # # Load the mapping function parameters from a file (e.g., CSV)
        # # Assuming the parameters are stored in a NumPy array
        # if os.path.exists('x_params.csv'):
        #     self.X_params = np.loadtxt('x_params.csv', delimiter=',')
        # else:
        #     print("'x_params.csv' does not exist")
        #     sys.exit()

        # if os.path.exists('y_params.csv'):
        #     self.Y_params = np.loadtxt('y_params.csv', delimiter=',')
        # else:
        #     print("'y_params.csv' does not exist")
        #     sys.exit()
        
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


    def compressed_image_callback(self, msg:CompressedImage):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        #cv_image = cv2.flip(cv_image, 0)
        #cv_image = cv2.flip(cv_image, 1)

        cv2.imshow("Original",cv_image)
        uv = self.extract_uv_points(cv_image)
        point_cloud_msg = self.project_image(uv)
        self.publisher.publish(point_cloud_msg)


    def image_callback(self, msg:Image):
        # Convert the binary image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        cv_image = cv2.flip(cv_image, 0)
        cv_image = cv2.flip(cv_image, 1)

        print(cv_image.shape)
        print(2)

        # Extract UV points from the binary image
        uv_points = self.extract_uv_points(cv_image)

        # Convert UV points to PointCloud2 message
        point_cloud_msg = self.project_image(uv_points) #self.convert_uv_to_point_cloud(uv_points)

        # Publish the PointCloud2 message  
        self.publisher.publish(point_cloud_msg)




    def calibrate_uv_xyz_transform(self,  calibration_type: str, calibration_file: str) -> np.ndarray:
        if calibration_type == "correlated":
            U, V, X, Y, Z = np.loadtxt(calibration_file,skiprows=2, unpack=True, delimiter=",")
            R = self.create_regressor(np.array([U,V]).T)
            sol = lsq_linear(LA.block_diag(R,R,R) ,np.append(X, [Y,Z]))
            Θ = sol.x.reshape(10,3, order="F")
            return Θ
        elif calibration_type == "params":
            Θ = np.loadtxt(calibration_file,skiprows=1, delimiter=",")
            return Θ
        



    def extract_uv_points(self, image:np.ndarray) -> np.ndarray:
        # This section is used for color images. Comment out if feeding a black and white image

        # image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # hsv_mask = cv2.inRange(image_hsv, self.hsv_bounds[0], self.hsv_bounds[1])
        # image_masked = cv2.bitwise_and(image,image, mask=hsv_mask)


        #These 2 lines are used to show an image of the black and white processed image that this node is subscribed to 
        cv2.imshow("Image",image)
        cv2.waitKey(1)


        #output is VU coordinates, need to flip them to UV, swap the columns
        #VU coordinates are all spots where image values are not 0,(Looking for coordinates wehre pixels are white)
        vu = np.argwhere(image != 0)
        #Flipping the columns to make uv coordinates 
        uv_unfiltered= vu[:, ::-1]
        
        #Input bounds for LLsq curve fitting measurements in fall2023 csv
        u_min=120
        u_max=904
        v_min=400
        v_max=700

        #filtering out all rows from retrieved uv Nx2 matrix that are outside our bounds
        masku=(uv_unfiltered[:,0]>=u_min )&(uv_unfiltered[:,0]<=u_max)
        maskv=(uv_unfiltered[:,1]>=v_min )&(uv_unfiltered[:,1]<=v_max)
        maskcombo=masku&maskv
        uv=uv_unfiltered[maskcombo]

        #return all filtered uv points for further processing
        return uv



    def project_image(self, uv: np.ndarray) -> PointCloud2:
        Φ = np.transpose([np.ones_like(uv[:,0]), uv[:,0], uv[:,0]**2, uv[:,0]**3, uv[:,1], uv[:,1]**2, uv[:,1]**3, uv[:,0]*uv[:,1], uv[:,0]*uv[:,1]**2, uv[:,0]**2 *uv[:,1]])
        P = Φ @ self.Θ

        # This commented block will plot all extracted uv points that are converted into xyz. It will stop the code on plot.show()
        # x=P[:,0]
        # y=P[:,1]
        # plt.scatter(x,y,.01)
        # plt.show()

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.height = 1
        msg.width = len(uv)
        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step =len(msg.fields)*4
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.asarray(P, dtype=np.float32).tobytes()
        
        return msg
    
    def create_regressor(self, uv: np.ndarray) -> np.ndarray:
        U, V = uv[:,0], uv[:,1]
        Φ = np.transpose([np.ones_like(U), U, U**2, U**3, V, V**2, V**3, U*V, U*V**2, U**2 *V])
        return Φ

def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()
    rclpy.spin(point_cloud_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
