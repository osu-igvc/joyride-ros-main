import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('point_cloud_publisher')

        # Load the mapping function parameters from a file (e.g., CSV)
        # Assuming the parameters are stored in a NumPy array
        if os.path.exists('x_params.csv'):
            self.X_params = np.loadtxt('x_params.csv', delimiter=',')
        else:
            print("'x_params.csv' does not exist")
            sys.exit()

        if os.path.exists('y_params.csv'):
            self.Y_params = np.loadtxt('y_params.csv', delimiter=',')
        else:
            print("'y_params.csv' does not exist")
            sys.exit()
        
        # Create a PointCloud2 publisher
        self.publisher = self.create_publisher(PointCloud2, 'perception/point_cloud', 10)

        # Create an OpenCV bridge
        self.bridge = CvBridge()

        # Create a subscriber for the binary image
        self.subscriber = self.create_subscription(
            Image,
            'perception/object_to_point_cloud',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        # Convert the binary image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # Extract UV points from the binary image
        uv_points = self.extract_uv_points(cv_image)

        # Convert UV points to PointCloud2 message
        point_cloud_msg = self.convert_uv_to_point_cloud(uv_points)

        # Publish the PointCloud2 message
        self.publisher.publish(point_cloud_msg)

    def extract_uv_points(self, binary_image):
        # Perform image processing to extract UV points
        # Here is a simple example that assumes white pixels represent UV points
        uv_points = np.argwhere(binary_image == 255)

        return uv_points

    def convert_uv_to_point_cloud(self, uv_points):
        point_cloud_data = []
        for uv in uv_points:
            # Apply the mapping function using the parameters
            x_val, y_val = self.uv_to_xyz(uv[1], uv[0])
            z_val = 0.0  # Assuming Z is always 0 in this case

            # Add the point to the point cloud data
            point_cloud_data.append([x_val, y_val, z_val])

        # Create the PointCloud2 message
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_objects'
        msg.height = 1
        msg.width = len(uv_points)
        msg.fields.append(PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1))
        msg.fields.append(PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1))
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.asarray(point_cloud_data, dtype=np.float32).tobytes()

        return msg

    def uv_to_xyz(self, u, v):
        uv_vec = np.array([1, u, u**2, u**3, v, v**2, v**3, u*v, u*v**2, u**2 * v])

        x_val = np.dot(uv_vec, self.X_params)
        y_val = np.dot(uv_vec, self.Y_params)

        return x_val, y_val


def main(args=None):
    rclpy.init(args=args)
    point_cloud_publisher = PointCloudPublisher()

    rclpy.spin(point_cloud_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
