import math
import numpy as np
from geometry_msgs.msg import Twist

from transforms3d.euler import *
from transforms3d.quaternions import *
import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import Imu

# def euler_from_quaternion(x, y, z, w):
#         """
#         Convert a quaternion into euler angles (roll, pitch, yaw)
#         roll is rotation around x in radians (counterclockwise)
#         pitch is rotation around y in radians (counterclockwise)
#         yaw is rotation around z in radians (counterclockwise)
#         """
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll_x = math.atan2(t0, t1)
     
#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch_y = math.asin(t2)
     
#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw_z = math.atan2(t3, t4)
     
#         return roll_x, pitch_y, yaw_z # in radians


class FrameListener(Node):

    def __init__(self):
        super().__init__('imu_tf')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Imu, '/tf/imu', 1)
        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', 1)
        self.imu_sub = self.create_subscription(Imu, '/vectornav/imu', self.on_imu_cb, 1)


    def on_imu_cb(self, cb_msg: Imu):
        imu_msg = cb_msg
        imu_msg.linear_acceleration_covariance
        imu_msg.header.frame_id = 'vectornav'

        self.publisher.publish(imu_msg)
        self.get_logger().warn('testing 123')
        
    

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()