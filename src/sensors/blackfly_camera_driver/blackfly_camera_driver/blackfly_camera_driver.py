# Blackfly camera driver lifecycle node
# Author: Max DeSantis


# Startup
# - Grab all parameters
# - Setup diagnostic publisher, image publisher
# - Attempt to open camera

# Running
# - Publish camera at set frequency (or full speed)

# Shutdown
# - Close camera

from typing import Optional

# ROS
import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from sensor_msgs.msg import Image, CompressedImage
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import cv2
from simple_pyspin import Camera
from PySpin import SpinnakerException
from cv_bridge import CvBridge
import numpy as np


class LifecycleBlackflyCameraDriver(Node):
    def __init__(self, node_name, **kwargs):
        
        self.bfly_camera: Optional[Camera] = None

        self.image_publisher: Optional[Publisher] = None
        self.image_publisher_timer: Optional[Timer] = None
        self.image_compressed_publisher: Optional[Publisher] = None

        self.diagnostic_publisher: Optional[Publisher] = None
        self.diagnostic_publisher_timer: Optional[Timer] = None

        self.cv_bridge = CvBridge()


        super().__init__(node_name, **kwargs)

    def new_image(self):
        try:
            if self.bfly_camera is not None:
                img = cv2.cvtColor(self.bfly_camera.get_array(), cv2.COLOR_RGB2BGR)
                img_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
                self.image_publisher.publish(img_msg)

                if self.compress_image:
                    msg = CompressedImage()
                    msg.format = 'jpeg'
                    msg.data = np.array(cv2.imencode('.jpg', img)[1]).tostring()
                    self.image_compressed_publisher.publish(msg)
                self.status.level = DiagnosticStatus.OK

        except:
            self.get_logger().error(f'Camera {self.serial_no} encounted an error when grabbing frame.')
            self.status.level = DiagnosticStatus.ERROR


    def new_diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status.append(self.status)
        self.diagnostic_publisher.publish(msg)

    # --------------------- Lifecycle Management --------------------- #

    def on_configure(self, state: State) -> TransitionCallbackReturn:

        self.get_logger().info(f'Configuring camera...')

        # Parameters
        self.serial_no = self.declare_parameter('serial_no', '18295828').get_parameter_value().string_value
        self.image_raw_publish_topic = self.declare_parameter('raw_image_topic', '/bfly_{}/image_raw'.format(self.serial_no)).get_parameter_value().string_value
        self.video_mode = self.declare_parameter('video_mode','Mode1').get_parameter_value().string_value
        self.pixel_format = self.declare_parameter('pixel_format', 'RGB8Packed').get_parameter_value().string_value
        self.frequency = self.declare_parameter('frequency', 100.0).get_parameter_value().double_value
        self.compress_image = self.declare_parameter('compress_image', False).get_parameter_value().bool_value

        self.diagnostics_publish_topic = '/diagnostics'

        self.get_logger().info(f'T camera...')
        self.get_logger().info(f'Compress: {self.compress_image}')
        # Pub/Sub/Timers
        self.image_publisher = self.create_lifecycle_publisher(Image, self.image_raw_publish_topic, 10)
        self.get_logger().info(f'Test camera: {self.serial_no}')

        if self.compress_image:
            self.get_logger().info('COMPRESSING')
            self.image_compressed_publisher = self.create_lifecycle_publisher(CompressedImage, f'{self.image_raw_publish_topic}/compressed', 10)


        try:
            self.diagnostic_publisher = self.create_lifecycle_publisher(DiagnosticArray, self.diagnostics_publish_topic, 10)
            self.diagnostic_publisher_timer = self.create_timer(1.0, self.new_diagnostics)
            self.status = DiagnosticStatus(name=self.get_name())
            self.status.level = DiagnosticStatus.OK

            self.get_logger().info(f'Opening camera: {self.serial_no}')

            self.bfly_camera = Camera(self.serial_no)
            if self.bfly_camera.running:
                self.bfly_camera.close()

            self.bfly_camera.init()
            self.bfly_camera.__setattr__('PixelFormat', self.pixel_format)
            self.bfly_camera.__setattr__('VideoMode', self.video_mode)
            self.bfly_camera.start()

            self.get_logger().info(f'Successfully opened camera: {self.serial_no}')
            self.image_publisher_timer = self.create_timer(1.0/self.frequency, self.new_image)

            return TransitionCallbackReturn.SUCCESS
        except:
            self.get_logger().error(f'Unable to open camera: {self.serial_no}')
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        return super().on_activate(state)
    
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # No mo' publishin
        return super().on_deactivate(state)
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.image_publisher_timer)
        self.destroy_publisher(self.image_publisher)
        self.destroy_timer(self.diagnostic_publisher_timer)
        self.destroy_publisher(self.diagnostic_publisher)
        self.bfly_camera.stop()
        self.bfly_camera.close()
        self.bfly_camera = None

        self.undeclare_parameter('serial_no')
        self.undeclare_parameter('raw_image_topic')
        self.undeclare_parameter('video_mode')
        self.undeclare_parameter('pixel_format')
        self.undeclare_parameter('frequency')
        

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.image_publisher_timer)
        self.destroy_publisher(self.image_publisher)
        self.destroy_timer(self.diagnostic_publisher_timer)
        self.destroy_publisher(self.diagnostic_publisher)
        self.bfly_camera.stop()
        self.bfly_camera.close()
        self.bfly_camera = None

        return TransitionCallbackReturn.SUCCESS


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()

    bfly_driver_node = LifecycleBlackflyCameraDriver('bfly_driver')

    executor.add_node(bfly_driver_node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        bfly_driver_node.destroy_node()


if __name__ == '__main__':
    main()