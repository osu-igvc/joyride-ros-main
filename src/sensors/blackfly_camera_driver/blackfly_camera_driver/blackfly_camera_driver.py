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
import diagnostic_updater

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
                self.status.message = ''

        except:
            self.status.level = DiagnosticStatus.ERROR
            self.status.message = 'Error getting frame'


    def new_diagnostics(self, stat):
        stat.summary(self.status.level, self.status.message)
        return stat

    def set_updater(self, updater):
        self.updater = updater

    # --------------------- Lifecycle Management --------------------- #

    def on_configure(self, state: State) -> TransitionCallbackReturn:


        # Parameters
        self.serial_no = self.declare_parameter('serial_no', '18295828').get_parameter_value().string_value
        self.image_raw_publish_topic = self.declare_parameter('raw_image_topic', '/bfly_{}/image_raw'.format(self.serial_no)).get_parameter_value().string_value
        self.video_mode = self.declare_parameter('video_mode','Mode1').get_parameter_value().string_value
        self.pixel_format = self.declare_parameter('pixel_format', 'RGB8Packed').get_parameter_value().string_value
        self.frequency = self.declare_parameter('frequency', 100.0).get_parameter_value().double_value
        self.compress_image = self.declare_parameter('compress_image', False).get_parameter_value().bool_value


        # Pub/Sub/Timers
        self.image_publisher = self.create_lifecycle_publisher(Image, self.image_raw_publish_topic, 10)

        if self.compress_image:
            self.image_compressed_publisher = self.create_lifecycle_publisher(CompressedImage, f'{self.image_raw_publish_topic}/compressed', 10)

        self.status = DiagnosticStatus()
        self.status.level = DiagnosticStatus.STALE
        self.updater.setHardwareID(self.serial_no)

        try:

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
            self.status.level = DiagnosticStatus.ERROR
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

    updater = diagnostic_updater.Updater(bfly_driver_node)
    updater.setHardwareID('none')
    bfly_driver_node.set_updater(updater)
    updater.add('', bfly_driver_node.new_diagnostics)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        bfly_driver_node.destroy_node()


if __name__ == '__main__':
    main()