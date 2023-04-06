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

from sensor_msgs.msg import Image

import cv2
from simple_pyspin import Camera
from cv_bridge import CvBridge


class LifecycleBlackflyCameraDriver(Node):
    def __init__(self, node_name, **kwargs):

        self.image_publisher: Optional[Publisher] = None
        self.image_publisher_timer: Optional[Timer] = None

        self.diagnostic_publisher: Optional[Publisher] = None
        self.diagnostic_publisher_timer: Optional[Timer] = None

        self.cv_bridge = CvBridge()


        super().__init__(node_name, **kwargs)

    def new_image(self):
        if self.bfly_camera is not None:
            img = cv2.cvtColor(self.bfly_camera.get_array(), cv2.COLOR_RGB2BGR)
            img_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
            self.image_publisher.publish(img_msg)
    

    # --------------------- Lifecycle Management --------------------- #

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        
        #self.image_raw_publish_topic = self.declare_parameter('raw_image_topic', '')
        self.video_mode = 'Mode1'
        self.pixel_format = 'RGB8Packed'
        self.serial_no = self.declare_parameter('serial_no', '18295828').get_parameter_value().string_value

        self.image_publisher = self.create_lifecycle_publisher(Image, '/image_raw', 10)
        self.image_publisher_timer = self.create_timer(1/100.0, self.new_image)

        try:
            self.bfly_camera = Camera(self.serial_no)
            self.bfly_camera.init()
            self.bfly_camera.__setattr__('PixelFormat', self.pixel_format)
            self.bfly_camera.__setattr__('VideoMode', self.video_mode)
            self.bfly_camera.start()

            self.get_logger().info(f'Successfully opened camera: {self.serial_no}')

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
        return super().on_cleanup(state)
    
    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.image_publisher_timer)
        self.destroy_publisher(self.image_publisher)
        return super().on_shutdown(state)


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