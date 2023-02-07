import cv2

from simple_pyspin import Camera

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class BlackflyNode(Node):

    def __init__(self):
        super().__init__('blackfly_publish_node')

        self.declare_parameter('serial_no', '00000000')
        self.declare_parameter('pixel_format', 'RGB8Packed')
        self.declare_parameter('video_mode', 'Mode1')


        self.serial_no = self.get_parameter('serial_no').get_parameter_value().string_value #'18295818'
        self.pixel_format = self.get_parameter('pixel_format').get_parameter_value().string_value # 'RGB8Packed'
        self.video_mode = self.get_parameter('video_mode').get_parameter_value().string_value #'Mode1'
        
        self.declare_parameter('img_pub_topic', '/cameras/bfly_{}/image_raw'.format(self.serial_no))
        self.img_pub_topic = self.get_parameter('img_pub_topic').get_parameter_value().string_value
        
        self.img_pub = self.create_publisher(Image, self.img_pub_topic, 10)

        self.cv_bridge = CvBridge()

        self.launchCamera()

    def launchCamera(self):

        # Need error handling
        with Camera(self.serial_no) as camera:
            camera.init()
            camera.__setattr__('PixelFormat', self.pixel_format)
            camera.__setattr__('VideoMode', self.video_mode)
            camera.start()

            while(True):
                img = cv2.cvtColor(camera.get_array(), cv2.COLOR_RGB2BGR)
                img_msg = self.cv_bridge.cv2_to_imgmsg(img, 'bgr8')
                self.img_pub.publish(img_msg)

def main():
    rclpy.init()
    bfly = BlackflyNode()
    rclpy.spin(bfly)
    bfly.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()