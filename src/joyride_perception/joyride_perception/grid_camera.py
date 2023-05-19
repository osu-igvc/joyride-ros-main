import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/sensors/cameras/center/image',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return

        # Get height and width of camera space
        height, width, _ = cv_image.shape

        size = 16
        # Set square size based off line size
        square_size = width // size
        # Get Start position
        start_x = (width - square_size * size) // 2
        start_y = (height - square_size * size) // 2
        for i in range(size + 1):
            x = start_x + i * square_size
            y = start_y + i * square_size
            cv2.line(cv_image, (x, start_y), (x, start_y + square_size * size), (0, 0, 255), 2)
            cv2.line(cv_image, (start_x, y), (start_x + square_size * size, y), (0, 0, 255), 2) 

        # Show center lines in Green
        cv2.line(cv_image, (width // 2, 0), (width // 2, height - 1), (0, 255, 0), 2)  # Vertical line
        cv2.line(cv_image, (0, height // 2), (width - 1, height // 2), (0, 255, 0), 2)  # Horizontal line

        cv2.imshow('Image with Lines', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
