import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageSubscriber(Node):
    def __init__(self):


        filePath = "/home/joyride-obc/joyride-ros-main/CurveFitParams_center.csv"

        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/sensors/cameras/center/image',
            self.image_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        #self.coeffs_x, self.coeffs_y = np.loadtxt(filePath, unpack = True, delimiter=",", skiprows=1)
        cv2.namedWindow("image")
        cv2.setMouseCallback('image',self.get_coordinates)

    def get_coordinates(self,event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            print('({}, {})'.format(x, y))

    def maskYellow(self, frame):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([41, 38, 143])
        upper_yellow = np.array([80, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        return yellow_mask

    def detectYellowPoints(self, masked_image):
        centers = []
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 0:  # set a threshold for minimum blob area
                x, y, w, h = cv2.boundingRect(contour)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    centers.append([cX,cY])
                else:
                    cX, cY = 0, 0
        return centers

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            cv_image = cv2.flip(cv_image, 0)
            cv_image = cv2.flip(cv_image, 1)
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return
        
        # cv_image = cv2.flip(cv_image, 0)
        # cv_image = cv2.flip(cv_image, 1)

        # masked = self.maskYellow(cv_image)
        # centers = self.detectYellowPoints(masked)

        # for p in centers:
        #     u, v = p[0], p[1]
        #     R = np.array([1, u, u**2, u**3, v, v**2, v**3, u*v, u*v**2, u**2*v ])
        #     x = np.dot(self.coeffs_x, R)
        #     y = np.dot(self.coeffs_y, R)
        #     z = 0
        #     cv2.circle(cv_image, (u, v), radius=5, color=(0, 255, 0), thickness = -1)
        #     cv2.putText(cv_image, f"xyz({x},{y},{z})", (u-20, v-20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)
            

        # # Get height and width of camera space
        # height, width, _ = cv_image.shape

        # size = 16
        # # Set square size based off line size
        # square_size = width // size
        # # Get Start position
        # start_x = (width - square_size * size) // 2
        # start_y = (height - square_size * size) // 2
        # for i in range(size + 1):
        #     x = start_x + i * square_size
        #     y = start_y + i * square_size
        #     cv2.line(cv_image, (x, start_y), (x, start_y + square_size * size), (0, 0, 255), 2)
        #     cv2.line(cv_image, (start_x, y), (start_x + square_size * size, y), (0, 0, 255), 2) 

        # # Show center lines in Green
        # cv2.line(cv_image, (width // 2, 0), (width // 2, height - 1), (0, 255, 0), 2)  # Vertical line
        # cv2.line(cv_image, (0, height // 2), (width - 1, height // 2), (0, 255, 0), 2)  # Horizontal line

     
        # height,width=cv_image.shape[:2]
        # print(height,",",width)
        
        cv2.imshow('image', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
