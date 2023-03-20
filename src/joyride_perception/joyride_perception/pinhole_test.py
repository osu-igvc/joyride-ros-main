
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml


from image_geometry.cameramodels import PinholeCameraModel

class ImageSubscriber(Node):

    def parseCalib(self, filename):
        stream = open(filename, 'r')
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.k = calib_data['camera_matrix']['data']
        cam_info.d = calib_data['distortion_coefficients']['data']
        cam_info.r = calib_data['rectification_matrix']['data']
        cam_info.p = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info



    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/sensors/cameras/lane/image_raw',
            self.image_callback,
            10)
        
        self.pinholeCam = PinholeCameraModel()
        self.camInfo = self.parseCalib('/home/joyride-obc/joyride-ros-main/src/joyride_core/config/lane_camera_calib.yaml')
        
        self.pinholeCam.fromCameraInfo(self.camInfo)

    def rayToGround(self, ray, camera_position):
        # calculate ray intersection with ground (z = 0)
        print('ray:{}'.format(ray))

        cX, cY, cZ = ray
        t = -camera_position[2] / cY

        real_x = cZ*t + camera_position[0]
        real_y = cX*t + camera_position[1]

        return (real_x, real_y, 0.0)
    

    def image_callback(self, msg):
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        centers = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # set a threshold for minimum blob area
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    centers.append([cX, cY])
                    cv2.circle(image, (cX, cY), radius=3, color=(0, 0, 255), thickness=-1)
                    pxRect = self.pinholeCam.rectifyPoint((cX, cY))
                    ray = self.pinholeCam.projectPixelTo3dRay(pxRect)
                    pos = self.rayToGround(ray, (0, 0, 45))
                    cv2.putText(image, "uv: ({},{})".format(cX, cY), (cX-20, cY-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                    cv2.putText(image, "xy: ({:.2f},{:.2f})".format(pos[0], pos[1]), (cX-20, cY+20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                else:
                    cX, cY = 0, 0
                




        cv2.imshow('Yellow Blobs', image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()