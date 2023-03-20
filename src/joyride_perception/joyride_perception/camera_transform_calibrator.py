# Camera Transform Calibrator

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import List
import cv2
import csv
import numpy as np
from scipy.optimize import curve_fit, lsq_linear
import math
import operator

# Given X,Y positions of a grid of squares
# - Assume Z = 0
# Given XYZ of camera with respect to "bottom left" point
# Find each colored point in the frame.
# - Take care to handle false positives
# - Allow "clicking" on image to remove false positives.
# Match image points to XY positions.
# - Display correlated points
# Save CSV of correlated points
# Fit mapping to points
# - Implement validation step?

class PointXYZ():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
    
    def getString(self):
        return 'xyz({},{},{})'.format(self.x, self.y, self.z)

    def __str__(self):
        return self.getString()
    def __repr__(self):
        return self.getString()

class PointUV():
    def __init__(self, u, v):
        self.u = u
        self.v = v

    def getString(self):
        return 'uv({},{})'.format(self.u, self.v)
    
    def __str__(self):
        return self.getString()
    def __repr__(self):
        return self.getString()

class CorrelatedPoint():
    def __init__(self, opticalPoint:PointUV, worldPoint:PointXYZ):
        self.optical = opticalPoint
        self.world = worldPoint

    def getString(self):
        return 'P: {} | {}'.format(self.optical.getString(), self.world.getString())
    def __str__(self):
        return self.getString()
    def __repr__(self):
        return self.getString()

def xDistanceFunction(optical_v, a, b, c, d):
    return a * np.exp(b*optical_v) + c*np.exp(d*optical_v)

def yDistanceFunction(OPTICAL_UV, a, b, c, d, e, f, g):
    u,v = OPTICAL_UV
    return a + b*u + c*v + d*u*v + e*np.power(v, 2) + f*u*np.power(v, 2) + g*np.power(v,3)

class OpticalTransformCalibrator(Node):
    def __init__(self):
        super().__init__('optical_transform_calibrator')
        self.world_data_file = self.declare_parameter('data', 'test_xyz_data.csv').get_parameter_value().string_value

        self.world_data = np.loadtxt(self.world_data_file, delimiter=',', dtype=float)

        self.paused = False
        self.CALIBRATION_OFFSET = [70, 22, -20] # XYZ distance from camera to bottom left point
        self.REMOVE_RADIUS = 10
        self.GRID_WIDTH = 3
        self.GRID_HEIGHT = 3
        self.GRID_SPACING = 1 #9*2.54 # cm

        self.WORLD_POINTS = []
        
        # for i in range (0, self.GRID_WIDTH):
        #     for j in range(0, self.GRID_HEIGHT):
        #         self.WORLD_POINTS.append(PointXYZ(j*self.GRID_SPACING + self.CAMERA_OFFSET[0], i*self.GRID_SPACING+ self.CAMERA_OFFSET[1], -self.CAMERA_OFFSET[2]))

        print(self.world_data)
        for row in self.world_data:
            self.WORLD_POINTS.append(PointXYZ(row[0] + self.CALIBRATION_OFFSET[0], row[1] + self.CALIBRATION_OFFSET[1], row[2] + self.CALIBRATION_OFFSET[2]))

        print(self.WORLD_POINTS)

        self.image_topic = self.declare_parameter('image', 'image_raw').get_parameter_value().string_value
        self.subscription = self.create_subscription(Image, self.image_topic, self.handleNewImage, 1)

        self.cv_bridge = CvBridge()
        cv2.namedWindow("Frame")
        cv2.setMouseCallback("Frame", self.mouseClick)
        

    def handleNewImage(self, msg:Image):
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_detect = self.maskYellow(image)
            
            point_list = self.detectYellowPoints(image_detect)
            image_marked = self.drawYellowPoints(image, point_list)
            
            if self.paused:
                cv2.imshow("Frame", self.image_marked)
                
                if cv2.waitKey(1) == ord(' '):
                    correlatedPoints = self.correlatePoints()
                    self.curvefitPoints(correlatedPoints)
            else:
                self.image_unmarked = image
                self.image_marked = image_marked
                self.point_list = point_list
                cv2.imshow("Frame", self.image_marked)
                cv2.waitKey(1)

    # ----------- GUI ----------- #
    def mouseClick(self, event, x, y, flags, params):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(x, ' ', y)

            if self.paused:
                self.removeNearbyPoint(x, y)

                if self.X_params is not None and self.Y_params is not None:
                    clickX, clickY = self.clickToPointAfterCalibrated(x, y)
                
                print('X: {}, Y: {}'.format(clickX, clickY))


        if event == cv2.EVENT_RBUTTONDOWN:
            if self.paused:
                self.paused = False
            else:
                self.paused = True
            
        pass

    def clickToPointAfterCalibrated(self, u, v):
        uv_vec = np.array([1, u, u**2, u**3, v, v**2, v**3, u*v, u*v**2, u**2 * v])

        x_val = np.dot(uv_vec, self.X_params)
        y_val = np.dot(uv_vec, self.Y_params)

        return x_val, y_val




    # ----------- CV UTILITY ----------- #

    

    def curvefitPoints(self, correlatedPoints:List[CorrelatedPoint]):

        U = np.array([p.optical.u for p in correlatedPoints])
        V = np.array([p.optical.v for p in correlatedPoints])
        X = np.array([p.world.x for p in correlatedPoints])
        Y = np.array([p.world.y for p in correlatedPoints])

        A = np.transpose([np.ones_like(U), U, U**2, U**3, V, V**2, V**3, U*V, U*V**2, U**2*V])
        res_x = lsq_linear(A, X)
        print(res_x)
        res_y = lsq_linear(A, Y)
        print(res_y)

        self.X_params = res_x.x
        self.Y_params = res_y.x



    def correlatePoints(self):

        if len(self.point_list) != len(self.WORLD_POINTS):
            print('Unable to correlate. Grid points: {}, Optical points: {}'.format(len(self.WORLD_POINTS), len(self.point_list)))
        else:
            # Sort points
            opticalSorted = sorted(self.point_list, key=lambda p: [p.u, p.v], reverse=False)
            #print(opticalSorted)

            groupedPoints = np.array([[p.u, p.v] for p in opticalSorted])
            #print(groupedPoints)

            # TODO Remove hardcoding here
            groupedPoints = groupedPoints.reshape(3,3,2)
            #print(groupedPoints)
            arrangedPoints = np.array([])
            for row in groupedPoints:
                
                row = row[row[:, 1].argsort()[::-1]]
                arrangedPoints = np.append(arrangedPoints, row)
            
            sortedVals = arrangedPoints.reshape(9, 2)
            print(sortedVals)

            # World is already sorted
            print(self.WORLD_POINTS)
            print(opticalSorted)

            correlatedPoints = []
            cPArray = np.empty((len(self.WORLD_POINTS), 5))

            for i in range(0, len(self.WORLD_POINTS)):
                newPoint = CorrelatedPoint(PointUV(sortedVals[i][0], sortedVals[i][1]), self.WORLD_POINTS[i])
                correlatedPoints.append(newPoint)
                cPArray[i] = [newPoint.optical.u, newPoint.optical.v, newPoint.world.x, newPoint.world.y, newPoint.world.z]
            
            print(*correlatedPoints, sep='\n')
            np.savetxt('calibration.csv', cPArray, delimiter=',', header='u, v, x, y, z')
            return correlatedPoints


    def removeNearbyPoint(self, click_u, click_v):
        if self.point_list is None:
            return

        newList = []
        for p in self.point_list:
            # Don't add point if it's close to mouse click
            if not (np.hypot(click_u - p.u, click_v - p.v) < self.REMOVE_RADIUS):
                newList.append(p)
            else:
                print('Removing point: {},{}'.format(p.u, p.v))
        self.point_list = newList
        self.image_marked = self.drawYellowPoints(self.image_unmarked, self.point_list)
    
    def maskYellow(self, frame):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        return yellow_mask

    def detectYellowPoints(self, masked_image):
        centers = []
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # set a threshold for minimum blob area
                x, y, w, h = cv2.boundingRect(contour)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    newPoint = PointUV(cX, cY)
                    centers.append(newPoint)
                else:
                    cX, cY = 0, 0
        
        return centers

    def drawYellowPoints(self, image, points):
        new_image = image.copy()
        for p in points:
            cv2.circle(new_image, (p.u, p.v), radius=5, color=(0, 255, 0), thickness = -1)
            cv2.putText(new_image, "uv:({},{})".format(p.u, p.v), (p.u-20, p.v-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return new_image


def main(args=None):
    rclpy.init(args=args)
    calib = OpticalTransformCalibrator()
    rclpy.spin(calib)
    rclpy.shutdown()

if __name__ == '__main__':
    main()