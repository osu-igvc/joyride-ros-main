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

def nothing(x):
    pass

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
        self.world_data = self.world_data - self.world_data[0]

        self.paused = False
        self.CALIBRATION_OFFSET = [116, 50+19, -64.5] # XYZ distance from camera to bottom left point -> Testing:  99, 43.5, -42.75 in  
        # to camera 64.5 , X: 116in Y: 50 // left/right camera Y: -/+19
        self.REMOVE_RADIUS = 10
        self.GRID_WIDTH = 8 # along y
        self.GRID_HEIGHT =  8 # along x
        # self.GRID_SPACING = 12 # in 

        self.WORLD_POINTS = []

        self.sliderWindowName = "HSV Slider"
        self.lowH = cv2.getTrackbarPos('lowH', self.sliderWindowName)
        self.owS = cv2.getTrackbarPos('lowS', self.sliderWindowName)
        self.lowV = cv2.getTrackbarPos('lowV', self.sliderWindowName)
        self.highH = cv2.getTrackbarPos('highH', self.sliderWindowName)
        self.highS = cv2.getTrackbarPos('highS', self.sliderWindowName)
        self.highV = cv2.getTrackbarPos('highV', self.sliderWindowName)

        self.minArea = cv2.getTrackbarPos('minArea', self.sliderWindowName)
        self.point_list = None

        
        # for i in range (0, self.GRID_WIDTH):
        #     for j in range(0, self.GRID_HEIGHT):
        #         self.WORLD_POINTS.append(PointXYZ(j*self.GRID_SPACING + self.CAMERA_OFFSET[0], i*self.GRID_SPACING+ self.CAMERA_OFFSET[1], -self.CAMERA_OFFSET[2]))

        print(self.world_data)
        for row in self.world_data:
            self.WORLD_POINTS.append(PointXYZ(row[0] + self.CALIBRATION_OFFSET[0], row[1] + self.CALIBRATION_OFFSET[1], row[2] + self.CALIBRATION_OFFSET[2]))

        print(self.WORLD_POINTS)

        self.image_topic = self.declare_parameter('image', '/sensors/cameras/right/image').get_parameter_value().string_value
        self.subscription = self.create_subscription(Image, self.image_topic, self.handleNewImage, 1)

        self.cv_bridge = CvBridge()
        cv2.namedWindow("Frame")
        cv2.setMouseCallback("Frame", self.mouseClick)
        

    def handleNewImage(self, msg:Image):
        #np_arr = np.fromstring(msg.data, np.uint8)
        #image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if self.paused:
            cv2.imshow("Frame", self.image_marked)
            
            if cv2.waitKey(1) == ord(' '):
                correlatedPoints = self.correlatePoints()
                self.curvefitPoints(correlatedPoints)
        else:
            image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image = cv2.flip(image,0)
            image = cv2.flip(image,1)
            image_detect = self.maskYellow(image)

            image_detect = cv2.dilate(image_detect, (1,1), iterations=2)
            image_detect = cv2.erode(image_detect, (1,1), iterations=2)
            
            point_list = self.detectYellowPoints(image_detect)
            image_marked = self.drawYellowPoints(image, point_list)
            self.image_unmarked = image
            self.image_marked = image_marked
            self.point_list = point_list
            cv2.imshow("Frame", self.image_marked)
            cv2.imshow("Mask", image_detect)
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
        
            #print('Grid points: {}, Optical points: {}'.format(len(self.WORLD_POINTS), len(self.point_list)))
            
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
        print("X HERE:\n")
        print(res_x)
        res_y = lsq_linear(A, Y)
        print("Y HERE:\n")
        print(res_y)

        self.X_params = res_x.x
        self.Y_params = res_y.x

        np.savetxt("CurveFitParams.csv",np.array([res_x.x,res_y.x]).T,delimiter=",",header="X_params, Y_params")



    def correlatePoints(self):

        if len(self.point_list) != len(self.WORLD_POINTS):
            print('Unable to correlate. Grid points: {}, Optical points: {}'.format(len(self.WORLD_POINTS), len(self.point_list)))
        else:
            # Sort points
            opticalSorted = sorted(self.point_list, key=lambda p: [p.u, p.v], reverse=False)
            #print(opticalSorted)

            groupedPoints = np.array([[p.u, p.v] for p in opticalSorted])
            #print(groupedPoints.shape)

            # TODO Remove hardcoding here
            groupedPoints = groupedPoints.reshape(int(self.GRID_WIDTH), int(self.GRID_HEIGHT),2)
            #print(groupedPoints)
            arrangedPoints = np.array([])
            for row in groupedPoints:
                
                row = row[row[:, 1].argsort()[::-1]]
                arrangedPoints = np.append(arrangedPoints, row)
            
            sortedVals = arrangedPoints.reshape(len(self.WORLD_POINTS), 2)
            print(sortedVals)

            # World is already sorted
            print(self.WORLD_POINTS)
            print(opticalSorted)

            correlatedPoints = []
            cPArray = np.empty((len(self.WORLD_POINTS), 5)) # 5-> u, v , x, y, z

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
        self.updateSliderValues()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.lowH, self.lowS, self.lowV])
        upper_yellow = np.array([self.highH, self.highS, self.highV])
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        return yellow_mask

    def detectYellowPoints(self, masked_image):
        centers = []
        contours, _ = cv2.findContours(masked_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > self.minArea:  # set a threshold for minimum blob area
                x, y, w, h = cv2.boundingRect(contour)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    newPoint = PointUV(cX, cY)
                    centers.append(newPoint)
                else:
                    cX, cY = 0, 0
        self.point_list = centers
        return centers

    def drawYellowPoints(self, image, points):
        new_image = image.copy()
        for p in points:
            cv2.circle(new_image, (p.u, p.v), radius=5, color=(0, 255, 0), thickness = -1)
            cv2.putText(new_image, "uv:({},{})".format(p.u, p.v), (p.u-20, p.v-20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 2)
        cv2.putText(new_image, "Num points: {}, {}".format(len(self.WORLD_POINTS), len(points)), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 2)
        return new_image
    
    # def uvToXYZ(self, uv):
    #     focalLength = 3.04 # in mm need to get from camera
    #     sensorWidth = 33 # in mm need to get from camera
    #     sensorHeight = 24 # in mm need to get from camera
    #     distanceToCamera = 2684.78 # in mm from hypotenuse from getting 97x42 in^2
        # angle = np.tan(42.0/97.0)

    def createSlider(self):
        cv2.namedWindow(self.sliderWindowName)
        cv2.createTrackbar('lowH', self.sliderWindowName, 41, 255, nothing)
        cv2.createTrackbar('lowS', self.sliderWindowName, 38, 255, nothing)
        cv2.createTrackbar('lowV', self.sliderWindowName, 143, 255, nothing)
        cv2.createTrackbar('highH', self.sliderWindowName, 80, 255, nothing)
        cv2.createTrackbar('highS', self.sliderWindowName, 255, 255, nothing)
        cv2.createTrackbar('highV', self.sliderWindowName, 255, 255, nothing)
        cv2.createTrackbar('minArea', self.sliderWindowName, 0, 150, nothing)

    def updateSliderValues(self):
        self.lowH = cv2.getTrackbarPos('lowH', self.sliderWindowName)
        self.lowS = cv2.getTrackbarPos('lowS', self.sliderWindowName)
        self.lowV = cv2.getTrackbarPos('lowV', self.sliderWindowName)
        self.highH = cv2.getTrackbarPos('highH', self.sliderWindowName)
        self.highS = cv2.getTrackbarPos('highS', self.sliderWindowName)
        self.highV = cv2.getTrackbarPos('highV', self.sliderWindowName)
        self.minArea = cv2.getTrackbarPos('minArea', self.sliderWindowName)

def main(args=None):
    rclpy.init(args=args)
    calib = OpticalTransformCalibrator()
    calib.createSlider()
    rclpy.spin(calib)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
