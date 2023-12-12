# Camera Transform Calibrator
"""
Camera Transform Calibrator

This module contains the OpticalTransformCalibrator class, which is a ROS node that calibrates the transformation
between camera image points and world coordinates. It uses OpenCV for image processing and handles user input for point
correlation and calibration.

"""
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
    """
    Class representing a point in 3D space with X, Y, and Z coordinates.

    Attributes:
        x (float): X-coordinate.
        y (float): Y-coordinate.
        z (float): Z-coordinate.

    """
    def __init__(self, x, y, z):
        """
        Initializes a PointXYZ object with given X, Y, and Z coordinates.

        Parameters:
            x (float): X-coordinate.
            y (float): Y-coordinate.
            z (float): Z-coordinate.
        """
        self.x = x
        self.y = y
        self.z = z
    
    def getString(self):
        """
        Returns a formatted string representation of the PointXYZ object.

        Returns:
            str: Formatted string representation of the point (e.g., 'xyz(1.0,2.0,3.0)').
        """
        return 'xyz({},{},{})'.format(self.x, self.y, self.z)

    def __str__(self):
        """
        Returns a string representation of the PointXYZ object.

        Returns:
            str: String representation of the point.
        """
        return self.getString()
    
    def __repr__(self):
        """
        Returns a string representation of the PointXYZ object.

        Returns:
            str: String representation of the point.
        """
        return self.getString()

class PointUV():
    """
    Class representing a point in 2D space with U and V coordinates.

    Attributes:
        u (float): U-coordinate.
        v (float): V-coordinate.

    """
    def __init__(self, u, v):
        """
        Initializes a PointUV object with given U and V coordinates.

        Parameters:
            u (float): U-coordinate.
            v (float): V-coordinate.
        """
        self.u = u
        self.v = v

    def getString(self):
        """
        Returns a formatted string representation of the PointUV object.

        Returns:
            str: Formatted string representation of the point (e.g., 'uv(1.0,2.0)').
        """
        return 'uv({},{})'.format(self.u, self.v)
    
    def __str__(self):
        """
        Returns a string representation of the PointUV object.

        Returns:
            str: String representation of the point.
        """
        return self.getString()
    
    def __repr__(self):
        """
        Returns a string representation of the PointUV object.

        Returns:
            str: String representation of the point.
        """
        return self.getString()

class CorrelatedPoint():
    """
    Class representing a correlated point with optical and world coordinates.

    Attributes:
        optical (PointUV): Optical coordinates.
        world (PointXYZ): World coordinates.

    """
    def __init__(self, opticalPoint:PointUV, worldPoint:PointXYZ):
        """
        Initializes a CorrelatedPoint object with given optical and world coordinates.

        Parameters:
            opticalPoint (PointUV): Optical coordinates.
            worldPoint (PointXYZ): World coordinates.
        """
        self.optical = opticalPoint
        self.world = worldPoint

    def getString(self):
        """
        Returns a formatted string representation of the CorrelatedPoint object.

        Returns:
            str: Formatted string representation of the correlated point (e.g., 'P: uv(1.0,2.0) | xyz(3.0,4.0,5.0)').
        """
        return 'P: {} | {}'.format(self.optical.getString(), self.world.getString())
    
    def __str__(self):
        """
        Returns a string representation of the CorrelatedPoint object.

        Returns:
            str: String representation of the correlated point.
        """
        return self.getString()
    
    def __repr__(self):
        """
        Returns a string representation of the CorrelatedPoint object.

        Returns:
            str: String representation of the correlated point.
        """
        return self.getString()

def xDistanceFunction(optical_v, a, b, c, d):
    """
    Calculate the X-distance based on the optical V-coordinate using a specified function.

    Parameters:
        optical_v (float): Optical V-coordinate.
        a, b, c, d (float): Coefficients for the function.

    Returns:
        float: X-distance calculated using the function.
    """
    return a * np.exp(b*optical_v) + c*np.exp(d*optical_v)

def yDistanceFunction(OPTICAL_UV, a, b, c, d, e, f, g):
    """
    Calculate the Y-distance based on the optical U and V coordinates using a specified function.

    Parameters:
        OPTICAL_UV (tuple): Tuple containing optical U and V coordinates.
        a, b, c, d, e, f, g (float): Coefficients for the function.

    Returns:
        float: Y-distance calculated using the function.
    """
    u,v = OPTICAL_UV
    return a + b*u + c*v + d*u*v + e*np.power(v, 2) + f*u*np.power(v, 2) + g*np.power(v,3)

class OpticalTransformCalibrator(Node):
    def __init__(self):
        super().__init__('optical_transform_calibrator')
        """
        Initialize the OpticalTransformCalibrator Node.

        This method sets up parameters, calibration offsets, grid properties, and initializes variables.
        It also subscribes to the image topic and sets up a mouse callback.

        Parameters:
            None

        Returns:
            None
        """
        self.world_data_file = self.declare_parameter('data', 'test_xyz_data.csv').get_parameter_value().string_value
        self.camera_frame = self.declare_parameter("camera_frame","").get_parameter_value().string_value

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
        """
        Callback function to handle new image messages.

        This method processes the incoming image, detects yellow points, and updates the displayed image.
        If paused, it waits for a spacebar press to perform additional actions.

        Parameters:
            msg (Image): The incoming image message.

        Returns:
            None
        """
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
        """
        Callback function for mouse clicks on the image window.

        This method handles left and right mouse clicks for point removal and pausing the calibration process.

        Parameters:
            event: The type of mouse event.
            x (int): The x-coordinate of the mouse click.
            y (int): The y-coordinate of the mouse click.
            flags: Additional flags for the mouse event.
            params: Additional parameters for the callback.

        Returns:
            None
        """
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
        """
        Convert image coordinates to world coordinates after calibration.

        This method takes image coordinates (u, v) and uses the calibration parameters to compute the corresponding
        world coordinates (x, y).

        Parameters:
            u (int): The image x-coordinate.
            v (int): The image y-coordinate.

        Returns:
            Tuple[float, float]: The corresponding world coordinates (x, y).
        """
        uv_vec = np.array([1, u, u**2, u**3, v, v**2, v**3, u*v, u*v**2, u**2 * v])

        x_val = np.dot(uv_vec, self.X_params)
        y_val = np.dot(uv_vec, self.Y_params)

        return x_val, y_val




    # ----------- CV UTILITY ----------- #

    

    def curvefitPoints(self, correlatedPoints:List[CorrelatedPoint]):
        """
        Perform curve fitting on correlated points to obtain calibration parameters.

        This method takes a list of correlated points and performs curve fitting separately for X and Y coordinates.

        Parameters:
            correlatedPoints (List[CorrelatedPoint]): List of correlated points.

        Returns:
            None
        """
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

        np.savetxt(f"CurveFitParams_{self.camera_frame}.csv",np.array([res_x.x,res_y.x]).T,delimiter=",",header="X_params, Y_params")

    def correlatePoints(self):
        """
        Correlate grid points and optical points.

        This method correlates the detected optical points with the predefined grid points.

        Parameters:
            None

        Returns:
            List[CorrelatedPoint]: List of correlated points.
        """
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
            np.savetxt(f'{self.camera_frame}_correlated_points.csv', cPArray, delimiter=',', header='u, v, x, y, z')
            return correlatedPoints


    def removeNearbyPoint(self, click_u, click_v):
        """
        Remove points near the clicked point within a specified radius.

        This method removes points from the detected optical points list that are close to the clicked point.

        Parameters:
            click_u (int): The clicked point's x-coordinate.
            click_v (int): The clicked point's y-coordinate.

        Returns:
            None
        """
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
        """
        Apply a yellow color mask to the input frame.

        This method converts the input frame to the HSV color space and applies a yellow color mask.

        Parameters:
            frame: The input frame.

        Returns:
            np.ndarray: The masked image.
        """
        self.updateSliderValues()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.lowH, self.lowS, self.lowV])
        upper_yellow = np.array([self.highH, self.highS, self.highV])
        yellow_mask = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        return yellow_mask

    def detectYellowPoints(self, masked_image):
        """
        Detect yellow points in a masked image.

        This method detects yellow points in the provided masked image using contour analysis.

        Parameters:
            masked_image: The input masked image.

        Returns:
            List[PointUV]: List of detected yellow points.
        """
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
        """
        Draw yellow points on an image.

        This method draws yellow points on the input image and displays additional information.

        Parameters:
            image: The input image.
            points (List[PointUV]): List of yellow points.

        Returns:
            np.ndarray: The image with drawn yellow points.
        """
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
        """
        Create a window with sliders for adjusting color filtering parameters.

        This method creates a window with sliders for adjusting color filtering parameters such as hue, saturation, and value.

        Parameters:
            None

        Returns:
            None
        """
        cv2.namedWindow(self.sliderWindowName)
        cv2.createTrackbar('lowH', self.sliderWindowName, 41, 255, nothing)
        cv2.createTrackbar('lowS', self.sliderWindowName, 38, 255, nothing)
        cv2.createTrackbar('lowV', self.sliderWindowName, 143, 255, nothing)
        cv2.createTrackbar('highH', self.sliderWindowName, 80, 255, nothing)
        cv2.createTrackbar('highS', self.sliderWindowName, 255, 255, nothing)
        cv2.createTrackbar('highV', self.sliderWindowName, 255, 255, nothing)
        cv2.createTrackbar('minArea', self.sliderWindowName, 0, 150, nothing)

    def updateSliderValues(self):
        """
        Update color filtering parameters based on slider values.

        This method updates the color filtering parameters based on the current values of the sliders.

        Parameters:
            None

        Returns:
            None
        """
        self.lowH = cv2.getTrackbarPos('lowH', self.sliderWindowName)
        self.lowS = cv2.getTrackbarPos('lowS', self.sliderWindowName)
        self.lowV = cv2.getTrackbarPos('lowV', self.sliderWindowName)
        self.highH = cv2.getTrackbarPos('highH', self.sliderWindowName)
        self.highS = cv2.getTrackbarPos('highS', self.sliderWindowName)
        self.highV = cv2.getTrackbarPos('highV', self.sliderWindowName)
        self.minArea = cv2.getTrackbarPos('minArea', self.sliderWindowName)

def main(args=None):
    """
    Main function to initialize and run the Optical Transform Calibrator node.

    This function initializes the ROS Python client library, creates an instance of the OpticalTransformCalibrator node,
    creates a slider window, and spins the node to handle callbacks.
    """
    rclpy.init(args=args)
    calib = OpticalTransformCalibrator()
    calib.createSlider()
    rclpy.spin(calib)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
