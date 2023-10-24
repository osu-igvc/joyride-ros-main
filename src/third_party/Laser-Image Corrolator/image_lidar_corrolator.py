import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from typing import List

from sklearn.cluster import KMeans

import numpy as np
import cv2

class PointCorresponder():
    def __init__(self):

        super().__init__('point_correspond_node')

        self.R = self.declare_parameter("max_detection_range", 25).get_parameter_value.double_value()
        self.L = self.declare_parameter("lidar_camera_baseline", 0.5).get_parameter_value.double_value()
        self.FOV =self.declare_parameter("FOV", np.deg2rad(53.23)).get_parameter_value.double_value() 
        self.threshold = self.declare_parameter().get_parameter_value._value()
        
        n_clusters = self.declare_parameter("clusters",5).get_parameter_value.int_value()
        laser_scan_topic = self.declare_parameter("image_topics").string_array_value()

        self.laser_scan_sub = self.create_subscriber()

        self.km = KMeans(n_clusters = n_clusters, n_init = 10, random_state = 170)
        self.θ_tilde = np.arcsin(self.L/self.R * np.sin(self.FOV))

    def window_data(self, data:np.ndarray, window: np.ndarray) -> np.ndarray:
        x, y = data[:,0], data[:,1]
        bounds = [np.argwhere(np.isclose(x, w))[0][0] for w in window[0]]
        temp = np.array([x[bounds[0]:bounds[1]], y[bounds[0]:bounds[1]]]).T
        windowed= [row for row in temp if (window[1][0]<=row[1]<=window[1][1])]
        return np.array(windowed)

    def polar_cartesian_convert(self, polar:np.array) -> np.array:
        return np.array([polar[:,1]*np.cos(polar[:,0]), polar[:,1]*np.sin(polar[:,0])]).T
    
    def points_polar(self, θ_obs: float, scan: np.ndarray,  θ_err: float = np.pi/6) -> np.array:
        windowed = self.window_data(scan, np.array([[θ_obs[0]-θ_err, θ_obs[0]+θ_err],[0,self.R]]))
        kmeans = self.km.fit(windowed)
        obs = kmeans.cluster_centers_.tolist()
        for θ in θ_obs[1:]:
            windowed = self.window_data(scan, np.array([[θ-θ_err, θ+θ_err],[0,self.R]]))
            kmeans = self.km.fit(windowed)
            clusters = kmeans.cluster_centers_
            new_clusters = [[c] for c in clusters if np.any(np.abs(obs -c) >= self.threshold)]
            obs.extend(clusters)
        return np.array(obs)