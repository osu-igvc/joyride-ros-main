import os
from typing import List

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path


class FunctionalTests(Node):
    def __init__(self):
        super().__init__("functional_tests")

        self.paths = self.load_paths("/home/joyride-obc/joyride-ros-main/src/igvc_competition/igvc_competition/waypoints/")
        
        self.optical_control_gains = 4.0
        direction = None
        self.behavior_sequence(direction)
    
    def load_waypoints(self,directory: str, frame: str = "base_link"):
        waypoints = {}
        if os.path.exists(directory):
            for S in [s for s in os.listdir(directory)]:
                coord = np.loadtxt(f"{directory}/{S}", delimiter=",", skiprows = 1, dtype=np.float32)
                waypoints[f'{S.split(".")[0]}'] = [self.pose_2d_to_pose_stamped(c, frame) for c in coord]
        return waypoints
                
    def pose_2d_to_pose_stamped(self,pose_2d: np.ndarray, frame: str = "base_link") -> List[PoseStamped]:
        pose = PoseStamped()
        pose.header.frame_id = frame

        pose.pose.position.x = float(pose_2d[0])
        pose.pose.position.y = float(pose_2d[1])

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0

        pose.pose.orientation.z = float(np.sin(pose_2d[2]))
        pose.pose.orientation.w = float(np.cos(pose_2d[2]))

        return pose

    def pose_sequence_to_path(self,sequence: List[PoseStamped], frame: str = "base_link"):
        path = Path()
        path.header.frame_id = frame
        path.poses = sequence
        return path
        
    def load_paths(self, directory: str):
        waypoints = self.load_waypoints(directory)
        paths = {w:self.pose_sequence_to_path(waypoints[w]) for w in waypoints}
        return paths

    def optical_lane_controller(self,err_opt: float, v: float = 1.7, K_opt: float = 0):
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = -K_opt*err_opt
        return cmd

    def behavior_sequence(self, turn: str = None):
        while True:
            self.optical_lane_controller(self.err_opt, K_opt= self.optical_control_gains)
            # publish_cmd

def main(args = None):
    rclpy.init(args=args)
    stateMachine = FunctionalTests()
    rclpy.spin(stateMachine)
    stateMachine.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()