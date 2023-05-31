# -------------------------------------------------------------------------
# This file defines a class for pose control for a TurtleBot3 and a ROS
# node. The center of the robot is moved and rotated to the desired pose.
# The desired pose is obtained from the topic /goal2D.
#
# Node:
#     /turtlebot3_pose_control
# Subscribed topics:
#     /odom, /goal2D
# Published topics:
#     /cmd_vel
# Parameters created on the parameter server:
#     /turtlebot3_pose_control/velocity_limits
#     /turtlebot3_pose_control/control_gains
#     /turtlebot3_pose_control/proximity
# -------------------------------------------------------------------------
# Start with some library imports
from typing import List

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Pose, Pose2D
from nav_msgs.msg import Odometry, Path

class SimpleGoToPose(Node):
    def __init__(self):
        super().__init__("simple_pose_controller")
        
        self.control_gains = self.declare_parameter("control_gains", [1.0,0.75,0.0]).get_parameter_value().double_array_value
        self.proximity = self.declare_parameter("proximity", 1.5).get_parameter_value().double_value
        self.L = self.declare_parameter("wheelbase", 1.75).get_parameter_value().double_value
        self.v_max = self.declare_parameter("maximum linear speed", 2.0).get_parameter_value().double_value
        self.ω_max = self.declare_parameter("maximum angular speed", np.pi/2).get_parameter_value().double_value

        self.odomSub = self.create_subscription(Odometry, '/odom', self.odometry_cb, 10)
        self.pathSub = self.create_subscription(Path, '/plan', self.path_cb, 10)
        
        self.controlTimer = self.create_timer(0.01,self.controller_cb)
        
        self.cmd_vel_pub = self.create_publisher(Twist,'/cmd_vel_simple', 1)
        self.cmd_vel_msg = Twist()

        self.path_poses = []

        self.current_pose = Pose2D()
        self.goal = Pose2D()

        self.control_gains = np.array(self.control_gains)
    
    def odometry_cb(self, odom: Odometry)->None:
        quat = odom.pose.pose.orientation
        R = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        
        self.current_pose.x = odom.pose.pose.position.x
        self.current_pose.y = odom.pose.pose.position.y
        self.current_pose.theta = R.as_euler("zyx",degrees=False)[0]

        self.controller()
    
    def path_cb(self, msg: Path) -> None:
        self.path_poses = msg.poses
    
    def controller_cb(self) -> None:

        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def increment_goal(self) -> None:
        if len(self.path_poses) > 0:
            goal = self.path_poses.pop(0).pose

            q = goal.orientation
            R = Rotation.from_quat([q.x, q.y, q.z, q.w])

            self.goal.x = goal.position.x
            self.goal.y = goal.position.y
            self.goal.theta = R.as_euler("zyx",degrees=False)[0]
        
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0


    def controller(self) -> None:

        ρ, α, β = self.tracking_errors(self.current_pose, self.goal)

        if ρ < self.proximity:
            self.increment_goal()
            pass

        if abs(α) > np.pi/2:
            multiplier = -1
            if α > 0:
                α = self.wrapAngles(np.pi-α)
            else:
                α = self.wrapAngles(-np.pi-α)
            if β > 0:
                β = self.wrapAngles(np.pi-β)
            else:
                β = self.wrapAngles(-np.pi-β)
        else:
            multiplier = 1
            
        v = multiplier*self.control_gains[0]*ρ
        ω = multiplier*self.control_gains[1]*α + multiplier*self.control_gains[2]*β

        self.cmd_vel_msg.linear.x = min(self.v_max, max(-self.v_max, v))
        self.cmd_vel_msg.angular.z = min(self.ω_max, max(-self.ω_max, ω)) 
    
    def tracking_errors(self, current: Pose2D, goal: Pose2D):
        ρ = np.hypot(goal.x - current.x, goal.y - current.y)
        α = self.wrapAngles(np.arctan2(goal.y - current.y ,goal.x - current.x) - current.theta)
        β = self.wrapAngles(α + current.theta - goal.theta)
        return ρ, α, β
        
    def wrapAngles(self,angle: float) -> float:
        angle = np.mod(angle+np.pi,2*np.pi)
        if angle < 0:
            angle = angle + 2*np.pi
        return angle - np.pi
    
    def stopControl(self) -> Twist:
        self._cmd = Twist()

def main(args = None):
    rclpy.init(args=args)
    controller = SimpleGoToPose()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()