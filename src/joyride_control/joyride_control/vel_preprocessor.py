
# Python Imports
import math

# ROS Imports
import rclpy
from rclpy.node import Node

# Message imports
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32


class VelocityPreprocessor(Node):
    
    def __init__(self):
        super().__init__('vel_preprocessor')

        self.WHEEL_BASE = 1.75 # meters
    

        self.cmdvel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmdvel_callback, 1)
        self.cmdack_pub = self.create_publisher(AckermannDriveStamped, '/cmd_ack', 1)

        self.fbsteerangle_sub = self.create_subscription(Float32, '/feedback/steer_angle', self.steer_angle_fb_callback, 1)

        self.prev_steer_angle = 0.0
        self.STEER_LPF_ALPHA = 0.7
        self.STEER_KP = 1.0

        self.steer_angle_measured = 0.0

    # -------------- ROS -------------- #
    def publishAckermannDrive(self, speed, steer_angle, steer_velocity):
        
        ackMsg = AckermannDriveStamped()
        ackMsg.header.stamp = self.get_clock().now().to_msg()
        ackMsg.drive.steering_angle = steer_angle
        ackMsg.drive.speed = speed
        ackMsg.drive.steering_angle_velocity = steer_velocity

        self.cmdack_pub.publish(ackMsg)
    
    def cmdvel_callback(self, msg:Twist):
        
        ackSpeed, ackAngle = self.computeAckermann(msg.linear.x, msg.angular.z)
        ackSteerVel = self.steeringPControl(ackAngle, self.steer_angle_measured)

        self.publishAckermannDrive(ackSpeed, ackAngle, ackSteerVel)

    def steer_angle_fb_callback(self, msg:Float32):
        self.steer_angle_measured = msg.data

    # -------------- Utility -------------- #

    def steeringPControl(self, desiredAngle:float, measuredAngle:float):
        error = desiredAngle - measuredAngle

        control = error * self.STEER_KP

        control = max(-5.0, min(control, 5.0))

        return control

    def computeAckermann(self, linearX:float, angularZ:float) -> float:
        if abs(linearX) < 0.01:
            return 0.0, 0.0
        else:

            # This is a temporary change before a bettery velocity controller is implemented

            if linearX > 0.1 and linearX < 1.5:
                linearX = 1.5

            phi = math.atan(angularZ * self.WHEEL_BASE / linearX)
            
            steer_ang = phi * 25.49

            steer_ang = max(-8.0, min(steer_ang, 8.0))

            steer_ang = self.STEER_LPF_ALPHA * steer_ang + (1 - self.STEER_LPF_ALPHA) * self.prev_steer_angle
            self.prev_steer_angle = steer_ang

            

            return linearX, steer_ang





def main():
    rclpy.init()
    velPP = VelocityPreprocessor()
    rclpy.spin(velPP)
    velPP.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()