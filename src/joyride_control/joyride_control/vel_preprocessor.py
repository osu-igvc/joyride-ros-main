
# Python Imports
import math

# ROS Imports
import rclpy
from rclpy.node import Node

# Message imports
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

from joyride_interfaces.msg import EPSPositionVelocityFeedback


class VelocityPreprocessor(Node):
    
    def __init__(self):
        super().__init__('vel_preprocessor')

        self.WHEEL_BASE = 1.75 # meters
    

        self.cmdvel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmdvel_callback, 1)
        self.cmdack_pub = self.create_publisher(AckermannDriveStamped, '/cmd_ack', 1)

        self.fbsteerangle_sub = self.create_subscription(EPSPositionVelocityFeedback, '/feedback/steer_angle', self.steer_angle_fb_callback, 1)

        self.prev_steer_angle = 0.0
        self.prev_control = 0.0
        self.STEER_LPF_ALPHA = 0.7
        self.STEER_KP = 1.0

        self.steer_angle_measured = 0.0
        self.STEER_ANGLE_ACC_MAX = 0.016

        self.WHEEL_BASE_ANGLE_STEERING_WHEEL_ANGLE = 25.49

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
        self.prev_control = ackSteerVel

        self.publishAckermannDrive(ackSpeed, ackAngle, ackSteerVel)

    def steer_angle_fb_callback(self, msg:EPSPositionVelocityFeedback):
        self.steer_angle_measured = msg.position_radians / self.WHEEL_BASE_ANGLE_STEERING_WHEEL_ANGLE

    # -------------- Utility -------------- #

    def steeringPControl(self, desiredAngle:float, measuredAngle:float):
        error = desiredAngle - measuredAngle

        control = error * self.STEER_KP

        control = max(-5.0, min(control, 5.0))

        self.get_logger().error('CTRL: {}, PREV: {}'.format(control, self.prev_control))
        if control - self.prev_control > self.STEER_ANGLE_ACC_MAX:
            control = self.prev_control + self.STEER_ANGLE_ACC_MAX
        elif control - self.prev_control < -self.STEER_ANGLE_ACC_MAX:
            control = self.prev_control - self.STEER_ANGLE_ACC_MAX

        return control

    def computeAckermann(self, linearX:float, angularZ:float) -> float:
        if abs(linearX) < 0.01:
            return 0.0, 0.0
        else:

            phi = math.atan(angularZ * self.WHEEL_BASE / linearX)
            
            steer_ang = phi

            steer_ang = max(-8.0 / self.WHEEL_BASE_ANGLE_STEERING_WHEEL_ANGLE, min(steer_ang, 8.0 / self.WHEEL_BASE_ANGLE_STEERING_WHEEL_ANGLE))

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