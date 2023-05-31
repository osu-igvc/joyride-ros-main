
# Python Imports
import math

# ROS Imports
import rclpy
from rclpy.node import Node

# Message imports
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32

from joyride_interfaces.msg import EPSPositionVelocityFeedback


class VelocityPreprocessor(Node):
    
    def __init__(self):
        super().__init__('vel_preprocessor')

        self.WHEEL_BASE = 1.75 # meters
    

        self.cmdvel_sub = self.create_subscription(Twist, '/cmd_vel_simple', self.cmdvel_callback, 1)
        self.cmdack_pub = self.create_publisher(AckermannDrive, '/cmd_ack', 1)

        self.fbsteerangle_sub = self.create_subscription(EPSPositionVelocityFeedback, '/feedback/steer_angle', self.steer_angle_fb_callback, 1)

        self.prev_steer_angle = 0.0
        self.prev_control = 0.0
        self.STEER_LPF_ALPHA = 0.2
        self.STEER_KP = 1.0
        self.STEER_KD = 0.05
        self.CMD_ANG_Z_LPF = 0.1
        self.prev_ang_z = 0.0
        self.prevError = 0.0
        self.tPrev = self.get_clock().now().nanoseconds

        self.steer_angle_measured = 0.0
        self.previous_ang_acceleration = 0.0
        self.STEER_ANGLE_ACC_MAX = 0.03
        self.STEER_ANGLE_JERK_MAX = 0.01
        
        self.linear_x_lpf_alpha = 0.3
        self.prev_linear_x = 0.0
        self.LIN_X_ACC_MAX = 0.03

        self.WHEEL_BASE_ANGLE_STEERING_WHEEL_ANGLE = 25.49

    # -------------- ROS -------------- #
    def publishAckermannDrive(self, speed, steer_angle, steer_velocity):
        
        ackMsg = AckermannDrive()
        #ackMsg.header.stamp = self.get_clock().now().to_msg()
        ackMsg.steering_angle = steer_angle
        ackMsg.speed = speed
        ackMsg.steering_angle_velocity = steer_velocity

        self.cmdack_pub.publish(ackMsg)
    
    def cmdvel_callback(self, msg:Twist):
        filteredAngZ = self.CMD_ANG_Z_LPF*msg.angular.z + (1-self.CMD_ANG_Z_LPF)*self.prev_ang_z

        linX_diff = msg.linear.x - self.prev_linear_x
        linX = msg.linear.x
        if linX_diff > self.LIN_X_ACC_MAX:
            linX = self.prev_linear_x + self.LIN_X_ACC_MAX
        elif linX_diff < -self.LIN_X_ACC_MAX:
            linX = self.prev_linear_x - self.LIN_X_ACC_MAX

        filteredLinX = self.linear_x_lpf_alpha*linX + (1-self.linear_x_lpf_alpha)*linX

        self.prev_linear_x = filteredLinX
        self.prev_ang_z = filteredAngZ

        ackSpeed, ackAngle = self.computeAckermann(filteredLinX, filteredAngZ)
        ackSteerVel = self.steeringPControl(ackAngle, self.steer_angle_measured)

        # Ignore jumps
        if abs(ackAngle - self.prev_steer_angle) > 0.1 and abs(ackAngle) > 0.1:
            ackAngle = self.prev_steer_angle

        self.publishAckermannDrive(ackSpeed, ackAngle, ackSteerVel)

    def steer_angle_fb_callback(self, msg:EPSPositionVelocityFeedback):
        self.steer_angle_measured = msg.position_radians / self.WHEEL_BASE_ANGLE_STEERING_WHEEL_ANGLE

    # -------------- Utility -------------- #

    def steeringPControl(self, desiredAngle:float, measuredAngle:float):
        error = desiredAngle - measuredAngle

        tNow = self.get_clock().now().nanoseconds
        deriv = (error - self.prevError) / (tNow/1.0e9 - self.tPrev/1.0e9)
        self.prevError = error
        self.tPrev = tNow
        control = error * self.STEER_KP #+ deriv * self.STEER_KD

        control = max(-5.0, min(control, 5.0))

        newAngularAcceleration = control - self.prev_control #diff in velocity commands
        angAccDiff = newAngularAcceleration - self.previous_ang_acceleration

        if angAccDiff > self.STEER_ANGLE_JERK_MAX:
            newAngularAcceleration = self.previous_ang_acceleration + self.STEER_ANGLE_JERK_MAX
        elif angAccDiff < -self.STEER_ANGLE_JERK_MAX:
            newAngularAcceleration = self.previous_ang_acceleration - self.STEER_ANGLE_JERK_MAX

        control = self.prev_control + newAngularAcceleration
        controlDiff = control - self.prev_control
        if controlDiff > self.STEER_ANGLE_ACC_MAX:
            control = self.prev_control + self.STEER_ANGLE_ACC_MAX
        elif controlDiff < -self.STEER_ANGLE_ACC_MAX:
            control = self.prev_control - self.STEER_ANGLE_ACC_MAX

        self.previous_ang_acceleration = newAngularAcceleration
        self.prev_control = control
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