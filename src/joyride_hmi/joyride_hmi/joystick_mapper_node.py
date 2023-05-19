# General
from math import pi
from enum import Enum
from urllib.request import Request

from requests import request

# ROS
import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.action import ActionClient

# PyGame - Joystick
from pygame.locals import *

# Custom
from joyride_interfaces.action import RequestEnableDisable
from joyride_interfaces.action import RequestAudioVisual



class JoystickSub(Node):

    class Buttons(Enum):
        TRIANGLE = 2
        OPTIONS = 9
        LEFT_BUMPER = 4
        RIGHT_BUMPER = 5
        LEFT_STICK = 11
        RIGHT_STICK = 12

    class Axes(Enum):
        LEFT_STICK_HORIZONTAL = 0
        LEFT_STICK_VERTICAL = 1
        LEFT_TRIGGER = 2
        RIGHT_STICK_VERTICAL = 4
        RIGHT_STICK_HORIZONTAL = 3
        RIGHT_TRIGGER = 5


    def __init__(self):
        super().__init__('joy_sub')     # Initilize the node with proper name (does not have to be the same as the class name)
        # Creating a subscriber that outputs joystick messages from publisher into /joy_input using the listener function. 
        # The limit for queued messgaes is still 1 
        self.subscription = self.create_subscription(Joy, '/joystick_raw', self.map_input_cb, 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.cmd_vel = Twist()

        self.fb_wheelspeed_pub = self.create_publisher(Float32, '/joy/wheelspeed', 1)
        self.fb_steerangle_pub = self.create_publisher(Float32, '/joy/steer_angle', 1)

        # Publish new commands at 100Hz
        self.publishTimer = self.create_timer(1.0/100.0, self.publishCMDVel)

        # Safety Enabling
        self.enable_status_sub = self.create_subscription(Bool, '/health/enable_status', self.systemEnableCallback, 1)
        self.enable_server_client = ActionClient(self, RequestEnableDisable, 'SystemEnable')
        self._systemEnableStatus = False
        self.ENABLE_AUTONOMY_BUTTON = self.Buttons.OPTIONS.value # Triangle on PS4 Controller
        self.enable_debounce_period = .3E9 # in nanoseconds
        self.enable_prev_time = 0

        # Audiovisual
        self.FRONT_LEFT_BLINKER_BUTTON = self.Buttons.LEFT_BUMPER.value
        self.FRONT_RIGHT_BLINKER_BUTTON = self.Buttons.RIGHT_BUMPER.value
        self.BUZZER_BUTTON = self.Buttons.RIGHT_STICK.value
        self.audiovisual_server_client = ActionClient(self, RequestAudioVisual, 'SystemAudioVisual')
        self.front_left_blinker_state = False
        self.front_right_blinker_state = False

        # Mapping Constants
        self.VEL_LIN_MAX = 1.5
        self.VEL_LIN_MIN = -1.5
        self.VEL_LPF_ALPHA = 0.3
        self.VEL_DEADZONE = 0.1
        self.vel_lin_prev = 0


        self.VEL_ANG_MAX = 0.585 # (2.3m/s)/(1.75m wheelbase) * tan(24deg max wheel angle) = angZ
        self.VEL_ANG_MIN = -0.585 
        self.VEL_ANG_LPF_ALPHA = 0.7
        self.VEL_ANG_DEADZONE = 0.05
        self.vel_ang_prev = 0

        # +V is down on analog stick. -V is up.
        self.JOY_V_MAX = 1
        self.JOY_V_MIN = -1
        self.JOY_V_AXIS = self.Axes.RIGHT_STICK_VERTICAL.value # VELOCITY AXIS
        self.JOY_V_DEADZONE = 0.05

        # +H is right on analog stick. -H is left.
        self.JOY_H_MAX = 1
        self.JOY_H_MIN = -1
        self.JOY_H_AXIS = self.Axes.LEFT_STICK_HORIZONTAL.value# STEERING AXIS
        self.JOY_H_DEADZONE = 0.15


    def systemEnableCallback(self, msg:Bool):
        self._systemEnableStatus = msg.data

    def requestSystemEnableDisable(self):
        
        # Debounce enable requests
        if self.get_clock().now().nanoseconds - self.enable_prev_time < self.enable_debounce_period:
            return

        self.enable_prev_time = self.get_clock().now().nanoseconds
        enable_msg = RequestEnableDisable.Goal()
        enable_msg.set_enabled = not self._systemEnableStatus
        self.enable_server_client.send_goal_async(enable_msg)

           

    def publishCMDVel(self):
        self.cmd_vel_publisher.publish(self.cmd_vel)
        
        fb_wheelspeed = Float32()
        fb_wheelspeed.data = self.cmd_vel.linear.x
        self.fb_wheelspeed_pub.publish(fb_wheelspeed)

        fb_steer = Float32()
        fb_steer.data = 0.0
        self.fb_steerangle_pub.publish(fb_steer)

    def updateCMDVel(self, linearX: float, angularZ: float):
        self.cmd_vel.linear.x = float(linearX)
        self.cmd_vel.angular.z = float(angularZ)

        #self.cmd_vel.header.stamp = self.get_clock().now().to_msg()
        #self.cmd_vel.header.frame_id = 'base_link'


    def map_input_cb(self, msg: Joy):
        self.msgParse_AutonomyEnableButton(msg)
        self.msgParse_AudioVisual(msg)
        self.msgParse_MotionAxes(msg)
        
    def msgParse_AudioVisual(self, msg:Joy):
        requestAV = RequestAudioVisual.Goal()

        if msg.buttons[self.BUZZER_BUTTON]:
            requestAV.activate_buzzer = True
        else:
            requestAV.activate_buzzer = False

        
        if msg.buttons[self.FRONT_LEFT_BLINKER_BUTTON]:
            requestAV.toggle_front_left_blinker = True
        else:
            requestAV.toggle_front_left_blinker = False
        
        if msg.buttons[self.FRONT_RIGHT_BLINKER_BUTTON]:
            requestAV.toggle_front_right_blinker = True
        else:
            requestAV.toggle_front_right_blinker = False


        self.audiovisual_server_client.send_goal_async(requestAV)


    def msgParse_AutonomyEnableButton(self, msg:Joy):
        if msg.buttons[self.ENABLE_AUTONOMY_BUTTON]:
            self.requestSystemEnableDisable()

    def msgParse_MotionAxes(self, msg:Joy):
        analogV_input = msg.axes[self.JOY_V_AXIS]
        analogH_input = msg.axes[self.JOY_H_AXIS]


        aV_in_trunc = - round(analogV_input, 3) # Truncate and invert so that +V is forward in robot frame
        aH_in_trunc = - round(analogH_input, 3) # Similar to above

        # Nullify negligible inputs to reduce noise
        if abs(aV_in_trunc) < self.JOY_V_DEADZONE: aV_in_trunc = 0
        if abs(aH_in_trunc) < self.JOY_H_DEADZONE: aH_in_trunc = 0

        cmd_linvel = self.mapLinearVelocity(aV_in_trunc)
        cmd_angvel = self.mapSteeringAngle(aH_in_trunc)

        cmd_linvel_filtered = self.vel_lin_prev * self.VEL_LPF_ALPHA + cmd_linvel * (1-self.VEL_LPF_ALPHA)
        cmd_angvel_filtered = self.vel_ang_prev * self.VEL_ANG_LPF_ALPHA + cmd_angvel * (1-self.VEL_ANG_LPF_ALPHA)

        # Nullify negligible inputs to reduce noise
        if abs(cmd_linvel_filtered) < self.VEL_DEADZONE: cmd_linvel_filtered = 0
        if abs(cmd_angvel_filtered) < self.VEL_ANG_DEADZONE: cmd_angvel_filtered = 0

        self.vel_lin_prev = cmd_linvel_filtered
        self.vel_ang_prev = cmd_angvel_filtered

        self.updateCMDVel(round(cmd_linvel_filtered, 5), round(cmd_angvel_filtered, 5))



    def mapLinearVelocity(self, analogInput):
        
        linVel = self.linearIshMap(analogInput, self.JOY_V_MIN, self.JOY_V_MAX, self.VEL_LIN_MIN, self.VEL_LIN_MAX)
        return linVel

    def mapSteeringAngle(self, analogInput):
        strAng = self.linearIshMap(analogInput, self.JOY_H_MIN, self.JOY_V_MAX, self.VEL_ANG_MIN, self.VEL_ANG_MAX)
        return strAng


    # Performs linear map. Due to potentially uneven bounds, simply "bottoms out" control input.
    # IE when reversing, will reach maximum speed before analog stick fully moved.
    def linearIshMap(self, value, inMin, inMax, outMin, outMax):
        #outRange = outMax - outMin
        #inRange = inMax - inMin

        #valScale = float(value - inMin) / inRange


        # Symmetrical range
        outRange = 2 * outMax
        inRange = 2 * inMax

        valScale = float(value + inMax) / inRange
        
        out = -outMax + (valScale * outRange)
        #self.get_logger().info('vs: ' + str(valScale) + ', out: ' + str(out))
        if out < outMin: out = outMin
        elif out > outMax: out = outMax

        return out

def main(args=None):
    rclpy.init(args=args)       # Initilizes rclpy before everything else
    joy_sub = JoystickSub()     # Calls subscriber
    rclpy.spin(joy_sub)         # Runs subscriber without timeout limit until keyboard interruption
    joy_sub.destroy_node()      # Destroys node when done
    rclpy.shutdown()            # Shuts down rclpy

if __name__ == '__main__':
    main()
    