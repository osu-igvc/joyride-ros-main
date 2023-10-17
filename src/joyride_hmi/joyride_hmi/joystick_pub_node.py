import rclpy
from sensor_msgs.msg import Joy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_updater import Updater

import time
import pygame
from pygame.locals import *
       

class JoystickPub(Node):
    def __init__(self):
        super().__init__('joy_pub')     # Initilize the node with proper name (does not have to be the same as the class name)

        # Initilizing pygame's joystick module
        pygame.joystick.quit()
        pygame.joystick.init()
        self.stick = None
        if pygame.joystick.get_count() == 0:
            return

        # Connecting to joystick (if there is more than 1 you would change it from 0th). 
        self.stick = pygame.joystick.Joystick(0)    
        
        # Initilize the connected joystick
        self.stick.init()          
        # Creating timer that updates every 0.05 seconds
        self.timer = self.create_timer(0.05, self.update)
        
        # Creating publisher that publishes standard Joy messages (imported from std_mssgs), the topic is /joy_input, 
        # and queued messages limit (if subscriber is not fast enough) is 1:
        self.publisher = self.create_publisher(Joy, '/joystick_raw', 1) 
         # Message type is set to joystick, a package built into ros2 made for controllers and joysticks:
        self.msg = Joy()       
         # Reports axis, must be in float32 format:
        self.msg.axes = [float(0)]*self.stick.get_numaxes()

        # Reports buttons, must be int32 format:
        self.msg.buttons = [0]*(self.stick.get_numbuttons())  
        self.rumbling = False

    def updateHeader(self):
         # Sending timestamp with each update 
        self.msg.header.stamp = self.get_clock().now().to_msg()    
    
    
     # Every time an event occurs this loop will determine if it is a button or axis motion 
     # and will assign values according to its state
    def update(self):
        if pygame.joystick.get_count() == 0:
            self.reinitialize_joystick()
            
        else:  
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:      
                    self.msg.buttons[event.button] = 1
                elif event.type == pygame.JOYBUTTONUP:
                    self.msg.buttons[event.button] = 0

                    if event.button == 9: # options button
                        self.stick.rumble(0.3, 1.0, 1000)
     
                elif event.type == pygame.JOYAXISMOTION:
                    self.msg.axes[event.axis] = event.value

        self.updateHeader()    #Updates the header each time update runs and outputs 
        self.publisher.publish(self.msg)    # Publishes values each time update runs

    def reinitialize_joystick(self):
        pygame.joystick.quit()
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            self.stick = pygame.joystick.Joystick(0)
            self.stick.init()
            self.msg.axes = [float(0)] * self.stick.get_numaxes()
            self.msg.buttons = [0] * (self.stick.get_numbuttons())
    
    def diagnostic(self, stat):
        if pygame.joystick.get_count() == 0:
            stat.summary(DiagnosticStatus.ERROR, "No joystick connected")
        else:
            stat.summary(DiagnosticStatus.OK, "Joystick Connected")
        return stat


def main():
    pygame.init()       # Initilize pygame before anything else
    rclpy.init()        # Initilize rclpy 
    joy_pub = JoystickPub()
    while joy_pub.stick == None:
        joy_pub = JoystickPub()
        time.sleep(0.5)
    

    updater = Updater(joy_pub)
    updater.setHardwareID("joystick")
    updater.add("/utility/joystick", joy_pub.diagnostic)

    # Spin node; runs node until keyboard interrupts such as crtl + c
    # This section needs to be changed to allow timeouts to shutdown the program 
    # Ideally there will be a shutdown sequence to allow the car to come to a stop safely
    rclpy.spin(joy_pub) 
    joy_pub.destroy_node()  # Destroy node at the end
    rclpy.shutdown()        # Shutdown rclpy at end


if __name__ == "__main__":
    main()
    
    