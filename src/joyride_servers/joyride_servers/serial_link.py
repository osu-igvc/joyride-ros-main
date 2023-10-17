
import serial
import os.path
from pathlib import Path
from typing import List


# ROS
import rclpy
from rclpy.node import Node

# ROS Messages
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from std_msgs.msg import String



class USBLinkNode(Node):
    def __init__(self):
        super().__init__('dbw_usb_link_node')

        # Open USB Port
        self.usbPort = Path('/dev/ttyACM1')
        self.serialPort = None

        self.retry_connection_timer = self.create_timer(0.5, self.tryConnect)

        self.tryConnect()

        self.rxUSB_Period = 1.0/100.0 # 100Hz reading

        self.txUSB_Period = 1.0/50.0 # 50Hz writing

        self.rxUSB_Timer = self.create_timer(self.rxUSB_Period, self.rxUSB)

        # Write USB
        #self.txUSB_Timer = self.create_timer(self.txUSB_Period, self.txUSB_Callback)

        # Receive info from ROS
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.newVelocity, 1)


    # ================== CONNECTION

    def tryConnect(self):
        try:
            if not self.usbPort.exists():
                if self.serialPort is not None:
                    # Lost connection, was connected before
                    self.get_logger().warning('Lost connection!')
                    self.serialPort = None
                else:
                    self.get_logger().warning('Failed to find USB port...')
            else:
                if self.serialPort is None:
                    # Still connected
                    self.get_logger().warning('Opening USB Port')
                    self.serialPort = serial.Serial('/dev/ttyACM1', baudrate=115200, timeout=0)
                    self.initThings()

        except serial.SerialException as error:
            self.get_logger().error('USB CONNECTION ERROR')

    def initThings(self):
        self.tx_CV_last = self.tx_HB_last = self.tx_CE_last = self.get_clock().now().nanoseconds / 1e9

        self.next_CE = self.next_CV = self.next_HB = None
        self.connectionStatus = False
        self.connectionBeatCount = 0

    # ================== SENDING ================== #

    def newVelocity(self, msg: TwistStamped):
        nextMsg = ""
        
        newLinX = msg.twist.linear.x
        newAngZ = msg.twist.angular.z

        nextMsg = self.packMSG_CMDVel(newLinX, newAngZ)

        if self.connectionStatus:
            self.next_CV = nextMsg

    # ================== USB TX/RX ================== #

    def txUSB(self, txString):
        try:
            if self.serialPort is None: return

            #self.get_logger().info('txing:'+str(txString.encode('utf-8')))

            if self.serialPort.isOpen():
                #self.serialPort.write(txString.encode('utf-8'))
                pass

        except OSError:
            self.get_logger().error('USB CONNECTION ERROR')

    def rxUSB(self):
        try:
            if self.serialPort is None: return

            if(self.serialPort.isOpen() and self.serialPort.in_waiting > 0):
                rxData = self.serialPort.readline() #.decode() #decode('utf-8')
                #rxData = self.serialPort.readline().decode('ascii')

                self.get_logger().info('USB RX: {}'.format(rxData))
                #self.parseMSG_Initial(rxData)

        except OSError:
            self.get_logger().error('USB CONNECTION ERROR')


    # ================== MSG PARSING ================== #

    def parseMSG_Initial(self, rxData:str):
        if len(rxData) < 2: return

        msgList = rxData.split(',')
        msgList.reverse()

        msgType = msgList.pop() # Sub command type. 

        if msgType == 'F':
            self.parseMSG_Feedback(msgList)
        elif msgType == 'H':
            self.parseMSG_Health(msgList)
    
    def parseMSG_Feedback(self, msg:List[str]):
        subType = msg.pop()
        if subType == 'E':
            rxMsg = int(msg.pop())
            self.dbw_enable_state = bool(rxMsg)

            #self.get_logger().info('rxUSB Test: En: ' + str(self.dbw_enable_state) + ', Str:' + str(rxMsg))
        elif subType == 'V':
            self.dbw_last_vel = float(msg.pop())
            self.dbw_last_str = float(msg.pop())
            #self.get_logger().info('rxUSB: Vx: ' + str(self.dbw_last_vel) + ', Wz: ' + str(self.dbw_last_str))
            

    
    # ================== MSG PACKING ================== #
    # All messages should have unique first two characters, separated by commas.
    # Once first two characters are parsed, the Interpreter can easily parse the remainder of the message.

    # ================== COMMAND MESSAGES

    def packMSG_CMDVel(self, linearX, angularZ) -> str:
        cmdMsg = f'C,V,{linearX:06.2f}\0,{angularZ:06.2f}\0'
        return cmdMsg


# ==================================== #

def main():
    rclpy.init()
    usbLink = USBLinkNode()
    rclpy.spin(usbLink)
    usbLink.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()