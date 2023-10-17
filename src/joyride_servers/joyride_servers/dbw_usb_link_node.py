
import serial
import os.path
from pathlib import Path
from typing import List


# ROS
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

# ROS Messages
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool
from std_msgs.msg import String

#from autonomy_util.heartbeat_manager import HeartbeatManager

# Custom messaging
from joyride_interfaces.msg import SystemHealth
from joyride_interfaces.action import RequestAudioVisual




class USBLinkNode(Node):
    def __init__(self):
        super().__init__('dbw_usb_link_node')

        # Enable protocol testing, verification
        self.TESTING_MODE = False

        # Open USB Port
        self.usbPort = Path('/dev/ttyACM0')
        self.serialPort = None

        self.retry_connection_timer = self.create_timer(0.5, self.tryConnect)

        self.tryConnect()

        self.rxUSB_Period = 1.0/100.0 # 100Hz reading
        self.txHeartbeat_Period = 0.5 # Every 800ms, timeout at 1000ms

        self.HEARTBEAT_TIMEOUT = 1.0

        self.txCV_PERIOD = 1/20.0
        self.txHB_PERIOD = 1/10.0
        self.txCE_PERIOD = 1/10.0
        self.txUSB_Period = 1.0/50.0 # 50Hz writing
        self.rxUSB_Timer = None
        self.txHeartbeat_Timer = None
        self.txUSB_Timer = None

        self.rxUSB_Timer = self.create_timer(self.rxUSB_Period, self.rxUSB)
        # Heartbeat/handshake timer
        self.txHeartbeat_Timer = self.create_timer(self.txHeartbeat_Period, self.txHeartbeat)
        # Write USB
        self.txUSB_Timer = self.create_timer(self.txUSB_Period, self.txUSB_Callback)

        self.tts_pub = self.create_publisher(String, '/HMI/tts', 1)
        
        self.audiovisual_action_server = ActionServer(
            self,
            RequestAudioVisual,
            'SystemAudioVisual',
            self.callback_AudioVisualServer
        )
        self.prev_audiovisual = None

        # Receive info from ROS
        self.cmd_vel_sub = self.create_subscription(TwistStamped, '/cmd_vel', self.newVelocity, 1)
        self.enable_state_sub = self.create_subscription(Bool, '/health/enable_status', self.newRequestEnable, 1)

        # Monitor state of DBW system
        self.dbw_enable_state = False
        self.connectionStatus = False
        self.connectionBeatCount = 0
        self.lastBeat = self.get_clock().now()


    # ================== CONNECTION

    def tryConnect(self):
        try:
            #self.get_logger().info('Tryin connection')
            if self.TESTING_MODE: return
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
                    self.serialPort = serial.Serial('/dev/ttyACM0', baudrate=115200, timeout=0)
                    self.initThings()

        except serial.SerialException as error:
            self.get_logger().error('USB CONNECTION ERROR')

    def initThings(self):
        self.tx_CV_last = self.tx_HB_last = self.tx_CE_last = self.get_clock().now().nanoseconds / 1e9

        self.next_CE = self.next_CV = self.next_HB = None
        self.connectionStatus = False
        self.connectionBeatCount = 0

    # ================== SENDING ================== #

    def txHeartbeat(self):
        # Attempt to connect
        if not self.connectionStatus:
            self.get_logger().info("Disconnected. Attempting to connect.")
            self.txUSB(self.packMSG_Handshake(self.connectionBeatCount))
        else:
            #self.get_logger().info("Connected. Transmitting heartbeat.")

            self.txUSB(self.packMSG_Heartbeat())

    def newVelocity(self, msg: TwistStamped):
        nextMsg = ""
        
        newLinX = msg.twist.linear.x
        newAngZ = msg.twist.angular.z

        nextMsg = self.packMSG_CMDVel(newLinX, newAngZ)

        if self.connectionStatus:
            self.next_CV = nextMsg


    def newRequestEnable(self, msg: Bool):
        nextMsg = ""
        requestedEnableState = msg.data

        if self.dbw_enable_state != requestedEnableState:
            nextMsg = self.packMSG_RequestEnable(requestedEnableState)
            if self.connectionStatus:
                self.next_CE = nextMsg

    def callback_AudioVisualServer(self, goal_handle):
        
        newMsg = self.packMSG_RequestAudioVisual(goal_handle.request.toggle_front_left_blinker, goal_handle.request.toggle_front_right_blinker, goal_handle.request.activate_buzzer)
        if newMsg != self.prev_audiovisual:
            self.txUSB(newMsg)
            self.prev_audiovisual = newMsg

        goal_handle.succeed()
        result = RequestAudioVisual.Result()
        return result

    def heartbeatTimeout(self):

        if self.connectionStatus:
            self.get_logger().error('Lost connection to DBW!')
            self.initThings()


    # ================== USB TX/RX ================== #


    def txUSB_Callback(self):

        if self.serialPort is None: return

        now = self.get_clock().now().nanoseconds/1e9
        if now - self.tx_CV_last >= self.txCV_PERIOD and self.next_CV is not None:
            self.txUSB(self.next_CV)
            self.tx_CV_last = now
        
        # if now - self.tx_HB_last >= self.txHB_PERIOD:
        #     self.txUSB(self.next_HB)
        #     self.tx_HB_last = now
        
        if now - self.tx_CE_last >= self.txCE_PERIOD and self.next_CE is not None:
            self.txUSB(self.next_CE)
            self.tx_CE_last = now

    def txUSB(self, txString):
        try:
            if self.serialPort is None: return

            self.get_logger().info('txing:'+str(txString.encode('utf-8')))

            if self.TESTING_MODE:
                self.txUSB_test(txString)
                return

            if self.serialPort.isOpen():
                self.serialPort.write(txString.encode('utf-8'))
        except OSError:
            self.get_logger().error('USB CONNECTION ERROR')

    def rxUSB(self):
        try:
            if self.serialPort is None: return

            if self.TESTING_MODE:
                self.rxUSB_test()
                return

            beatDiff = (self.get_clock().now().nanoseconds - self.lastBeat.nanoseconds) / 1e9
            #self.get_logger().info("bd:"+str(beatDiff))
            if(self.serialPort.isOpen() and self.serialPort.in_waiting > 0):
                rxData = self.serialPort.readline().decode('utf-8')
               #self.get_logger().info("received: " + rxData)

                self.parseMSG_Initial(rxData)
            elif  beatDiff > self.HEARTBEAT_TIMEOUT:
                #self.heartbeatTimeout
                pass
        except OSError:
            self.get_logger().error('USB CONNECTION ERROR')



    # ================== Testing Protocols

    def txUSB_test(self, txString):
        self.get_logger().info('txUSB Test:' + txString)

    def rxUSB_test(self):
        if self.test_index > len(self.TEST_MSGS) - 1:
            self.test_index = 0
        newTestData = self.TEST_MSGS[self.test_index]
        self.test_index += 1


        self.parseMSG_Initial(newTestData)


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
    
    def parseMSG_Health(self, msg:List[str]):
        subType = msg.pop()
        #self.get_logger().info("Health msg:" + subType)
        # Handshake
        if subType[0] == 'H':
            beatCount = int(msg.pop())

            self.get_logger().info("Beat Count: " + str(beatCount))
            if beatCount == (self.connectionBeatCount + 1):
                self.connectionBeatCount += 1
                newMsg = self.packMSG_Handshake(self.connectionBeatCount)

                self.get_logger().info("new msg:" + str(newMsg))

                if beatCount == 4:
                    self.connectionStatus = True
                    self.get_logger().info("HANDSHAKE COMPLETE")
                    newStr = String()
                    newStr.data = 'CONNECTED'
                    self.tts_pub.publish(newStr)
                elif beatCount > 4:
                    self.get_logger().warn("HANDSHAKE FAILURE, RESTARTING")
                    self.connectionBeatCount = 0
                    newMsg = self.packMSG_Handshake(self.connectionBeatCount)
                    self.txUSB(newMsg)
                else:
                    self.txUSB(newMsg)
            
        # Beat
        elif subType[0] == 'B':
            #self.get_logger().info("Received heartbeat from DBW")
            self.lastBeat = self.get_clock().now()

    
    # ================== MSG PACKING ================== #
    # All messages should have unique first two characters, separated by commas.
    # Once first two characters are parsed, the Interpreter can easily parse the remainder of the message.

    # ================== COMMAND MESSAGES

    def packMSG_CMDVel(self, linearX, angularZ) -> str:
        cmdMsg = f'C,V,{linearX:06.2f}\0,{angularZ:06.2f}\0'
        return cmdMsg

    def packMSG_RequestEnable(self, requestedEnableState) -> str:
        if requestedEnableState:
            return 'C,E,1'
        else:
            return 'C,E,0'

    def packMSG_RequestAudioVisual(self, toggle_fl_blinker, toggle_fr_blinker, activate_buzzer) -> str:
        cmdMsg = f'C,B,{int(toggle_fl_blinker)},{int(toggle_fr_blinker)},{int(activate_buzzer)}'
        #self.get_logger().info('AV REQUEST: ' + str(cmdMsg))
        return cmdMsg

    # ================== Health Messages
    # Handshake protocol
    def packMSG_Handshake(self, count:int) -> str:
        return ("H,H," + str(count))
    
    def packMSG_Heartbeat(self) ->str:
        return ("H,B")


    # ================== ACTION SERVER HANDLING ================== #



# ==================================== #

def main():
    rclpy.init()
    usbLink = USBLinkNode()
    rclpy.spin(usbLink)
    usbLink.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()