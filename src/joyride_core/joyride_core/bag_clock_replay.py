
import rclpy
from rclpy.node import Node

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix

class TimeNode(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.time_sub = self.create_subscription(NavSatFix, '/vectornav/gnss', self.time_cb, 10)
        self.time_pub = self.create_publisher(Clock, '/clock', 1)
        self.time_msg = Clock()
    
    def time_cb(self, msg:NavSatFix):
        self.time_msg.clock.sec = msg.header.stamp.sec
        self.time_msg.clock.nanosec = msg.header.stamp.nanosec
        self.time_pub.publish(self.time_msg)

def main():
    rclpy.init()

    timeNode = TimeNode()
    
    rclpy.spin(timeNode)

    timeNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()