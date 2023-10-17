
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import diagnostic_msgs
import diagnostic_updater

class DiagTest(Node):
    def __init__(self):
        super().__init__('diag_test')

        self.timer = self.create_timer(1.0, self.cb)
        self.t_pub = self.create_publisher(Bool, '/testing', 1)

    def cb(self):
        b = Bool()
        b.data = False
        self.t_pub.publish(b)

    def updateDiagnostics(self, stat):
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN,
        'Test updater')
        stat.add('Variable maybe', '1000')
        return stat



def main():
    rclpy.init()

    node = DiagTest()

    updater = diagnostic_updater.Updater(node)

    updater.setHardwareID('Device Test')

    updater.add('DIag method updater', node.updateDiagnostics)

    updater.broadcast(diagnostic_msgs.msg.DiagnosticStatus.OK, 'Initializing things sorta')

    rclpy.spin(node)


if __name__ == "__main__":
    main