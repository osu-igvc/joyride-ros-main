from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class StaticTransformNode(Node):

    def __init__(self):
        super().__init__('static_transform_broadcaster')

        self.transform_attr = [".frame_id",
                               ".child_frame_id",
                               ".translation.x",
                               ".translation.y",
                               ".translation.z",
                               ".rotation.x",
                               ".rotation.y",
                               ".rotation.z",
                               ".rotation.w"]
        self.frames = []

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

        self.create_transform_list()

    def create_transform_list(self):

        self.declare_parameters(namespace='',parameters=[('frames',[])])
        frames = self.get_parameter('frames').value

        for f in frames:
            self.declare_parameters(namespace='',parameters = [(f+a,[]) for a in self.transform_attr])
            params = self.get_parameters([f+a for a in self.transform_attr])

            F = TransformStamped()
            F.header.frame_id           = params[0].get_parameter_value().string_value
            F.child_frame_id            = params[1].get_parameter_value().string_value
            F.transform.translation.x   = params[2].get_parameter_value().double_value
            F.transform.translation.y   = params[3].get_parameter_value().double_value
            F.transform.translation.z   = params[4].get_parameter_value().double_value
            F.transform.rotation.x      = params[5].get_parameter_value().double_value
            F.transform.rotation.y      = params[6].get_parameter_value().double_value
            F.transform.rotation.z      = params[7].get_parameter_value().double_value
            F.transform.rotation.w      = params[8].get_parameter_value().double_value

            self.frames.append(F)

    def broadcast_timer_callback(self):
        t = self.get_clock().now().to_msg()
        for frame in self.frames:
            frame.header.stamp = t
        self.tf_broadcaster.sendTransform(self.frames)


def main():
    rclpy.init()
    stn = StaticTransformNode()
    rclpy.spin(stn)
    stn.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()