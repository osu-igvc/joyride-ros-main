from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from rcl_interfaces.msg import ParameterDescriptor

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

        self.declare_parameters(namespace='',parameters=[('frames', rclpy.Parameter.Type.STRING_ARRAY)], )
        config_frames = self.get_parameter('frames').value
        #self.get_logger().warn('All frames:' + str(self.frames))

        for f in config_frames:
            self.declare_parameters(namespace='',parameters = [(f+a,[], ParameterDescriptor(dynamic_typing=True)) for a in self.transform_attr])
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
            #self.get_logger().warn('frame:' + str(f) + ', header: '+F.header.frame_id)

            self.frames.append(F)
        
        #self.get_logger().warn('All frames:' + str(self.frames))

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