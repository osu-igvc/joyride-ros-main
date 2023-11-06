
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import KeyValue
import diagnostic_msgs
import diagnostic_updater

from joyride_interfaces.srv import RequestAutoEnableDisable
from joyride_interfaces.msg import SystemDiagnosticSummary, DriveByWireSystemInfo

class AutoModeManagerServerNode(Node):
    def __init__(self):
        super().__init__('automode_manager_server_node')

        self.get_logger().info('Auto Mode Manager Server node spawning')

        # ROS Parameters
        self.system_status_topic = self.declare_parameter('system_status_topic', '/system/health').get_parameter_value().string_value
        self.auto_enable_disable_topic = self.declare_parameter('auto_enable_disable_topic', 'requestAutoEnableDisable').get_parameter_value().string_value
        self.status_valid_timeout = self.declare_parameter('status_valid_timeout', 1.5).get_parameter_value().double_value
        self.diagnostic_topic = self.declare_parameter('diagnostic_msg_topic', '/diagnostics_agg').get_parameter_value().string_value
        self.dbw_system_topic = self.declare_parameter('dbw_system_topic', '/feedback/dbw_system_info').get_parameter_value().string_value
        self.declare_parameter('tracked_nodes', value=rclpy.Parameter.Type.STRING_ARRAY)
        self.tracked_nodes = self.get_parameter('tracked_nodes').value
        self.dbw_enable_request_timeout = self.declare_parameter('dbw_enable_timeout', 1.5).get_parameter_value().double_value


        # Get params into tuple
        self.buildNodeList()

        # ROS Timers
        self.system_status_timer = self.create_timer(0.8, self.publishSystemStatus_Timer_Callback)

        # ROS Publisher
        self.system_status_pub = self.create_publisher(SystemDiagnosticSummary, self.system_status_topic,1)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 1)

        # ROS Subscriber
        self.diagnostic_sub = self.create_subscription(DiagnosticArray, self.diagnostic_topic, self.handleDiagnosticStatusMsg_Callback, 10)
        self.dbw_sub = self.create_subscription(DriveByWireSystemInfo, self.dbw_system_topic, self.handleDBWSystem_Callback, 10)
        
        # ROS Service Server
        self.request_auto_enable_disable_srv = self.create_service(RequestAutoEnableDisable, self.auto_enable_disable_topic, self.requestAutoEnableDisable_Callback)

        # State Management
        self.SYSTEM_STATUS = SystemDiagnosticSummary()
        self.SYSTEM_STATUS.stamp = self.get_clock().now().to_msg()
        self.SYSTEM_STATUS.system_status = SystemDiagnosticSummary.STALE
        self.SYSTEM_STATUS.auto_software_enabled = False
        self.SYSTEM_STATUS.control_input = SystemDiagnosticSummary.MANUAL
        
        self.own_status = DiagnosticStatus.OK
        
        self.get_logger().info('Auto Mode Manager Server node loaded')

        self.publishSystemStatus_Timer_Callback()
        

    # ------------- ROS Callbacks ------------- #
    def publishSystemStatus_Timer_Callback(self):
        # self.get_logger().warn('Initial state in timer: {}'.format(self.SYSTEM_STATUS))
        # self.get_logger().warn('Auto state: {}'.format(self.SYSTEM_STATUS.auto_software_enabled))
        # Update system status
        if self.SYSTEM_STATUS.system_status != SystemDiagnosticSummary.OK:
            self.SYSTEM_STATUS.auto_software_enabled = False
        
        self.SYSTEM_STATUS.stamp = self.get_clock().now().to_msg()

        self.system_status_pub.publish(self.SYSTEM_STATUS)

        newStatus = DiagnosticArray()
        newStatus.status.append(DiagnosticStatus(level=self.own_status, name='/servers/automode_manager', values=[KeyValue(key='Enabled',
                            value=str(self.SYSTEM_STATUS.auto_software_enabled))]))
        newStatus.header.stamp = self.get_clock().now().to_msg()

        self.diagnostic_pub.publish(newStatus)
        
        self.last_request_time = self.get_clock().now().nanoseconds
        
    def handleDBWSystem_Callback(self, msg:DriveByWireSystemInfo):
        # self.get_logger().warn('Initial state in DBW callback: {}'.format(self.SYSTEM_STATUS))
        # self.get_logger().warn('Auto state: {}'.format(self.SYSTEM_STATUS.auto_software_enabled))
        # If timout passed, allow us to exit based on hardware. Otherwise ignore it.
        if self.get_clock().now().nanoseconds - self.last_request_time > self.dbw_enable_request_timeout * 1e9:
            if not msg.hardware_auto_enable:
                self.SYSTEM_STATUS.auto_software_enabled = False
            

    def handleDiagnosticStatusMsg_Callback(self, msg:DiagnosticArray):
        presentMsgs_ids = []
        supplementalDiag = DiagnosticArray()
        newStatus = SystemDiagnosticSummary.OK

        for status in msg.status:
            presentMsgs_ids.append(status.hardware_id)

            # Update if not OK
            for node in self.nodeList:
                if node[0] == status.name and node[2]:
                    if newStatus < status.level:
                        newStatus = status.level
                        
        self.SYSTEM_STATUS.system_status = newStatus
        
        for node in self.nodeList:
            # node[2] checks to see if diag is required for that node for enabling.
            # id count is to see if the node is already accounted for
            if node[2] and presentMsgs_ids.count(node[1]) < 1:
                # ID is not present, so add it to list of statues to publish
                supplementalDiag.status.append(DiagnosticStatus(level=DiagnosticStatus.STALE, name=node[0], hardware_id=str(node[1]), message='MISSING'))

        
        supplementalDiag.header.stamp = self.get_clock().now().to_msg()
        self.diagnostic_pub.publish(supplementalDiag)

    def requestAutoEnableDisable_Callback(self, request, response):
        self.get_logger().warn('Initial state in Auto enable: {}'.format(self.SYSTEM_STATUS))
        self.get_logger().warn('Auto state: {}'.format(self.SYSTEM_STATUS.auto_software_enabled))
        # Ensure last status isn't out of date

        if self.get_clock().now().nanoseconds - self.get_clock().now().from_msg(self.SYSTEM_STATUS.stamp).nanoseconds > self.status_valid_timeout * 1E9: # time in nano
            self.get_logger().warn('Request set enable: {}, received by {}. Ignoring - system status is out of date.'.format(request.set_auto_enabled, request.sender_name))
            response.response = RequestAutoEnableDisable.Response.STATUS_TIMEOUT
            self.SYSTEM_STATUS.auto_software_enabled = False
            return response
        
        # Ensure system is healthy
        if self.SYSTEM_STATUS.system_status != SystemDiagnosticSummary.OK:
            self.get_logger().warn('Request set enable: {}, received by {}. Ignoring - system is unhealthy.'.format(request.set_auto_enabled, request.sender_name))
            response.response = RequestAutoEnableDisable.Response.SYSTEM_UNHEALTHY
            self.SYSTEM_STATUS.auto_software_enabled = False
            return response
        
        # Check new status is different
        if self.SYSTEM_STATUS.auto_software_enabled == request.set_auto_enabled:
            self.get_logger().warn('Previous state: {}'.format(self.SYSTEM_STATUS.auto_software_enabled))
            self.get_logger().warn('Request set enable: {}, received by {}. Ignoring - system is already in that state.'.format(request.set_auto_enabled, request.sender_name))
            response.response = RequestAutoEnableDisable.Response.ALREADY_IN_STATE
            return response

        # Checks are passed, make request
        self.makeAutoRequest(request.set_auto_enabled)
        self.get_logger().warn('Request set enable: {}, received by {}. Obeying.'.format(request.set_auto_enabled, request.sender_name))
        response.response = RequestAutoEnableDisable.Response.REQUEST_MADE
        self.SYSTEM_STATUS.auto_software_enabled = request.set_auto_enabled
        if request.set_auto_enabled:
            self.last_request_time = self.get_clock().now().nanoseconds
        return response
    


    # ------------- Make Requests ------------- #
    def makeAutoRequest(self, newStatus:bool):
        # Begin publishing enable to ROSCAN
        pass

    # ------------- Diagnostics  ------------- #
    def diagnosticUpdate(self, stat):
        stat.summary(self.own_status, 'OK')
        stat.add('Enabled', str(self.SYSTEM_STATUS.enabled))
        stat.add('Control', str(self.SYSTEM_STATUS.control_input))
        return stat
    
    def buildNodeList(self):

        self.nodeList = []
        for item in self.tracked_nodes:

            name = self.declare_parameter(item+'.name', value=rclpy.Parameter.Type.STRING).get_parameter_value().string_value
            id = self.declare_parameter(item+'.id', value=rclpy.Parameter.Type.INTEGER).get_parameter_value().integer_value
            require = self.declare_parameter(item+'.require', value=rclpy.Parameter.Type.BOOL).get_parameter_value().bool_value

            self.nodeList.append( (name, id, require) )

# --------------------- Node Startup --------------------- #

def main():

    rclpy.init()

    automodeManagerServerNode = AutoModeManagerServerNode()
    # updater = diagnostic_updater.Updater(automodeManagerServerNode)
    # updater.add('status', automodeManagerServerNode.diagnosticUpdate)
    rclpy.spin(automodeManagerServerNode)
    automodeManagerServerNode.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()