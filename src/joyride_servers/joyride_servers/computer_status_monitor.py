
import psutil
import collections

# ROS
import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class ComputerStatusMonitor(Node):
    def __init__(self):
        super().__init__('computer_status_monitor_node')

        self.cpu_warning_percentage = self.declare_parameter('cpu_warning_percentage', 90).get_parameter_value().integer_value
        self.diagnostic_topic = self.declare_parameter('diagnostic_topic', '/diagnostics').get_parameter_value().string_value
        self.cpu_diagnostic_name = self.declare_parameter('cpu_diagnostic_name', '/utility/cpu').get_parameter_value().string_value
        self.cpu_hardware_id = str(self.declare_parameter('cpu_hardware_id', 0xf0).get_parameter_value().integer_value)
        self.frequency = self.declare_parameter('frequency', 1.0).get_parameter_value().double_value
        self.period = 1.0 / self.frequency

        self.update_timer = self.create_timer(self.period, self.publishDiagnostics_Callback)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 1)

        self.cpu_readings = collections.deque(maxlen=1)

        self.cpuStatus = None
    
    # From diagnostics noetic-devel branch, diagnostics_common
    def get_average_cpu_readings(self):
        def avg(lst):
            return float(sum(lst)) / len(lst) if lst else float('nan')

        return [avg(cpu_percentages) for cpu_percentages in zip(*self.cpu_readings)]
    
    def updateStatuses(self):
        self.updateCPUStatus()

    def updateCPUStatus(self):
        self.cpu_readings.append(psutil.cpu_percent(percpu=True))
        cpu_percents = self.get_average_cpu_readings()
        cpu_average = sum(cpu_percents) / len(cpu_percents)
        cpuStatus = DiagnosticStatus(name=self.cpu_diagnostic_name, hardware_id=self.cpu_hardware_id)

        cpuStatus.values.append(KeyValue(key='CPU Load Average', value='{:.2f}'.format(cpu_average)))

        warn = False
        for idx, val in enumerate(cpu_percents):
            cpuStatus.values.append(KeyValue(key='CPU {} Load'.format(idx), value='{:.2f}'.format(val)))
                                    
            if val > self.cpu_warning_percentage:
                warn = True

        if warn:
            cpuStatus.level = DiagnosticStatus.WARN
            cpuStatus.message = 'At least one CPU exceeds {:d} percent'.format(self.cpu_warning_percentage)
        else:
            cpuStatus.level = DiagnosticStatus.OK
            cpuStatus.message = 'CPU Average {:.2f} percent'.format(cpu_average)

        self.cpuStatus = cpuStatus
    
    def publishDiagnostics_Callback(self):
        
        self.updateStatuses()
        diagArray = DiagnosticArray()
        
        if self.cpuStatus is not None:
            diagArray.status.append(self.cpuStatus)
        diagArray.header.stamp = self.get_clock().now().to_msg()

        self.diagnostic_pub.publish(diagArray)




def main():

    rclpy.init()

    computerStatusMonitor = ComputerStatusMonitor()

    rclpy.spin(computerStatusMonitor)
    computerStatusMonitor.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()