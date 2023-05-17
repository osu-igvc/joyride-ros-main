
import psutil
import collections

# ROS
import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class ComputerStatusMonitor(Node):
    def __init__(self):
        super().__init__('computer_status_monitor_node')

        self.frequency = self.declare_parameter('frequency', 1.0).get_parameter_value().double_value
        self.period = 1.0 / self.frequency
        self.diagnostic_topic = self.declare_parameter('diagnostic_topic', '/diagnostics').get_parameter_value().string_value

        self.cpu_warning_temperature = self.declare_parameter('cpu_warning_temperature', 80).get_parameter_value().integer_value
        self.cpu_warning_percentage = self.declare_parameter('cpu_warning_percentage', 90).get_parameter_value().integer_value
        self.cpu_diagnostic_name = self.declare_parameter('cpu_diagnostic_name', '/utility/cpu').get_parameter_value().string_value
        self.cpu_hardware_id = str(self.declare_parameter('cpu_hardware_id', 0xf0).get_parameter_value().integer_value)

        self.ram_warning_percentage = self.declare_parameter('ram_warning_percentage', 90).get_parameter_value().integer_value
        self.ram_diagnostic_name = self.declare_parameter('ram_diagnostic_name', '/utility/ram').get_parameter_value().string_value
        self.ram_hardware_id = str(self.declare_parameter('ram_hardware_id', 0xf1).get_parameter_value().integer_value)

        self.update_timer = self.create_timer(self.period, self.publishDiagnostics_Callback)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, self.diagnostic_topic, 1)

        self.cpu_readings = collections.deque(maxlen=1)
        self.ram_readings = collections.deque(maxlen=1)

        self.cpuStatus = None
    
    # From diagnostics noetic-devel branch, diagnostics_common
    def get_average_cpu_readings(self):
        def avg(lst):
            return float(sum(lst)) / len(lst) if lst else float('nan')

        return [avg(cpu_percentages) for cpu_percentages in zip(*self.cpu_readings)]
    
    def get_cpu_temperature(self):
        try:
            return psutil.sensors_temperatures()['coretemp'][0].current
        except:
            self.get_logger().error(str(psutil.sensors_temperatures()))
            return float('nan')
    
    def updateStatuses(self):
        self.updateCPUStatus()
        self.updateRAMStatus()


    def updateCPUStatus(self):
        self.cpu_readings.append(psutil.cpu_percent(percpu=True))
        cpu_percents = self.get_average_cpu_readings()
        cpu_average = sum(cpu_percents) / len(cpu_percents)
        cpuStatus = DiagnosticStatus(name=self.cpu_diagnostic_name, hardware_id=self.cpu_hardware_id)

        cpuStatus.values.append(KeyValue(key='CPU Temperature', value='{:.2f}'.format(self.get_cpu_temperature())))
        cpuStatus.values.append(KeyValue(key='CPU Load Average', value='{:.2f}'.format(cpu_average)))

        if cpu_average > self.cpu_warning_percentage:
            cpuStatus.level = DiagnosticStatus.WARN
            cpuStatus.message = 'CPU usage exceeds {:d} percent'.format(self.cpu_warning_percentage)
        
        elif self.get_cpu_temperature() > self.cpu_warning_temperature:
            cpuStatus.level = DiagnosticStatus.WARN
            cpuStatus.message = 'CPU temperature exceeds {:d} degrees'.format(self.cpu_warning_temperature)

        else:
            cpuStatus.level = DiagnosticStatus.OK
            cpuStatus.message = f"CPU: {cpu_average:.2f}%, {self.get_cpu_temperature():.2f}°C"

        self.cpuStatus = cpuStatus

    def updateRAMStatus(self):
        self.ram_readings.append(psutil.virtual_memory().percent)
        ram_average = sum(self.ram_readings) / len(self.ram_readings)

        newRAMStatus = DiagnosticStatus(name=self.ram_diagnostic_name, hardware_id=self.ram_hardware_id)

        newRAMStatus.values.append(KeyValue(key='RAM Load Average', value='{:.2f}'.format(ram_average)))

        if ram_average > self.ram_warning_percentage:
            newRAMStatus.level = DiagnosticStatus.WARN
            newRAMStatus.message = 'RAM Average exceeds {:d} percent'.format(self.ram_warning_percentage)
        else:
            newRAMStatus.level = DiagnosticStatus.OK
            newRAMStatus.message = 'RAM Average {:.2f} percent'.format(ram_average)

        self.ramStatus = newRAMStatus
    
    def publishDiagnostics_Callback(self):
        
        self.updateStatuses()
        diagArray = DiagnosticArray()
        
        if self.cpuStatus is not None:
            diagArray.status.append(self.cpuStatus)
        
        if self.ramStatus is not None:
            diagArray.status.append(self.ramStatus)

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