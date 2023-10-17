
import rclpy
from rclpy.node import Node


import pyttsx3

from std_msgs.msg import String

class TTS_Node(Node):
    def __init__(self):
        super().__init__('tts_node')

        self.engine = pyttsx3.init()

        self.engine.setProperty('rate',100)
        #self.engine.setProperty('voice', self.engine.getProperty('voices')[11].id)
        self.engine.setProperty('gender', 'female')

        self.text_sub = self.create_subscription(String, '/HMI/tts', self.tts_cb, 10)

    
    def tts_cb(self, msg:String):
        self.engine.say(msg.data)
        self.engine.runAndWait()

def main():
    rclpy.init()

    ttsNode = TTS_Node()
    
    rclpy.spin(ttsNode)

    ttsNode.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()