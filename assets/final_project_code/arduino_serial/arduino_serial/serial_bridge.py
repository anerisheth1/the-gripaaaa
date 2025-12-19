#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# important: run this code after ssh into turtlebot

class ArduinoSerialBridge(Node):
   def __init__(self):
       super().__init__('arduino_serial_bridge')


       # Connect to Arduino
       self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=0.1)


       # Subscriber to listen for commands
       self.sub = self.create_subscription(
           String,
           '/arduino_cmd',
           self.send_to_arduino,
           10
       )


       self.get_logger().info("Serial bridge connected to Arduino.")


   def send_to_arduino(self, msg):
       cmd = msg.data.strip() + "\n"
       self.ser.write(cmd.encode())
       self.get_logger().info(f"Sent to Arduino: {cmd}")


def main(args=None):
   rclpy.init(args=args)
   node = ArduinoSerialBridge()
   rclpy.spin(node)
   node.destroy_node()
   rclpy.shutdown()
