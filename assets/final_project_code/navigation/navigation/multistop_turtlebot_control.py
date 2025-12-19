#!/usr/bin/env python3
import sys
import numpy as np


import rclpy
from rclpy.node import Node
from rclpy.time import Time


from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


from std_msgs.msg import String
import time




class TurtleBotController(Node):
   def __init__(self, robot_frame, target_frames):
       super().__init__('turtlebot_controller')


       self.robot_frame = robot_frame
       self.target_frames = target_frames
       self.current_goal_idx = 0
       self.current_target = self.target_frames[self.current_goal_idx]


       self.K1 = 0.2
       self.K2 = 0.5
       self.stop_threshold = 0.3  # meters
       self.wait_duration = 25.0  # seconds


       # -----------------------------
       # State machine
       # -----------------------------
       self.state = "MOVING"   # MOVING | WAITING | DONE
       self.wait_start_time = None


       # -----------------------------
       # TF + publishers
       # -----------------------------
       self.tf_buffer = Buffer()
       self.tf_listener = TransformListener(self.tf_buffer, self)


       self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
       self.arduino_pub = self.create_publisher(String, "/arduino_cmd", 10)


       self.timer = self.create_timer(0.1, self.loop)


       self.sent_start = False


       self.get_logger().info(
           f"Robot frame: {self.robot_frame}, "
           f"Targets queued: {self.target_frames}"
       )


   # ------------------------------------------------
   def loop(self):
       if self.state == "DONE":
           self.cmd_pub.publish(Twist())
           return


       if self.state == "WAITING":
           elapsed = time.time() - self.wait_start_time
           if elapsed >= self.wait_duration:
               self.advance_to_next_target()
           return


       # ---------------- MOVING ----------------
       try:
           tf = self.tf_buffer.lookup_transform(
               self.robot_frame,
               self.current_target,
               Time()
           )


           x_error = tf.transform.translation.x
           y_error = tf.transform.translation.y
           distance = np.sqrt(x_error**2 + y_error**2)


           self.get_logger().info(
               f"Target: {self.current_target}, "
               f"x={x_error:.3f}, y={y_error:.3f}, d={distance:.3f}"
           )


           if distance < self.stop_threshold:
               self.handle_goal_reached()
               return


           cmd = Twist()
           cmd.linear.x = self.K1 * x_error
           cmd.angular.z = self.K2 * y_error
           self.cmd_pub.publish(cmd)


       except TransformException:
           self.cmd_pub.publish(Twist())


   # ------------------------------------------------
   def handle_goal_reached(self):
       self.cmd_pub.publish(Twist())
       self.get_logger().info(f"🎯 Reached {self.current_target}")


       # if not self.sent_start:
       #     msg = String()
       #     msg.data = "START"
       #     self.arduino_pub.publish(msg)
       #     self.sent_start = True
       #     self.get_logger().info("📡 Sent START to Arduino")


       self.state = "WAITING"
       self.wait_start_time = time.time()
       self.get_logger().info("⏳ Waiting 15 seconds...")


   # ------------------------------------------------
   def advance_to_next_target(self):
       self.current_goal_idx += 1


       if self.current_goal_idx >= len(self.target_frames):
           self.state = "DONE"
           self.get_logger().info("✅ All targets completed")
           return


       self.current_target = self.target_frames[self.current_goal_idx]
       self.state = "MOVING"
       self.sent_start = False


       self.get_logger().info(f"➡️ Moving to next target: {self.current_target}")


   # ------------------------------------------------
   def destroy_node(self):
       self.cmd_pub.publish(Twist())
       super().destroy_node()




def main(args=None):
   rclpy.init(args=args)


   if len(sys.argv) < 3:
       print("Usage: ros2 run turtlebot_controller turtlebot_control robot_frame target1 [target2 ...]")
       rclpy.shutdown()
       return


   robot_frame = sys.argv[1]
   target_frames = sys.argv[2:]


   node = TurtleBotController(robot_frame, target_frames)


   try:
       rclpy.spin(node)
   finally:
       node.destroy_node()
       rclpy.shutdown()
