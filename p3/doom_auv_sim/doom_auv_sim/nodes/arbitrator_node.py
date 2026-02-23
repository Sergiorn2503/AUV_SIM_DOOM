#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import json

class ArbitratorNode(Node):
    def __init__(self):
        super().__init__('arbitrator_node')
        self.get_logger().info("COMMAND ARBITRATOR STARTED")
        
        # Subscriptions
        self.sub_auto = self.create_subscription(Twist, '/cmd_vel/autonomous', self.auto_callback, 10)
        self.sub_mode = self.create_subscription(String, '/auv/control_mode', self.mode_callback, 10)
        self.sub_acoustic = self.create_subscription(String, '/comm/auv_rx', self.acoustic_rx_callback, 10)
        
        # Publisher to Sim
        self.pub_cmd = self.create_publisher(Twist, '/auv/thruster_cmd', 10)
        
        # State
        self.mode = "MANUAL" # Default
        self.last_manual_cmd = Twist()
        self.last_auto_cmd = Twist()
        self.last_manual_time = 0
        self.last_auto_time = 0
        
        # Timer (20Hz)
        self.create_timer(0.05, self.control_loop)

    def acoustic_rx_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data.get("type") == "manual_control":
                # Convert JSON to Twist
                t = Twist()
                t.linear.x = float(data.get("linear", 0.0))
                t.angular.z = float(data.get("angular", 0.0))
                t.linear.z = float(data.get("vertical", 0.0))
                
                self.last_manual_cmd = t
                self.last_manual_time = time.time()
                # self.get_logger().info("Received Acoustic Manual Command")
        except:
            pass

    def auto_callback(self, msg):
        self.last_auto_cmd = msg
        self.last_auto_time = time.time()

    def mode_callback(self, msg):
        if msg.data in ["MANUAL", "AUTO"]:
            if self.mode != msg.data:
                self.get_logger().info(f"SWITCHING CONTROL MODE TO: {msg.data}")
                self.mode = msg.data

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        
        if self.mode == "MANUAL":
            # Pass manual if recent (< 0.5s)
            if now - self.last_manual_time < 0.5:
                cmd = self.last_manual_cmd
            else:
                # Stop if no command
                pass 
                
        elif self.mode == "AUTO":
            # Pass auto if recent (< 0.5s)
            if now - self.last_auto_time < 0.5:
                cmd = self.last_auto_cmd
            else:
                pass
                
        self.pub_cmd.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArbitratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
