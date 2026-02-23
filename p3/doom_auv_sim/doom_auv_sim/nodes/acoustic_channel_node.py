#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import time
import random
import json
from doom_auv_sim.communication_logic import CommunicationPhysics

class AcousticChannelNode(Node):
    def __init__(self):
        super().__init__('acoustic_channel_node')
        
        # Bi-directional Communication
        # Topside -> AUV
        self.sub_topside = self.create_subscription(String, '/comm/topside_tx', self.topside_callback, 10)
        self.pub_auv = self.create_publisher(String, '/comm/auv_rx', 10)
        
        # AUV -> Topside
        self.sub_auv = self.create_subscription(String, '/comm/auv_tx', self.auv_callback, 10)
        self.pub_topside = self.create_publisher(String, '/comm/topside_rx', 10)
        
        # State for Physics
        self.sub_state = self.create_subscription(PoseStamped, '/auv/state', self.state_callback, 10)
        self.current_pose = None
        self.comm_physics = CommunicationPhysics()
        
        self.message_queue = [] # (send_time, publisher, msg)
        self.timer = self.create_timer(0.05, self.process_queue)
        
        self.get_logger().info("Acoustic Channel Node Started (Bi-Directional) - DEBUG MODE (FORCED LINK)")

    def state_callback(self, msg):
        self.current_pose = msg.pose

    def get_distance(self):
        if self.current_pose:
            # Assume Topside is at (0,0) or (50*32, 50*32) - Let's use 0,0 relative for simple distance
            # Or better, use the Map Center: 50*32, 50*32
            # Let's assume Topside is the starting point: 50*32, 50*32
            tx, ty = 1600.0, 1600.0 
            px = self.current_pose.position.x
            py = self.current_pose.position.y
            dist = ((px - tx)**2 + (py - ty)**2)**0.5
            return dist / 32.0 # Convert pixels to meters (approx)
        return 10.0

    def calculate_delay(self):
        dist_m = self.get_distance()
        speed_sound = 1500.0 # m/s
        delay = dist_m / speed_sound
        # Add some processing overhead/noise
        return delay + random.uniform(0.01, 0.05)

    def topside_callback(self, msg):
        self.handle_msg(msg, self.pub_auv, direction="downlink")

    def auv_callback(self, msg):
        self.handle_msg(msg, self.pub_topside, direction="uplink")

    def handle_msg(self, msg, publisher, direction):
        # SIMULATE PHYSICS (Queue with Delay)
        delay = self.calculate_delay()
        send_time = time.time() + delay
        
        # Packet Loss Simulation (Simple)
        if random.random() < 0.05: # 5% packet loss
             self.get_logger().warn(f"Packet Loss in {direction}!")
             return

        self.message_queue.append((send_time, publisher, msg))
        # self.get_logger().info(f"Queued {direction} msg. Delay: {delay:.3f}s")

    def process_queue(self):
        now = time.time()
        for item in self.message_queue[:]:
            send_time, pub, msg = item
            if now >= send_time:
                self.get_logger().info(f"Publishing to {pub.topic_name if hasattr(pub, 'topic_name') else 'endpoint'}")
                pub.publish(msg)
                self.message_queue.remove(item)

def main(args=None):
    rclpy.init(args=args)
    node = AcousticChannelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
