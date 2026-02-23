#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LinkVerifier(Node):
    def __init__(self):
        super().__init__('link_verifier')
        print("--- TOPIC VERIFIER STARTED ---")
        
        self.sub_auv_tx = self.create_subscription(String, '/comm/auv_tx', self.cb_auv_tx, 10)
        self.sub_topside_rx = self.create_subscription(String, '/comm/topside_rx', self.cb_topside_rx, 10)
        self.sub_topside_tx = self.create_subscription(String, '/comm/topside_tx', self.cb_topside_tx, 10)
        self.sub_auv_rx = self.create_subscription(String, '/comm/auv_rx', self.cb_auv_rx, 10)
        
        self.counts = {
            '/comm/auv_tx': 0,
            '/comm/topside_rx': 0,
            '/comm/topside_tx': 0,
            '/comm/auv_rx': 0
        }

    def cb_auv_tx(self, msg):
        self.counts['/comm/auv_tx'] += 1
        print(f"[{self.counts['/comm/auv_tx']}] RECEIVED /comm/auv_tx: {msg.data[:50]}...")

    def cb_topside_rx(self, msg):
        self.counts['/comm/topside_rx'] += 1
        print(f"[{self.counts['/comm/topside_rx']}] RECEIVED /comm/topside_rx (Relayed): {msg.data[:50]}...")

    def cb_topside_tx(self, msg):
        self.counts['/comm/topside_tx'] += 1
        print(f"[{self.counts['/comm/topside_tx']}] RECEIVED /comm/topside_tx: {msg.data[:50]}...")

    def cb_auv_rx(self, msg):
        self.counts['/comm/auv_rx'] += 1
        print(f"[{self.counts['/comm/auv_rx']}] RECEIVED /comm/auv_rx (Relayed): {msg.data[:50]}...")

def main(args=None):
    rclpy.init(args=args)
    node = LinkVerifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
