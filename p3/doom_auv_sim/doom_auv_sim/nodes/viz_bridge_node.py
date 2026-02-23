#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
import time
import random
from collections import deque

class VizBridgeNode(Node):
    def __init__(self):
        super().__init__('viz_bridge_node')
        self.get_logger().info("VISUALIZATION BRIDGE STARTED - REALISTIC LAG ENABLED")
        
        # Buffer for delayed messages
        self.msg_queue = deque()
        
        # Subscriptions (Ground Truth topics from main_sim_node)
        self.create_subscription(PoseStamped, '/sim/state', lambda m: self.queue_msg(m, '/auv/state'), 10)
        self.create_subscription(LaserScan, '/sim/sonar_scan', lambda m: self.queue_msg(m, '/auv/sonar_scan'), 10)
        self.create_subscription(PointCloud2, '/sim/point_cloud', lambda m: self.queue_msg(m, '/auv/point_cloud'), 10)
        self.create_subscription(PointCloud2, '/sim/map_cloud', lambda m: self.queue_msg(m, '/auv/map_cloud'), 10)
        self.create_subscription(Marker, '/sim/marker', lambda m: self.queue_msg(m, '/auv/marker'), 10)
        
        # Publishers (Delayed / Visualization topics for RViz)
        self.pub_pose = self.create_publisher(PoseStamped, '/auv/state', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/auv/sonar_scan', 10)
        self.pub_pcl = self.create_publisher(PointCloud2, '/auv/point_cloud', 10)
        self.pub_map = self.create_publisher(PointCloud2, '/auv/map_cloud', 10)
        self.pub_marker = self.create_publisher(Marker, '/auv/marker', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to process queue
        self.create_timer(0.05, self.process_queue)
        
    def queue_msg(self, msg, topic_out):
        # Simulate Delay based on "acoustic" constraints
        # Random delay between 0.1s and 0.3s (Smoother experience requested)
        delay = random.uniform(0.1, 0.3)
        release_time = time.time() + delay
        self.msg_queue.append((release_time, topic_out, msg))
        
    def process_queue(self):
        now = time.time()
        
        # Process queue
        # Since delays are random, the queue isn't strictly sorted by release_time.
        # We iterate through all items and publish ready ones.
        
        remaining = deque()
        
        while self.msg_queue:
            release_time, topic, msg = self.msg_queue.popleft()
            
            if now >= release_time:
                self.publish_msg(topic, msg)
            else:
                remaining.append((release_time, topic, msg))
        
        self.msg_queue = remaining

    def publish_msg(self, topic, msg):
        # We update the timestamp to 'now' so RViz accepts it as "current" visualization
        # even though the data is effectively old. This prevents RViz from discarding it
        # due to "Message too old" errors if TF is current.
        # However, for true realism, we might want to keep the old stamp, but then TF needs to handle history.
        # Given the task is visualization, updating stamp is safer for RViz stability while showing the "jumpy" movement.
        
        msg.header.stamp = self.get_clock().now().to_msg()
        
        if topic == '/auv/state':
            self.pub_pose.publish(msg)
            # Broadcast TF separately using the delayed pose
            t = TransformStamped()
            t.header.stamp = msg.header.stamp # Synced with message
            t.header.frame_id = "map"
            t.child_frame_id = "base_link"
            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z
            t.transform.rotation = msg.pose.orientation
            self.tf_broadcaster.sendTransform(t)
            
        elif topic == '/auv/sonar_scan':
            self.pub_scan.publish(msg)
        elif topic == '/auv/point_cloud':
            self.pub_pcl.publish(msg)
        elif topic == '/auv/map_cloud':
            self.pub_map.publish(msg)
        elif topic == '/auv/marker':
            self.pub_marker.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VizBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
