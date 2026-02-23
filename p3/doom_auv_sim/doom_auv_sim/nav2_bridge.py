#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('nav2_bridge')
        
        self.subscription_goal = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
            
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            '/auv/state',
            self.pose_callback,
            10)
            
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.current_pose = None
        self.target_pose = None
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Nav2 Bridge Started. Waiting for /goal_pose...")

    def goal_callback(self, msg):
        self.target_pose = msg
        self.get_logger().info(f"New Goal Received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def pose_callback(self, msg):
        self.current_pose = msg

    def control_loop(self):
        if self.current_pose is None or self.target_pose is None:
            return
            
        # Simple P-Controller
        dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        
        distance = math.sqrt(dx**2 + dy**2)
        target_heading = math.atan2(dy, dx)
        
        # Get current yaw from quaternion (simplified)
        # Assuming z/w are set correctly in doom_sim_node
        # qz = sin(yaw/2), qw = cos(yaw/2) -> yaw = 2 * atan2(qz, qw)
        qz = self.current_pose.pose.orientation.z
        qw = self.current_pose.pose.orientation.w
        current_yaw = 2 * math.atan2(qz, qw)
        
        heading_error = target_heading - current_yaw
        # Normalize angle
        while heading_error > math.pi: heading_error -= 2*math.pi
        while heading_error < -math.pi: heading_error += 2*math.pi
        
        cmd = Twist()
        
        if distance > 1.0: # Tolerance
            cmd.linear.x = min(distance, 1.0) # Cap speed
            cmd.angular.z = heading_error * 2.0 # P-gain
        else:
            # Reached
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            # self.target_pose = None # Keep holding position or stop?
            
        self.publisher_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
