#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Point
from std_msgs.msg import Float32 # Added for battery sync
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import random
import time
import json

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.get_logger().info("ONBOARD AUTONOMY NODE STARTED (CRITICAL PANIC MODE ENABLED)")
        
        # Publishers
        self.pub_auto_vel = self.create_publisher(Twist, '/cmd_vel/autonomous', 10)
        self.pub_arm = self.create_publisher(String, '/auv/arm_cmd', 10)
        self.pub_tx = self.create_publisher(String, '/comm/auv_tx', 10)
        self.pub_mode_ctrl = self.create_publisher(String, '/auv/control_mode', 10) 
        self.pub_sys = self.create_publisher(String, '/auv/sys_cmd', 10) # Added for system commands 
        
        # Subscribers
        self.sub_rx = self.create_subscription(String, '/comm/auv_rx', self.rx_callback, 10)
        self.create_subscription(PoseStamped, '/auv/state', self.state_callback, 10)
        self.create_subscription(PoseArray, '/auv/detected_targets', self.target_callback, 10)
        self.create_subscription(String, '/auv/mission_event', self.event_callback, 10)
        self.create_subscription(LaserScan, '/auv/sonar_scan', self.scan_callback, 10)
        self.create_subscription(Point, '/auv/arm_state', self.arm_state_callback, 10)
        self.create_subscription(Float32, '/auv/battery', self.battery_callback, 10) # Added
        
        # Internal State
        self.mode = "MANUAL" 
        self.state = "EXPLORE" 
        self.robot_pose = None
        self.scan_data = None
        self.target_pose = None
        self.target_screen_pos = (320, 240)
        self.create_subscription(Point, '/auv/target_screen_pos', self.target_screen_callback, 10)
        self.arm_pos = (320, 240) 
        self.arm_angle = 0.0
        self.rejected_targets = []
        self.interaction_count = 0
        self.battery = 100.0 
        
        # Panic Mode State
        self.avoid_start_time = 0.0
        self.previous_state = "EXPLORE"

        # Waypoints
        self.waypoints = []
        for y in range(15, 95, 20): 
             row = range(15, 95, 20)
             if (y // 20) % 2 == 1: row = reversed(row) 
             for x in row:
                 self.waypoints.append((float(x), float(y), float(y % 40 + 20))) 
                 
        self.nav_goal = None 
        self.target_depth = 10.0 
        self.last_state_time = time.time()
        self.align_step = 0
        self.align_error = 0.0 # Added for telemetry
        
        # Timers
        self.create_timer(0.5, self.send_telemetry) # 2 Hz Telemetry
        self.create_timer(0.1, self.control_loop)   # 10 Hz Control
        
        self.avoid_turn_direction = 0.0 # Initialize safety variable

    def rx_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['type'] == 'mission':
                if data['action'] == 'start':
                    self.mode = "AUTO"
                    self.state = "EXPLORE"
                    self.get_logger().info("SWITCHING TO AUTO MODE")
                    m = String(); m.data = "AUTO"
                    self.pub_mode_ctrl.publish(m)
                elif data['action'] == 'stop':
                    self.mode = "MANUAL"
                    self.get_logger().info("SWITCHING TO MANUAL MODE")
                    m = String(); m.data = "MANUAL"
                    self.pub_mode_ctrl.publish(m)
                elif data['action'] == 'confirm':
                    if self.state == "WAIT_CONFIRM":
                        self.state = "APPROACH"
                elif data['action'] == 'reject':
                    if self.state == "WAIT_CONFIRM" and self.target_pose:
                        self.rejected_targets.append((self.target_pose.position.x, self.target_pose.position.y))
                        self.state = "EXPLORE"
                        self.target_pose = None
                        self.nav_goal = None
            elif data['type'] == 'arm':
                m = String()
                m.data = data['cmd']
                self.pub_arm.publish(m)
                if data['cmd'] == 'depth_up': self.target_depth -= 1.0
                elif data['cmd'] == 'depth_down': self.target_depth += 1.0
                self.target_depth = max(0.0, min(100.0, self.target_depth)) # Clamp
        except Exception as e:
            self.get_logger().error(f"RX Error: {e}")

    def send_telemetry(self):
        # Build strict JSON for Dashboard
        # Required keys: mode, state, battery, depth, targets_collected, nearest_obstacle
        
        current_depth = 0.0
        if self.robot_pose: current_depth = abs(self.robot_pose.position.z)
        
        min_dist = 99.9
        if self.scan_data:
             valid_ranges = [r for r in self.scan_data.ranges if r > 0.1]
             if valid_ranges: min_dist = min(valid_ranges)

        telemetry = {
            "mode": self.mode,
            "state": self.state if self.mode == "AUTO" else "STANDBY",
            "battery": round(self.battery, 1),
            "depth": round(current_depth, 1),
            "target_z": round(self.target_depth, 1),
            "targets_collected": self.interaction_count,
            "nearest_obstacle": round(min_dist, 1),
            "align_error": round(self.align_error, 1) if self.state == "ALIGN" else 0.0
        }
        
        msg = String()
        msg.data = json.dumps(telemetry)
        self.pub_tx.publish(msg)
        
        # Debug Log to confirm transmission
        # Check if we are sending WAIT_CONFIRM
        if self.state == "WAIT_CONFIRM":
             self.get_logger().info(f"TX Telemetry (WAIT_CONFIRM): {msg.data}") 
        else:
             self.get_logger().info(f"TX Telemetry: {msg.data[:50]}...") 

    def state_callback(self, msg):
        self.robot_pose = msg.pose
        self.last_state_rx_time = time.time() 
        # Removed internal battery simulation

    def battery_callback(self, msg):
        self.battery = msg.data

    def arm_state_callback(self, msg):
        self.arm_pos = (msg.x, msg.y)
        self.arm_angle = msg.z
                
    def scan_callback(self, msg):
        self.scan_data = msg

    def target_screen_callback(self, msg):
        self.target_screen_pos = (msg.x, msg.y)

    def target_callback(self, msg):
        if self.mode == "AUTO" and self.state == "EXPLORE" and len(msg.poses) > 0:
            potential_target = msg.poses[0]
            if self.is_rejected(potential_target): return
            
            self.target_pose = potential_target
            self.state = "WAIT_CONFIRM" 
            self.target_depth = abs(self.target_pose.position.z)
            self.get_logger().info(f"Target Found! Depth: {self.target_depth:.1f}m")
            self.send_telemetry() # FORCE UPDATE IMMEDIATELY

    def is_rejected(self, target):
        tx, ty = target.position.x, target.position.y
        for (rx, ry) in self.rejected_targets:
            if math.sqrt((tx-rx)**2 + (ty-ry)**2) < 5.0: return True
        return False

    def event_callback(self, msg):
        if msg.data == "target_collected":
            if self.mode == "AUTO":
                self.state = "RECOVER"
                self.last_state_time = time.time()
                self.target_pose = None
                self.interaction_count += 1
                if self.interaction_count >= 5:
                    self.get_logger().info("MISSION COMPLETE. SAVING MAP.")
                    
                    # Trigger Map Save
                    s = String(); s.data = "save_map"
                    self.pub_sys.publish(s)
                    
                    self.state = "SURFACING"
                    self.target_depth = 0.0

    def control_loop(self):
        # Watchdog
        if not hasattr(self, 'last_state_rx_time') or time.time() - self.last_state_rx_time > 2.0:
            if self.mode == "AUTO":
                self.mode = "MANUAL"
                self.get_logger().warn("LOST SIM CONNECTION -> MANUAL SAFE MODE")
            return

        if not self.robot_pose: return

        # --- BATTERY FAILSAFE (Critical Priority - WORKS IN MANUAL) ---
        if self.battery <= 10.0 and self.state != "SURFACING":
            self.get_logger().warn(f"Batería baja ({self.battery:.1f}%). Ascenso de emergencia iniciado.")
            self.mode = "AUTO" # Force Auto
            self.state = "SURFACING"
            
            # TRIGGER VISUAL WARNING (Link to main_sim_node)
            s_msg = String()
            s_msg.data = "surface"
            self.pub_arm.publish(s_msg)
            
            # Notify Arbitrator
            m = String(); m.data = "AUTO"
            self.pub_mode_ctrl.publish(m)

        # --- CRITICAL AVOIDANCE LOGIC (Highest Priority - OVERRIDES MANUAL) ---
        # Trigger Condition
        # Runs in ALL MODES to prevent collision
        if self.state != "AVOID_CRITICAL" and self.state != "EMERGENCY_REVERSE" and self.state != "RECOVER" and self.state != "SURFACING":
            if self.scan_data:
                valid_ranges = [r for r in self.scan_data.ranges if r > 0.1]
                if valid_ranges:
                    min_r = min(valid_ranges)
                    
                    # 1. CRITICAL EMERGENCY REVERSE (Very close < 1.5m)
                    if min_r < 1.5:
                         self.get_logger().error(f"CRITICAL PROXIMITY ({min_r:.2f}m)! ENGAGING EMERGENCY REVERSE.")
                         
                         # FORCE AUTO IF IN MANUAL
                         if self.mode == "MANUAL":
                             self.mode = "AUTO"
                             self.get_logger().warn("SAFETY INTERVENTION: AUTOPILOT ENGAGED")
                             m = String(); m.data = "AUTO"
                             self.pub_mode_ctrl.publish(m)

                         self.previous_state = self.state
                         self.state = "EMERGENCY_REVERSE"
                         self.avoid_start_time = now
                    
                    # 2. STANDARD AVOIDANCE (< 3.0m) - Increased from 2.5 for safety
                    elif min_r < 3.0: 
                        self.get_logger().warn(f"OBSTACLE TOO CLOSE (<3.0m)! Distance: {min_r:.2f}m. ENGAGING EVASION.")
                        
                        # FORCE AUTO IF IN MANUAL
                        if self.mode == "MANUAL":
                             self.mode = "AUTO"
                             self.get_logger().warn("SAFETY INTERVENTION: AUTOPILOT ENGAGED")
                             m = String(); m.data = "AUTO"
                             self.pub_mode_ctrl.publish(m)

                        self.previous_state = self.state
                        self.state = "AVOID_CRITICAL"
                        self.avoid_start_time = now
                        
                        # Determine best turn direction
                        count = len(self.scan_data.ranges)
                        mid = count // 2
                        left_sum = sum(self.scan_data.ranges[mid:])
                        right_sum = sum(self.scan_data.ranges[:mid])
                        
                        self.avoid_turn_direction = 1.0 if left_sum > right_sum else -1.0 # Turn towards more space
                        self.get_logger().info(f"Evasion Plan: Reverse & Turn {'LEFT' if self.avoid_turn_direction > 0 else 'RIGHT'}")


        if self.mode != "AUTO": return

        cmd = Twist()
        now = time.time()



        # State Execution
        if self.state == "EMERGENCY_REVERSE":
            # Action: FULL REVERSE, NO TURN (to clear immediate danger)
            cmd.linear.x = -1.0 
            cmd.angular.z = 0.0
            
            if self.state != "SURFACING": self.target_depth = abs(self.robot_pose.position.z)
            
            if now - self.avoid_start_time > 2.0: # 2s Reverse
                self.state = "AVOID_CRITICAL" # Then transition to standard avoidance to turn away
                self.avoid_start_time = now # Reset timer for next phase
                self.get_logger().info("EMERGENCY REVERSE DONE. SWITCHING TO ROTATIONAL EVASION.")

        elif self.state == "AVOID_CRITICAL":
            # MANDATORY ACTION: BACKUP & TURN AWAY
            cmd.linear.x = -0.5 # Reverse (halved speed for control)
            cmd.angular.z = self.avoid_turn_direction # Turn towards open space
            
            # Maintain Depth
            if self.state != "SURFACING": self.target_depth = abs(self.robot_pose.position.z)
            
            # Exit Condition (Timer) - Increased to 3.0s for better clearance
            if now - self.avoid_start_time > 3.0: 
                self.state = getattr(self, 'previous_state', "EXPLORE")
                self.get_logger().info("EVASION MANEUVER DONE. RESUMING.")
        
        # --- NORMAL MISSION STATES ---
        elif self.state == "WAIT_CONFIRM":
            cmd.linear.x = 0.0; cmd.angular.z = 0.0
            
        elif self.state == "SURFACING":
            cmd.linear.x = 0.0; cmd.angular.z = 0.0; self.target_depth = 0.0
            if self.robot_pose.position.z > -0.5: self.mode = "MANUAL"
            
        elif self.state == "EXPLORE":
            self.behavior_explore(cmd)
            
        elif self.state == "APPROACH":
            self.behavior_approach(cmd)
            
        elif self.state == "ALIGN":
            self.behavior_align(cmd)
            
        elif self.state == "RECOVER":
             if now - self.last_state_time > 2.0:
                 self.state = "EXPLORE"; self.nav_goal = None
        
        # --- DEPTH CONTROL (Always Active) ---
        current_depth = abs(self.robot_pose.position.z)
        depth_err = self.target_depth - current_depth
        # Reduced gain from 1.0 to 0.3 to prevent vertical oscillation
        cmd.linear.z = max(min(-0.3 * depth_err, 2.0), -2.0)
        
        self.pub_auto_vel.publish(cmd)

    def behavior_explore(self, cmd):
        if not self.nav_goal:
            if self.waypoints:
                self.nav_goal = self.waypoints.pop(0)
                self.target_depth = self.nav_goal[2]
            else:
                 self.nav_goal = (random.uniform(10, 90), random.uniform(10, 90), 40.0) 
            
        dx = self.nav_goal[0] - self.robot_pose.position.x
        dy = self.nav_goal[1] - self.robot_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 5.0: self.nav_goal = None; return

        goal_heading = math.atan2(dy, dx)
        q = self.robot_pose.orientation
        current_yaw = 2.0 * math.atan2(q.z, q.w)
        err_yaw = (goal_heading - current_yaw + math.pi) % (2*math.pi) - math.pi
        
        # Reduced gain from 1.5 to 0.5 to prevent oscillation
        cmd.angular.z = max(min(err_yaw * 0.5, 1.0), -1.0)
        cmd.linear.x = 1.0 if abs(err_yaw) < 0.5 else 0.2

    def behavior_approach(self, cmd):
        if not self.target_pose: self.state = "EXPLORE"; return

        dx = self.target_pose.position.x - self.robot_pose.position.x
        dy = self.target_pose.position.y - self.robot_pose.position.y
        dist = math.sqrt(dx*dx + dy*dy)
        
        goal_heading = math.atan2(dy, dx)
        q = self.robot_pose.orientation
        current_yaw = 2.0 * math.atan2(q.z, q.w)
        err_yaw = (goal_heading - current_yaw + math.pi) % (2*math.pi) - math.pi
        
        # Reduced gain to prevent aggressive turning
        cmd.angular.z = max(min(err_yaw * 0.3, 0.5), -0.5) 

        # Proportional Speed Control (Slow down as we get closer)
        # Max speed 0.5, Min speed 0.1
        target_speed = max(min(dist * 0.3, 0.5), 0.1)
        
        # Stop forward motion if we are turning too much or very close but not aligned
        if abs(err_yaw) > 0.2:
             cmd.linear.x = 0.0
        else:
             cmd.linear.x = target_speed
        
        current_depth = abs(self.robot_pose.position.z)
        
        # Relaxed Transition to ALIGN (0.7 -> 1.5m) to catch it earlier
        # FIXED: Relaxed params (Depth < 2.0, Yaw < 0.3) to prevent circling/overshoot
        if dist < 1.5 and abs(self.target_depth - current_depth) < 2.0 and abs(err_yaw) < 0.3:
            self.state = "ALIGN"; self.align_step = 0; self.last_state_time = time.time()
            cmd.linear.x = 0.0
            
        # Hard Brake to prevent fly-by if transition misses
        if dist < 0.5:
            cmd.linear.x = 0.0

    def behavior_align(self, cmd):
        if not self.target_pose: self.state = "EXPLORE"; return
        now = time.time(); dt = now - self.last_state_time
        
        # USE VISUAL SERVOING (Dynamic Target from Topic)
        target_x, target_y = self.target_screen_pos # Updated via callback
        cur_x, cur_y = self.arm_pos
        
        # --- CALCULATE ALIGNMENT ERROR CONTINUOUSLY ---
        # 1. Orientation (FIXED: Absolute Target Orientation)
        q_tgt = self.target_pose.orientation
        # Re-verify quaternion to yaw conversion just to be safe, but standard is:
        tgt_yaw = math.atan2(2.0*(q_tgt.w*q_tgt.z + q_tgt.x*q_tgt.y), 1.0 - 2.0*(q_tgt.y*q_tgt.y + q_tgt.z*q_tgt.z))
        
        # We DO NOT subtract robot_yaw because renderer compares gripper_rotation (absolute/state) 
        # directly with target['orientation'] (absolute).
        
        target_arm_deg = math.degrees(tgt_yaw)
        current_arm_deg = self.arm_angle
        
        diff_deg = target_arm_deg - current_arm_deg
        diff_deg = (diff_deg + 180) % 360 - 180
        
        self.align_error = diff_deg # Update Global State
        
        # Calculate Distance to Target (3D)
        dist_3d = math.sqrt((self.target_pose.position.x - self.robot_pose.position.x)**2 + 
                            (self.target_pose.position.y - self.robot_pose.position.y)**2)
        
        # Check if we are close enough to grab (Match Renderer logic ~1.5m)
        # We try to get closer than 1.5m to be safe (e.g., 0.8m - 1.2m)
        distance_ok = dist_3d < 1.2
        # -----------------------------------------------

        if self.align_step == 0: # Centering, Rotating & Approaching
            m = String()
            
            # --- BODY FROZEN: No rotation or lateral movement ---
            # The drone arrived facing the target from APPROACH.
            # Any body rotation moves the target on screen, creating a
            # feedback loop with arm corrections. Keep body completely still.
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
            
            # PRIORITY: Rotation BEFORE Translation (or concurrent if safe)
            # Relaxed rotation check to allow position tracking while rotating
            rotation_aligned = abs(diff_deg) < 5.0 
            rotation_good_enough = abs(diff_deg) < 15.0 # Check for interleaving
            
            # Position Adjustment (XY on Screen)
            # We want to be centered on screen
            pos_aligned = False
            
            # 1. First check Rotation (Strict)
            # BUT if we are "close enough" (15 deg), allow position fix to interleave
            if not rotation_aligned and (not rotation_good_enough or int(now*10)%2 == 0):
                 m.data = "rot_right" if diff_deg > 0 else "rot_left"
                 self.get_logger().info(f"Aligning Rot: {m.data} | Err: {diff_deg:.1f}")
            
            # 2. Then Position
            elif abs(cur_x - target_x) > 5: # Tightened to 5 pixels
                 m.data = "right" if cur_x < target_x else "left"
            elif abs(cur_y - target_y) > 5: # Tightened to 5 pixels
                 m.data = "down" if cur_y < target_y else "up"
            else:
                 pos_aligned = True
            
            # APPROACH LOGIC: Only creep forward when arm is centered and need to get closer
            if pos_aligned and rotation_aligned and not distance_ok and dist_3d > 0.15:
                 cmd.linear.x = 0.1 # Slow creep only when fully aligned
            else:
                 cmd.linear.x = 0.0
            
            # Debug Logic
            if int(now * 2) % 2 == 0:
                 self.get_logger().info(f"Grip Check: Rot={rotation_aligned} Pos={pos_aligned} Dist={distance_ok} | Err={diff_deg:.1f} Dist={dist_3d:.2f}")

            # Transition to GRIP only if EVERYTHING is ready
            if rotation_aligned and pos_aligned and distance_ok:
                self.align_step = 1; self.last_state_time = now; m.data = ""
            
            if m.data: 
                self.pub_arm.publish(m)
                # self.get_logger().info(f"Aligning Arm: {m.data}")

        elif self.align_step == 1: # Grip (Immediate)
             # We are already close and aligned. Just Grip.
             cmd.linear.x = 0.0
             
             m = String(); m.data = "grip"
             self.pub_arm.publish(m)
             self.align_step = 2; self.last_state_time = now
             
        elif self.align_step == 2: # Wait
             if dt > 2.0: 
                 self.align_step = 0 # Retry loop
                 # Ensure gripper is OPEN for next attempt (toggle)
                 # If we missed, we are likely closed. Toggle to open.
                 m = String(); m.data = "grip"
                 self.pub_arm.publish(m)

        # Abort if target lost or drifting away (increased to 4.0 margin)
        # FIX: prevent override of AVOID_CRITICAL
        if self.state in ["APPROACH", "ALIGN", "WAIT_CONFIRM"] and self.target_pose:
            dx = self.target_pose.position.x - self.robot_pose.position.x
            dy = self.target_pose.position.y - self.robot_pose.position.y
            if math.sqrt(dx*dx+dy*dy) > 4.0: self.state = "APPROACH"

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
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
