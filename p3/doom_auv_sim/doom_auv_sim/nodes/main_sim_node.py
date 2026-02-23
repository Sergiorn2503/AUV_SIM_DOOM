#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import Header, String, Float32
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import pygame
import struct
import math
import time
from tf2_ros import TransformBroadcaster
from PIL import Image, ImageDraw

from doom_auv_sim.modules.map_manager import MapManager
from doom_auv_sim.modules.physics import PhysicsEngine
from doom_auv_sim.modules.renderer import DoomRenderer, MAX_DEPTH, TILE_SIZE

class DoomSimNode(Node):
    def __init__(self):
        super().__init__('doom_sim_node')
        print("\n" + "="*60)
        print("!!! DOOM SIMULATOR STARTING - CLEAN VERSION (v3) !!!")
        print("!!! NO GRAVITY - DEEP CONTROL FIX APPLIED !!!")
        print("!!! INITIAL DEPTH FORCED TO 10.0m !!!")
        print("="*60 + "\n")
        
        # Modules
        self.map_manager = MapManager()
        self.physics = PhysicsEngine(self.map_manager)
        
        # FORCE DEPTH RESET
        self.physics.depth = 10.0
        print(f"DEBUG: Physics Initialized. Depth = {self.physics.depth}")
        
        self.renderer = DoomRenderer()
        
        # ROS 2 Interface
        self.subscription = self.create_subscription(
            Twist,
            '/auv/thruster_cmd',
            self.thruster_cmd_callback,
            10)
            
        self.last_cmd_vel = Twist()
        self.last_cmd_time = 0.0
            
        self.sub_arm = self.create_subscription(String, '/auv/arm_cmd', self.arm_callback, 10)
        self.sub_sys = self.create_subscription(String, '/auv/sys_cmd', self.sys_callback, 10)
            
        # QoS Profile for Sensors
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pose_publisher = self.create_publisher(PoseStamped, '/sim/state', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/sim/sonar_scan', qos_sensor)
        self.pcl_publisher = self.create_publisher(PointCloud2, '/sim/point_cloud', qos_sensor)
        self.map_publisher = self.create_publisher(PointCloud2, '/sim/map_cloud', qos_sensor)
        self.marker_publisher = self.create_publisher(Marker, '/sim/marker', 10)
        # self.tf_broadcaster = TransformBroadcaster(self) # DISABLED: Bridge handles TF now
        
        # Timer for main loop
        self.timer = self.create_timer(0.05, self.game_loop) # 20 Hz
        self.map_timer = self.create_timer(1.0, self.publish_full_map) # 1 Hz for full map

        # Mission / Autonomy Interface
        from geometry_msgs.msg import PoseArray, Pose, Point
        self.target_publisher = self.create_publisher(PoseArray, '/auv/detected_targets', 10)
        self.event_publisher = self.create_publisher(String, '/auv/mission_event', 10)
        self.arm_state_publisher = self.create_publisher(Point, '/auv/arm_state', 10)
        self.target_screen_publisher = self.create_publisher(Point, '/auv/target_screen_pos', 10)
        self.battery_publisher = self.create_publisher(Float32, '/auv/battery', 10)

    def save_map(self):
        """Saves the current explored map to disk."""
        timestamp = time.strftime("%Y%m%d-%H%M%S")
        filename_pcd = f"map_{timestamp}.pcd"
        points = self.generate_map_points()
        with open(filename_pcd, 'w') as f:
            f.write("VERSION .7\n")
            f.write("FIELDS x y z rgb\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")
            f.write(f"WIDTH {len(points)}\n")
            f.write("HEIGHT 1\n")
            f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
            f.write(f"POINTS {len(points)}\n")
            f.write("DATA ascii\n")
            for p in points:
                f.write(f"{p[0]:.3f} {p[1]:.3f} {p[2]:.3f} {p[3]:.3e}\n")
        self.get_logger().info(f"Map saved to {filename_pcd}")
        
        # Also save as PNG
        self.save_map_png(timestamp)
        
        # Also save as OBJ (3D)
        self.save_map_obj(timestamp)
        return filename_pcd

    def save_map_obj(self, timestamp):
        """Saves the map as a Wavefront OBJ 3D model."""
        filename_obj = f"mission_result_{timestamp}.obj"
        
        vertices = []
        faces = []
        
        # Helper to add a cube
        # x, y, z are center coordinates
        # dx, dy, dz are half-dimensions (radius)
        def add_cube(cx, cy, Cz_top, Cz_bottom, r_xy):
            # Vertices
            # Top Face (z = Cz_top)
            v_start = len(vertices) + 1
            
            # 8 corners
            # Top: 
            vertices.append((cx - r_xy, cy - r_xy, Cz_top)) # 1: TL
            vertices.append((cx + r_xy, cy - r_xy, Cz_top)) # 2: TR
            vertices.append((cx + r_xy, cy + r_xy, Cz_top)) # 3: BR
            vertices.append((cx - r_xy, cy + r_xy, Cz_top)) # 4: BL
            
            # Bottom:
            vertices.append((cx - r_xy, cy - r_xy, Cz_bottom)) # 5: TL
            vertices.append((cx + r_xy, cy - r_xy, Cz_bottom)) # 6: TR
            vertices.append((cx + r_xy, cy + r_xy, Cz_bottom)) # 7: BR
            vertices.append((cx - r_xy, cy + r_xy, Cz_bottom)) # 8: BL
            
            # Faces (Quads)
            # Top
            faces.append((v_start, v_start+3, v_start+2, v_start+1))
            # Bottom
            faces.append((v_start+4, v_start+5, v_start+6, v_start+7))
            # Front (y+)
            faces.append((v_start+3, v_start+7, v_start+6, v_start+2))
            # Back (y-)
            faces.append((v_start, v_start+1, v_start+5, v_start+4))
            # Left (x-)
            faces.append((v_start, v_start+4, v_start+7, v_start+3))
            # Right (x+)
            faces.append((v_start+1, v_start+2, v_start+6, v_start+5))

        # 1. Walls
        map_w = self.map_manager.map.shape[1]
        map_h = self.map_manager.map.shape[0]
        
        for r in range(map_h):
            for c in range(map_w):
                 if self.map_manager.fog_map[r, c]:
                     if self.map_manager.map[r, c] == 1: # Wall
                         # Add Pillar (0 to -100)
                         # Grid coords: x=c, y=r
                         add_cube(float(c), float(r), 0.0, -100.0, 0.5)

        # 2. Obstacles
        all_obstacles = self.map_manager.get_all_obstacles()
        for obs in all_obstacles:
            c = int(obs.x / TILE_SIZE)
            r = int(obs.y / TILE_SIZE)
            if 0 <= c < map_w and 0 <= r < map_h:
                if self.map_manager.fog_map[r, c]:
                     # Add Cube
                     ox = obs.x / TILE_SIZE
                     oy = obs.y / TILE_SIZE
                     # Depth is positive downwards in physics, but negative z in 3D usually
                     z_top = -obs.min_depth
                     z_bot = -obs.max_depth
                     add_cube(ox, oy, z_top, z_bot, 0.5)
        
        with open(filename_obj, 'w') as f:
            f.write(f"# Doom AUV Sim Map - {timestamp}\n")
            for v in vertices:
                f.write(f"v {v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n")
            for face in faces:
                f.write(f"f {face[0]} {face[1]} {face[2]} {face[3]}\n")
                
        self.get_logger().info(f"3D Map Object saved to {filename_obj}")

    def save_map_png(self, timestamp):
        """Saves the map as a PNG image using PIL."""
        scale = 10 # Pixels per cell
        map_w = self.map_manager.map.shape[1]
        map_h = self.map_manager.map.shape[0]
        img_w = map_w * scale
        img_h = map_h * scale
        
        # Create Image (Black background for unexplored)
        img = Image.new('RGB', (img_w, img_h), color='black')
        draw = ImageDraw.Draw(img)
        
        # 1. Draw Map (Walls/Empty)
        for r in range(map_h):
            for c in range(map_w):
                if self.map_manager.fog_map[r, c]:
                    cell_id = self.map_manager.map[r, c]
                    x1 = c * scale
                    y1 = r * scale
                    x2 = x1 + scale
                    y2 = y1 + scale
                    
                    if cell_id == 1: # Wall
                        draw.rectangle([x1, y1, x2, y2], fill='gray')
                    else: # Empty (Water)
                        draw.rectangle([x1, y1, x2, y2], fill='blue')
                        
        # 2. Draw Obstacles
        all_obstacles = self.map_manager.get_all_obstacles()
        for obs in all_obstacles:
            # Check if obstacle center is explored
            c = int(obs.x / TILE_SIZE)
            r = int(obs.y / TILE_SIZE)
            
            if 0 <= c < map_w and 0 <= r < map_h:
                if self.map_manager.fog_map[r, c]:
                    # Draw Obstacle
                    ox = (obs.x / TILE_SIZE) * scale
                    oy = (obs.y / TILE_SIZE) * scale
                    
                    color = 'white'
                    if obs.color_id == 3: color = 'red' # Ship
                    elif obs.color_id == 4: color = 'cyan' # Creature
                    elif obs.color_id == 5: color = 'brown' # Reef
                    elif obs.color_id == 6: color = 'darkgray' # Mine
                    
                    # Draw as circle (radius ~0.5m -> 0.5 * scale)
                    rad = 0.5 * scale
                    draw.ellipse([ox-rad, oy-rad, ox+rad, oy+rad], fill=color)

        # 3. Draw Targets
        for t in self.map_manager.targets:
            # Check if target location is explored (or if it was seen/collected)
            c = int(t['x'] / TILE_SIZE)
            r = int(t['y'] / TILE_SIZE)
            
            if 0 <= c < map_w and 0 <= r < map_h:
                 # Logic: if collected -> Green, if seen but not collected -> Yellow
                 if t['collected']:
                     tx = (t['x'] / TILE_SIZE) * scale
                     ty = (t['y'] / TILE_SIZE) * scale
                     rad = 0.4 * scale
                     draw.ellipse([tx-rad, ty-rad, tx+rad, ty+rad], fill='green', outline='white')
                 elif self.map_manager.fog_map[r, c]:
                     tx = (t['x'] / TILE_SIZE) * scale
                     ty = (t['y'] / TILE_SIZE) * scale
                     rad = 0.4 * scale
                     draw.ellipse([tx-rad, ty-rad, tx+rad, ty+rad], fill='yellow', outline='black')

        # 4. Draw Trajectory (Robot Path) - Optional, but useful
        # We don't have history here easily unless we tracking it. 
        # Skipping for now to keep it simple.
        
        # 5. Draw Current Robot Position
        rx = (self.physics.x / TILE_SIZE) * scale
        ry = (self.physics.y / TILE_SIZE) * scale
        
        # Draw Robot Arrow or Circle
        draw.ellipse([rx-scale, ry-scale, rx+scale, ry+scale], outline='white', width=2)
        
        # Save
        filename_png = f"mission_result_{timestamp}.png"
        img.save(filename_png)
        self.get_logger().info(f"Map Image saved to {filename_png}")

    def generate_map_points(self):
        points = []
        for r in range(self.map_manager.map.shape[0]):
            for c in range(self.map_manager.map.shape[1]):
                if self.map_manager.fog_map[r, c]:
                    cell_id = self.map_manager.map[r, c]
                    mx = c 
                    my = r
                    if cell_id == 1: # Wall
                        rgb_int = (0 << 16) | (255 << 8) | 0
                        rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
                        for z in range(-100, 0, 5):
                            points.append([float(mx), float(my), float(z), rgb_float])
                    else:
                        rgb_int = (0 << 16) | (0 << 8) | (50)
                        rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
                        points.append([float(mx), float(my), -100.0, rgb_float])

        all_obstacles = self.map_manager.get_all_obstacles()
        for obs in all_obstacles:
            c = int(obs.x / TILE_SIZE)
            r = int(obs.y / TILE_SIZE)
            if 0 <= c < self.map_manager.map.shape[1] and 0 <= r < self.map_manager.map.shape[0]:
                if self.map_manager.fog_map[r, c]:
                    mx = obs.x / TILE_SIZE
                    my = obs.y / TILE_SIZE
                    r_col, g_col, b_col = (255, 255, 255)
                    if obs.color_id == 3: r_col, g_col, b_col = (255, 0, 0)
                    elif obs.color_id == 4: r_col, g_col, b_col = (0, 0, 255)
                    elif obs.color_id == 5: r_col, g_col, b_col = (139, 69, 19)
                    elif obs.color_id == 6: r_col, g_col, b_col = (50, 50, 50)
                    rgb_int = (int(r_col) << 16) | (int(g_col) << 8) | int(b_col)
                    rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
                    for z in range(int(-obs.max_depth), int(-obs.min_depth), 2):
                        points.append([mx, my, float(z), rgb_float])
                        points.append([mx + 0.5, my, float(z), rgb_float])
                        points.append([mx - 0.5, my, float(z), rgb_float])
                        points.append([mx, my + 0.5, float(z), rgb_float])
                        points.append([mx, my - 0.5, float(z), rgb_float])
        return points

    def publish_full_map(self):
        points = self.generate_map_points()
        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 16
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True
        buffer = []
        for p in points:
            buffer.append(struct.pack('ffff', p[0], p[1], p[2], p[3]))
        pc2.data = b"".join(buffer)
        self.map_publisher.publish(pc2)

    def thruster_cmd_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_time = time.time()

    def process_control_logic(self):
        # Apply Thruster Command (Simulate Motor Response / Acceleration)
        # ACELERACION SUAVE: Usamos FORCE constants para acumular velocidad lentamente
        # en lugar de asignarla directamente. La física (physics.py) se encarga del drag.
        
        FORCE_LINEAR = 0.2  # Reducido de 5.0 directo a 0.2 acumulativo
        FORCE_ANGULAR = 0.3
        FORCE_Z = 0.2
        
        if time.time() - self.last_cmd_time < 2.0:
            # Add to current velocity (Acceleration)
            self.physics.linear_vel += self.last_cmd_vel.linear.x * FORCE_LINEAR
            self.physics.angular_vel += self.last_cmd_vel.angular.z * FORCE_ANGULAR
            self.physics.z_vel += self.last_cmd_vel.linear.z * FORCE_Z
        else:
             # Si no hay comando, el drag en physics.py frenará el robot naturalmente.
             pass

    def publish_state(self):
        now = self.get_clock().now().to_msg()
        # Pose
        msg = PoseStamped()
        msg.header.stamp = now
        msg.header.frame_id = "map"
        msg.pose.position.x = self.physics.x / TILE_SIZE
        msg.pose.position.y = self.physics.y / TILE_SIZE
        msg.pose.position.z = -self.physics.depth
        cy = math.cos(self.physics.angle * 0.5)
        sy = math.sin(self.physics.angle * 0.5)
        msg.pose.orientation.z = sy
        msg.pose.orientation.w = cy
        self.pose_publisher.publish(msg)
        
        # TF
        # TF (DISABLED - Bridge handles it)
        # t = TransformStamped()
        # t.header.stamp = now
        # t.header.frame_id = "map"
        # t.child_frame_id = "base_link"
        # t.transform.translation.x = self.physics.x / TILE_SIZE
        # t.transform.translation.y = self.physics.y / TILE_SIZE
        # t.transform.translation.z = -self.physics.depth
        # t.transform.rotation.z = sy
        # t.transform.rotation.w = cy
        # self.tf_broadcaster.sendTransform(t)
        
        # Marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = now
        marker.ns = "auv"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 1.5
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        self.marker_publisher.publish(marker)

        # Publish Detected Targets
        from geometry_msgs.msg import PoseArray, Pose, Point
        target_msg = PoseArray()
        target_msg.header.stamp = now
        target_msg.header.frame_id = "map"
        for t in self.map_manager.targets:
            if not t['collected'] and t.get('visible', False):
                pose = Pose()
                pose.position.x = t['x'] / TILE_SIZE
                pose.position.y = t['y'] / TILE_SIZE
                pose.position.z = -float(t['depth'])
                rad = math.radians(t['orientation'])
                pose.orientation.z = math.sin(rad * 0.5)
                pose.orientation.w = math.cos(rad * 0.5)
                target_msg.poses.append(pose)
        self.target_publisher.publish(target_msg)
        
        # Publish Arm State
        arm_msg = Point()
        arm_msg.x = float(self.renderer.arm_x)
        arm_msg.y = float(self.renderer.arm_y)
        arm_msg.z = float(self.renderer.gripper_rotation)
        self.arm_state_publisher.publish(arm_msg)

        # Publish Target Screen Coordinates (For Arm Visual Servoing)
        # Find the first valid target (matching mission_node selection)
        target_screen_msg = Point()
        target_screen_msg.x = 320.0 # Default center
        target_screen_msg.y = 240.0
        for t in self.map_manager.targets:
             if not t['collected'] and t.get('visible', False):
                 # This is the target we are likely pursuing
                 target_screen_msg.x = float(t.get('screen_x', 320.0))
                 target_screen_msg.y = float(t.get('screen_y', 240.0))
                 break
        self.target_screen_publisher.publish(target_screen_msg)
        
        # Publish Battery
        bat_msg = Float32()
        bat_msg.data = float(self.physics.battery)
        self.battery_publisher.publish(bat_msg)

    def publish_scan(self, walls):
        # Create LaserScan message
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi / 3 / 2  # -30 deg
        scan.angle_max = math.pi / 3 / 2   # +30 deg
        scan.angle_increment = (math.pi / 3) / len(walls)
        scan.range_min = 0.1
        scan.range_max = 100.0
        
        ranges = []
        for depth, _, _, _, _ in walls:
            ranges.append(float(depth))
        
        # Reverse to match ROS standard (Left to Right) if needed, 
        # but configured CAST_RAYS usually goes -FOV/2 to +FOV/2 which is Right to Left in some engines 
        # or Left to Right. Assuming standard loop order matches logic.
        scan.ranges = ranges
        self.scan_publisher.publish(scan)

    def publish_point_cloud(self, walls):
        points = []
        # 1. Walls
        for depth, height, color, wx, wy in walls:
            if depth < MAX_DEPTH:
                for z in range(-100, 0, 5): 
                     mx = wx / TILE_SIZE
                     my = wy / TILE_SIZE
                     mz = z
                     r, g, b = color
                     rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                     rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
                     points.append([mx, my, mz, rgb_float])
        # 2. Obstacles
        all_obstacles = self.map_manager.get_all_obstacles()
        for obs in all_obstacles:
            if obs.min_depth <= self.physics.depth <= obs.max_depth:
                mx = obs.x / TILE_SIZE
                my = obs.y / TILE_SIZE
                r, g, b = (255, 255, 255)
                if obs.color_id == 3: r, g, b = (255, 0, 0)
                elif obs.color_id == 4: r, g, b = (0, 0, 255)
                elif obs.color_id == 5: r, g, b = (139, 69, 19)
                elif obs.color_id == 6: r, g, b = (50, 50, 50)
                rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
                for z in range(int(-obs.max_depth), int(-obs.min_depth), 2):
                    points.append([mx, my, z, rgb_float])
                    points.append([mx + 1, my, z, rgb_float])
                    points.append([mx - 1, my, z, rgb_float])
                    points.append([mx, my + 1, z, rgb_float])
                    points.append([mx, my - 1, z, rgb_float])

        header = Header()
        header.frame_id = "map"
        header.stamp = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 16
        pc2.row_step = pc2.point_step * pc2.width
        pc2.is_dense = True
        buffer = []
        for p in points:
            buffer.append(struct.pack('ffff', p[0], p[1], p[2], p[3]))
        pc2.data = b"".join(buffer)
        self.pcl_publisher.publish(pc2)

    def arm_callback(self, msg):
        cmd = msg.data
        if cmd == "up": self.renderer.arm_y -= 5 
        elif cmd == "down": self.renderer.arm_y += 5
        elif cmd == "left": self.renderer.arm_x -= 5
        elif cmd == "right": self.renderer.arm_x += 5
        elif cmd == "rot_left": self.renderer.gripper_rotation -= 2
        elif cmd == "rot_right": self.renderer.gripper_rotation += 2
        elif cmd == "grip":
            self.renderer.arm_open = not self.renderer.arm_open
            if self.renderer.try_collect_target(self.physics, self.map_manager):
                 msg = String()
                 msg.data = "target_collected"
                 self.event_publisher.publish(msg)
        elif cmd == "surface":
            self.emergency_surfacing = True 
        
        # Clamp
        if self.renderer.arm_x < 0: self.renderer.arm_x = 0
        if self.renderer.arm_x > self.renderer.screen.get_width(): self.renderer.arm_x = self.renderer.screen.get_width()
        if self.renderer.arm_y < 0: self.renderer.arm_y = 0
        if self.renderer.arm_y > self.renderer.screen.get_height(): self.renderer.arm_y = self.renderer.screen.get_height()

    def sys_callback(self, msg):
        if msg.data == "save_map":
            self.save_map()

    def game_loop(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                pygame.quit()
                return

        # Update Logic
        self.process_control_logic()
        self.physics.update()
        self.map_manager.update_dynamic_obstacles()
        
        # EMERGENCY ASCENT LOGIC (FIX: Reduced speed to 0.05)
        if getattr(self, 'emergency_surfacing', False):
            self.physics.depth -= 0.05 
            if self.physics.depth <= 0.0:
                self.physics.depth = 0.0
                self.emergency_surfacing = False 
            self.renderer.emergency_mode = True
        else:
            self.renderer.emergency_mode = False
        
        # Render
        walls = self.renderer.cast_rays(self.physics, self.map_manager)
        self.renderer.screen.fill((0, 0, 20)) 
        self.renderer.draw_3d(walls)
        self.renderer.draw_sprites(self.physics, self.map_manager)
        self.renderer.draw_hud(self.physics, self.map_manager)
        self.renderer.draw_sonar(self.physics, self.map_manager)
        self.renderer.draw_controls(self.physics)
        self.renderer.draw_arm()
        
        self.publish_state()
        self.publish_scan(walls)
        self.publish_point_cloud(walls)
        
        if int(time.time()) % 2 == 0 and int(time.time() * 10) % 10 == 0:
            self.get_logger().info(f"State: x={self.physics.x:.1f}, y={self.physics.y:.1f}, Depth={self.physics.depth:.1f}")
            
        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = DoomSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
