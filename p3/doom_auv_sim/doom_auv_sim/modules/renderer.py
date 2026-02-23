import pygame
import math
import time
import numpy as np

# Constants
SCREEN_WIDTH = 640
SCREEN_HEIGHT = 480
FOV = math.pi / 3
HALF_FOV = FOV / 2
CASTED_RAYS = 120
STEP_ANGLE = FOV / CASTED_RAYS
MAX_DEPTH = 100
TILE_SIZE = 32
MAP_SIZE = 100

# Colors
COLOR_BG = (0, 0, 20)
COLOR_HUD = (0, 255, 0)
COLOR_TEXT = (0, 255, 0)

class DoomRenderer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Girona 500 AUV - DOOM Sim")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont('monospace', 16, bold=True)
        self.font_warning = pygame.font.SysFont('monospace', 24, bold=True)
        
        # Arm State
        self.arm_x = SCREEN_WIDTH // 2
        self.arm_y = SCREEN_HEIGHT - 50
        self.arm_open = True
        self.arm_length = 100
        self.arm_angle = -math.pi / 2 
        self.gripper_rotation = 0.0 
        
        self.last_collection_time = 0
        self.collection_message = ""

    def cast_rays(self, physics, map_manager):
        start_angle = physics.angle - HALF_FOV
        walls = []
        
        for ray in range(CASTED_RAYS):
            for depth in range(MAX_DEPTH):
                target_x = physics.x + math.cos(start_angle) * depth * 5 
                target_y = physics.y + math.sin(start_angle) * depth * 5
                
                col = int(target_x / TILE_SIZE)
                row = int(target_y / TILE_SIZE)
                
                if 0 <= col < MAP_SIZE and 0 <= row < MAP_SIZE:
                    # Reveal Fog of War
                    map_manager.fog_map[row, col] = True
                    
                    cell_id = map_manager.map[row, col]
                    if cell_id == 1: # Wall
                        # Fix fisheye
                        depth *= math.cos(physics.angle - start_angle)
                        
                        # Calculate wall height
                        wall_height = 21000 / (depth + 0.0001)
                        
                        # Color based on depth (fog)
                        color_intensity = 255 / (1 + depth * 0.05)
                        color = (0, color_intensity, 0)
                        
                        walls.append((depth, wall_height, color, target_x, target_y))
                        break
            else:
                walls.append((MAX_DEPTH, 0, (0,0,0), 0, 0))
                
            start_angle += STEP_ANGLE
            
        return walls

    def draw_3d(self, walls):
        # Ceiling
        pygame.draw.rect(self.screen, (0, 0, 40), (0, 0, SCREEN_WIDTH, SCREEN_HEIGHT // 2))
        # Floor
        pygame.draw.rect(self.screen, (10, 10, 10), (0, SCREEN_HEIGHT // 2, SCREEN_WIDTH, SCREEN_HEIGHT // 2))
        
        scale = SCREEN_WIDTH // CASTED_RAYS
        
        for i, (depth, height, color, _, _) in enumerate(walls):
            pos_x = (CASTED_RAYS - 1 - i) * scale
            pos_y = (SCREEN_HEIGHT // 2) - (height // 2)
            pygame.draw.rect(self.screen, color, (pos_x, pos_y, scale, height))

    def draw_sprites(self, physics, map_manager):
        # Targets
        render_list = []
        for target in map_manager.targets:
            target['visible'] = False # Reset visibility
            if not target['collected']:
                render_list.append({
                    'type': 'target',
                    'x': target['x'],
                    'y': target['y'],
                    'depth': target['depth'],
                    'obj': target
                })
        
        # Obstacles
        all_obstacles = map_manager.get_all_obstacles()
        for obs in all_obstacles:
            render_list.append({
                'type': 'obstacle',
                'x': obs.x,
                'y': obs.y,
                'min_depth': obs.min_depth,
                'max_depth': obs.max_depth,
                'color_id': obs.color_id,
                'obj': obs
            })

        # Sort by distance
        for item in render_list:
            dx = item['x'] - physics.x
            dy = item['y'] - physics.y
            item['dist'] = math.sqrt(dx*dx + dy*dy)
            
        render_list.sort(key=lambda x: x['dist'], reverse=True)
        
        for item in render_list:
            dx = item['x'] - physics.x
            dy = item['y'] - physics.y
            
            sprite_angle = math.atan2(dy, dx) - physics.angle
            while sprite_angle < -math.pi: sprite_angle += 2 * math.pi
            while sprite_angle > math.pi: sprite_angle -= 2 * math.pi
            
            if -HALF_FOV < sprite_angle < HALF_FOV:
                screen_x = (HALF_FOV - sprite_angle) / FOV * SCREEN_WIDTH
                dist = item['dist']
                
                if item['type'] == 'obstacle':
                    obj_height_m = item['max_depth'] - item['min_depth']
                    sprite_height = int((obj_height_m / 10.0) * (2000 / (dist + 0.1)))
                    
                    cid = item['color_id']
                    aspect_ratio = 1.0
                    if cid == 3: aspect_ratio = 2.0 
                    elif cid == 4: aspect_ratio = 2.0 
                    elif cid == 5: aspect_ratio = 1.5 
                    elif cid == 6: aspect_ratio = 1.0 
                    
                    sprite_width = int(sprite_height * aspect_ratio)
                    
                else: 
                    sprite_height = int(2000 / (dist + 0.1))
                    sprite_width = sprite_height
                
                # Cap sizes
                if sprite_height < 5: sprite_height = 5
                if sprite_height > 600: sprite_height = 600
                if sprite_width < 5: sprite_width = 5
                if sprite_width > 800: sprite_width = 800
                
                # Y Position
                if item['type'] == 'obstacle':
                    obj_center_depth = (item['min_depth'] + item['max_depth']) / 2.0
                    delta_z = obj_center_depth - physics.depth
                else:
                    delta_z = item['depth'] - physics.depth
                    
                scale_y = 10000 / (dist + 0.1)
                screen_y = (SCREEN_HEIGHT // 2) + (delta_z * scale_y * 0.05)
                
                # Draw
                if item['type'] == 'target':
                    pygame.draw.circle(self.screen, (0, 255, 255), (int(screen_x), int(screen_y)), sprite_width // 2)
                    pygame.draw.circle(self.screen, (255, 255, 255), (int(screen_x), int(screen_y)), sprite_width // 2, 2)
                    
                    target = item['obj']
                    rad_ori = math.radians(target['orientation'])
                    end_x = screen_x + math.cos(rad_ori) * (sprite_width // 2)
                    end_y = screen_y + math.sin(rad_ori) * (sprite_width // 2)
                    pygame.draw.line(self.screen, (255, 0, 0), (int(screen_x), int(screen_y)), (int(end_x), int(end_y)), 3)
                    
                    target['screen_x'] = screen_x
                    target['screen_y'] = screen_y
                    target['visible'] = True
                    
                elif item['type'] == 'obstacle':
                    cid = item['color_id']
                    in_depth = item['min_depth'] <= physics.depth <= item['max_depth']
                    
                    if cid == 3: # Ship
                        w, h = sprite_width, sprite_height
                        rect = pygame.Rect(screen_x - w//2, screen_y - h//2, w, h)
                        color = (255, 0, 0) if in_depth else (100, 0, 0)
                        pygame.draw.polygon(self.screen, color, [
                            (rect.left, rect.top), (rect.right, rect.top),
                            (rect.right - w//4, rect.bottom), (rect.left + w//4, rect.bottom)
                        ])
                    elif cid == 4: # Whale
                        w, h = sprite_width, sprite_height
                        rect = pygame.Rect(screen_x - w//2, screen_y - h//2, w, h)
                        color = (0, 0, 255) if in_depth else (0, 0, 100)
                        pygame.draw.ellipse(self.screen, color, rect)
                        pygame.draw.polygon(self.screen, color, [
                            (rect.right, screen_y), (rect.right + w//4, screen_y - h//4), (rect.right + w//4, screen_y + h//4)
                        ])
                    elif cid == 5: # Reef
                        w, h = sprite_width, sprite_height
                        rect = pygame.Rect(screen_x - w//2, screen_y - h//2, w, h)
                        color = (139, 69, 19) if in_depth else (70, 35, 10)
                        pygame.draw.ellipse(self.screen, color, rect)
                    elif cid == 6: # Mine
                        r = min(sprite_width, sprite_height) // 2
                        color = (50, 50, 50) if in_depth else (25, 25, 25)
                        pygame.draw.circle(self.screen, color, (int(screen_x), int(screen_y)), r)
                        for ang in range(0, 360, 45):
                            rad = math.radians(ang)
                            sx = screen_x + math.cos(rad) * r
                            sy = screen_y + math.sin(rad) * r
                            ex = screen_x + math.cos(rad) * (r * 1.5)
                            ey = screen_y + math.sin(rad) * (r * 1.5)
                            pygame.draw.line(self.screen, (255, 0, 0), (sx, sy), (ex, ey), 2)
                            
                    label = f"Z:{item['min_depth']}-{item['max_depth']}m"
                    lbl_surf = self.font.render(label, 1, (255, 255, 255))
                    self.screen.blit(lbl_surf, (screen_x - lbl_surf.get_width()//2, screen_y - sprite_height // 2 - 20))

    def draw_hud(self, physics, map_manager):
        # Background
        pygame.draw.rect(self.screen, (0, 0, 0), (0, SCREEN_HEIGHT - 100, SCREEN_WIDTH, 100))
        pygame.draw.line(self.screen, COLOR_HUD, (0, SCREEN_HEIGHT - 100), (SCREEN_WIDTH, SCREEN_HEIGHT - 100), 2)
        
        acoustic_snr, optical_snr = physics.calculate_snr()
            
        collected_count = sum(1 for t in map_manager.targets if t['collected'])
        texts = [
            f"DEPTH: {physics.depth:.1f}m",
            f"BATTERY: {physics.battery:.1f}%",
            f"ACOUSTIC SNR: {acoustic_snr:.1f} dB",
            f"OPTICAL SNR: {optical_snr:.1f} dB",
            f"POS: ({physics.x/TILE_SIZE:.1f}, {physics.y/TILE_SIZE:.1f})",
            f"TARGETS: {collected_count}/{len(map_manager.targets)}"
        ]
        
        for i, text in enumerate(texts):
            label = self.font.render(text, 1, COLOR_TEXT)
            self.screen.blit(label, (20, SCREEN_HEIGHT - 90 + i * 18))
            
        cx, cy = SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2
        pygame.draw.line(self.screen, (0, 255, 0), (cx - 10, cy), (cx + 10, cy), 1)
        pygame.draw.line(self.screen, (0, 255, 0), (cx, cy - 10), (cx, cy + 10), 1)
        
        if physics.collision_warning:
            if physics.collision_distance < 1.7:
                 warn_text = f"AUTOPILOT: AVOIDING OBSTACLE! ({physics.collision_distance:.1f}m)"
            else:
                 warn_text = f"COLLISION WARNING! {physics.collision_distance:.1f}m"
            warn_label = self.font_warning.render(warn_text, 1, (255, 0, 0))
            if int(time.time() * 5) % 2 == 0:
                text_rect = warn_label.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT - 120))
                self.screen.blit(warn_label, text_rect)
                
        if time.time() - self.last_collection_time < 3.0:
            msg_label = self.font_warning.render(self.collection_message, 1, (0, 255, 255))
            text_rect = msg_label.get_rect(center=(SCREEN_WIDTH // 2, 100))
            self.screen.blit(msg_label, text_rect)

    def draw_sonar(self, physics, map_manager):
        map_scale = 1 
        sonar_size = MAP_SIZE * map_scale
        margin = 10
        
        sonar_rect = pygame.Rect(SCREEN_WIDTH - sonar_size - margin, margin, sonar_size, sonar_size)
        pygame.draw.rect(self.screen, (0, 50, 0), sonar_rect)
        pygame.draw.rect(self.screen, COLOR_HUD, sonar_rect, 1)
        
        for r in range(MAP_SIZE):
            for c in range(MAP_SIZE):
                if map_manager.fog_map[r, c]:
                    cell_id = map_manager.map[r, c]
                    rect = (sonar_rect.x + c * map_scale, sonar_rect.y + r * map_scale, map_scale, map_scale)
                    if cell_id == 1:
                        pygame.draw.rect(self.screen, (0, 150, 0), rect)
                    else:
                        pygame.draw.rect(self.screen, (0, 20, 0), rect)
        
        all_obstacles = map_manager.get_all_obstacles()
        for obs in all_obstacles:
            if obs.min_depth <= physics.depth <= obs.max_depth:
                tx = (obs.x / TILE_SIZE) * map_scale
                ty = (obs.y / TILE_SIZE) * map_scale
                
                color = (255, 255, 255)
                if obs.color_id == 3: color = (255, 0, 0)
                elif obs.color_id == 4: color = (0, 0, 255)
                elif obs.color_id == 5: color = (139, 69, 19)
                elif obs.color_id == 6: color = (100, 100, 100)
                
                pygame.draw.rect(self.screen, color, (sonar_rect.x + tx, sonar_rect.y + ty, 4, 4))

        for target in map_manager.targets:
            if not target['collected']:
                 tx = (target['x'] / TILE_SIZE) * map_scale
                 ty = (target['y'] / TILE_SIZE) * map_scale
                 pygame.draw.circle(self.screen, (0, 255, 255), (int(sonar_rect.x + tx), int(sonar_rect.y + ty)), 2)
                                     
        px = (physics.x / TILE_SIZE) * map_scale
        py = (physics.y / TILE_SIZE) * map_scale
        pygame.draw.circle(self.screen, (255, 0, 0), (int(sonar_rect.x + px), int(sonar_rect.y + py)), 3)
        
        end_x = px + math.cos(physics.angle) * 10
        end_y = py + math.sin(physics.angle) * 10
        pygame.draw.line(self.screen, (255, 0, 0), (sonar_rect.x + px, sonar_rect.y + py), (sonar_rect.x + end_x, sonar_rect.y + end_y), 1)

    def draw_controls(self, physics):
        center_x = SCREEN_WIDTH - 60
        center_y = SCREEN_HEIGHT - 160
        radius = 40
        
        pygame.draw.circle(self.screen, (0, 50, 0), (center_x, center_y), radius)
        pygame.draw.circle(self.screen, COLOR_HUD, (center_x, center_y), radius, 1)
        
        speed_ratio = physics.linear_vel / 5.0
        bar_height = int(speed_ratio * (radius - 5))
        pygame.draw.line(self.screen, (255, 255, 0), (center_x, center_y), (center_x, center_y - bar_height), 3)
        
        turn_ratio = physics.angular_vel / 1.0
        turn_len = int(turn_ratio * (radius - 5))
        pygame.draw.line(self.screen, (255, 255, 0), (center_x, center_y), (center_x + turn_len, center_y), 3)
        
        lbl = self.font.render("CTRL", 1, COLOR_HUD)
        self.screen.blit(lbl, (center_x - 20, center_y + radius + 5))

    def draw_arm(self):
        gripper_color = (0, 255, 0) if self.arm_open else (255, 0, 0)
        rad_grip = math.radians(self.gripper_rotation)
        
        pygame.draw.circle(self.screen, (100, 100, 100), (int(self.arm_x), int(self.arm_y)), 15)
        
        offset = 20 if self.arm_open else 10
        
        f1_x = self.arm_x + math.cos(rad_grip + math.pi) * offset
        f1_y = self.arm_y + math.sin(rad_grip + math.pi) * offset
        pygame.draw.circle(self.screen, gripper_color, (int(f1_x), int(f1_y)), 8)
        
        f2_x = self.arm_x + math.cos(rad_grip) * offset
        f2_y = self.arm_y + math.sin(rad_grip) * offset
        pygame.draw.circle(self.screen, gripper_color, (int(f2_x), int(f2_y)), 8)
        
        end_x = self.arm_x + math.cos(rad_grip) * 30
        end_y = self.arm_y + math.sin(rad_grip) * 30
        pygame.draw.line(self.screen, (255, 255, 0), (self.arm_x, self.arm_y), (end_x, end_y), 2)
        
        # instr_text = f"ARM: I/K(Y) J/L(X) U(GRIP) O/P(ROT {int(self.gripper_rotation)%360})"
        # lbl = self.font.render(instr_text, 1, (255, 255, 255))
        # self.screen.blit(lbl, (10, 10))
        
        # Emergency Mode Warning
        if getattr(self, 'emergency_mode', False):
             warn_font = pygame.font.SysFont('Arial', 48, bold=True)
             warn_lbl = warn_font.render("!! EMERGENCY ASCENT !!", 1, (255, 0, 0))
             cx = SCREEN_WIDTH // 2 - warn_lbl.get_width() // 2
             cy = SCREEN_HEIGHT // 4
             self.screen.blit(warn_lbl, (cx, cy))

    def update_arm(self, keys):
        if keys[pygame.K_i]: self.arm_y -= 5
        if keys[pygame.K_k]: self.arm_y += 5
        if keys[pygame.K_j]: self.arm_x -= 5
        if keys[pygame.K_l]: self.arm_x += 5
        if keys[pygame.K_o]: self.gripper_rotation -= 5
        if keys[pygame.K_p]: self.gripper_rotation += 5
        
        if self.arm_x < 0: self.arm_x = 0
        if self.arm_x > SCREEN_WIDTH: self.arm_x = SCREEN_WIDTH
        if self.arm_y < 0: self.arm_y = 0
        if self.arm_y > SCREEN_HEIGHT: self.arm_y = SCREEN_HEIGHT

    def try_collect_target(self, physics, map_manager):
        if not self.arm_open: # Closing gripper
            for target in map_manager.targets:
                if not target['collected'] and target.get('visible', False):
                    dx = target['x'] - physics.x
                    dy = target['y'] - physics.y
                    dist_real = math.sqrt(dx*dx + dy*dy)
                    
                    if dist_real < 1.5 * TILE_SIZE: # Increased to 1.5m
                        diff = abs(target['orientation'] - self.gripper_rotation) % 360
                        if diff > 180: diff = 360 - diff
                        
                        # Relaxed to 30 degrees
                        if diff < 30: 
                            sx = target['screen_x']
                            sy = target['screen_y']
                            d_screen = math.sqrt((sx - self.arm_x)**2 + (sy - self.arm_y)**2)
                            
                            if d_screen < 50: 
                                target['collected'] = True
                                self.collection_message = "TARGET COLLECTED!"
                                self.last_collection_time = time.time()
                                print("Target Collected!")
                                return True
                        else:
                             self.collection_message = f"ALIGNMENT ERROR {diff:.1f}° > 30°"
                             self.last_collection_time = time.time()
                    else:
                        self.collection_message = f"TOO FAR: {dist_real/TILE_SIZE:.2f}m > 1.5m"
                        self.last_collection_time = time.time()
        return False
