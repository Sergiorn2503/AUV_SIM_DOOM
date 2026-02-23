import math
from doom_auv_sim.communication_logic import CommunicationPhysics

# Constants
TILE_SIZE = 32
MAP_SIZE = 100
BATTERY_DRAIN_IDLE = 0.001 # Per tick
BATTERY_DRAIN_MOVE = 0.01 # Per unit of movement/rotation

# Physics Constants (SUAVIZADO DE MOVIMIENTO)
WATER_DRAG_LINEAR = 0.92  # Retiene 92% de velocidad por tick (Simula inercia)
WATER_DRAG_ANGULAR = 0.85 # Mayor resistencia al giro para evitar oscilaciones

class PhysicsEngine:
    def __init__(self, map_manager):
        self.map_manager = map_manager
        self.comm_physics = CommunicationPhysics()
        
        # AUV State
        self.x = 50.0 * TILE_SIZE # Start in middle
        self.y = 50.0 * TILE_SIZE
        self.angle = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.z_vel = 0.0
        self.depth = 10.0 # meters
        self.battery = 100.0
        
        # Collision State
        self.collision_warning = False
        self.collision_distance = float('inf')

    def update(self):
        # Collision Avoidance Check
        # Cast ray forward
        check_dist = 20.0 # meters
        self.collision_warning = False
        self.collision_distance = float('inf')
        
        for d in range(1, int(check_dist) + 1):
            tx = self.x + math.cos(self.angle) * d * TILE_SIZE 
            ty = self.y + math.sin(self.angle) * d * TILE_SIZE
            
            c = int(tx / TILE_SIZE)
            r = int(ty / TILE_SIZE)
            
            if 0 <= c < MAP_SIZE and 0 <= r < MAP_SIZE:
                if self.map_manager.map[r, c] == 1: # Wall
                    self.collision_distance = float(d)
                    self.collision_warning = True
                    break
        
        # Check Obstacles (Dynamic + Static)
        all_obstacles = self.map_manager.get_all_obstacles()
        for obs in all_obstacles:
            # Check if active
            if obs.min_depth <= self.depth <= obs.max_depth:
                dx = obs.x - self.x
                dy = obs.y - self.y
                dist = math.sqrt(dx*dx + dy*dy)
                dist_m = dist / TILE_SIZE
                
                if dist_m < 20.0:
                    if dist_m < self.collision_distance:
                        self.collision_distance = dist_m
                        self.collision_warning = True
        
        # Emergency Stop
        if self.collision_warning and self.collision_distance < 0.9:
            # Allow reversing
            if self.linear_vel > 0:
                self.linear_vel = 0.0
            
        # Move AUV
        # 1. Apply Drag (Water Resistance) causing velocity decay
        # Esto previene que el robot se mueva infinitamente y suaviza el frenado
        self.linear_vel *= WATER_DRAG_LINEAR
        self.angular_vel *= WATER_DRAG_ANGULAR
        self.z_vel *= WATER_DRAG_LINEAR
        
        # Stop completely if very slow (deadband)
        if abs(self.linear_vel) < 0.01: self.linear_vel = 0.0
        if abs(self.angular_vel) < 0.01: self.angular_vel = 0.0
        if abs(self.z_vel) < 0.01: self.z_vel = 0.0

        # 2. Update Position
        self.angle += self.angular_vel * 0.1
        
        # Proposed new position
        new_x = self.x + math.cos(self.angle) * self.linear_vel
        new_y = self.y + math.sin(self.angle) * self.linear_vel
        
        # Check if new position is valid (Basic collision)
        col = int(new_x / TILE_SIZE)
        row = int(new_y / TILE_SIZE)
        
        can_move = True
        if 0 <= col < MAP_SIZE and 0 <= row < MAP_SIZE:
            if self.map_manager.map[row, col] == 1: # Wall
                can_move = False
                
        # Check Obstacle Collision for Movement
        for obs in all_obstacles:
            if obs.min_depth <= self.depth <= obs.max_depth:
                # Calculate distance from OLD position to check if we are already trapped
                dist_old = math.sqrt((obs.x - self.x)**2 + (obs.y - self.y)**2)
                
                # Check New Position
                dx = obs.x - new_x
                dy = obs.y - new_y
                dist_new = math.sqrt(dx*dx + dy*dy)
                
                if dist_new < TILE_SIZE: # 1 meter radius
                    # FIX: Allow moving AWAY if we are already stuck
                    if dist_new > dist_old: # We are escaping!
                        pass
                    else:
                        can_move = False
                        break
        
        # Emergency Stop Logic Override
        if self.collision_warning and self.collision_distance < 2.0 and self.linear_vel > 0:
            can_move = False
            self.linear_vel = 0.0 # Force stop visual
            
        if can_move:
            self.x = new_x
            self.y = new_y

        # Update Depth (Neutral Buoyancy)
        self.depth -= self.z_vel * 0.1 
        
        # Safety Clamps
        if self.depth < 0.0: self.depth = 0.0
        if self.depth > 120.0: self.depth = 120.0 # Just a safe max
        
        # Battery Consumption
        # Base drain
        drain = BATTERY_DRAIN_IDLE
        # Movement drain (proportional to effort)
        drain += abs(self.linear_vel) * 0.005
        drain += abs(self.angular_vel) * 0.01
        drain += abs(self.z_vel) * 0.01 # Add Z drain
        
        self.battery -= drain
        if self.battery < 0: self.battery = 0.0

    def calculate_snr(self):
        # Calculate SNR logic 
        virtual_buoys = [(20 * TILE_SIZE, 20 * TILE_SIZE), (80 * TILE_SIZE, 80 * TILE_SIZE)]
        min_dist = float('inf')
        for bx, by in virtual_buoys:
            dist = math.sqrt((self.x - bx)**2 + (self.y - by)**2) / TILE_SIZE
            if dist < min_dist:
                min_dist = dist
        
        acoustic_snr = self.comm_physics.calculate_acoustic_snr(min_dist, self.depth)
        optical_snr = self.comm_physics.calculate_optical_snr(min_dist)
        scintillation = self.comm_physics.check_scintillation((self.x, self.y))
        
        if scintillation:
            acoustic_snr = 0.0
            optical_snr = 0.0
            
        return acoustic_snr, optical_snr
