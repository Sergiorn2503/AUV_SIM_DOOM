import numpy as np
import random

# Constants
TILE_SIZE = 32
MAP_SIZE = 100

class DynamicObstacle:
    def __init__(self, x, y, dx, dy, min_depth, max_depth, color_id):
        self.x = float(x)
        self.y = float(y)
        self.dx = dx
        self.dy = dy
        self.min_depth = min_depth
        self.max_depth = max_depth
        self.color_id = color_id # 3 for Ship (Red), 4 for Deep Creature (Blue), 5 for Static (Green/Brown)

class MapManager:
    def __init__(self):
        self.map = self.generate_map()
        self.fog_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=bool)
        self.dynamic_obstacles = []
        self.static_obstacles = []
        self.targets = []
        
        self._init_obstacles()
        self._init_targets()

    def generate_map(self):
        # 1 = Wall, 0 = Empty
        game_map = np.zeros((MAP_SIZE, MAP_SIZE), dtype=int)
        # Borders
        game_map[0, :] = 1
        game_map[-1, :] = 1
        game_map[:, 0] = 1
        game_map[:, -1] = 1
        
        # Random obstacles (Camarinal Sill features)
        for _ in range(50):
            rx, ry = np.random.randint(1, MAP_SIZE-1, 2)
            game_map[rx, ry] = 1
            
        return game_map

    def _init_obstacles(self):
        # Surface Ships (0-10m)
        for _ in range(10): 
            ox = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            oy = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            dx = random.choice([-2.0, -1.5, -1.0, 1.0, 1.5, 2.0])
            dy = random.choice([-2.0, -1.5, -1.0, 1.0, 1.5, 2.0])
            self.dynamic_obstacles.append(DynamicObstacle(ox, oy, dx, dy, 0, 10, 3))
            
        # Deep Creatures (40-60m)
        for _ in range(10):
            ox = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            oy = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            dx = random.choice([-1.0, -0.5, 0.5, 1.0])
            dy = random.choice([-1.0, -0.5, 0.5, 1.0])
            self.dynamic_obstacles.append(DynamicObstacle(ox, oy, dx, dy, 40, 60, 4))
            
        # Static Obstacles (Fixed Depth Ranges)
        # Shallow Reefs (17-37m)
        for _ in range(15):
            ox = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            oy = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            self.static_obstacles.append(DynamicObstacle(ox, oy, 0, 0, 17, 37, 5))
            
        # Deep Mines (70-90m)
        for _ in range(15):
            ox = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            oy = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            self.static_obstacles.append(DynamicObstacle(ox, oy, 0, 0, 70, 90, 6))

    def _init_targets(self):
        import math
        count = 0
        while count < 5:
            tx = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            ty = random.randint(10, MAP_SIZE-10) * TILE_SIZE
            
            # 1. Check if in Wall
            c = int(tx / TILE_SIZE)
            r = int(ty / TILE_SIZE)
            if self.map[r, c] == 1: continue
            
            # 2. Check if overlapping with Obstacles
            valid = True
            for obs in self.get_all_obstacles():
                dist = math.sqrt((tx - obs.x)**2 + (ty - obs.y)**2)
                if dist < 2.0 * TILE_SIZE: # Keep at least 2m away
                    valid = False
                    break
            
            if valid:
                td = random.randint(5, 95) 
                t_ori = random.randint(0, 360)
                self.targets.append({'x': tx, 'y': ty, 'depth': td, 'orientation': t_ori, 'collected': False})
                count += 1

    def update_dynamic_obstacles(self):
        for obs in self.dynamic_obstacles:
            obs.x += obs.dx
            obs.y += obs.dy
            
            # Bounce off borders
            if obs.x < TILE_SIZE or obs.x > (MAP_SIZE - 2) * TILE_SIZE: obs.dx *= -1
            if obs.y < TILE_SIZE or obs.y > (MAP_SIZE - 2) * TILE_SIZE: obs.dy *= -1

    def get_all_obstacles(self):
        return self.dynamic_obstacles + self.static_obstacles
