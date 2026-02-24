# DOOM AUV Simulation

## Execution Instructions

To run the simulation, follow these steps:

1. **Build the workspace** (if not already built):
   ```bash
   colcon build
   ```

2. **Source the setup script**:
   ```bash
   source install/setup.bash
   ```

3. **Launch the simulation**:
   ```bash
   ros2 launch doom_auv_sim doom_sim.launch.py
   ```

4. **Run the Teleoperation Node** (in a separate terminal):
   ```bash
   source install/setup.bash
   ros2 run doom_auv_sim teleop_node
   ```

## Nodes
- `doom_sim_node`: Main simulation logic using Pygame.
- `acoustic_channel_node`: Simulates acoustic communication with delay/loss.
- `viz_bridge_node`: Bridges simulation state to RViz for visualization.
- `arbitrator_node`: Manages command arbitration between autonomous and manual control.
- `mission_node`: Handles autonomous mission logic.
- `teleop_node`: Provides manual control interface.
