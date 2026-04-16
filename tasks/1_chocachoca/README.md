# Activity 1: Sweeping Robot (Chocachoca)

## Objective
Design and implement a reactive control system for a robot to cover the largest possible area of a confined space in the shortest time, avoiding collisions using LiDAR feedback.

## Implementation Details
The solution, named **Chocachoca**, uses a Finite State Machine (FSM) to switch between difference behaviors:

### State Machine
1. **SPIRAL**: The robot moves in an Archimedean spiral trajectory to maximize coverage in open areas. It increases linear speed ($v_s$) and decreases angular speed ($\omega_s$) gradually.
2. **TURN**: Triggered when an obstacle is detected within a threshold ($D_T \approx 600$ mm). The robot rotates until the path is clear.
3. **FORWARD**: Moves straight until an obstacle is detected.
4. **FOLLOW_WALL**: Maintains a constant distance from the wall using proportional control, ideal for perimeter coverage.

### Perception
- **LiDAR Filtering**: The raw point cloud is filtered to remove noise and grouped by angle to reduce computational load.
- **Visualisation**: A custom GUI displays the robot's state and the filtered LiDAR data in real-time.

## Dependencies
- **RoboComp**: Framework for component execution.
- **Webots**: Simulation environment.
- **Qt6**: For the Graphical User Interface.

## Configuration
Example configuration can be found in `etc/config`. Key parameters include:
- `Period.Compute`: Control loop cycle time (ms).
- `Robot.MaxSpeed`: Maximum linear velocity.

## Execution
To compile and run the component:

```bash
cd tasks/1_chocachoca
cmake -B build && make -C build -j$(nproc)
bin/chocachoca etc/config
```