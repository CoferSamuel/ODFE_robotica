# Activity 2: Geometry-based Localiser

## Objective
Implement a localizer that estimates and incrementally corrects the robot's pose within a known rectangular room, compensating for odometry drift using 2D LiDAR data.

## Implementation Details
The system corrects the robot's position by matching detected environmental features (corners) with a known map.

### Algorithm Pipeline
1. **Line Extraction (RANSAC)**: Robustly fits lines to the LiDAR point cloud to identify room walls, filtering out noise.
2. **Corner Detection**: Computes intersections of perpendicular lines to find potential room corners.
3. **Data Association (Hungarian Algorithm)**: Matches detected corners with the map's varying corners by minimizing the total Euclidean distance.
4. **Pose Correction**: Solves a linearized least-squares system to compute small corrections $(\Delta x, \Delta y, \Delta \theta)$ and update the robot's pose.

### Modes
- **Estimation-only**: Calculates position without moving the robot or updating the odometry (passive).
- **Closed-loop**: Updates the robot's pose in real-time, allowing for accurate navigation.

## Dependencies
- **RoboComp**
- **Webots**
- **Eigen**: For efficient matrix operations and linear algebra solving.

## Execution
To compile and run the component:

```bash
cd tasks/2_localiser
cmake -B build && make -C build -j$(nproc)
bin/localiser etc/config
```
