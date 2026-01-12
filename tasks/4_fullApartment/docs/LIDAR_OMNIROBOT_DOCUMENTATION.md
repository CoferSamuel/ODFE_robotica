# RoboComp Lidar3D and OmniRobot Interface Documentation

## Table of Contents
1. [Overview](#overview)
2. [Lidar3D Interface](#lidar3d-interface)
   - [Data Types](#lidar3d-data-types)
   - [Methods](#lidar3d-methods)
   - [Usage Examples](#lidar3d-usage-examples)
3. [OmniRobot Interface](#omnirobot-interface)
   - [Data Types](#omnirobot-data-types)
   - [Methods](#omnirobot-methods)
   - [Usage Examples](#omnirobot-usage-examples)
4. [GenericBase Types](#genericbase-types)
5. [Best Practices](#best-practices)

---

## Overview

This document provides comprehensive documentation for the **Lidar3D** and **OmniRobot** interfaces in RoboComp. These interfaces are used for controlling omnidirectional robots and reading 3D LiDAR sensor data.

Both interfaces are generated from Ice (Internet Communications Engine) definitions and provide both synchronous and asynchronous methods for communication.

---

## Lidar3D Interface

The Lidar3D interface provides methods to retrieve 3D point cloud data from LiDAR sensors. It supports multiple data formats and filtering options.

### Lidar3D Data Types

#### `TPoint`
Represents a single 3D point in the LiDAR point cloud.

```cpp
struct TPoint
{
    float x;           // X coordinate in millimeters (robot reference frame)
    float y;           // Y coordinate in millimeters (robot reference frame)
    float z;           // Z coordinate in millimeters (robot reference frame)
    int intensity;     // Reflection intensity value
    float phi;         // Horizontal angle in radians
    float theta;       // Vertical angle in radians
    float r;           // Distance from sensor origin in millimeters
    float distance2d;  // 2D distance (projection on XY plane) in millimeters
    int pixelX;        // X pixel coordinate if projected onto an image
    int pixelY;        // Y pixel coordinate if projected onto an image
};
```

**Field Descriptions:**
- **x, y, z**: Cartesian coordinates of the point in the robot's reference frame (typically centered at the robot's base)
- **intensity**: Reflection intensity, useful for detecting different material properties
- **phi**: Azimuth angle (horizontal rotation), measured from the robot's front
- **theta**: Elevation angle (vertical rotation)
- **r**: Euclidean distance from the sensor to the point
- **distance2d**: Distance in the horizontal plane, useful for 2D navigation
- **pixelX, pixelY**: Image coordinates when LiDAR data is projected onto a camera image

#### `TPoints`
```cpp
using TPoints = std::vector<TPoint>;
```
A collection of TPoint objects representing the entire point cloud.

#### `TData`
Main structure containing LiDAR scan data.

```cpp
struct TData
{
    TPoints points;          // Vector of 3D points
    float period;            // Scan period in milliseconds
    long long int timestamp; // Unix timestamp in milliseconds
};
```

**Field Descriptions:**
- **points**: The actual point cloud data
- **period**: Time between consecutive scans (useful for velocity estimation)
- **timestamp**: When the scan was captured, for temporal synchronization

#### `TDataImage`
LiDAR data organized as arrays, optimized for image projection.

```cpp
struct TDataImage
{
    long long int timestamp;  // Unix timestamp in milliseconds
    TFloatArray XArray;       // Array of X coordinates
    TFloatArray YArray;       // Array of Y coordinates
    TFloatArray ZArray;       // Array of Z coordinates
    TIntArray XPixel;         // Array of X pixel coordinates
    TIntArray YPixel;         // Array of Y pixel coordinates
};
```

**Use Case:** Efficient when projecting LiDAR points onto camera images or when processing data in array format.

#### `TDataCategory`
LiDAR data with semantic categories.

```cpp
struct TDataCategory
{
    TFloatArray XArray;          // Array of X coordinates
    TFloatArray YArray;          // Array of Y coordinates
    TFloatArray ZArray;          // Array of Z coordinates
    TCategories CategoriesArray; // Array of category IDs for each point
    float period;                // Scan period in milliseconds
    long long int timestamp;     // Unix timestamp in milliseconds
};
```

**Use Case:** When points have been classified (e.g., ground, walls, obstacles) using semantic segmentation or SLAM algorithms.

#### `TCategories`
```cpp
using TCategories = std::vector<int>;
```
Array of integer category identifiers.

#### Type Aliases
```cpp
using TFloatArray = std::vector<float>;
using TIntArray = std::vector<int>;
```

---

### Lidar3D Methods

#### 1. `getLidarData`
Retrieves LiDAR data for a specific angular range.

```cpp
TData getLidarData(
    const std::string& name,          // LiDAR sensor name (e.g., "pearl", "helios")
    float start,                      // Start angle in radians
    float len,                        // Angular length in radians
    int decimationDegreeFactor,       // Decimation factor (1 = no decimation)
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Parameters:**
- **name**: Identifier of the LiDAR sensor (configured in the component)
- **start**: Starting angle of the scan sector (0 = front, π/2 = left, -π/2 = right)
- **len**: Angular width of the sector to retrieve
- **decimationDegreeFactor**: Point reduction factor for downsampling the point cloud
  - **Value 1**: No decimation - returns all points (full resolution)
  - **Value 2**: Returns every 2nd point (50% reduction)
  - **Value 3**: Returns every 3rd point (66% reduction)
  - **Value N**: Returns every Nth point
  - **Example**: If LiDAR produces 10,000 points, decimation=5 returns ~2,000 points
  - **Use when**: Processing power is limited or you need faster updates over accuracy

**Returns:** `TData` containing the filtered point cloud.

**Example Use Cases:**
- Get only front-facing points: `start=0, len=π/4`
- Get full 360° scan: `start=0, len=2π`
- Reduce data rate: use `decimationDegreeFactor=2` or higher

**Understanding decimationDegreeFactor:**

The `decimationDegreeFactor` parameter controls point cloud density by skipping points. Think of it as a "thinning" factor:

```
Original points (decimation=1):  • • • • • • • • • •  (10 points)
Decimation=2:                    • - • - • - • - • -  (5 points, keep every 2nd)
Decimation=3:                    • - - • - - • - - •  (3-4 points, keep every 3rd)
Decimation=5:                    • - - - - • - - - -  (2 points, keep every 5th)
```

**When to use different values:**

| Factor | Points Kept | Use Case |
|--------|-------------|----------|
| 1 | 100% | High-precision mapping, detailed object detection |
| 2 | 50% | Balanced performance, most navigation tasks |
| 3-5 | 20-33% | Real-time obstacle avoidance, fast processing |
| 10+ | <10% | Rough environment sensing, very fast updates |

**Example:**
```cpp
// High detail for mapping (slow but accurate)
auto detailed = lidar3d_proxy->getLidarData("pearl", 0, 2*M_PI, 1);
// detailed.points.size() might be 50,000 points

// Fast obstacle detection (fast but less detail)
auto fast = lidar3d_proxy->getLidarData("pearl", 0, 2*M_PI, 5);
// fast.points.size() might be 10,000 points
```

---

#### 2. `getLidarDataWithThreshold2d`
Retrieves LiDAR data filtered by maximum 2D distance. **Returns full 360° scan** filtered by distance.

```cpp
TData getLidarDataWithThreshold2d(
    const std::string& name,          // LiDAR sensor name
    float distance,                   // Maximum 2D distance in millimeters
    int decimationDegreeFactor,       // Decimation factor
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Parameters:**
- **name**: LiDAR sensor identifier
- **distance**: Maximum horizontal distance threshold (points farther are excluded)
- **decimationDegreeFactor**: Point reduction factor (1=all points, 2=half, 3=one third, etc.)
  - Higher values = faster processing but lower spatial resolution
  - Lower values = more detail but slower processing

**Returns:** `TData` with points within the distance threshold from a **complete 360° scan**.

**Key Differences from `getLidarData`:**
- ✓ Always returns full 360° coverage (you cannot specify angular range)
- ✓ Filters by 2D distance threshold (removes far points)
- ✗ Cannot select specific angular sectors (use `getLidarData` for that)

**Example Usage:**
```cpp
// Get points within 12 meters, no decimation
auto data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 12000, 1);
```

**Use Cases:**
- Navigation: Focus on nearby obstacles
- Performance: Reduce point cloud size for real-time processing
- Range filtering: Remove distant noisy points

---

#### 3. `getLidarDataProyectedInImage`
Retrieves LiDAR data projected onto camera image coordinates.

```cpp
TData getLidarDataProyectedInImage(
    const std::string& name,          // LiDAR sensor name
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Returns:** `TData` with populated `pixelX` and `pixelY` fields in each `TPoint`.

**Use Cases:**
- Sensor fusion: Combine LiDAR and camera data
- Visualization: Overlay point cloud on camera images
- Object detection: Correlate 3D points with 2D image features

---

#### 4. `getLidarDataArrayProyectedInImage`
Similar to `getLidarDataProyectedInImage` but returns data in array format.

```cpp
TDataImage getLidarDataArrayProyectedInImage(
    const std::string& name,          // LiDAR sensor name
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Returns:** `TDataImage` with coordinate and pixel arrays.

**Advantages:**
- More efficient for array-based processing
- Better cache locality
- Easier to interface with numpy/OpenCV

---

#### 5. `getLidarDataByCategory`
Retrieves LiDAR data filtered by semantic categories.

```cpp
TDataCategory getLidarDataByCategory(
    const TCategories& categories,    // Vector of category IDs to retrieve
    long long int timestamp,          // Timestamp for synchronization
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Parameters:**
- **categories**: List of category IDs to include (e.g., {1, 3, 5} for specific object types)
- **timestamp**: Reference timestamp for data synchronization

**Returns:** `TDataCategory` containing only points matching the specified categories.

**Example Categories:**
- 0: Unknown/unclassified
- 1: Ground
- 2: Walls
- 3: Dynamic obstacles
- 4: Static obstacles
- 5: Ceiling

---

### Lidar3D Usage Examples

#### Example 1: Basic Point Cloud Acquisition
```cpp
// Get all points within 10 meters
auto data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 10000, 1);

// Access individual points
for (const auto& point : data.points) {
    std::cout << "Point: x=" << point.x 
              << " y=" << point.y 
              << " z=" << point.z 
              << " distance=" << point.distance2d << std::endl;
}
```

#### Example 2: Filtering Points by Angle
```cpp
// Get front-facing points (±45 degrees)
float start = -M_PI/4;  // -45 degrees
float len = M_PI/2;     // 90 degrees total
auto data = lidar3d_proxy->getLidarData("helios", start, len, 1);

// Process only front points
std::cout << "Front points: " << data.points.size() << std::endl;
```

#### Example 3: Distance-Based Obstacle Detection
```cpp
// Get nearby points
auto data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 2000, 1);

// Find closest obstacle
float min_distance = std::numeric_limits<float>::max();
TPoint closest_point;

for (const auto& point : data.points) {
    if (point.distance2d < min_distance) {
        min_distance = point.distance2d;
        closest_point = point;
    }
}

std::cout << "Closest obstacle at: " << min_distance << " mm" << std::endl;
```

#### Example 4: Asynchronous Data Acquisition
```cpp
// Non-blocking call
auto future = lidar3d_proxy->getLidarDataWithThreshold2dAsync("pearl", 12000, 1);

// Do other work...
doOtherProcessing();

// Get result when needed
auto data = future.get();
```

#### Example 5: Filtering and Processing
```cpp
// Custom filtering function
auto filter_by_height = [](const TPoints& points) {
    TPoints filtered;
    for (const auto& p : points) {
        // Keep only points between 100mm and 2000mm height
        if (p.z > 100 && p.z < 2000) {
            filtered.push_back(p);
        }
    }
    return filtered;
};

// Get data and filter
auto data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 12000, 1);
auto filtered_points = filter_by_height(data.points);
```

---

## OmniRobot Interface

The OmniRobot interface controls omnidirectional robots, providing methods for motion control and odometry management.

### OmniRobot Data Types

#### `TMechParams`
Mechanical parameters of the robot.

```cpp
struct TMechParams
{
    float temp;         // Motor/driver temperature in Celsius
    std::string device; // Hardware device identifier
    std::string handler;// Driver/handler name
    float maxVelAdv;    // Maximum advance velocity (mm/s)
    float maxVelRot;    // Maximum rotation velocity (rad/s)
};
```

**Field Descriptions:**
- **temp**: Current temperature for thermal monitoring
- **device**: Hardware interface (e.g., "/dev/ttyUSB0", "CAN0")
- **handler**: Software driver identifier
- **maxVelAdv**: Hardware limit for linear speed
- **maxVelRot**: Hardware limit for angular speed

---

### OmniRobot Methods

#### 1. `setSpeedBase`
Sets the velocity of the omnidirectional base.

```cpp
void setSpeedBase(
    float advx,                       // Forward/backward velocity (mm/s)
    float advz,                       // Lateral (strafe) velocity (mm/s)
    float rot,                        // Rotation velocity (rad/s)
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Parameters:**
- **advx**: Velocity along robot's X-axis (forward=positive, backward=negative)
- **advz**: Velocity along robot's Z-axis (left=positive, right=negative)
- **rot**: Angular velocity (counterclockwise=positive, clockwise=negative)

**Coordinate System:**
- **X-axis**: Forward direction of the robot
- **Z-axis**: Lateral direction (perpendicular to forward)
- **Rotation**: Around vertical Y-axis

**Example:**
```cpp
// Move forward at 300 mm/s
omnirobot_proxy->setSpeedBase(300, 0, 0);

// Strafe left at 200 mm/s
omnirobot_proxy->setSpeedBase(0, 200, 0);

// Rotate counterclockwise at 0.5 rad/s
omnirobot_proxy->setSpeedBase(0, 0, 0.5);

// Combined motion: forward + rotate
omnirobot_proxy->setSpeedBase(300, 0, 0.3);
```

---

#### 2. `stopBase`
Immediately stops the robot.

```cpp
void stopBase(const Ice::Context& context = Ice::noExplicitContext);
```

**Use Cases:**
- Emergency stops
- End of motion sequence
- Obstacle detection response

**Example:**
```cpp
// Emergency stop
omnirobot_proxy->stopBase();
```

---

#### 3. `getBasePose`
Retrieves the current robot pose from odometry.

```cpp
void getBasePose(
    int& x,                           // Output: X position (mm)
    int& z,                           // Output: Z position (mm)
    float& alpha,                     // Output: Orientation (radians)
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Parameters:**
- **x**: X coordinate in global reference frame (millimeters)
- **z**: Z coordinate in global reference frame (millimeters)
- **alpha**: Robot heading angle in radians (0 = facing +X axis)

**Example:**
```cpp
int x, z;
float alpha;
omnirobot_proxy->getBasePose(x, z, alpha);

std::cout << "Robot position: (" << x << ", " << z << ")" << std::endl;
std::cout << "Robot heading: " << alpha << " rad" << std::endl;
```

---

#### 4. `getBaseState`
Retrieves comprehensive robot state including velocities.

```cpp
void getBaseState(
    RoboCompGenericBase::TBaseState& state,  // Output: Complete state
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Returns:** `TBaseState` structure (see GenericBase Types section).

**Example:**
```cpp
RoboCompGenericBase::TBaseState state;
omnirobot_proxy->getBaseState(state);

std::cout << "Position: (" << state.x << ", " << state.z << ")" << std::endl;
std::cout << "Corrected position: (" << state.correctedX << ", " << state.correctedZ << ")" << std::endl;
std::cout << "Velocity: " << state.advVx << " mm/s" << std::endl;
std::cout << "Is moving: " << (state.isMoving ? "Yes" : "No") << std::endl;
```

---

#### 5. `resetOdometer`
Resets odometry to origin (0, 0, 0).

```cpp
void resetOdometer(const Ice::Context& context = Ice::noExplicitContext);
```

**Use Cases:**
- Starting position initialization
- After relocalization
- When odometry drift is too large

**Example:**
```cpp
// Reset to origin
omnirobot_proxy->resetOdometer();
```

---

#### 6. `setOdometerPose`
Manually sets the odometry pose.

```cpp
void setOdometerPose(
    int x,                            // X position (mm)
    int z,                            // Z position (mm)
    float alpha,                      // Orientation (radians)
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Use Cases:**
- Relocalization after visual SLAM
- Correction from external localization system
- Initial pose setting

**Example:**
```cpp
// Set robot at position (1000, 500) facing 45 degrees
omnirobot_proxy->setOdometerPose(1000, 500, M_PI/4);
```

---

#### 7. `setOdometer`
Sets complete odometer state.

```cpp
void setOdometer(
    const RoboCompGenericBase::TBaseState& state,  // Complete state to set
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Use Case:** Restore a previously saved complete state.

---

#### 8. `correctOdometer`
Applies a correction to current odometry.

```cpp
void correctOdometer(
    int x,                            // X correction (mm)
    int z,                            // Z correction (mm)
    float alpha,                      // Angle correction (radians)
    const Ice::Context& context = Ice::noExplicitContext
);
```

**Use Cases:**
- Incremental corrections from loop closure
- Gradual drift compensation
- Sensor fusion updates

**Example:**
```cpp
// Apply small correction
omnirobot_proxy->correctOdometer(50, -30, 0.02);
```

---

### OmniRobot Usage Examples

#### Example 1: Basic Movement Control
```cpp
// Move forward for 2 seconds
omnirobot_proxy->setSpeedBase(300, 0, 0);
std::this_thread::sleep_for(std::chrono::seconds(2));
omnirobot_proxy->stopBase();

// Strafe right for 1 second
omnirobot_proxy->setSpeedBase(0, -200, 0);
std::this_thread::sleep_for(std::chrono::seconds(1));
omnirobot_proxy->stopBase();
```

#### Example 2: Target-Based Navigation
```cpp
// Function to move to a target position
void moveToTarget(float target_x, float target_z) {
    const float KP_LINEAR = 0.5;  // Proportional gain for linear motion
    const float KP_ANGULAR = 1.0; // Proportional gain for rotation
    
    while (true) {
        // Get current pose
        int x, z;
        float alpha;
        omnirobot_proxy->getBasePose(x, z, alpha);
        
        // Calculate error
        float dx = target_x - x;
        float dz = target_z - z;
        float distance = sqrt(dx*dx + dz*dz);
        
        // Check if reached
        if (distance < 50) {  // 50mm threshold
            omnirobot_proxy->stopBase();
            break;
        }
        
        // Calculate desired angle
        float target_angle = atan2(dz, dx);
        float angle_error = target_angle - alpha;
        
        // Normalize angle error to [-π, π]
        while (angle_error > M_PI) angle_error -= 2*M_PI;
        while (angle_error < -M_PI) angle_error += 2*M_PI;
        
        // Calculate velocities
        float adv = KP_LINEAR * distance;
        float rot = KP_ANGULAR * angle_error;
        
        // Apply limits
        adv = std::clamp(adv, -500.0f, 500.0f);
        rot = std::clamp(rot, -0.5f, 0.5f);
        
        // Send command
        omnirobot_proxy->setSpeedBase(adv, 0, rot);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Use it
moveToTarget(2000, 1500);  // Move to (2000mm, 1500mm)
```

#### Example 3: Omnidirectional Motion
```cpp
// Move diagonally forward-right while rotating
omnirobot_proxy->setSpeedBase(
    300,   // Forward at 300 mm/s
    -200,  // Right at 200 mm/s
    0.1    // Slight counterclockwise rotation
);

std::this_thread::sleep_for(std::chrono::seconds(3));
omnirobot_proxy->stopBase();
```

#### Example 4: Odometry Monitoring
```cpp
// Monitor position over time
for (int i = 0; i < 100; i++) {
    RoboCompGenericBase::TBaseState state;
    omnirobot_proxy->getBaseState(state);
    
    std::cout << "Time: " << i*0.1 << "s | "
              << "Pos: (" << state.x << ", " << state.z << ") | "
              << "Angle: " << state.alpha << " | "
              << "Vel: " << state.advVx << " mm/s" << std::endl;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
```

#### Example 5: Obstacle Avoidance with LiDAR Integration
```cpp
void obstacleAvoidance() {
    while (true) {
        // Get LiDAR data
        auto data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 5000, 1);
        
        // Find minimum distance in front
        float min_front_dist = std::numeric_limits<float>::max();
        for (const auto& point : data.points) {
            // Check points in front sector (±30 degrees)
            if (abs(point.phi) < M_PI/6 && point.distance2d < min_front_dist) {
                min_front_dist = point.distance2d;
            }
        }
        
        // Control logic
        if (min_front_dist < 500) {  // Too close
            omnirobot_proxy->stopBase();
            std::cout << "Obstacle too close!" << std::endl;
            break;
        } else if (min_front_dist < 1000) {  // Slow down
            omnirobot_proxy->setSpeedBase(100, 0, 0);
        } else {  // Safe to move
            omnirobot_proxy->setSpeedBase(300, 0, 0);
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
```

#### Example 6: Circular Motion
```cpp
// Move in a circle
float radius = 1000;  // mm
float angular_vel = 0.3;  // rad/s
float linear_vel = radius * angular_vel;  // v = r * ω

omnirobot_proxy->setSpeedBase(linear_vel, 0, angular_vel);

// Move for one complete circle (2π / ω seconds)
float duration = (2 * M_PI) / angular_vel;
std::this_thread::sleep_for(std::chrono::duration<float>(duration));

omnirobot_proxy->stopBase();
```

#### Example 7: Relocalization After Loop Closure
```cpp
// Detect loop closure and correct odometry
void handleLoopClosure(int detected_x, int detected_z, float detected_angle) {
    // Get current odometry
    int odom_x, odom_z;
    float odom_alpha;
    omnirobot_proxy->getBasePose(odom_x, odom_z, odom_alpha);
    
    // Calculate correction
    int correction_x = detected_x - odom_x;
    int correction_z = detected_z - odom_z;
    float correction_alpha = detected_angle - odom_alpha;
    
    // Apply correction
    omnirobot_proxy->correctOdometer(correction_x, correction_z, correction_alpha);
    
    std::cout << "Applied odometry correction: (" 
              << correction_x << ", " << correction_z << ", " 
              << correction_alpha << ")" << std::endl;
}
```

---

## GenericBase Types

### `TBaseState`
Complete robot base state information.

```cpp
struct TBaseState
{
    float x;              // X position from odometry (mm)
    float correctedX;     // X position after corrections (mm)
    float z;              // Z position from odometry (mm)
    float correctedZ;     // Z position after corrections (mm)
    float alpha;          // Orientation from odometry (radians)
    float correctedAlpha; // Orientation after corrections (radians)
    float advVx;          // Current forward velocity (mm/s)
    float advVz;          // Current lateral velocity (mm/s)
    float rotV;           // Current rotational velocity (rad/s)
    bool isMoving;        // True if robot is in motion
};
```

**Field Descriptions:**

| Field | Description |
|-------|-------------|
| `x` | Raw odometry X position in millimeters |
| `correctedX` | Corrected X position after applying external corrections (SLAM, visual odometry) |
| `z` | Raw odometry Z position in millimeters |
| `correctedZ` | Corrected Z position after applying external corrections |
| `alpha` | Raw odometry heading angle in radians |
| `correctedAlpha` | Corrected heading after applying external corrections |
| `advVx` | Instantaneous forward/backward velocity in mm/s |
| `advVz` | Instantaneous lateral (strafe) velocity in mm/s |
| `rotV` | Instantaneous rotational velocity in rad/s |
| `isMoving` | Motion flag, useful for knowing when commands have taken effect |

**Use Cases:**
- **Raw vs Corrected**: Use raw values for short-term motion control; use corrected values for global localization
- **Velocity Monitoring**: Check actual velocities to ensure commands are executed
- **Motion Detection**: Use `isMoving` flag to synchronize actions with robot motion

---

## Best Practices

### LiDAR Data Processing

1. **Distance Filtering**
   ```cpp
   // Always use threshold methods when possible for efficiency
   auto data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 10000, 1);
   ```

2. **Decimation for Performance**
   ```cpp
   // Use decimation when processing speed is critical
   auto data = lidar3d_proxy->getLidarData("helios", 0, 2*M_PI, 2);  // Every other point
   ```

3. **Angle-Based Filtering**
   ```cpp
   // Filter specific sectors for targeted obstacle detection
   auto filter_sector = [](const TPoints& points, float center_angle, float width) {
       TPoints filtered;
       float half_width = width / 2;
       for (const auto& p : points) {
           float angle_diff = abs(p.phi - center_angle);
           if (angle_diff < half_width) {
               filtered.push_back(p);
           }
       }
       return filtered;
   };
   ```

4. **Height-Based Segmentation**
   ```cpp
   // Separate ground, obstacles, and overhead points
   TPoints ground_points, obstacle_points, overhead_points;
   for (const auto& p : data.points) {
       if (p.z < 100) {
           ground_points.push_back(p);
       } else if (p.z < 2000) {
           obstacle_points.push_back(p);
       } else {
           overhead_points.push_back(p);
       }
   }
   ```

### Robot Motion Control

1. **Velocity Limiting**
   ```cpp
   // Always clamp velocities to safe limits
   float adv = std::clamp(desired_adv, -500.0f, 500.0f);
   float rot = std::clamp(desired_rot, -0.5f, 0.5f);
   omnirobot_proxy->setSpeedBase(adv, 0, rot);
   ```

2. **Smooth Acceleration**
   ```cpp
   // Gradually increase velocity to avoid jerky motion
   float target_vel = 500;
   float current_vel = 0;
   float accel = 50;  // mm/s²
   
   while (current_vel < target_vel) {
       current_vel += accel * 0.1;  // 100ms timestep
       omnirobot_proxy->setSpeedBase(current_vel, 0, 0);
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
   }
   ```

3. **Emergency Stop Handling**
   ```cpp
   // Always have a way to stop quickly
   try {
       omnirobot_proxy->setSpeedBase(300, 0, 0);
   } catch (const std::exception& e) {
       omnirobot_proxy->stopBase();
       throw;
   }
   ```

4. **Dead Reckoning Compensation**
   ```cpp
   // Periodically check if you've reached expected position
   void checkOdometryDrift(int expected_x, int expected_z) {
       int actual_x, actual_z;
       float alpha;
       omnirobot_proxy->getBasePose(actual_x, actual_z, alpha);
       
       float error = sqrt(pow(expected_x - actual_x, 2) + pow(expected_z - actual_z, 2));
       if (error > 200) {  // 200mm threshold
           std::cerr << "Warning: Large odometry drift detected: " << error << "mm" << std::endl;
       }
   }
   ```

### Performance Optimization

1. **Minimize Proxy Calls**
   ```cpp
   // Bad: Multiple calls per loop
   for (int i = 0; i < 1000; i++) {
       int x, z;
       float alpha;
       omnirobot_proxy->getBasePose(x, z, alpha);  // Slow
       processData(x, z, alpha);
   }
   
   // Good: Cache and reuse when possible
   int x, z;
   float alpha;
   omnirobot_proxy->getBasePose(x, z, alpha);
   for (int i = 0; i < 1000; i++) {
       processData(x, z, alpha);
   }
   ```

2. **Async Calls for Concurrency**
   ```cpp
   // Start async operations
   auto lidar_future = lidar3d_proxy->getLidarDataWithThreshold2dAsync("pearl", 10000, 1);
   auto pose_future = omnirobot_proxy->getBasePoseAsync();
   
   // Do other work
   doSomeProcessing();
   
   // Get results
   auto lidar_data = lidar_future.get();
   auto pose_result = pose_future.get();
   ```

3. **Use Appropriate Data Structures**
   ```cpp
   // For frequent spatial queries, use spatial indexing
   // For example, octree or KD-tree for 3D points
   ```

### Safety Considerations

1. **Timeout Handling**
   ```cpp
   // Set reasonable timeouts for Ice calls
   Ice::Context ctx;
   ctx["timeout"] = "5000";  // 5 second timeout
   omnirobot_proxy->setSpeedBase(300, 0, 0, ctx);
   ```

2. **Sensor Validation**
   ```cpp
   // Always validate sensor data
   auto data = lidar3d_proxy->getLidarDataWithThreshold2d("pearl", 10000, 1);
   if (data.points.empty()) {
       std::cerr << "Error: No LiDAR data received" << std::endl;
       omnirobot_proxy->stopBase();
       return;
   }
   ```

3. **Motion Monitoring**
   ```cpp
   // Verify robot responds to commands
   RoboCompGenericBase::TBaseState state_before, state_after;
   omnirobot_proxy->getBaseState(state_before);
   omnirobot_proxy->setSpeedBase(300, 0, 0);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));
   omnirobot_proxy->getBaseState(state_after);
   
   if (!state_after.isMoving) {
       std::cerr << "Warning: Robot not responding to motion commands" << std::endl;
   }
   ```

---

## Coordinate System Conventions

### LiDAR Coordinate System
- **Origin**: Sensor location (typically robot center)
- **X-axis**: Forward direction
- **Y-axis**: Upward (vertical)
- **Z-axis**: Lateral (right-hand rule)
- **phi**: Azimuth angle, 0 = forward (+X), π/2 = left (+Z)
- **theta**: Elevation angle, 0 = horizontal, π/2 = upward (+Y)

### Robot Base Coordinate System
- **Origin**: Robot geometric center or wheel center
- **X-axis**: Forward direction
- **Z-axis**: Lateral direction (left positive)
- **Y-axis**: Vertical (upward)
- **alpha**: Heading angle, 0 = facing +X axis in global frame

### Units Summary
| Measurement | Unit | Example |
|-------------|------|---------|
| Position (x, z) | millimeters (mm) | 1000 = 1 meter |
| Distance | millimeters (mm) | 500 = 50 cm |
| Velocity (linear) | mm/s | 300 = 0.3 m/s |
| Angle | radians | π/2 = 90 degrees |
| Angular velocity | rad/s | 0.5 = ~28.6 deg/s |
| Timestamp | milliseconds (Unix time) | 1700000000000 |

---

## Troubleshooting

### Common Issues

1. **No LiDAR Data Received**
   - Check sensor name matches configuration
   - Verify Ice proxy connection
   - Ensure LiDAR component is running

2. **Robot Not Moving**
   - Check velocity limits in configuration
   - Verify OmniRobot component is active
   - Check for emergency stop conditions
   - Examine logs for hardware errors

3. **Odometry Drift**
   - Implement periodic corrections using visual landmarks
   - Use sensor fusion (IMU, visual odometry)
   - Reset odometry at known locations

4. **High CPU Usage**
   - Increase decimation factor
   - Reduce point cloud processing frequency
   - Use distance thresholds to limit data

5. **Jerky Motion**
   - Implement velocity smoothing
   - Reduce control loop frequency
   - Check for network latency issues

---

## Additional Resources

- **RoboComp Documentation**: https://robocomp.readthedocs.io/
- **Ice Middleware**: https://zeroc.com/products/ice
- **Point Cloud Processing**: PCL (Point Cloud Library)
- **Robotics Best Practices**: "Probabilistic Robotics" by Thrun, Burgard, Fox

---

**Document Version**: 1.0  
**Last Updated**: November 21, 2025  
**Generated from**: RoboComp Ice Interface Definitions
