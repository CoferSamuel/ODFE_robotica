# Activity 3: Visually-Guided Navigation (Door Finding)

## Objective
Extend navigation capabilities to autonomously detect and traverse doors, enabling transition between rooms. The system uses a combination of LiDAR (for geometry) and visual cues (patches) to identify exits.

## Implementation Details

### Finite State Machine (FSM)
The task is orchestrated by a robust FSM:
1. **GOTO_ROOM_CENTER**: Navigates to the geometric center of the room to have a clear vantage point.
2. **TURN**: Rotates the robot to scan for visual markers (colored patches) indicating an exit.
3. **GO_TO_DOOR**: Navigates towards the detected door patch until a standoff distance is reached (1 meter).
4. **ORIENT_TO_DOOR**: Aligns the robot with the door normal vector. Uses a "sticky" target angle to prevent jitter and ensures the robot faces *out* of the room using vector analysis.
5. **CROSS_DOOR**: Traverses the doorway into the next room.

### Features
- **Door Detection**: Uses a `door_detector` module to find openings in the LiDAR scan.
- **Orientation Correction**: Solves the $180^\circ$ ambiguity problem by ensuring the target vector points away from the room center.
- **Sticky Targets**: Prevents oscillation by locking onto a valid target orientation once computed.

## Dependencies
- **RoboComp**
- **Webots**
- **OpenCV**: required for detecting color patches in the camera stream (Red/Green badges).

## Execution
To compile and run the component:

```bash
cd tasks/3_multiRoom
cmake -B build && make -C build -j$(nproc)
bin/multiroom etc/config
```
