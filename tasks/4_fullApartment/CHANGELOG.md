# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

### Fixed
### Fixed
- **Door Selection Logic (Anti-Oscillation Algorithm)**: 
    - **Problem**: Previously, the robot selected doors based on their index in the Lidar vector (e.g., `doors[0]`). As the robot turned, the door order fluctuated, causing the robot to switch targets mid-maneuver ("oscillation").
    - **Solution**: Implemented a **Sticky World-Coordinate** algorithm:
        1.  **Room 0**: Upon first detection (which happens **while the robot is facing the badge**), the robot randomly selects a door, calculates its **World Position** (`P_world = RobotPose * DoorLocal`), and memorizes it.
            - *Side Definition*: It also records whether the door is to the "Left" or "Right" relative to the robot's perspective **at this exact moment** (looking at the badge). This fixes the reference frame for the next room.
            - *Sticky Tracking*: In subsequent frames, it ignores indices and selects the door closest to this stored world coordinate.
        2.  **Room 1 (Lateral Inheritance)**: The robot recalls the choice from Room 0 (e.g., "We picked the Left door").
            - **Logic**: It scans all visible doors in Room 1.
            - **Matching (The "One-Shot" Decision)**: This specific check (Min X / Max X) occurs **only once**, at the precise moment the robot enters the `GOTO_DOOR` state (immediately after the `TURN` maneuver finishes).
                - *Why then?*: The robot has just centered its camera on the visual badge, so its position (localization) is stable. We lock the door target instantly before the robot starts moving towards it.
                - *Subsequent Frames*: Immediately after this first frame, the algorithm switches back to the **Sticky World-Coordinate** method (Section 1) to track that specific door as the robot drives towards it.
            - **Goal**: This ensures the robot maintains a consistent side preference ("Left Hand" or "Right Hand") relative to its own view throughout the traversal.
        
        **Detailed Example Scenario**:
        1.  **Room 0 Start**: The robot turns and faces the Red Badge (Up). It sees **Door A** (Right) and **Door B** (Left).
        2.  **Random Choice**: It randomly selects **Door B** (West Wall).
        3.  **Locking the Reference**:
            - It calculates Door B's coordinates.
            - It records: "I chose the **Left** side".
        4.  **Sticky Navigation**: It drives to Door B.
        5.  **Room 1 Start**: The robot enters Room 1, turns 180 degrees, and faces the Green Badge (Down). It sees **Door C** (Right) and **Door D** (Left).
        6.  **The "One-Shot" Decision**:
            - **Moment**: The *instant* the Turn finishes and `GOTO_DOOR` starts.
            - **Logic**: "My memory says **Left**. I look for the door on *my* Left."
            - **Action**: The robot selects **Door D** (East Wall).
        7.  **Execution**: It locks onto Door D.
    - **Result**: Because the robot turned 180 degrees, "Left" is now the opposite physical wall. The robot performs a **Diagonal (Zig-Zag)** traversal.

### Changed
- **Rotation Speed**: Increased rotational velocity parameters to improve responsiveness:
    - `TURN` state speed increased from 0.4 to **0.8**.
    - `ORIENT_TO_DOOR` state speed increased from 0.2 to **0.5**.
