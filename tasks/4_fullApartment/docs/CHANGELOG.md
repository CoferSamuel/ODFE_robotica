# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased]

## Commit 31-12-2025 13:00

### Added
- **Door Decision Logic (Ultra-Localiser + Graph Integration)**:
    A robust, memory-based navigation system that combines geometric localization with topological learning:
    1.  **Localization**: Upon entering a room, the **Ultra-Localiser** (Grid Search) estimates the robot's pose to handle arbitrary entry positions.
    2.  **Entry Identification (Learning)**:
        -   **First Visit**: Uses the robot's pose to find the closest door ("Proximity") and registers it as the Entry Door. Crucially, it **learns** a bidirectional connection in the Graph between this door and the previous room's exit door.
        -   **Re-entry**: Uses the **Learned Connection** from the Graph to identify the entry door instantly. This bypasses proximity checks, making the system immune to localization symmetry errors (e.g., if the robot localizes upside-down).
    3.  **Target Selection Strategy**:
        -   **Room 0**: Implements **True Random Selection** (choosing from ALL doors, including the entry door). This prevents deterministic loops and allows the robot to start/stop or change direction randomly.
        -   **Room 1**: Implements **Traversal Preference** (prioritizing non-entry doors) to ensure the robot crosses the room ("Lateral Preference").
        -   **Exploration**: The Graph now correctly tracks "Explored" status (doors with >= 2 connections) to guide this selection.

    #### Topological Graph Map (Deep Dive)
    
    The robot now builds a **semantic graph** of the environment in real-time. This replaces transient decision-making with a persistent memory structure.
    
    **Graph Structure**
    
    The graph consists of two types of nodes:
    - **ROOM Nodes**: Represent physical rooms (e.g., Room 0, Room 1). Store metadata like dimensions (`5500x4000`), entry heading, and the ID of the door used to enter.
    - **DOOR Nodes**: Represent physical connections between rooms. Store geometric endpoints (`p1`, `p2`) and their center position.
    
    **How It Works**
    
    1.  **Dynamic Construction**:
        -   When the robot enters a new room, it calls `add_room()`.
        -   Detected doors are added via `add_door()` and immediately connected to the current Room node.
        
    2.  **Topological Linking (The "Smarts")**:
        -   The system handles the fact that the same physical door is seen as two different "Door Nodes" from different rooms (different coordinates in local frames).
        -   When moving from Room A to Room B, the graph **links** the exit door of Room A to the entry door of Room B.
        -   **Result**: The topology is `Room_A <-> Door_Node_A <-> Door_Node_B <-> Room_B`.
        
    3.  **Exploration State**:
        -   A door is considered **Explored** if it has connections to 2 different neighbors (Parent Room + Linked Door).
        -   This allows the robot to query `get_unexplored_doors()` to intelligently select new targets, prioritizing unknown areas over backtracking.

    **Data Structure**
    
    ```cpp
    struct GraphNode {
        int id;
        NodeType type;       // ROOM or DOOR
        Eigen::Vector2f pos; // Center coordinates
        // Room specifics
        int entry_door_id;   // How we got here
        float reference_heading;
        // Door specifics
        Eigen::Vector2f p1, p2;
    };
    ```
    
    **Visualization (Log Example)**
    
    The system generates a human-readable trace in `logs/GRAPH.log`:
    ```
    [12:51:38] Added ROOM node: id=0 name="Room_0"
    [12:51:38] Added DOOR node: id=1 name="Door_R0_0"
    [12:51:38] Connected: Room_0 (id=0) <-> Door_R0_0 (id=1)
    ...
    [12:51:58] Connected: Door_R0_1 (id=2) <-> Door_R1_0 (id=4) <-- Topology Loop Closed!
    ```

### Changed
- **Targeting & Motion Control Improvements**:
    Refined the `robot_controller` for faster and smoother target approach:
    -   **Adaptive Deceleration**: Added distance-based speed scaling (`dist_factor`) to slow down smoothly when within 500mm of the target.
    -   **Aggressive Rotation**: Increased rotational gain (`theta_dot_e`) to align with targets more quickly.
    -   **Velocity Profile**: Widened the Gaussian curve (`sigma`) to maintain higher linear speeds even during minor orientation corrections, reducing "stop-and-turn" behavior.

### Fixed
- **Door Projection Artifacts**: Fixed a visual bug where doors appeared distorted or on adjacent walls. The system now projects both door endpoints to the **single wall closest to the door's center**, ensuring consistent geometry.
- **Compilation**: Fixed `is_node_in_graph` vs `has_node` API mismatch.

## Commit 29-12-2025 18:10

### Added
- **Ultra-Localiser: Position-Independent Localization System**:
    
    The Ultra-Localiser is a robust localization algorithm that enables the robot to determine its position within a room **from any starting point**, without requiring the robot to begin at the center or rely on external visual markers (like the badge).
    
    #### The Problem
    
    Previously, the localizer only worked when the robot started at the **center of the room** (position 0, 0). This was because:
    
    1. The `robot_pose` was initialized to identity (0, 0, 0°) at startup.
    2. The Hungarian corner-matching algorithm compared **detected corners** (from LIDAR) against **nominal room corners** transformed by the current `robot_pose`.
    3. When the robot was not at the center, the pose assumption was wrong, causing match errors to exceed the threshold (3500mm), and localization would fail.
    
    This meant the robot could only navigate correctly if manually placed at the room center before starting — an impractical constraint for real-world deployment.
    
    #### The Solution: Multi-Hypothesis Grid Search
    
    The Ultra-Localiser implements a **bootstrap phase** that runs once at startup:
    
    1. **Grid Search**: The algorithm tests **200 candidate poses** across the room:
       - **5 × 5 spatial grid** covering 80% of the room dimensions
       - **8 angular orientations** (every 45°)
       - Total: 25 positions × 8 angles = 200 hypotheses
    
    2. **Hypothesis Evaluation**: For each candidate pose:
       - Transform nominal room corners to the robot frame using the candidate pose
       - Match detected LIDAR corners against transformed nominal corners (Hungarian algorithm)
       - Compute the maximum matching error
    
    3. **Best Pose Selection**: The pose with the **lowest maximum matching error** is selected as the initial robot pose.
    
    4. **Continuous Tracking**: After bootstrap, the standard incremental pose correction (`solve_pose()`) takes over, updating the robot's position continuously as it moves.
    
    #### Key Design Decisions
    
    - **Badge Independence**: The localization decision and pose updates are now **completely independent** of badge detection. The system relies purely on geometric corner matching.
    - **Error Threshold**: Localization is considered successful when:
      - Match error < 3500mm (configurable via `LOCALISATION_MATCH_ERROR_THRESHOLD`)
      - At least 3 corners are matched
    - **One-Time Bootstrap**: The grid search runs only once (`initial_localisation_done` flag) to avoid computational overhead during normal operation.
    
    #### How Corner Matching Works (Deep Dive)
    
    **The Core Question**: Given detected LIDAR corners, where is the robot?
    
    **Step 1: We know the room's shape (Nominal Corners in World Frame)**
    ```
                  WORLD FRAME (Room coordinates)
                  
        Corner A                    Corner B
        (-2750, 2000) ●━━━━━━━━━━━━━● (2750, 2000)
                      ┃            ┃
                      ┃   ROOM     ┃
                      ┃            ┃
        Corner D      ┃            ┃      Corner C
        (-2750, -2000)●━━━━━━━━━━━━● (2750, -2000)
    ```
    
    **Step 2: LIDAR detects corners in the Robot Frame**
    ```
                  ROBOT FRAME (What LIDAR sees)
                  
                  ● (-500, 600)     ← Detected Corner 1
                 ╱
                ╱
        🤖═════════►  (Robot at origin, facing forward)
                ╲
                 ╲
                  ● (-500, -2400)   ← Detected Corner 2
    ```
    
    **Step 3: For each hypothesis, transform nominal corners to robot frame**
    
    ```
    Transform formula: P_robot = R(-θ) × (P_world - Robot_position)
    
    Example for Hypothesis (-2200, 1600, 180°):
    
    Corner A at (-2750, 2000) in world frame:
      Step 1: Translate → (-2750, 2000) - (-2200, 1600) = (-550, 400)
      Step 2: Rotate by -180° → (550, -400)
      
    Result: Corner A SHOULD appear at (550, -400) in robot frame
    ```
    
    **Step 4: Compare expected vs. detected corners (Hungarian Algorithm)**
    
    ```
    Expected (from hypothesis)     Detected (from LIDAR)      Distance
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    A_expected: (550, -400)    ↔   Detected_1: (540, -380)   = 22mm ✓
    D_expected: (550, 3600)    ↔   Detected_2: (560, 3590)   = 14mm ✓
    B_expected: (-4950, -400)  ↔   Detected_3: (-4940, -410) = 14mm ✓
    C_expected: (-4950, 3600)  ↔   Detected_4: (-4960, 3595) = 11mm ✓
                                                            ──────────
                                                 Max error = 22mm ✓✓✓
    ```
    
    The hypothesis with the **smallest max error** wins!
    
    #### Two-Phase Localization: Coarse + Fine
    
    **Important**: The grid search finds the **closest grid point**, not the exact robot position!
    
    ```
    Grid points (discrete):
      (-2200, 1600), (-1100, 1600), (0, 1600), (1100, 1600), (2200, 1600)
      (-2200, 800),  (-1100, 800),  (0, 800),  ...
    
    If robot is at (-2050, 1450) ← NOT on grid!
      → Nearest grid point (-2200, 1600) wins with ~300mm error
    ```
    
    | Phase | What It Does | Accuracy |
    |-------|--------------|----------|
    | **1. Grid Search** | Finds approximate position | Within ~300-500mm |
    | **2. `solve_pose()`** | Refines pose each cycle | Within ~20mm after a few cycles |
    
    **Example Convergence**:
    ```
    t=0: Grid search → robot_pose = (-2200, 1600, 180°)   [~300mm off]
    t=1: solve_pose() → robot_pose = (-2120, 1520, 179°)  [~150mm off]
    t=2: solve_pose() → robot_pose = (-2060, 1460, 178°)  [~50mm off]
    t=3: solve_pose() → robot_pose = (-2050, 1450, 178°)  [~10mm off] ✓
    ```
    
    **Why This Works**: With a 5×5 grid, the maximum distance from any point to the nearest grid point is:
    ```
    Room: 5500mm × 4000mm
    Grid spacing: ~1100mm × 800mm
    Max distance to nearest grid: √(550² + 400²) ≈ 680mm (well below 3500mm threshold)
    ```
    
    #### Algorithm Pseudocode
    
    ```
    function execute_localiser():
        if NOT initial_localisation_done AND corners detected:
            robot_pose = find_best_initial_pose(detected_corners)
            initial_localisation_done = true
        
        // Standard matching pipeline
        robot_corners = transform_nominal_to_robot_frame(robot_pose.inverse())
        matched = hungarian_match(detected_corners, robot_corners)
        max_error = max(matched.errors)
        
        // Continuous pose update (badge-independent)
        if matched.count >= 3 AND max_error < THRESHOLD:
            localised = true
            pose_correction = solve_pose(detected_corners, matched)
            robot_pose = robot_pose + pose_correction
        
        update_robot_graphic(robot_pose)
    ```
    
    #### Algorithm Flowchart
    
    ```mermaid
    flowchart TD
        A[execute_localiser] --> B{initial_localisation_done?}
        B -->|No| C[Grid search 200 poses]
        C --> D[Select best match]
        D --> E[Set robot_pose]
        E --> F[Continue]
        B -->|Yes| F
        F --> G{match_error < 3500mm<br/>and matches ≥ 3?}
        G -->|Yes| H[Update pose with solve_pose]
        G -->|No| I[Keep current pose]
        H --> J[Update robot graphic]
        I --> J
    ```
    
    #### Files Modified
    
    | File | Change |
    |------|--------|
    | `specificworker.h` | Added `initial_localisation_done` flag, `find_best_initial_pose()` declaration |
    | `specificworker.cpp` | Added grid search function, bootstrap logic in `execute_localiser()`, removed `badge_found` dependency |
    
    #### Usage Example
    
    **Scenario**: Robot starts in the top-left corner of Room 0 at position (-2200, 1600) facing down (180° rotation).
    
    ```
    ╔══════════════════════════════════════════════════════════════════════════════╗
    ║                              ROOM 0 LAYOUT                                    ║
    ║                           (5500mm × 4000mm)                                   ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║                                                                              ║
    ║    Corner A                                              Corner B            ║
    ║    (-2750, 2000) ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━● (2750, 2000)      ║
    ║                  ┃                                      ┃                    ║
    ║                  ┃     🤖 ← Robot starts here           ┃                    ║
    ║                  ┃        (-2200, 1600)                 ┃                    ║
    ║                  ┃        facing ↓ (180°)               ┃                    ║
    ║                  ┃                                      ┃                    ║
    ║                  ┃              ⊕ ← Room Center         ┃                    ║
    ║                  ┃                (0, 0)                ┃                    ║
    ║                  ┃                                      ┃                    ║
    ║                  ┃                                      ┃                    ║
    ║    Corner D      ┃                                      ┃      Corner C      ║
    ║    (-2750,-2000) ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━● (2750, -2000)     ║
    ║                                                                              ║
    ╚══════════════════════════════════════════════════════════════════════════════╝
    ```
    
    ---
    
    **BEFORE Ultra-Localiser** (Old Behavior):
    
    ```
    ╔══════════════════════════════════════════════════════════════════════════════╗
    ║  PROBLEM: Robot pose initialized at (0, 0, 0°) — WRONG!                       ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║                                                                              ║
    ║    Actual Robot Position          vs.       Assumed Robot Position           ║
    ║    ─────────────────────                    ────────────────────────         ║
    ║                                                                              ║
    ║    ●━━━━━━━━━━━━━━━━━━━━●                   ●━━━━━━━━━━━━━━━━━━━━●           ║
    ║    ┃                    ┃                   ┃                    ┃           ║
    ║    ┃  🤖 ← HERE         ┃                   ┃         🤖         ┃           ║
    ║    ┃  (-2200, 1600)     ┃                   ┃       (0, 0) ← WRONG!          ║
    ║    ┃                    ┃                   ┃                    ┃           ║
    ║    ●━━━━━━━━━━━━━━━━━━━━●                   ●━━━━━━━━━━━━━━━━━━━━●           ║
    ║                                                                              ║
    ║  Result: Match error = 4500mm (> 3500mm threshold)                           ║
    ║          Localised = FALSE                                                   ║
    ║          Robot graphic stuck at center, doesn't track real position!         ║
    ╚══════════════════════════════════════════════════════════════════════════════╝
    ```
    
    **Console Output (Before)**:
    ```
    [IDLE] Localised=false max_error=inf matches=0
    [GOTO_ROOM_CENTER] Match error=4500mm (above threshold)
    localiser: Updated robot graphic at (0, 0) angle=0 deg   ← WRONG POSITION!
    ```
    
    ---
    
    **AFTER Ultra-Localiser** (New Behavior):
    
    ```
    ╔══════════════════════════════════════════════════════════════════════════════╗
    ║  STEP 1: Grid Search Bootstrap (runs once at startup)                         ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║                                                                              ║
    ║    Testing 200 candidate poses (5×5 grid × 8 angles):                        ║
    ║                                                                              ║
    ║    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●                        ║
    ║    ┃  ○     ○     ○     ○     ○  ← Test positions   ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ┃  ○     ○     ○     ○     ○                     ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ┃  ○     ○     ⊕     ○     ○  ← Center (0,0)     ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ┃  ○     ○     ○     ○     ○                     ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ┃  ○     ○     ○     ○     ○                     ┃                        ║
    ║    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●                        ║
    ║                                                                              ║
    ║    For each ○: test 8 angles (0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°)   ║
    ║    Score each by corner matching error                                        ║
    ╚══════════════════════════════════════════════════════════════════════════════╝
    
    ╔══════════════════════════════════════════════════════════════════════════════╗
    ║  STEP 2: Best Pose Found!                                                     ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║                                                                              ║
    ║    Winner: (-2200, 1600, 180°) with match error = 306mm ✓                    ║
    ║                                                                              ║
    ║    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●                        ║
    ║    ┃  ★ ← BEST MATCH                                ┃                        ║
    ║    ┃     Error: 306mm (well below 3500mm threshold) ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ┃                    ⊕                           ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●                        ║
    ║                                                                              ║
    ║    robot_pose = (-2200, 1600, 180°)                                          ║
    ║    initial_localisation_done = true                                          ║
    ╚══════════════════════════════════════════════════════════════════════════════╝
    
    ╔══════════════════════════════════════════════════════════════════════════════╗
    ║  STEP 3: Continuous Tracking (every cycle)                                    ║
    ╠══════════════════════════════════════════════════════════════════════════════╣
    ║                                                                              ║
    ║    As robot moves toward center, pose is refined:                            ║
    ║                                                                              ║
    ║    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●                        ║
    ║    ┃  🤖 → → → → → → → → → ⊕                        ┃                        ║
    ║    ┃  Start              Target                     ┃                        ║
    ║    ┃  (-2200, 1600)      (0, 0)                     ┃                        ║
    ║    ┃                                                ┃                        ║
    ║    ┃  Pose updated each cycle:                      ┃                        ║
    ║    ┃  t=0: (-2200, 1600) → t=1: (-2150, 1580) → ... ┃                        ║
    ║    ●━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━●                        ║
    ║                                                                              ║
    ║    Robot graphic in viewer_room correctly follows actual position! ✓          ║
    ╚══════════════════════════════════════════════════════════════════════════════╝
    ```
    
    **Console Output (After)**:
    ```
    [IDLE] Grid search best error: 306mm at x=-2200 y=1600
    [IDLE] localiser: Initial pose found via grid search at x=-2200 y=1600
    [IDLE] localiser: Localised=true max_error=306mm matches=4
    [GOTO_ROOM_CENTER] Target: -2164, 1587 Dist: 2684mm
    [GOTO_ROOM_CENTER] localiser: Pose updated: x=-2195 y=1592 theta=-179.8°
    localiser: Updated robot graphic at (-2195, 1592) angle=-179.8 deg   ← CORRECT!
    ```
    
    ---
    
    **Debug Output Interpretation**:
    
    | Log Message | Meaning |
    |-------------|---------|
    | `Grid search best error: 306mm` | Algorithm tested 200 poses, found one with 306mm corner match error |
    | `Initial pose found via grid search` | Bootstrap phase complete, pose locked |
    | `Localised=true` | Match error below threshold, continuous tracking active |
    | `Pose updated: x=... y=... theta=...` | Incremental refinement happening each cycle |
    | `matches=4` | All 4 room corners successfully matched |
    
    #### Performance Characteristics
    
    - **Bootstrap Time**: ~1-2ms for 200 hypotheses (negligible, runs once)
    - **Accuracy**: Initial pose typically within ±200mm of true position
    - **Robustness**: Works with partial corner visibility (requires ≥3 corners)


## Commit 29-12-2025 17:30

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


