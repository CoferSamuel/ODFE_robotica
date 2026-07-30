# Activity 4: Multi-Room Autonomous Navigation & AI Semantic Mapping

[![RoboComp](https://img.shields.io/badge/Framework-RoboComp-blue)](https://github.com/robocomp/robocomp)
[![Language](https://img.shields.io/badge/Language-C%2B%2B23%2FPython3-green)](https://isocpp.org/)
[![PyTorch](https://img.shields.io/badge/AI-PyTorch-orange)](https://pytorch.org/)
[![OpenCV](https://img.shields.io/badge/Vision-OpenCV4-red)](https://opencv.org/)

Activity 4 represents the culmination of the project, integrating global localization (**Ultra-Localiser**), persistent spatial memory (**Topological Graph Mapping**), intelligent door target selection, and deep learning-based semantic room identification (**MNIST PyTorch Detector**) for the **OmniRobot** mobile platform in **Webots**.

Developed as part of the *Robótica Avanzada* course at the **Universidad de Extremadura** (taught by Prof. Pablo Bustos García).

📹 **Video Demonstration:** [Watch Multi-Room Navigation & AI Execution (Google Drive)](https://drive.google.com/file/d/1vdQ3AvWZ5x7y0ox2yeZf0NKtGY7jAHUi/view?usp=sharing)

---

## 🌟 Key Subsystems & Features

```text
+-----------------------------------------------------------------------+
|                    ULTRA-LOCALISER (BOOTSTRAP)                       |
|   200-Hypothesis Grid Search  --->  SVD Continuous Fine Solver        |
+-----------------------------------+-----------------------------------+
                                    |
                                    v
+-----------------------------------+-----------------------------------+
|                  TOPOLOGICAL GRAPH EXPLORATION                        |
|   ROOM Nodes <--> DOOR Nodes (Learned Links)  | Priority Target Selector|
+-----------------------------------+-----------------------------------+
                                    |
                                    v
+-----------------------------------+-----------------------------------+
|               AI SEMANTIC PERCEPTION (MNIST DETECTOR)                 |
|   HSV Red Mask -> Bitwise Invert -> PyTorch CNN -> Digit (Room ID)    |
+-----------------------------------------------------------------------+
```

---

## 🌐 1. Ultra-Localiser (Global Grid Search Bootstrap)

Solves the **kidnapped robot problem** when starting from arbitrary unknown poses without prior position information:

### Two-Phase Convergence Algorithm
1. **Coarse Grid Search (Bootstrap):** Evaluates $25 \times 8 = 200$ candidate pose hypotheses:
   - $5 \times 5$ spatial grid covering 80% of room dimensions.
   - 8 angular orientations ($45^\circ$ step increments).
   - Transforms room corners for each candidate pose, performs Hungarian matching against LiDAR detections, and selects the candidate pose minimizing maximum match error ($E_k < 3500\text{ mm}$, $\ge 3$ matched corners).
2. **Fine Incremental Solver:** SVD linearized least-squares solver takes over, refining position accuracy from $\sim 300\text{--}500\text{ mm}$ down to $\sim 20\text{ mm}$.

| Phase | Function | Accuracy Range |
| :--- | :--- | :---: |
| **Grid Search Bootstrap** | Global initial pose estimate | $\sim 300\text{--}500\text{ mm}$ |
| **Incremental SVD** | Real-time continuous refinement | $\sim 20\text{ mm}$ |

---

## 🗺️ 2. Topological Graph Mapping & Exploration

Replaces purely reactive behaviors with a persistent semantic graph representation of the environment.

### Graph Data Structure
- **`ROOM` Nodes:** Represent physical rooms. Store dimensions, reference heading, entry door ID, and room index.
- **`DOOR` Nodes:** Represent physical connection thresholds. Store geometric endpoints $(p_1, p_2)$ and center coordinates.

### Learned Bidirectional Links
When moving from Room A to Room B, the graph automatically registers a persistent link connecting the exit door node of Room A to the entry door node of Room B:

$$\text{Room A} \longleftrightarrow \text{Door Node A} \longleftrightarrow \text{Door Node B} \longleftrightarrow \text{Room B}$$

### Priority Door Selection Algorithm
The `select_door_from_graph()` function selects next navigation targets based on a 4-tier priority system:
1. **Priority 1:** Unexplored, Non-Entry Doors (maximizes discovery of new rooms).
2. **Priority 2:** Any Unexplored Door.
3. **Priority 3 (Traversal Logic):** If all doors are explored:
   - **Room 0:** Random selection among all doors to prevent deterministic deadlocks.
   - **Room 1+:** Non-entry doors only (forces crossing the room).
4. **Priority 4:** Backtrack through entry door as last resort.

---

## 🧠 3. AI Room Digit Recognition (`mnist_detector`)

A dedicated Python PyTorch component (`mnist_detector`) provides semantic room identification by reading handwritten digits (0–9) posted on room walls.

### Component Architecture & Ice Remote Interface
- **Asynchronous Execution:** Runs in a separate Python process communicating via Ice interface `getNumber()`, preventing deep learning inference from blocking real-time control loops.
- **Dynamic Period:** Runs at 5 Hz when actively queried by the controller, dropping to 0.5 Hz when idle.

### Computer Vision & Neural Network Pipeline
1. **Red Badge Masking:** Converts camera stream to HSV space and applies a red color mask to locate candidate number badges.
2. **Preprocessing:** Crops ROI, converts to grayscale, resizes to $28 \times 28$ pixels, and applies **bitwise inversion** (matching white digits on black background expected by MNIST models).
3. **PyTorch CNN Architecture:**
   - Input: $28 \times 28 \times 1$ tensor.
   - Conv2D Layer 1 ($32\text{ filters}, 3\times3$) + ReLU.
   - Conv2D Layer 2 ($64\text{ filters}, 3\times3$) + ReLU + MaxPool2D.
   - Fully Connected Layer ($128\text{ units}$) + Softmax Output ($10\text{ digit classes}$).

---

## 📐 4. Wall Projection Door Visualization

Door endpoints $(p_1, p_2)$ detected by LiDAR are orthogonally projected onto the closest nominal wall line segment to prevent rendering artifacts caused by sensor noise:

$$\text{proj}(\mathbf{p}) = \mathbf{w}_1 + \frac{(\mathbf{p} - \mathbf{w}_1) \cdot (\mathbf{w}_2 - \mathbf{w}_1)}{\|\mathbf{w}_2 - \mathbf{w}_1\|^2} (\mathbf{w}_2 - \mathbf{w}_1)$$

Rendered in real-time in the Qt GUI as cyan lines with dark blue endpoint markers.

---

## 💻 Compilation and Execution

### Build C++ Navigation Controller
```bash
cd tasks/4_multiroom_advanced

# 1. Build C++ multiroom component
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(nproc)

# 2. Run main navigation component
./bin/multiroom etc/config
```

### Launch Python Deep Learning Detector
In a separate terminal:
```bash
cd tasks/4_multiroom_advanced
python3 src/mnist_detector/src/specificworker.py src/mnist_detector/etc/config
```