# Activity 2: Geometry-Based Room Localiser

[![RoboComp](https://img.shields.io/badge/Framework-RoboComp-blue)](https://github.com/robocomp/robocomp)
[![Language](https://img.shields.io/badge/Language-C%2B%2B23-green)](https://isocpp.org/)
[![Eigen](https://img.shields.io/badge/Math-Eigen3-orange)](https://eigen.tuxfamily.org/)

This component implements a geometry-based 2D LiDAR localiser for the **OmniRobot** mobile platform in **Webots**. Developed as part of the *Robótica* course at the **Universidad de Extremadura** (taught by Prof. Pablo Bustos García), the system estimates and incrementally corrects the robot's pose $(x, y, \theta)$ within a known rectangular room, eliminating accumulated odometry drift.

🎥 **Video Demonstration:** [Local MP4 File](../../docs/multimedia/task2/localiser_demo.mp4) | [Google Drive Mirror](https://drive.google.com/file/d/1f_QV6La_op_kgW51XC4HKma_aQV0oxyi/view?usp=sharing)

---

## 📐 Mathematical Foundations & Pipeline

The Localiser extracts geometric features (room corners) from 2D LiDAR range scans and aligns them with a known nominal room model using linearized least-squares optimization.

```text
+-------------------+      +-------------------+      +-------------------+
| 2D LiDAR Scans    | ---> | RANSAC Line       | ---> | Corner Extraction |
| Preprocessing     |      | Fitting (Walls)   |      | & Intersections  |
+-------------------+      +-------------------+      +---------+---------+
                                                                |
                                                                v
+-------------------+      +-------------------+      +---------+---------+
| Pose Update       | <--- | SVD Linear Solver | <--- | Hungarian Data    |
| (x, y, theta)     |      | (Least-Squares)   |      | Association       |
+-------------------+      +-------------------+      +-------------------+
```

### 1. $SE(2)$ Rigid Frame Transformations
To compare nominal map corners $C_{\text{map}}$ with sensor observations in the robot's local coordinate frame, points are transformed using the robot's estimated translation $T = (T_x, T_y)$ and rotation matrix $R(\theta)$:

$$\tilde{C} = R(\theta)^T (C_{\text{map}} - T)$$

### 2. Wall Line Extraction (RANSAC)
The **Random Sample Consensus (RANSAC)** algorithm robustly fits straight lines to 2D LiDAR scans, isolating wall segments from environmental noise:
- Randomly samples 2 points to generate a candidate line model.
- Counts inliers within a distance threshold.
- Iteratively selects the line with maximum consensus.

### 3. Corner Detection
Computes intersection points of perpendicular RANSAC lines. Candidate intersections are validated if:
1. The angle between intersecting line normals is $90^\circ \pm \epsilon$.
2. The intersection point falls within valid wall segment boundaries.
3. Duplicates within a proximity threshold are merged.

### 4. Data Association (Hungarian Algorithm)
Associates detected LiDAR corners with transformed nominal map corners using **Kuhn's Hungarian Assignment Algorithm**, minimizing the total sum of squared Euclidean distances.

### 5. Linearized Least-Squares Pose Solver (SVD)
For small pose errors, using small-angle approximations ($\cos\Delta\theta \approx 1$, $\sin\Delta\theta \approx \Delta\theta$), the non-linear transformation is linearized:

$$\tilde{C}_x \approx T_x + m_x - \Delta\theta \cdot m_y$$

$$\tilde{C}_y \approx T_y + m_y + \Delta\theta \cdot m_x$$

Stacking all matched corner pairs yields an overdetermined linear system:

$$W r = b \quad \text{where } r = [\Delta x, \Delta y, \Delta\theta]^T$$

The optimal pose correction vector $r$ is solved via **Singular Value Decomposition (SVD)** pseudoinverse, guaranteeing numerical stability.

### 6. Outlier Rejection Consensus Filter
A secondary RANSAC-like consensus check filters out erroneous corner pairings before applying the pose update to prevent single bad matches from corrupting the pose.

---

## ⚙️ Operating Modes

1. **Estimation-Only Mode:** Computes real-time localization error metrics with actuators disabled to benchmark solver accuracy and convergence rate.
2. **Closed-Loop Mode:** Feeds corrected pose estimates continuously into the reactive motion controller, compensating for odometry drift during navigation.

---

## 💻 Compilation and Execution

### Requirements
- **RoboComp Framework**
- **Webots Simulator**
- **Eigen3** (`libeigen3-dev`)

### Build & Run
```bash
cd tasks/2_localiser

# 1. Build component
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(nproc)

# 2. Execute localiser
./bin/localiser etc/config
```
