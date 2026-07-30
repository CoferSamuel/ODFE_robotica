# Activity 3: Visually-Guided Autonomous Door Traversal

[![RoboComp](https://img.shields.io/badge/Framework-RoboComp-blue)](https://github.com/robocomp/robocomp)
[![Language](https://img.shields.io/badge/Language-C%2B%2B23-green)](https://isocpp.org/)
[![OpenCV](https://img.shields.io/badge/Vision-OpenCV4-red)](https://opencv.org/)

This component extends the navigation capabilities of the **OmniRobot** mobile platform in **Webots** to achieve autonomous door detection, visual patch alignment, and inter-room traversal. Developed as part of the *Robótica* course at the **Universidad de Extremadura** (taught by Prof. Pablo Bustos García).

🎥 **Video Demonstration:** [Local MP4 File](../../docs/multimedia/task3/multiroom_simulation.mp4) | [Google Drive Mirror](https://drive.google.com/file/d/10n_1iG7BUhLd8JUsoaLkpJoZF9h5ERZR/view?usp=sharing)

---

## 🎯 Objectives & Multi-State Architecture

The robot transitions from a starting position to room center, locates visual landmarks (green/red fiducial panels), aligns perpendicular to the doorway, and executes a straight open-loop crossing.

```tikz
\begin{tikzpicture}[scale=0.85, transform shape, node distance=2.5cm, auto]
    \node[draw, ellipse, thick, fill=gray!10] (s0) {INITIAL CHECK};
    \node[draw, ellipse, thick, fill=gray!10] (s1) [left=3.5cm of s0] {GOTO ROOM CENTER};
    \node[draw, ellipse, thick, fill=orange!10] (s2) [below=2cm of s0] {TURN};
    \node[draw, ellipse, thick, fill=gray!10] (s3) [below=2cm of s2] {GO TO DOOR};
    \node[draw, ellipse, thick, fill=gray!10] (s4) [below=2cm of s3] {ORIENT TO DOOR};
    \node[draw, ellipse, thick, fill=green!10] (s5) [right=3cm of s4] {CROSS DOOR};

    \path[->, thick, \small]
    (s0) edge node [above] {Not Centered} (s1)
    (s0) edge node [right] {Is Centered} (s2)
    (s1) edge node [below left] {Arrived} (s2)
    (s2) edge node [right] {Patch Found} (s3)
    (s3) edge node [right] {Dist $< 1$m} (s4)
    (s4) edge node [above] {Aligned} (s5)
    (s5) edge [bend right=60] node [left] {Room Changed} (s0);
\end{tikzpicture}
```

---

## ⚙️ Behavioral State Machine Logic

### 1. `GOTO_ROOM_CENTER`
Navigates the robot to the room's geometric centroid to obtain an unobstructed vantage point. Terminates when distance $d < 100\text{ mm}$.

### 2. `TURN` (Active Visual Perception)
Executes an in-place rotation at $\omega = 0.2\text{ rad/s}$ while applying OpenCV color segmentation (HSV mask and blob analysis) on the camera stream to center the target panel (green or red).

### 3. `GO_TO_DOOR` & Geometric Ambiguity Solver
Navigates to a 1.0-meter standoff position in front of the door detected by 2D LiDAR.

#### 180° Directional Ambiguity Resolution
Since door line segments have a $180^\circ$ orientation ambiguity, the system compares the door's normal vector $\mathbf{n}$ with the vector pointing towards the room centroid ($\mathbf{v}_{\text{inward}}$) using the dot product:

$$\mathbf{v}_{\text{target}} = \begin{cases} -\mathbf{v}_{\text{door}} & \text{if } \mathbf{n} \cdot \mathbf{v}_{\text{inward}} > 0 \\ \mathbf{v}_{\text{door}} & \text{otherwise} \end{cases}$$

This guarantees that the target vector points *out* of the room rather than back inside.

### 4. `ORIENT_TO_DOOR` (Bang-Bang Heading Control)
Decouples approach from turning. Rotates the robot in place to align with a locked ("sticky") world target angle using a constant velocity controller:

$$\omega_z = -\text{sgn}(\theta_{\text{err}}) \cdot \omega_{\text{fixed}} \quad (\omega_{\text{fixed}} = 0.2\text{ rad/s})$$

Terminates when absolute angular error $|\theta_{\text{err}}| < 0.2\text{ rad}$ ($\approx 11.5^\circ$).

### 5. `CROSS_DOOR` (Ballistic Traversal)
Executes an open-loop, straight-line trajectory at constant speed $v_x = 400\text{ mm/s}$ for a fixed duration $T_{\text{limit}} = 5.0\text{ s}$. Open-loop control avoids oscillatory sensor feedback caused by narrow door frame reflections.

---

## 🕹 Motion Controller & Velocity Modulation

The closed-loop kinematic controller modulates linear speed $v$ using two scaling factors:

1. **Gaussian Alignment Factor ($\Omega$):** Suppresses linear speed during sharp heading turns:
   $$\Omega = \exp\left( \frac{-\theta_e^2}{\pi / 4} \right)$$
2. **Distance Deceleration Ramp ($D$):** Prevents overshoot when approaching target standoff:
   $$D = \min\left(1.0, \frac{\|\mathbf{t}\|}{500}\right)$$

$$v = V_{\max} \cdot \Omega \cdot D \quad (V_{\max} = 1000\text{ mm/s})$$

---

## 💻 Compilation and Execution

```bash
cd tasks/3_multiRoom

# 1. Build component
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(nproc)

# 2. Execute multiroom component
./bin/multiroom etc/config
```
