# Activity 1: Autonomous Sweeping Robot (Chocachoca)

[![RoboComp](https://img.shields.io/badge/Framework-RoboComp-blue)](https://github.com/robocomp/robocomp)
[![Language](https://img.shields.io/badge/Language-C%2B%2B23-green)](https://isocpp.org/)

This component implements a reactive control system for the **OmniRobot** mobile platform in **Webots**. Developed as part of the *Robótica* course at the **Universidad de Extremadura** (taught by Prof. Pablo Bustos García), the component aims to maximize room floor coverage within a 3-minute evaluation trial using 2D/3D LiDAR sensor feedback while avoiding collisions.

---

## 🎯 Objectives & Performance Evaluation

- **Primary Goal:** Cover the maximum possible floor surface in a enclosed rectangular environment within 180 seconds.
- **Evaluation Tool:** Managed and evaluated by the `Aspirator` GUI component, which tracks swept area and execution metrics over time.
- **Platform:** OmniRobot omnidirectional mobile platform equipped with a 360° LiDAR (`Lidar3D`).

---

## 🔍 LiDAR Perception & Preprocessing Pipeline

Raw LiDAR scans undergo a two-stage filtering process before feeding into the control logic:

1. **Angle-Based Grouping:** Groups raw points by azimuth angle and retains only the closest point per angle direction (up to a 12-meter maximum range).
2. **Isolation Noise Filtering:** Discards sparse noise points that lack at least one neighboring detection within a 200 mm radius.

The filtered point cloud is rendered in real-time on a custom Qt GUI alongside the robot's current odometric pose $(x, z, \alpha)$.

---

## ⚙️ Operational Algorithms & Behavioral State Machine (FSM)

The control system is driven by a Finite State Machine (FSM) comprising four operational states:

```text
       +------------------------------------+
       |               SPIRAL               |
       +-----------------+------------------+
                         | (Obstacle < D_T)
                         v
       +-----------------+------------------+
       |                TURN                |
       +--------+------------------+--------+
                |                  |
   (Clear Path  |                  | (Clear Path
   70% prob.)   v                  v 30% prob.)
+---------------+----+       +----+---------------+
|      FORWARD       |       |    FOLLOW_WALL     |
+--------------------+       +--------------------+
```

### 1. `SPIRAL`
Executes an Archimedean spiral trajectory to maximize coverage in open spaces.
- **Control Law:**
  - Initial forward velocity $v_s = 500.0\text{ mm/s}$, increasing by $+2.5\text{ mm/s}$ per cycle up to $V_{s,\max} = 1000.0\text{ mm/s}$.
  - Initial rotational velocity $\omega_s = 3.0\text{ rad/s}$, decreasing by $-0.01\text{ rad/s}$ per cycle down to $\omega_{\min} = 0.0\text{ rad/s}$.
- **Transition:** If any frontal obstacle enters $D_T = 600.0\text{ mm}$, abort spiral and transition to `TURN`.

### 2. `TURN`
Executes in-place rotation to clear frontal obstacles blocking the robot's path.
- Filters points within the $\pm 45^\circ$ frontal cone.
- **Stuck Recovery:** Increments a counter $c$. If $c \ge 100$ iterations, forces positive rotation to escape corners.
- **Transition:** Once the nearest frontal obstacle exceeds $700.0\text{ mm}$, transitions with 70% probability to `FORWARD` and 30% probability to `FOLLOW_WALL`.

### 3. `FORWARD`
Drives the robot straight forward at $V_{\max} = 800.0\text{ mm/s}$ with proportional braking upon obstacle approach.
- **Linear Speedup:** Increases $v$ smoothly as a function of distance:
  $$v = (d_{\min} - T_{\min}) \cdot \frac{V_{\max}}{D_T - T_{\min}}$$
- **Proportional Braking Law:** As distance $d_{\min} < D_T$, computes a linear braking term $v_b$:
  $$v_b = (D_T - d_{\min}) \cdot \frac{B_{\max}}{D_T - T_{\min}}$$
  Angular turning speed is assigned as $\omega = v_b / 2$.
- **Transition:** If $d_{\min} > D_S = 2000.0\text{ mm}$, transitions back to `SPIRAL`.

### 4. `FOLLOW_WALL`
Implements wall-following using proportional control to maintain a constant distance corridor around $D_T = 600.0\text{ mm}$.
- **Ideal Corridor ($D_T \le r < D_T + 150\text{ mm}$):** Drives straight at $v = 150\text{ mm/s}, \omega = 0$.
- **Too Far ($r > D_T + 150\text{ mm}$):** Rotates slightly towards the wall ($\omega = \mp 0.2\text{ rad/s}$).
- **Too Close ($r < D_T - 100\text{ mm}$):** Rotates away from the wall ($\omega = \pm 0.2\text{ rad/s}$).

---

## 📊 Experimental Results

Tested across three Webots benchmark environments (3-minute trials):

| Scenario | Description | Max Coverage Achieved |
| :--- | :--- | :---: |
| **Test 1: SimpleWorld** | Rectangular room without obstacles | **70%** |
| **Test 2: 1 Box World** | Room with 1 central rectangular obstacle | **67%** |
| **Test 3: 2 Box World** | Room with 2 obstacles | **59%** |

---

## 💻 Compilation and Execution

```bash
cd tasks/1_chocachoca

# 1. Build component
cmake -B build -DCMAKE_BUILD_TYPE=Release
make -C build -j$(nproc)

# 2. Run component
./bin/chocachoca etc/config
```