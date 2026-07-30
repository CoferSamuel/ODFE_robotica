# ODFE Robotics – Component-Based Autonomous Mobile Robotics

[![RoboComp](https://img.shields.io/badge/Framework-RoboComp-blue)](https://github.com/robocomp/robocomp)
[![Simulator](https://img.shields.io/badge/Simulator-Webots-red)](https://cyberbotics.com/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B23%2FPython3-green)](https://isocpp.org/)

This repository contains the software codebase, control algorithms, and academic documentation for the practical projects developed in the **Robotics** and **Advanced Robotics** courses at the **University of Extremadura** (Escuela Politécnica de Cáceres), taught by **Prof. Pablo Bustos García**.

The codebase implements a complete reactive and deliberative navigation pipeline for the **OmniRobot** mobile platform in the **Webots** simulation environment, using **Component-Based Software Engineering (CBSE)** principles enabled by the **RoboComp** robotics framework.

---

## 🎓 Academic Context

- **Institution:** Universidad de Extremadura – Escuela Politécnica de Cáceres (Spain)
- **Degree:** Grado en Ingeniería Informática en Ingeniería de Computadores
- **Subject:** Robótica y Robótica Avanzada (2025/2026)
- **Instructor:** Prof. Pablo Bustos García (*RoboLab*)
- **Authors:**
  - **Ismael González Loro** (`igonzaleoa@alumnos.unex.es`)
  - **Samuel Corrionero Fernández** (`scorrion@alumnos.unex.es`)
  - **José Pulido Delgado** (`jopulidod@alumnos.unex.es`)

📄 **Full Academic Report:** [docs/main.pdf](docs/main.pdf) | [LaTeX Source](docs/main.tex)

---

## 🏛 System Architecture & CBSE Paradigm

The software follows a modular **Component-Based Software Engineering (CBSE)** architecture. Individual software components communicate asynchronously or synchronously using Ice (Internet Communications Engine) middleware and RoboComp interface definitions (IDSL / CDSL).

```text
               +-------------------+
               |  Lidar3D Sensor   |
               +---------+---------+
                         | (3D Point Cloud)
                         v
+--------------+   +-----+------+   +-------------------+
|  OmniRobot   |<--| Webots     |<--| Chocachoca /      |
|  (Actuators) |   | Bridge     |   | Localiser / FSM   |
+--------------+   +-----+------+   +---------+---------+
                         ^                    |
                         | (Manual Override)  v
               +---------+---------+   +------+------------+
               | JoystickPublish   |   | Aspirator GUI     |
               +-------------------+   | Telemetry Metrics |
                                       +-------------------+
```

---

## 🚀 Projects & Activities Overview

| Activity | Component | Description & Key Algorithms | Video Demo |
| :--- | :--- | :--- | :---: |
| **Activity 1** | [`1_chocachoca`](tasks/1_chocachoca/README.md) | **Sweeping Robot:** Reactive floor coverage algorithm using a 4-state FSM (`SPIRAL`, `TURN`, `FORWARD`, `FOLLOW_WALL`) with 2-stage LiDAR filtering and linear braking laws. | - |
| **Activity 2** | [`2_localiser`](tasks/2_localiser/README.md) | **Geometry-Based Localiser:** $SE(2)$ rigid pose correction in rectangular rooms using RANSAC line extraction, Hungarian corner association, and SVD linearized least-squares solving. | [📹 Demo](https://drive.google.com/file/d/1f_QV6La_op_kgW51XC4HKma_aQV0oxyi/view?usp=sharing) |
| **Activity 3** | [`3_multiRoom`](tasks/3_multiRoom/README.md) | **Visually-Guided Door Traversal:** Autonomous door detection and multi-room crossing using dot-product orientation ambiguity resolution, sticky world targets, and a Gaussian-modulated motion controller. | [📹 Demo](https://drive.google.com/file/d/10n_1iG7BUhLd8JUsoaLkpJoZF9h5ERZR/view?usp=sharing) |
| **Activity 4** | [`4_multiroom_advanced`](tasks/4_multiroom_advanced/README.md) | **Multi-Room Autonomous Navigation & AI:** Ultra-Localiser with 200-hypothesis grid search, real-time Topological Graph mapping, priority door selection, and PyTorch CNN MNIST room digit recognition. | [📹 Demo](https://drive.google.com/file/d/1vdQ3AvWZ5x7y0ox2yeZf0NKtGY7jAHUi/view?usp=sharing) |

---

## 📂 Repository Structure

```text
ODFE_robotica/
├── docs/                        # Complete IEEE academic paper and media
│   ├── main.pdf                 # Compiled IEEE conference paper
│   ├── main.tex                 # LaTeX source file
│   └── multimedia/              # Experimental plots, diagrams, and figures
│
├── tasks/                       # Core course activities
│   ├── 0_pretasks/              # Initial RoboComp & Qt introductory exercises
│   ├── 1_chocachoca/            # Activity 1: Reactive sweeping robot
│   ├── 2_localiser/             # Activity 2: Geometry-based room localiser
│   ├── 3_multiRoom/             # Activity 3: Single-door room crossing
│   └── 4_multiroom_advanced/    # Activity 4: Multi-room topological graph & AI
│
└── others/                      # Utilities & startup scripts
    └── initializeComponents/    # Automated component launch configuration
```

---

## 🛠 Technology Stack & Tools

- **Operating System:** Ubuntu Linux 22.04 LTS
- **Programming Languages:** C++23, Python 3.10
- **Frameworks & Libraries:**
  - **RoboComp:** Component middleware and IDSL generation tools.
  - **Webots 2023+:** Professional 3D mobile robot simulator.
  - **Qt6:** Real-time Graphical User Interfaces (GUI) and event management.
  - **Eigen3:** High-performance matrix manipulation and linear solvers (SVD).
  - **OpenCV 4:** Color space conversion and contour segmentation.
  - **PyTorch:** Deep learning inference engine for MNIST digit recognition.
- **Build System:** CMake $\ge 3.16$ & Make

---

## ⚡ Quick Start & Execution

1. **Clone the repository:**
   ```bash
   git clone https://github.com/CoferSamuel/ODFE_robotica.git
   cd ODFE_robotica
   ```

2. **Build a specific activity (e.g., Activity 2 Localiser):**
   ```bash
   cd tasks/2_localiser
   cmake -B build -DCMAKE_BUILD_TYPE=Release
   make -C build -j$(nproc)
   ```

3. **Run the component (with Webots running):**
   ```bash
   ./bin/localiser etc/config
   ```

---

## 📜 Citation & Credits

If you refer to this work or codebase, please cite the main technical report:

```bibtex
@techreport{ODFE2025Robotics,
  title       = {Component-Based Reactive Navigation and Autonomous Multi-Room Exploration for the OmniRobot Platform},
  author      = {González Loro, Ismael and Corrionero Fernández, Samuel and Pulido Delgado, José},
  institution = {Universidad de Extremadura, Escuela Politécnica de Cáceres},
  year        = {2025},
  note        = {Supervised by Prof. Pablo Bustos García}
}
```
