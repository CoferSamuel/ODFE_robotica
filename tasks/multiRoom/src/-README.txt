Localiser src directory: file overview

This note explains what each file under `src/` is for. It helps navigate the Localiser component and understand how pieces fit together.

- common_types.h
	Shared data types used across the project: Lidar point aliases, `LineSegment`, `Lines`, `Corner(s)`, `Match`, and helpers. Centralises geometry and container shapes so detectors, matchers, and the worker agree on formats.

- hungarian.h / hungarian.cpp
	High-level wrapper for the Hungarian (Munkres) assignment algorithm to match detected room corners to nominal model corners. Exposes `rc::Hungarian::match` and distance utilities.

- munkres.hpp
	Header-only implementation of the Munkres/Hungarian algorithm. Consumed by `hungarian.cpp` to compute minimum-cost bipartite matchings.

- ransac_line_detector.h / ransac_line_detector.cpp
	RANSAC-based 2D line detector. Extracts line segments from 2D point clouds (e.g., projected lidar) and returns `LineSegment` objects with endpoints, direction, inliers, and quality score.

- room_detector.h / room_detector.cpp
	Room feature extraction utilities. Computes corners from points/lines (multiple overloads), optionally estimates room size, filters lines, and provides drawing helpers to visualise lines and corners in a `QGraphicsScene`.

- specificworker.h / specificworker.cpp
	Main runtime component (RoboComp `SpecificWorker`). Orchestrates sensing (LIDAR), detection (RANSAC/Room_Detector), matching (Hungarian), visualisation (two viewers), and a simple behaviour state machine (FORWARD, TURN, FOLLOW_WALL, SPIRAL). Contains `initialize()`, `compute()`, behaviours, and speed-setting logic.

- mainUI.ui
	Qt Designer UI definition for the component window and its frames/viewers.

- readme.txt
	This file.

Notes
- Typical flow: LIDAR points → RANSAC line segments → room corners (`Room_Detector`) → Hungarian matching vs nominal room → state machine decisions in `SpecificWorker`.
- Geometry units are millimetres unless stated otherwise. Angles typically in radians; some Qt helpers may use degrees.
