/*
 * Copyright (C) 2025 by YOUR NAME HERE
 *
 * This file is part of RoboComp (GPL v3 or later). See LICENSE for details.
 *
 * Purpose: Declaration of `SpecificWorker`, the main runtime component for the
 * Localiser. It orchestrates sensing, detection (lines/corners), matching
 * (Hungarian), visualization and motion behaviors (state machine).
 */

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// Qt & project includes (keep minimal in headers)
#include <qtmetamacros.h>                         // Q_OBJECT macro
#include <genericworker.h>                        // Base class provided by RoboComp
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>

// Local components
#include "hungarian.h"
#include "ransac_line_detector.h"
#include "room_detector.h"

// Forward declarations to reduce compile-time coupling
class QGraphicsPolygonItem;
class QGraphicsScene;
class QRectF;

// Nominal room model used for drawing and basic transforms
struct NominalRoom
{
	float width;                 // Room width in mm
	float length;                // Room length in mm
	Corners corners;             // Room corners in scene coordinates

	explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners corners_ = {})
		: width(width_), length(length_), corners(std::move(corners_)) {}

	// Transform room corners with an arbitrary 2D affine transform
	// For room->robot you can pass the inverse of the robot pose
	Corners transform_corners_to(const Eigen::Affine2d &transform) const
	{
		Corners transformed_corners;
		for (const auto &[p, _, __] : corners)
		{
			auto ep = Eigen::Vector2d{p.x(), p.y()};
			Eigen::Vector2d tp = transform * ep;
			transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
		}
		return transformed_corners;
	}
};

// Definition of the global nominal room (previously defined in the header which caused multiple-definition errors)
extern NominalRoom room;

// Main worker class -----------------------------------------------------------
class SpecificWorker : public GenericWorker
{
	Q_OBJECT

public:
	// Types ------------------------------------------------------------------
	enum class State { IDLE, FORWARD, TURN, FOLLOW_WALL, SPIRAL };

	// Construction -----------------------------------------------------------
	/**
	 * @brief Construct the Localiser worker.
	 * @param configLoader Configuration source (reads properties, proxies, etc.).
	 * @param tprx Tuple of Ice proxies provided by the framework.
	 * @param startup_check If true, run a short self-test and exit.
	 */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
	 * @brief Destructor. Releases owned Qt objects via parent hierarchy.
	 */
	~SpecificWorker();

public slots: // DOUBT what is a slot?
	// Lifecycle --------------------------------------------------------------
	/**
	 * @brief One-time initialisation of viewers, scene geometry and state.
	 * Creates the main viewer (robot + lidar) and the room viewer (model).
	 */
	void initialize();

	void new_target_slot(QPointF);

	/**
	 * @brief Main periodic loop. Polls sensors, updates visuals and state.
	 * Fetches LIDAR data, filters points, draws them and advances the state machine.
	 */
	void compute();

	// Emergency & restore ----------------------------------------------------
	/** @brief Called while in emergency state. Keep the component safe. */
	void emergency();
	/** @brief Restore from emergency to normal operation. */
	void restore();

	// Startup check ----------------------------------------------------------
	/**
	 * @brief Quick startup check. Schedules app quit and returns status.
	 * @return 0 on success (by convention).
	 */
	int startup_check();

signals: // DOUBT what is this for?
	// void customSignal();

private:
	// Constants --------------------------------------------------------------
	const int ROBOT_LENGTH = 400;        // Robot visual size in mm for drawing
	const int ROBOT_WIDTH  = 400;        // Robot visual size in mm for drawing

	// Viewers & graphics -----------------------------------------------------
	QRectF                    dimensions;       // Scene extents for main viewer
	AbstractGraphicViewer*    viewer = nullptr; // Main viewer (left)
	QGraphicsPolygonItem*     robot_polygon{};  // Robot graphic in main viewer

	AbstractGraphicViewer*    viewer_room{};    // Room viewer (right)
	QGraphicsPolygonItem*     robot_room_draw{};// Robot graphic in room viewer

	// State & models ---------------------------------------------------------
	bool                      startup_check_flag = false; // run self-check?
	Eigen::Affine2d           robot_pose;        // 2D pose used for transforms
												 
	rc::Room_Detector         room_detector;     // Line/corner detection
	rc::Hungarian             hungarian;         // Matching
	State     state = State::IDLE; // Initial state

	// Helpers ---------------------------------------------------------------
	/**
	 * @brief For each beam angle, keep the nearest LIDAR point (optional result).
	 * @param points Input LIDAR points (polar/cartesian fields available).
	 * @return Optional filtered set; empty if input had no points.
	 */
	std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints &points);

	/**
	 * @brief Remove isolated LIDAR points that lack close neighbours.
	 * @param points Input points after previous filtering.
	 * @param d Neighbourhood radius in millimetres.
	 * @return Points that have at least one neighbour within @p d.
	 */
	RoboCompLidar3D::TPoints  filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);

	/**
	 * @brief Draw LIDAR points as small rectangles in a scene (debug).
	 * @param points Points to draw.
	 * @param scene Target QGraphicsScene.
	 */
	void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene);

	/**
	 * @brief Detect and visualize corners from LIDAR points in the main viewer.
	 * 
	 * This method performs the complete corner detection pipeline:
	 *   1. Extracts line segments from LIDAR points using RANSAC
	 *   2. Computes line intersections at ~90° angles to identify corners
	 *   3. Applies non-maximum suppression to avoid duplicate corners
	 *   4. Draws red semi-transparent circles at detected corner positions
	 *   5. Manages dynamic visualization (removes old markers, adds new ones)
	 * 
	 * @param points Filtered LIDAR points to process for corner detection.
	 */
	void detect_and_draw_corners(const RoboCompLidar3D::TPoints &points);

	// Behaviours ------------------------------------------------------------
	/**
	 * @brief Forward behaviour: advance if free, otherwise transition to turn.
	 * @param points Filtered LIDAR points.
	 * @return Next state and commanded speeds {state, adv, rot}.
	 */
	std::tuple<State, float, float> forward(const RoboCompLidar3D::TPoints &points);

	/**
	 * @brief Turn behaviour: rotate to clear obstacles or transition onward.
	 * @param points Filtered LIDAR points.
	 * @return Next state and commanded speeds {state, adv, rot}.
	 */
	std::tuple<State, float, float> turn(const RoboCompLidar3D::TPoints &points);

	/**
	 * @brief Follow wall behaviour: regulate distance while moving forward.
	 * @param points Filtered LIDAR points.
	 * @return Next state and commanded speeds {state, adv, rot}.
	 */
	std::tuple<State, float, float> follow_wall(const RoboCompLidar3D::TPoints &points);

	/**
	 * @brief Spiral behaviour: increase advance and reduce rotation over time.
	 * @param points Filtered LIDAR points.
	 * @return Next state and commanded speeds {state, adv, rot}.
	 */
	std::tuple<State, float, float> spiral(const RoboCompLidar3D::TPoints &points);

	/**
	 * @brief Send computed speeds to the omni-robot proxy.
	 * @param result Triplet {state, adv, rot} from a behaviour.
	 */
	void                      SetMachineSpeed(std::tuple<State, float, float> result);

	/**
	 * @brief Evaluate the state machine and delegate to the appropriate behaviour.
	 * @param filtered_points Points used for decision-making.
	 * @return Triplet {state, adv, rot} for this cycle.
	 */
	std::tuple<State, float, float> StateMachine(RoboCompLidar3D::TPoints filtered_points);
};

#endif // SPECIFICWORKER_H