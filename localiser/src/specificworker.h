/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Purpose: Declaration of `SpecificWorker`, the main runtime component for
	the Localiser. It orchestrates sensing, detection (lines/corners), matching
	(Hungarian), visualization and motion behaviors (state machine). */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <qtmetamacros.h>
#include <QGraphicsItem>
#include <vector>
#include "hungarian.h"
#include "ransac_line_detector.h"
#include "room_detector.h"


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include <genericworker.h>


struct NominalRoom
{
	float width; //  mm
	float length;
	Corners corners;
	explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) : width(width_), length(length_), corners(std::move(corners_)){};
	Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
	{
		Corners transformed_corners;
		for(const auto &[p, _, __] : corners)
		{
			auto ep = Eigen::Vector2d{p.x(), p.y()};
			Eigen::Vector2d tp = transform * ep;
			transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
		}
		return transformed_corners;
	}
};

// Declare global nominal room (defined in a single translation unit to avoid multiple-definition link errors)
extern NominalRoom room;

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT


public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();


public slots:
	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();


    void new_target_slot(QPointF);





	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();

private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	QRectF dimensions;
	AbstractGraphicViewer *viewer;
	const int ROBOT_LENGTH = 400;
	QGraphicsPolygonItem *robot_polygon;
	bool startup_check_flag;
	
	AbstractGraphicViewer *viewer_room;
	QGraphicsPolygonItem *robot_room_draw;
	// Persistent room polygon shown in the room viewer (created on first compute)
	QGraphicsPolygonItem *roomItemRoom;
	// Corner marker items stored to avoid re-adding them every compute
	std::vector<QGraphicsItem*> room_corner_items_main;
	std::vector<QGraphicsItem*> room_corner_items_room;
	Eigen::Affine2d robot_pose;
	rc::Room_Detector room_detector;
	rc::Hungarian hungarian;

	enum class State{IDLE, FORWARD, TURN, FOLLOW_WALL, SPIRAL};
	SpecificWorker::State state = SpecificWorker::State::SPIRAL;


	std::optional<RoboCompLidar3D::TPoints>  filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints &points);

    RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);

    void draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene);

	std::tuple<SpecificWorker::State, float, float> forward(const RoboCompLidar3D::TPoints &points);
    std::tuple<SpecificWorker::State, float, float> turn(const RoboCompLidar3D::TPoints &points);
	std::tuple<SpecificWorker::State, float, float> follow_wall(const RoboCompLidar3D::TPoints &points);
	std::tuple<SpecificWorker::State, float, float> spiral(const RoboCompLidar3D::TPoints &points);
	void SetMachineSpeed(std::tuple<State, float, float> result);
	std::tuple<SpecificWorker::State, float, float> StateMachine(RoboCompLidar3D::TPoints filtered_points);
signals:
	//void customSignal();
};

#endif