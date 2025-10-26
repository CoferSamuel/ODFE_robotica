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

#include "specificworker.h"                       // Header that declares SpecificWorker class
#include <iostream>                                // std::cout, std::endl
#include <qcolor.h>                                // QColor used for drawing
#include <QRect>                                   // QRectF / QRect types
#include <cppitertools/groupby.hpp>                // grouping helper for filter_min_distance_cppitertools
#include <cppitertools/enumerate.hpp>              // enumerate helper for iteration with indices
#include <cppitertools/zip.hpp>                    // zip helper (not currently used but included)
#include <algorithm>                               // std::min_element, std::min, std::max, std::clamp
#include <optional>                                // std::optional used by filter_min_distance_cppitertools
#include <numeric>                                 // std::iota used in filter_isolated_points
#include <cmath>                                   // std::floor, std::pow used in grouping
using namespace std;                                // bring standard names into scope for readability

// --- Tunable constants used by behaviour/state logic ---
const float MIN_DISTANCE_TURN = 600.0f;             // minimum safe distance (mm) to stop turning and resume forward
const float MAX_ADV = 800.0f;                       // maximum forward speed (mm/s)
const float MIN_THRESHOLD = 50.0f;                  // threshold distance where braking begins (mm)
const float MAX_BRAKE = 100.0f;                     // maximum braking contribution to linear speed (mm/s)
const float DIST_CHANGE_TO_SPIRAL= 2000.0f;         // if nearest obstacle further than this, switch to spiral exploration

// SpecificWorker constructor: sets up initial state and optional startup check
SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	// store whether we are running in startup-check mode (used by tests/CI)
	this->startup_check_flag = startup_check;
	// If running a startup check, run it and return immediately
	if(this->startup_check_flag)
	{
		this->startup_check();                        // run quick startup check
	}
	else
	{
		// If hibernation support is enabled, start the hibernation checker timer
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		// Example of how to configure a state machine is provided (commented) — not active by default
		/***
		// Example setup for a custom state in the GRAFCET state machine
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period,
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function called periodically
															std::bind(&SpecificWorker::customEnter, this), // Called when entering the state
															std::bind(&SpecificWorker::customExit, this)); // Called when exiting the state

		// Example of how to add transitions between states (signals triggered externally)
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); // Define 'customSignal' in .h Signals section

		// Add the custom state to the state machine
		statemachine.addState(states["CustomState"].get());
		***/

		// Configure the state machine to have exclusive child mode (only one active substate)
		statemachine.setChildMode(QState::ExclusiveStates);
		// Start the state machine now that it's configured
		statemachine.start();

		// Check the state machine for configuration errors and throw if any
		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;                         // print Qt warning
			throw error;                                 // propagate as exception to fail fast
		}
	}
}

// Destructor: called when the object is destroyed
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl; // log for debugging
}


// initialize(): sets up UI viewer, scene dimensions and connections
void SpecificWorker::initialize()
{
	std::cout << "initialize worker" << std::endl;      // print initialization message

	// Placeholder where device initialization and parameter fetching would go
	/////////GET PARAMS, OPEN DEVICES....////////
	// int period = configLoader.get<int>("Period.Compute");
	// NOTE: If you want get period of compute use getPeriod("compute")
	// std::string device = configLoader.get<std::string>("Device.name");

	// Define the scene/world dimensions in millimetres for the viewer
	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	// Create a graphics viewer using the previously defined frame and dimensions
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	// Resize the widget to a sensible default
	this->resize(900,450);
	// Show the viewer widget (Qt GUI)
	viewer->show();
	// Add a robot graphic to the viewer and keep its polygon handle for later updates
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);                      // extract the polygon item from returned tuple

	// Connect mouse events from the viewer to a slot that handles new targets (clicks)
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
	srand(time(NULL));

}


// compute(): main periodic loop executed at configured frequency
void SpecificWorker::compute()
{

	std::cout << "Compute worker" << std::endl;         // print each compute invocation
	// The behavior functions return a tuple: (next state, linear_speed, angular_speed)
	std::tuple<SpecificWorker::State, float, float> result;
	RoboCompLidar3D::TPoints filtered_points;             // will hold lidar points after filtering

	try
	{
		// Request 2D-thresholded lidar data named "helios" up to 12 metres, decimation factor 1
		const auto data= lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
		qInfo()<<"full"<< data.points.size();            // log number of received points
		// Collapse points to one per angle (nearest in each angle bin)
		const auto filter_data = filter_min_distance_cppitertools(data.points);
		// Defensive: if the raw data is empty, warn and return early
		if (data.points.empty()){qWarning()<<"No points received"; return ;}
		// Remove isolated points (likely noise) using a neighborhood threshold of 200 mm
		filtered_points = filter_isolated_points(filter_data.value(), 200.0f);

		// If we have filtered data, draw it on the viewer scene for debugging/visualization
		if (filter_data.has_value())
			draw_lidar(filter_data.value(), &viewer->scene);

	}
	catch (const Ice::Exception &e){ std::cout << e.what() << std::endl; } // handle Ice exceptions gracefully


	// Decide which behavior to execute depending on current finite state
	switch(state)
	{
		case SpecificWorker::State::IDLE:
			// In IDLE simply transition immediately to FORWARD (example behaviour)
			state = SpecificWorker::State::FORWARD;
			break;

		case SpecificWorker::State::FORWARD:
			// Execute forward behaviour which computes speeds and possibly next state
			result = forward(filtered_points);
			break;

		case SpecificWorker::State::TURN:
			// Execute turn behaviour
			result = turn(filtered_points);
			break;

		case SpecificWorker::State::FOLLOW_WALL:
			// Execute wall-following behaviour
			result = follow_wall(filtered_points);
			break;

		case SpecificWorker::State::SPIRAL:
			// Execute spiral exploration behaviour
			result = spiral(filtered_points);
			break;
	}
	// Extract the computed next state and speeds from the behavior result
	state = std::get<0>(result);
	float adv = std::get<1>(result);
	float rot = std::get<2>(result);
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);} 
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;} // send speeds to robot base
}

// forward(): behaviour when trying to move forward while avoiding frontal obstacles
// It returns a tuple: (next state, linear speed [mm/s], angular speed [rad/s])
std::tuple<SpecificWorker::State, float, float> SpecificWorker::forward(const RoboCompLidar3D::TPoints& points)
{
	// Prepare return tuple
	std::tuple<SpecificWorker::State, float, float> result;

	// Collect only points that are roughly in front of the robot (-45° .. +45°)
	RoboCompLidar3D::TPoints frontal_points;
	frontal_points.reserve(points.size());                // reserve to avoid reallocation
	for (const auto &p : points)
	{
		if (p.phi > (-M_PI_4 )&& p.phi < (M_PI_4))  // -45° < phi < +45°
		{
			frontal_points.push_back(p);              // keep frontal points
		}
	}
	// Find the closest point (the most limiting obstacle), spiral needs all points
	auto min_dist_spiral = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });    // Punto más cercano

	// Find the closest frontal point (the most limiting obstacle)
	auto min_dist = std::min_element(std::begin(frontal_points), std::end(frontal_points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });    // Punto más cercano

	// If the nearest frontal obstacle is very far, switch to spiral exploration
	if (min_dist_spiral->r >DIST_CHANGE_TO_SPIRAL) {
		return{State::SPIRAL,0.0f,0.0f};              // change behaviour to SPIRAL

	}else{ // If the nearest frontal obstacle is close
		// Decide whether to keep going forward or enter turning state based on min distance
		if (min_dist->r > MIN_DISTANCE_TURN) {
			// Compute forward speed proportional to how far the nearest obstacle is, clamped to MAX_ADV
			// Either go at maximum speed or scale down based on proximity to obstacle
			float adv_speed = std::min(MAX_ADV, (min_dist->r - MIN_THRESHOLD) * (MAX_ADV / (MIN_DISTANCE_TURN - MIN_THRESHOLD)));
			result = std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::FORWARD, adv_speed, 0.f);
		} else {
			qInfo() << "FORWARD -> TURN";                  // log state transition
			// Compute a braking term proportional to how close we are to the critical distance
			float brake_speed = std::min(MAX_BRAKE, (MIN_DISTANCE_TURN - min_dist->r) * (MAX_BRAKE / (MIN_DISTANCE_TURN - MIN_THRESHOLD)));
			float brake_rot = brake_speed / 2.0f;               // reduce rotation while braking 
			result = std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::TURN, 0.f, brake_rot);
		}
	}	
	return result;                                      // return chosen state and speeds
}

// turn(): behaviour executed while the robot is turning to avoid obstacles
// State machine: if obstacle removed -> sometimes follow wall or go forward; else keep turning
std::tuple<SpecificWorker::State, float, float> SpecificWorker::turn(const RoboCompLidar3D::TPoints& points)
{
	static int cont=0;                                  // persistent counter across calls
	std::tuple<SpecificWorker::State, float, float> result; // For return value
	static float rot = 0.6f;                        	// default rotation speed

	// Select frontal points (-45°..45°) to reason about the immediate front
	RoboCompLidar3D::TPoints frontal_points;
	frontal_points.reserve(points.size()); // To avoid reallocations when pushing back
												// When we reserve space, we are telling the vector to allocate enough memory
												// for the specified number of elements (points.size()). This way, when we
												// push_back elements into the vector, it doesn't have to reallocate memory
												// each time, which can be inefficient. Instead, it can just use the already
												// allocated space.
	// Select frontal points (-45°..45°) to reason about the immediate front
	for (const auto &p : points)
	{
		if (p.phi > (-M_PI_4 )&& p.phi < (M_PI_4))
		{
			frontal_points.push_back(p);
		}
	}
	// find the nearest frontal point
	auto min_dist = std::min_element(std::begin(frontal_points), std::end(frontal_points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });    // Punto más cercano

	// If obstacle has moved sufficiently away, decide randomly between FOLLOW_WALL and FORWARD
	if (min_dist->r > MIN_DISTANCE_TURN+100)
	{
		// Randomly choose rotation speed for next turns
		// 30 percent chance to turn more aggressively, number more bigger means less turns
		rot = (rand() % 100 < 70) ? 0.6f : 3.0f;
		cont=0;                                        // reset counter
		// Randomly decide whether to follow wall or continue forward
		if (std::rand() % 2 == 0)
		{
			qInfo() << "TURN -> FOLLOW WALL";
			result = {State::FOLLOW_WALL, MAX_ADV, 0.0f};  // resume with forward velocity
		}
		else
		{
			qInfo() << "TURN -> FORWARD";
			result = {State::FORWARD, MAX_ADV, 0.0f};
		}

	}
	else { // still obstacle ahead, continue turning

		qInfo() << "TURN AGAIN";                     
		cont++;
		if (cont>=100)
			result=  {State::TURN, 0.0f, 1.0f};        // force a positive rotation after many TURN AGAIN
		else {
			if ( rot > 0.6f ) qInfo() << "TURN MORE AGGRESSIVELY" << ", rot:" << rot;
			if (min_dist->phi < 0)
				result =  {State::TURN, 0.0f, rot};  // obstacle to the left -> turn one direction
			else
				result = {State::TURN, 0.0f, -rot};  // obstacle to the right -> turn opposite direction
		}


	}
	return result;                                      // return decision
}

// follow_wall(): simple heuristic wall-following behaviour
// It inspects the closest point overall and adjusts rotation to keep a roughly constant distance
std::tuple<SpecificWorker::State, float, float> SpecificWorker::follow_wall(const RoboCompLidar3D::TPoints &points)
{

	// Find absolute nearest point in the pointcloud (anywhere, not only frontal)
	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
		{ return p1.r < p2.r; });
	qInfo() << "DISTANCIA PARED------------------"<<min_dist->r; // debug log
	// If in a narrow band near MIN_DISTANCE_TURN, keep following
	if (min_dist->r >= MIN_DISTANCE_TURN+50 && min_dist->r < MIN_DISTANCE_TURN+200)
	{
		qInfo() << "FOLLOW WALL";
		return {State::FOLLOW_WALL, 550.0f, 0.0f};  // proceed forward slowly
	}

	// If too far from wall, gently steer towards it depending on which side it's located
	if (min_dist->r > MIN_DISTANCE_TURN+200)
	{
		if (min_dist->phi < 0)
		{
			qInfo() << "FOLLOW WALL LEFT FAR";
			return {State::FOLLOW_WALL, 270.0f, -1.5f}; // turn right slightly
		}

		qInfo() << "FOLLOW WALL RIGHT FAR";
		return {State::FOLLOW_WALL, 270.0f, 1.5f};     // turn left slightly

	}
	// If very close to wall, steer away slightly depending on side
	if (MIN_DISTANCE_TURN-120 < min_dist->r && min_dist->r < MIN_DISTANCE_TURN+50)
	{
		
		if (min_dist->phi < 0)
		{
			qInfo() << "FOLLOW WALL LEFT NEAR";
			return {State::FOLLOW_WALL, 270.0f, 2.2f};  // rotate to increase distance
		}

		qInfo() << "FOLLOW WALL RIGHT NEAR";

		return {State::FOLLOW_WALL, 270.0f, -2.2f};

	}

	// If none of the above, switch back to fast forward
	qInfo() << "FOLLOW WALL -> FORWARD";
	return {State::FORWARD, 600.0f, 0.0f};
}

// spiral(): exploration behaviour that gradually increases forward speed and reduces rotation to create an outward spiral
std::tuple<SpecificWorker::State, float, float> SpecificWorker::spiral(const RoboCompLidar3D::TPoints &points)
{

	static float adv_speed_spiral = 600.0f;   // initial linear speed for spiral (persistent)
	static float rot_speed_spiral = 1.8f;     // initial angular speed for spiral (persistent)
	const float MAX_ADV_SPIRAL = 1200.0f;     // cap for forward speed
	const float MIN_ROT = 0.0f;              // minimum angular speed (we allow it to go to 0)

	// Check for any close obstacles: if found -> go to TURN behaviour and reset spiral speeds
	auto min_dist = std::min_element(points.begin(), points.end(),[](const auto &a, const auto &b){ return a.r < b.r; });

	if (min_dist->r < MIN_DISTANCE_TURN)
	{
		qInfo() << "SPIRAL -> OBSTÁCULO DETECTADO. Cambiando a TURN.";
		adv_speed_spiral = 600.0f;   // reset linear speed for next time
		rot_speed_spiral = 1.8f;     // reset rotational speed
		return {State::TURN, 0.0f, 0.0f}; // switch to TURN immediately
	}

	// Gradually increase forward speed and slowly reduce rotation to create an outward spiral trajectory
	adv_speed_spiral = std::min(adv_speed_spiral + 2.5f, MAX_ADV_SPIRAL);

	if (rot_speed_spiral > MIN_ROT)
        rot_speed_spiral -= 0.03f * (rot_speed_spiral / 5.0f);  // reduce rotation gradually

	qInfo() << "SPIRAL: adv" << adv_speed_spiral << "rot" << rot_speed_spiral;
	return {State::SPIRAL, adv_speed_spiral, rot_speed_spiral};

}

// new_target_slot: (currently unused) -- called when user clicks on viewer to set new goal
// It queries the robot base state and updates the robot polygon in the scene accordingly
void SpecificWorker::new_target_slot(QPointF)
{
	try {
		RoboCompGenericBase::TBaseState bState;         // variable to receive odometry/state
		omnirobot_proxy->getBaseState(bState);          // populate bState with x, z, alpha

		// Update graphic representing robot orientation (Qt rotation uses degrees internally for items)
		robot_polygon->setRotation(bState.alpha + M_PI_2);

		// Update graphic position using robot's x,z coordinates
		robot_polygon->setPos(bState.x, bState.z);

		// Print state for debugging
		std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
	 }
	 catch (const Ice::Exception &e)  {
		 // Print ICE exception in case of RPC failure
		 std::cout << e.what() << std::endl;
		 return;
	 }
}


// draw_lidar: visualize a set of 2D lidar points in a QGraphicsScene
// This function clears the previously drawn points and draws the current set as small rectangles
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene)
{
	// static container keeps pointers to drawn items between calls so we can remove them later
	static std::vector<QGraphicsItem*> draw_points;

	// Remove any previously drawn items from the scene and free memory
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);  // remove from scene graph
		delete p;              // free the QGraphicsItem
	}
	draw_points.clear();       // clear the list of pointers

	// Choose a color and pen for drawing points
	const QColor color("Pink");
	const QPen pen(color, 10);  // pen width 10 (visual thickness)

	// For each lidar point, add a small rectangle centered on the point coordinates
	for (const auto &p : points)
	{
		const auto dp = scene->addRect(-25, -25, 50, 50, pen); // 50x50 px rect centered at origin
		dp->setPos(p.x, p.y);                      // move rect to lidar point coordinates
		draw_points.push_back(dp);                 // store pointer to remove later
	}

}

// Emergency, restore and startup_check implementations
// emergency(): slot potentially called by the framework when an emergency state is entered
void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl; // Log that emergency handler was called
	// emergencyCODE: place emergency shutdown or safe-stop code here
	// if (SUCCESSFUL) // The component is safe to continue
	//     emit goToRestore();
}


// restore(): slot executed when leaving emergency state to restore normal operation
void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl; // Log that restore handler was called
	// restoreCODE: reinitialize components and restore normal operation
}


// startup_check(): small self-test used during startup-check mode
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl; // Log performing startup checks
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit())); // Quit after 200 ms (simple auto-exit for startup test)
	return 0; // Return 0 to indicate success
}

// Esta función recibe un conjunto de puntos del LiDAR (cada punto tiene coordenadas x, y, r y phi)
// y devuelve, para cada ángulo 'phi', el punto más cercano (menor distancia 'r').
// Se usa std::optional para devolver un resultado vacío si no hay puntos válidos.
std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points)
{
	// 🧩 1️⃣ Si no hay puntos, se devuelve un optional vacío.
	if (points.empty())
	{
		return {};
	}

	// 🧩 2️⃣ Se crea un contenedor para los puntos filtrados.
	// Se reserva memoria para evitar realocaciones (mejora de rendimiento).
	RoboCompLidar3D::TPoints result;
	result.reserve(points.size());

	// 🧩 3️⃣ Se agrupan los puntos por su ángulo 'phi' con una precisión de 2 decimales.
	//    Esto significa que todos los puntos que tengan un ángulo muy similar se procesan juntos.
	for (auto &&[angle, group] : iter::groupby(points, [](const auto& p)
	{
		// Redondeo de 'phi' a dos decimales.
		float multiplier = std::pow(10.0f, 2);
		return std::floor(p.phi * multiplier) / multiplier;
	}))
	{
		// 🧩 4️⃣ Dentro de cada grupo de puntos con el mismo ángulo,
		// se busca el punto con menor distancia 'r' (el más cercano al robot).
		auto min_it = std::min_element(std::begin(group), std::end(group),
			[](const auto& a, const auto& b) { return a.r < b.r; });

		// 🧩 5️⃣ Se añade ese punto al vector de resultados.
		// Este punto representa el obstáculo más cercano en esa dirección.
		result.emplace_back(*min_it);
	}

	// 🧩 6️⃣ Se devuelve el conjunto de puntos filtrados (uno por ángulo).
	return result;
}


RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d)
{
    if (points.empty()) return {};

    const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
    std::vector<bool> hasNeighbor(points.size(), false);

    // Create index vector for parallel iteration
    std::vector<size_t> indices(points.size());
    std::iota(indices.begin(), indices.end(), size_t{0});

    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto& p1 = points[i];
        for (size_t j = 0; j < points.size(); ++j)
        {
            if (i == j) continue;
            const auto& p2 = points[j];
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            if (dx*dx + dy*dy <= d_squared)
            {
                hasNeighbor[i] = true;
                break;
            }
        }
    }

    // Collect results
    std::vector<RoboCompLidar3D::TPoint> result;
    result.reserve(points.size());
    for (auto &&[i, p] : iter::enumerate(points))
        if (hasNeighbor[i])
             result.push_back(points[i]);
    return result;
}





/*
	Fordward -> Basicamente set velocidad

		Codigo para avanzar rapido si no hay obstaculo cerca
					auto point = points.front();
					// point.distance2d es la distancia euclidiana al punto (por ejemplo, sqrt(x² + z²)).
					// 		si distance2d es grande, el resultado es grande;
					//		si es pequeño (obstáculo cerca), el resultado es pequeño.
					// std::clamp(valor, 1.f, 0.f) → limita el valor al rango [0, 1].
					// Por lo tanto, break_adv será 1 cuando no haya obstáculos cerca y 0 cuando estén muy cerca.
					float break_adv = std::clamp(point.distance2d * (1/MIN_THRESHOLD), 1.f, 0.f);
					// atan2(y, x) devuelve el ángulo en radianes entre el eje X y el punto (x, y).
					// Calcula el ángulo de rotación hacia el obstáculo:
					float rot = atan2(point.x, point.z);

					// Este término controla cuánto debe frenar la rotación en función del ángulo.
					// Si rot es positivo (obstáculo a la derecha), brake_rot disminuye con rot.
					// Si rot es negativo (obstáculo a la izquierda), brake_rot aumenta con rot
					float brake_rot = rot>=0 ? std::clamp(rot+1, 1.f, 0.f) : std::clamp(1-rot, 0.f, 1.f);

					// Combina los dos factores anteriores:
					// 		MAX_ADV: velocidad máxima (por ejemplo, 200 mm/s)
					//		break_adv: freno por distancia
					// 		brake_rot: freno por dirección
					float adv_speed = MAX_ADV * break_adv * brake_rot;

	Turn -> Basicamente set rotation,
							rotation se le mete velocidad y rota hasta que no encuentre una pared (lo hace solo)
	FollowWall -> El más complicado, al lidar ser una mierda debes de ir reajustando el angulo del robot
									 porque no es capaz de seguir recto correctamente
									 A todo esto tiene que ir detectando la pared
	Spining -> En teoria no es muy complicado
*/







/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float start, float len, int decimationDegreeFactor)
// RoboCompLidar3D::TDataImage this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long timestamp)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance, int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z, float alpha)
// RoboCompOmniRobot::void this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams