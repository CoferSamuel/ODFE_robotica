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

/* Purpose: Implementation of the `SpecificWorker` runtime logic. This file
	contains the worker constructor/destructor, lifecycle hooks (`initialize`,
	`compute`) and the behaviour state machine implementations used to control
	the robot based on detected geometry and matches. */
#include "specificworker.h"
#include <iostream>
#include <qcolor.h>
#include <QRect>
#include <cppitertools/groupby.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/zip.hpp>
#include <algorithm>
#include "common_types.h"


using namespace std;


const float MIN_DISTANCE_TURN = 600.0f; // Distancia mínima para salir del estado TURN
const float MAX_ADV = 600.0f;         // Velocidad máxima de avance en mm/s
const float MIN_THRESHOLD = 50.0f;  // Distancia mínima para empezar a frenar en mm
const float MAX_BRAKE = 100.0f;       // Velocidad máxima de frenado en mm/s
const float DIST_CHANGE_TO_SPIRAL= 200000.0f;//Distancia a la que no consideramos que no hay nada cerca para la espiral

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
	this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period,
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}


/**
 * @brief Initialize the SpecificWorker GUI and internal state.
 *
 * This method sets up both the main viewer (left pane) used for live
 * visualization of the robot and LIDAR points, and the dedicated "room"
 * viewer (right pane) used to display the nominal room and a scaled view.
 */
void SpecificWorker::initialize()
{
   if(this->startup_check_flag)
   {
      this->startup_check();
   }
   else
   {
	std::cout << "initialize worker" << std::endl; // announce initialisation on stdout

	// --- Prepare GUI geometry -------------------------------------------------
	/*
	 * "viewer" (left pane) shows the robot at the center with a +/-6m x, +/-3m y
	 * scene extent. It will display live LIDAR points and robot movement.
	 *
	 * "viewer_room" (right pane) shows a fixed-size room with grid and
	 * nominal room outline. The robot will move inside this room according to its pose.
	 */
		// Setup the auto-generated GUI from Qt Designer
		setupUi(this);
	
       viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
       auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
       robot_draw = r;
       
       viewer_room = new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);
       auto [rr, re] = viewer_room->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
       robot_room_draw = rr;
       // draw room in viewer_room

       viewer_room->scene.addRect(nominal_rooms[0].rect(), QPen(Qt::black, 30));
	

       // initialise robot pose
       robot_pose.setIdentity();
       robot_pose.translate(Eigen::Vector2d(0.0,0.0));

	   	this->show();


		// Connect mouse events from the viewer to a slot that handles new targets (clicks)
		connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);
		srand(time(NULL));       // Viewer

		// time series plotter for match error
       TimeSeriesPlotter::Config plotConfig;
       plotConfig.title = "Maximum Match Error Over Time";
       plotConfig.yAxisLabel = "Error (mm)";
       plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
       plotConfig.autoScaleY = false;       // We will set a fixed range
       plotConfig.yMin = 0;
       plotConfig.yMax = 1000;
    	
	   SpecificWorker::time_series_plotter = std::make_unique<TimeSeriesPlotter>(frame_plot_error, plotConfig);
       match_error_graph = time_series_plotter->addGraph("", Qt::blue);

	   //stop robot
	   move_robot(0,0,0);
   }
}


void SpecificWorker::new_target_slot(QPointF target)
{
	try {
		RoboCompGenericBase::TBaseState bState;         // variable to receive odometry/state
		omnirobot_proxy->getBaseState(bState);          // populate bState with x, z, alpha

		// Update graphic representing robot orientation (Qt rotation uses degrees internally for items)
		robot_draw->setRotation(bState.alpha + M_PI_2);

		// Update graphic position using robot's x,z coordinates
		robot_draw->setPos(bState.x, bState.z);

		// Print state for debugging
		std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
	 }
	 catch (const Ice::Exception &e)  {
		 // Print ICE exception in case of RPC failure
		 std::cout << e.what() << std::endl;
		 return;
	 }
}

void SpecificWorker::compute()
{
	QThread::msleep(500); // wait for viewer to be ready
    std::cout << "Compute worker" << std::endl;
	// Resultado devuelto por las funciones de comportamiento
    RoboCompLidar3D::TPoints data = read_data();
	data= door_detector.filter_points(data, &viewer->scene);



	// ========== CORNER DETECTION AND VISUALIZATION ==========
	// Detect corners from LIDAR points and visualize them in the main viewer
	// This call encapsulates the entire pipeline: RANSAC line extraction, 
	// 90° intersection detection, and dynamic circle visualization

	// Cogemos las esquinas detectadas o mesuradas
	const auto &[corners, lines] = room_detector.compute_corners(data, &viewer->scene);

	// ========== ROBOT POSE ESTIMATION VIA CORNER MATCHING ==========
	// Estimate and update the robot's pose based on matched corners
	const auto center_opt = room_detector.estimate_center_from_walls(lines);
	draw_lidar(data, center_opt, &viewer->scene);


	// Transform nominal room corners to robot frame for matching
	Corners robot_corners = nominal_rooms[0].transform_corners_to(robot_pose.inverse());
	
	// Match detected corners to nominal room corners using Hungarian algorithm
	auto matched_corners = hungarian.match(corners, robot_corners);

	  // compute max of  match error
   float max_match_error = 99999.f;  
   if (not matched_corners.empty())
   	{
       	const auto max_error_iter = std::ranges::max_element(matched_corners, [](const auto &a, const auto &b)
    	{ return std::get<2>(a) < std::get<2>(b); });

       	max_match_error = static_cast<float>(std::get<2>(*max_error_iter));
       	time_series_plotter->addDataPoint(match_error_graph,max_match_error);
       	//print_match(match, max_match_error); //debugging
   	}
	Eigen::Vector3d pose = solve_pose(corners, matched_corners);
    // update robot pose
	if (localised)
		update_robot_pose(pose);

	RetVal ret_val = process_state(data, corners, matched_corners, viewer);
   	auto [st, adv, rot] = ret_val;
   	state = st;

	//move_robot(adv, rot, max_match_error);


	// Update robot graphic position in the room viewer
	robot_room_draw->setPos(robot_pose.translation().x(), robot_pose.translation().y());
	double angle = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
	robot_room_draw->setRotation(qRadiansToDegrees(angle));



	// Descomentar para que se mueva solos
	//std::tuple<SpecificWorker::State, float, float> result= StateMachine(filtered_points);
	// Aplicar las velocidades calculadas al robot
	//SetMachineSpeed(result);
}

void SpecificWorker::emergency()
{
}

// Removed: new_target_slot(QPointF) no longer used




//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
    //restoreCODE
    //Restore emergency component

}


int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

	/**
 * @brief Dibuja los puntos del LiDAR en una escena gráfica, con un desplazamiento opcional.
 * * Esta función limpia los puntos dibujados en la llamada anterior y dibuja
 * los nuevos puntos (filtered_points). Si se proporciona 'center',
 * todos los puntos se dibujan relativos a esa coordenada (actúa como un offset).
 * * @param filtered_points Puntos del LiDAR a dibujar.
 * @param center          Posición (opcional) que se usará como origen para dibujar los puntos.
 * @param scene           Puntero a la QGraphicsScene donde se deben dibujar los puntos.
 */
void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, 
                                std::optional<Eigen::Vector2d> center, 
                                QGraphicsScene *scene)
{
    // Vector estático para almacenar los puntos dibujados y poder borrarlos después.
    static std::vector<QGraphicsItem*> draw_points;

    // 1. Limpia los puntos de la iteración anterior
    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    // 2. Define el estilo de los nuevos puntos
    const QColor color("Pink");
    const QPen pen(color, 10); // Un trazo rosa de 10px de grosor

    // 3. Determina el desplazamiento (offset) basado en el nuevo parámetro 'center'
    // Si 'center' no tiene valor, el offset es (0, 0).
    double offsetX = 0.0;
    double offsetY = 0.0;
    
    if (center.has_value()) // Comprueba si se ha pasado un centro
    {
        offsetX = center.value().x(); // Obtiene la coordenada x del centro
        offsetY = center.value().y(); // Obtiene la coordenada y del centro
    }

    // 4. Dibuja los nuevos puntos usando el nombre de variable 'filtered_points'
    for (const auto &p : filtered_points)
    {
        // Crea un pequeño rectángulo para representar el punto
        const auto dp = scene->addRect(-25, -25, 50, 50, pen);

        // Coloca el punto en su coordenada (p.x, p.y) MÁS el offset
        dp->setPos(offsetX + p.x, offsetY + p.y);

        // Guarda el punto dibujado para borrarlo en la siguiente llamada
        draw_points.push_back(dp);
    }
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

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::read_data()
{
    // 1. Declara 'filter_data' en el alcance de la función (aquí fuera)
    RoboCompLidar3D::TPoints filter_data = {}; // Inicialízala vacía

    try
    {
        const auto data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
        
        // 2. Asigna el valor (sin 'RoboCompLidar3D::TPoints' delante)
        //    Usamos .value_or() para desenvolver el 'optional' que devuelve la función
        filter_data = filter_min_distance_cppitertools(data.points).value_or(RoboCompLidar3D::TPoints{});
        
        if (data.points.empty())
        {
            qWarning() << "No points received"; 
            return {};
        }
    }
    catch (const Ice::Exception &e)
    { 
        std::cout << e.what() << std::endl; 
        return {}; // Devuelve vacío también si hay un error
    }

    // 3. Ahora 'filter_data' SÍ existe aquí, y es un TPoints (un vector), 
    //    así que no uses .value()
    return filter_isolated_points(filter_data, 200.0f);
}

std::expected<int, std::string> SpecificWorker::closest_lidar_index_to_given_angle(const auto &points, float angle)
{
    return std::expected<int, std::string>();
}

RoboCompLidar3D::TPoints SpecificWorker::filter_same_phi(const RoboCompLidar3D::TPoints &points)
{
    return RoboCompLidar3D::TPoints();
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

void SpecificWorker::print_match(const Match &match, const float error) const
{
}


bool SpecificWorker::update_robot_pose(Eigen::Vector3d pose)
{

	std::cout << "Nueva pose estimada: " << std::endl;
	std::cout << "X: " << pose(0) << " Y: " << pose(1) << " Theta: " << pose(2) << std::endl;
	qInfo() << "--------------------";


	if (pose.array().isNaN().any())
		return	false;

	robot_pose.translate(Eigen::Vector2d(pose(0), pose(1)));
	robot_pose.rotate(pose(2));

	return true;
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error)
{
	//TODO comprobar si el error es muy grande y no mover el robot
	if (max_match_error>500){
		std::cout << "Error muy grande, no muevo el robot" << std::endl;
		return;
	}
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;}
}

Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners, const Match &match)
{
	Eigen::MatrixXd W(match.size() * 2, 3);
	Eigen::VectorXd b(match.size() * 2);

	for (auto &&[i, m]: match | iter::enumerate)
	{
		auto &[meas_c, nom_c, _] = m;
		auto &[p_meas, __, ___] = meas_c;
		auto &[p_nom, ____, _____] = nom_c;
		
		b(2 * i)     = p_nom.x() - p_meas.x();
		b(2 * i + 1) = p_nom.y() - p_meas.y();
		W.block<1, 3>(2 * i, 0)     << 1.0, 0.0, -p_meas.y();
		W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
	}
	// estimate new pose with pseudoinverse
	const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
    return r;
}

std::tuple<SpecificWorker::STATE, float, float> SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer)
{

	// TURN
    return {SpecificWorker::STATE::TURN, 0.0f, 0.5f};
}









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


