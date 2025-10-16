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
#include "specificworker.h"
#include <iostream>
#include <qcolor.h>
#include <QRect>
#include <cppitertools/groupby.hpp>
#include <cppitertools/enumerate.hpp>
//Filtrar lider solo por parte delantera
//Un if que va delante que compruebe anguilo
//ipot o icot con valor 400
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


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

    //initializeCODE

    /////////GET PARAMS, OPEND DEVICES....////////
    // int period = configLoader.get<int>("Period.Compute"); 
	//NOTE: If you want get period of compute use getPeriod("compute")
    // std::string device = configLoader.get<std::string>("Device.name");

	this->dimensions = QRectF(-6000, -3000, 12000, 6000);
	viewer = new AbstractGraphicViewer(this->frame, this->dimensions);
	this->resize(900,450);
	viewer->show();
	const auto rob = viewer->add_robot(ROBOT_LENGTH, ROBOT_LENGTH, 0, 190, QColor("Blue"));
	robot_polygon = std::get<0>(rob);

	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::new_target_slot);

}


void SpecificWorker::compute()
{
    std::cout << "Compute worker" << std::endl;

	std::vector<RoboCompLidar3D::TPoint> points;
	std::vector<RoboCompLidar3D::TPoint> arrayFrontal;
 

	try
	{


		const auto data= lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
		qInfo()<<"full"<< data.points.size();
		if (data.points.empty()){qWarning()<<"No points received"; return ;}
		const auto filter_data = filter_min_distance_cppitertools(data.points);
		qInfo() << filter_data.value().size();
		if (filter_data.has_value()){
			draw_lidar(filter_data.value(), &viewer->scene);
			points = filter_data.value();
		}


	}
	catch (const Ice::Exception &e){ std::cout << e.what() << std::endl; }

	// new_target_slot(QPointF());

	int x, z; float alpha;


	try {
		this->omnirobot_proxy->getBasePose(x, z, alpha);

		RoboCompGenericBase::TBaseState state;

		// Esto deberia de ir en el inicialize para que asi solo se ejecute una vez
		float advx = 0.0;
		float advz = 400.0;
		float rot = 0.0;
		this->omnirobot_proxy->setSpeedBase(advx, advz, rot);

		std::string name; float start, len; int decimationDegreeFactor;
		const auto data= lidar3d_proxy->getLidarDataWithThreshold2d("helios", len, decimationDegreeFactor);

		if (len == 0 && decimationDegreeFactor == 0)
			std::cout << "sa chocao " << std::endl;


		// Comprobamos si hay puntos delante y a menos de 450 cm de distancia
		//if(p.phi < M_PI_2 && p.phi > -M_PI_2 && p.r < 450)
		// 	if(derecha libre)
		// 		rot = -1.0;
		// 	else if(izquierda libre)
		// 		rot = 1.0;
		// 	else
		// 		set rotation(180)
	}
	catch(const Ice::Exception &e)
	{
	  std::cout << "Error in OmniRobot: " << e << std::endl;
	}

	// Lidar tests
	try {
		std::string name; float start, len; int decimationDegreeFactor;
		this->lidar3d_proxy->getLidarData(name, start, len, decimationDegreeFactor);
		std::cout << "LIDAR: " << "name: " << name << ", start: " << start << ", len: "  << len << ", decimationDegreeFactor: " << decimationDegreeFactor << std::endl;
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error in Lidar" << e << std::endl;
	}

//computeCODE
//try
//{
//  camera_proxy->getYImage(0,img, cState, bState);
//    if (img.empty())
//        emit goToEmergency()
//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
//  searchTags(image_gray);
//}
//catch(const Ice::Exception &e)
//{
//  std::cout << "Error reading from Camera" << e << std::endl;
//}

    // El min
    // si menor distancia en el centro del array es menor the 500
    // el minimo elemento está en la mitad del array. Para leerlo calcula el largo del array/2, y algo de Begin
    //    adv = 0, rot = 1
    // else adv 1000 rot = 0
  	// Si no hay puntos, no continuamos




    //   if (points.empty())
    //      return;


    //   // Obtener el punto central del array (zona frontal)
    //   int mid_index = points.size() / 2;
    //   float frontal_dist = points[mid_index].distance2d;


    //   // Comportamiento simple de evasión
    //   float side = 0.f;
    //   float adv = 0.f;
    //   float rot = 0.f;


    //   if (frontal_dist < 2000)  // obstáculo cerca
    //   {
    //      adv = 0.f;
    //      rot = 1.f;
    //   }
    //   else  // despejado
    //   {
    //      adv = 1000.f;
    //      rot = 0.f;
    //   }
    //   try
    //   {
    //      omnirobot_proxy->setSpeedBase(side, adv, rot);
    //   }catch (const Ice::Exception &e) {
    //      std::cout << e.what() << std::endl;
    //   }



}

void SpecificWorker::new_target_slot(QPointF)
{
    try {
        // Declaramos una variable para almacenar el estado actual del robot
        RoboCompGenericBase::TBaseState bState;

        // Llamamos al proxy del robot omni para obtener su estado actual
        // Esto llena bState con x, z, alpha (posición y orientación)
        omnirobot_proxy->getBaseState(bState);

        // Actualizamos la orientación del polígono que representa el robot en la vista
        // bState.alpha está en radianes, Qt usa grados, así que convertimos
        robot_polygon->setRotation(bState.alpha + M_PI_2);

        // Actualizamos la posición del polígono en la escena (x, z)
        // Nota: aquí se usa x y z porque Qt probablemente representa Y vertical
        robot_polygon->setPos(bState.x, bState.z);

        // Imprimimos por consola la orientación y posición para depuración
        std::cout << bState.alpha << " " << bState.x << " " << bState.z << std::endl;
     }
     catch (const Ice::Exception &e)  {
         // Capturamos cualquier excepción de Ice y la mostramos
         std::cout << e.what() << std::endl;
         return;
     }
}


void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene)
{
	// Se declara un vector estático para almacenar los puntos dibujados en la escena.
	// 'static' hace que el vector mantenga su contenido entre llamadas sucesivas a la función.
	static std::vector<QGraphicsItem*> draw_points;

	// Se recorren todos los puntos gráficos previamente dibujados para eliminarlos de la escena
	for (const auto &p : draw_points)
	{
		scene->removeItem(p);  // Se elimina el objeto visual de la escena
		delete p;              // Se libera la memoria ocupada por el objeto gráfico
	}
	draw_points.clear();       // Se limpia el vector para volver a llenarlo con nuevos puntos


	// Se define el color con el que se dibujarán los nuevos puntos (verde claro)
	const QColor color("Pink");

	// Se define el grosor y el color del trazo (pen) para los rectángulos que representarán los puntos
	const QPen pen(color, 10);

	// (Opcional) Se podría definir un pincel sólido para rellenar los rectángulos, pero está comentado
	// const QBrush brush(color, Qt::SolidPattern);

	// Se recorren todos los puntos recibidos del LiDAR
	for (const auto &p : points)
	{
		// Se crea un pequeño rectángulo (50x50 px) centrado en el origen (-25, -25)
		// para representar visualmente el punto en la escena
		const auto dp = scene->addRect(-25, -25, 50, 50, pen);

		// Se coloca el rectángulo en las coordenadas (x, y) del punto LiDAR
		dp->setPos(p.x, p.y);
		
		// Se guarda el puntero al rectángulo en el vector para poder eliminarlo la próxima vez
		draw_points.push_back(dp);
	}
}




void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
    //emergencyCODE
    //
    //if (SUCCESSFUL) //The componet is safe for continue
    //  emmit goToRestore()
}



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


std::optional<RoboCompLidar3D::TPoints> SpecificWorker::filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints& points)
{
	// non-empty condition
	if (points.empty())
	{
		return {};
	}
	RoboCompLidar3D::TPoints result;
	result.reserve(points.size());

	// loop over the groups produced by item::groupBy
	for (auto &&[angle, group] : iter::groupby(points, [](const auto& p)
	{

		float multiplier = std::pow(10.0f, 2);
		return std::floor(p.phi * multiplier) / multiplier;
	}
		)
	)
	{
		// 'group' is an iterable object containing all points for the current angle.
		auto min_it = std::min_element(std::begin(group), std::end(group),[]( const auto& a, const auto& b) { return a.r < b.r; });
        // El filtro por distancia y ángulo se aplica aquí. 

			result.emplace_back(*min_it);
        	
	}

	return result;
}

// RoboCompLidar3D::TPoints SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d)
// {
//     if (points.empty()) return {};

//     const float d_squared = d * d;  // Avoid sqrt by comparing squared distances
//     std::vector<bool> hasNeighbor(points.size(), false);

//     // Create index vector for parallel iteration
//     std::vector<size_t> indices(points.size());
//     std::iota(indices.begin(), indices.end(), size_t{0});

//     // Parallelize outer loop - each thread checks one point
//     std::for_each(std::execution::par, indices.begin(), indices.end(), [&](size_t i)
//         {
//             const auto& p1 = points[i];
//             // Sequential inner loop (avoid nested parallelism)
//             for (auto &&[j,p2] : iter::enumerate(points))
//                 {
//                     if (i == j) continue;
//                     const float dx = p1.x - p2.x;
//                     const float dy = p1.y - p2.y;
//                     if (dx * dx + dy * dy <= d_squared)
//                     {
//                         hasNeighbor[i] = true;
//                         break;
//                     }
//                 }
//         });

//     // Collect results
//     std::vector<RoboCompLidar3D::TPoint> result;
//     result.reserve(points.size());
//     for (auto &&[i, p] : iter::enumerate(points))
//         if (hasNeighbor[i])
//              result.push_back(points[i]);
//     return result;
// }







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