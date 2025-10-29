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
#include <cppitertools/zip.hpp>
#include <algorithm>
using namespace std;

const float MIN_DISTANCE_TURN = 600.0f; // Distancia mínima para salir del estado TURN
const float MAX_ADV = 600.0f;         // Velocidad máxima de avance en mm/s
const float MIN_THRESHOLD = 50.0f;  // Distancia mínima para empezar a frenar en mm
const float MAX_BRAKE = 100.0f;       // Velocidad máxima de frenado en mm/s
const float DIST_CHANGE_TO_SPIRAL= 2000.0f;//Distancia a la que no consideramos que no hay nada cerca para la espiral

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
	// Resultado devuelto por las funciones de comportamiento
	RoboCompLidar3D::TPoints filtered_points;

	try
	{
	 	const auto data= lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
		qInfo()<<"full"<< data.points.size();
		const auto filter_data = filter_min_distance_cppitertools(data.points);
		if (data.points.empty()){qWarning()<<"No points received"; return ;}
		filtered_points = filter_isolated_points(filter_data.value(), 200.0f);

//		qInfo() << filter_data.size();
		if (filter_data.has_value())
			draw_lidar(filter_data.value(), &viewer->scene);




	}
	catch (const Ice::Exception &e){ std::cout << e.what() << std::endl; }

	std::tuple<SpecificWorker::State, float, float> result= StateMachine(filtered_points);
	// Ejemplo de uso en un switch
	// Aplicar las velocidades calculadas al robot
	SetMachineSpeed(result);
}

void SpecificWorker::SetMachineSpeed(std::tuple<SpecificWorker::State, float, float> result){
	state = std::get<0>(result);
	float adv = std::get<1>(result);
	float rot = std::get<2>(result);
	try{ omnirobot_proxy->setSpeedBase(0, adv, rot);}
	catch (const Ice::Exception &e){ std::cout << e << " " << "Conexión con Laser" << std::endl; return;}
}
std::tuple<SpecificWorker::State, float, float> SpecificWorker::StateMachine(RoboCompLidar3D::TPoints filtered_points){
	std::tuple<SpecificWorker::State, float, float> result;

	switch(state)
	{
		case SpecificWorker::State::IDLE:
			// Aquí va la lógica para cuando el robot está parado
			state = SpecificWorker::State::FORWARD;
			break;

		case SpecificWorker::State::FORWARD:
			result = forward(filtered_points);
			break;


		case SpecificWorker::State::TURN:
			// Lógica de giro
			result = turn(filtered_points);
			break;

		case SpecificWorker::State::FOLLOW_WALL:
			// Lógica de seguimiento de pared
			result = follow_wall(filtered_points);
			break;

		case SpecificWorker::State::SPIRAL:
			// Lógica de espiral
			result = spiral(filtered_points);
			break;
	}
	return result;
}
//El robot avanza rápido cuando no hay obstáculos y frena o gira cuanto más cerca o frontal está el obstáculo.
std::tuple<SpecificWorker::State, float, float> SpecificWorker::forward(const RoboCompLidar3D::TPoints& points)
{
	std::tuple<SpecificWorker::State, float, float> result;

	RoboCompLidar3D::TPoints frontal_points;
	frontal_points.reserve(points.size());
	// for, para filtrar los puntos que estan al frente del robot
	for (const auto &p : points)
	{
		if (p.phi > (-M_PI_4 )&& p.phi < (M_PI_4))  // -45° < phi < +45°
		{
			frontal_points.push_back(p);
		}
	}
	auto min_dist = std::min_element(std::begin(frontal_points), std::end(frontal_points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });	// Punto más cercano
	if (min_dist->r>DIST_CHANGE_TO_SPIRAL) {
		return{State::SPIRAL,0.0f,0.0f};
	}
	// Estos tres calculos, se pueden quitar y poner valores fijos si se desea

	// Cálculo de velocidad de avance basada en la distancia al obstáculo más cercano
	float adv_speed = std::min(MAX_ADV, (min_dist->r - MIN_THRESHOLD) * (MAX_ADV / (MIN_DISTANCE_TURN - MIN_THRESHOLD)));
	// Cálculo del freno por distancia
	float brake_speed = std::min(MAX_BRAKE, (MIN_DISTANCE_TURN - min_dist->r) * (MAX_BRAKE / (MIN_DISTANCE_TURN - MIN_THRESHOLD)));
	float brake_rot = brake_speed / 2.0f; // Reducción de la velocidad de rotación al frenar

	// Decisión de estado basada en la distancia al obstáculo más cercano
	if (min_dist->r > MIN_DISTANCE_TURN) {
		result = std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::FORWARD, adv_speed, 0.f);
	} else {
		qInfo() << "FORWARD -> TURN";
		result = std::tuple<SpecificWorker::State, float, float>(SpecificWorker::State::TURN, 0.f, brake_rot);
	}
	return result;
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::turn(const RoboCompLidar3D::TPoints& points)
{
	static int cont=0;
	std::tuple<SpecificWorker::State, float, float> result;

	RoboCompLidar3D::TPoints frontal_points;
	frontal_points.reserve(points.size());
	// for, para filtrar los puntos que estan al frente del robot
	for (const auto &p : points)
	{
		if (p.phi > (-M_PI_4 )&& p.phi < (M_PI_4))  // -45° < phi < +45°
		{
			frontal_points.push_back(p);
		}
	}
	auto min_dist = std::min_element(std::begin(frontal_points), std::end(frontal_points),[](const auto& p1, const auto& p2)
			{ return p1.r < p2.r; });	// Punto más cercano

	// Si ya no hay obstáculo cerca, volvemos a FORWARD
	if (min_dist->r > MIN_DISTANCE_TURN+100)
	{
		cont=0;
		// Aleatoriamente decido si seguir pared o avanzar
		if (std::rand() % 2 == 0)
		{
			qInfo() << "TURN -> FOLLOW WALL";
			result = {State::FOLLOW_WALL, MAX_ADV, 0.0f};  // Podemos avanzar
		}
		else
		{
			qInfo() << "TURN -> FORWARD";
			result = {State::FORWARD, MAX_ADV, 0.0f};  // Podemos avanzar
		}

	}
	else {

		qInfo() << "TURN AGAIN";
		cont++;
		if (cont>=100)
			result=  {State::TURN, 0.0f, 1.0f};
		else
			if (min_dist->phi < 0)
				result =  {State::TURN, 0.0f, 1.0f};
			else
				result = {State::TURN, 0.0f, -1.0f};

	}
	return result;
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::follow_wall(const RoboCompLidar3D::TPoints &points)
{

	auto min_dist = std::min_element(std::begin(points), std::end(points),[](const auto& p1, const auto& p2)
		{ return p1.r < p2.r; });
	qInfo() << "DISTANCIA PARED------------------"<<min_dist->r;
	// Si ya no hay obstáculo cerca, volvemos a FORWARD
	if (min_dist->r >= MIN_DISTANCE_TURN && min_dist->r < MIN_DISTANCE_TURN+150)
	{
		qInfo() << "FOLLOW WALL";
		return {State::FOLLOW_WALL, 200.0f, 0.0f};  // Podemos avanzar
	}

	//Estamos muy lejos de la pared en este caso 750
	if (min_dist->r > MIN_DISTANCE_TURN+150)//Si estoy lejos de la pared corrijo el movimiento
	{
		//Detectamos en que lado esta la pared que seguimos y reducimos la velocidad para rotar.
		if (min_dist->phi < 0)
		{
			qInfo() << "FOLLOW WALL LEFT FAR";
			return {State::FOLLOW_WALL, 150.0f, -0.1f};
		}

			qInfo() << "FOLLOW WALL RIGHT FAR";

			return {State::FOLLOW_WALL, 150.0f, 0.1f};

	}
	if (MIN_DISTANCE_TURN-100 < min_dist->r && min_dist->r < MIN_DISTANCE_TURN+10)//Esto es si estoy cerca de la pared y corrijo el movimiento
	{
		//Detectamos en que lado esta la pared que seguimos y reducimos la velocidad para rotar.
		if (min_dist->phi < 0)
		{
			qInfo() << "FOLLOW WALL LEFT NEAR";
			return {State::FOLLOW_WALL, 150.0f, 0.1f};
		}

		qInfo() << "FOLLOW WALL RIGHT NEAR";

		return {State::FOLLOW_WALL, 150.0f, -0.1f};

	}

	qInfo() << "FOLLOW WALL -> FORWARD";
	return {State::FORWARD, 600.0f, 0.0f};
}

std::tuple<SpecificWorker::State, float, float> SpecificWorker::spiral(const RoboCompLidar3D::TPoints &points)
{

	static float adv_speed_spiral = 500.0f;   // velocidad lineal inicial (mm/s)
	static float rot_speed_spiral = 3.0f;     // velocidad angular inicial (rad/s)
	const float MAX_ADV_SPIRAL = 1000.0f;        // velocidad máxima de avance
	const float MIN_ROT = 0.0f;        // rotación mínima

	// 1️⃣ Comprobamos si hay obstáculos cerca
	auto min_dist = std::min_element(points.begin(), points.end(),
									 [](const auto &a, const auto &b){ return a.r < b.r; });

	if (min_dist != points.end() && min_dist->r < MIN_DISTANCE_TURN)
	{
		qInfo() << "SPIRAL -> OBSTÁCULO DETECTADO. Cambiando a TURN.";
		adv_speed_spiral = 100.0f;   // reseteamos valores
		rot_speed_spiral = 4.0f;
		return {State::TURN, 0.0f, 0.0f};
	}

	// 2️⃣ Aumentamos velocidad de avance y reducimos rotación (efecto espiral)
	adv_speed_spiral = std::min(adv_speed_spiral + 2.5f, MAX_ADV_SPIRAL);
	rot_speed_spiral = std::max(rot_speed_spiral - 0.01f, MIN_ROT);

	qInfo() << "SPIRAL: adv" << adv_speed_spiral << "rot" << rot_speed_spiral;
	return {State::SPIRAL, adv_speed_spiral, rot_speed_spiral};

}

// NO SE USA ACTUALMENTE
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