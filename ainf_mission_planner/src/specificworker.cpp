/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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
#include <algorithm>
#include <limits>
#include <cmath>
#include <QtMath>
#include <QSettings>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QRegularExpression>
#include <thread>

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
	QSettings settings("robocomp", "ainf_mission_planner");
	settings.setValue("window/geometry", this->saveGeometry());
	std::cout << "Destroying SpecificWorker" << std::endl;
}


void SpecificWorker::initialize()
{
    std::cout << "initialize worker" << std::endl;

	viewer = new AbstractGraphicViewer(this->frame, QRectF(-8, -8, 16, 16), true);
	viewer->add_robot(0.46f, 0.48f, 0.0f, 0.2f, QColor("Blue"));
	viewer->show();
	connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this, &SpecificWorker::slot_new_target);

	// Signals para actualizar chat en el hilo principal
	connect(this, &SpecificWorker::chatMessageReceived, this, [this](const QString &msg) {
		textEdit_chat->append(msg);
	});
	connect(this, &SpecificWorker::chatTokenReceived, this, [this](const QString &token) {
		textEdit_chat->insertPlainText(token);
		textEdit_chat->ensureCursorVisible();
	});

	// Conectar botón enviar chat
	connect(pushButton_sendChat, &QPushButton::clicked, this, [this]() {
		QString text = lineEdit_prompt->text();
		if(text.trimmed().isEmpty()) return;
		lineEdit_prompt->clear();
		emit chatMessageReceived("<b>Yo:</b> " + text);
		emit chatMessageReceived("<b>Ollama:</b> ");
		try {
			obtenerDestinoIA("Misión: " + text.toStdString());
		} catch(const Ice::Exception &e) {
			qWarning() << "Error starting AI evaluation:" << e.what();
		}
	});
	connect(lineEdit_prompt, &QLineEdit::returnPressed, pushButton_sendChat, &QPushButton::click);

	connect(pushButton_startFollow, &QPushButton::clicked, this, [this]()
	{
		std::cout << "===========================================" << std::endl;
		std::cout << "BOTON START FOLLOW PULSADO" << std::endl;
		try
		{
			std::cout << "Iniciando peticion a IA..." << std::endl;
			obtenerDestinoIA();
		}
		catch(const Ice::Exception &e)
		{
			std::cerr << "Error starting AI evaluation:" << e.what() << std::endl;
		}
	});

	connect(pushButton_stopFollow, &QPushButton::clicked, this, [this]()
	{
		try
		{
			navigator_proxy->stop();
			qInfo() << "Path following stopped";
		}
		catch(const Ice::Exception &e)
		{
			qWarning() << "Error stopping path following:" << e.what();
		}
	});

	connect(pushButton_resumeFollow, &QPushButton::clicked, this, [this]()
	{
		try
		{
			navigator_proxy->resume();
			qInfo() << "Path following resumed";
		}
		catch(const Ice::Exception &e)
		{
			qWarning() << "Error resuming path following:" << e.what();
		}
	});

	QSettings settings("robocomp", "ainf_mission_planner");
	if(settings.contains("window/geometry"))
		this->restoreGeometry(settings.value("window/geometry").toByteArray());

	try
	{
		const auto map_data = navigator_proxy->getLayout();

		auto draw_polyline = [this](const RoboCompNavigator::TLayout &polyline, const QPen &pen)
		{
			if(polyline.size() < 2)
				return;

			for(size_t i = 0; i < polyline.size() - 1; ++i)
			{
				const auto &p1 = polyline[i];
				const auto &p2 = polyline[i + 1];
				viewer->scene.addLine(QLineF(p1.x, p1.y, p2.x, p2.y), pen);
			}
			const auto &first = polyline.front();
			const auto &last = polyline.back();
			viewer->scene.addLine(QLineF(last.x, last.y, first.x, first.y), pen);
		};

		draw_polyline(map_data.layout, QPen(QColor("magenta"), 0.16));

		for(size_t i = 0; i < map_data.objects.size(); ++i)
		{
			const auto &obj = map_data.objects[i];
			const QColor object_color = QColor::fromHsv(static_cast<int>((i * 47) % 360), 210, 230);

			if(obj.layout.size() >= 3)
			{
				QPolygonF polygon;
				polygon.reserve(static_cast<int>(obj.layout.size()));
				for(const auto &point : obj.layout)
					polygon << QPointF(point.x, point.y);

				viewer->scene.addPolygon(polygon,
				                         QPen(object_color, 0.07),
				                         QBrush(object_color, Qt::SolidPattern));
			}
			else
			{
				draw_polyline(obj.layout, QPen(object_color, 0.07));
			}

			if(not obj.layout.empty())
			{
				const auto &p = obj.layout.front();
				auto *text = viewer->scene.addText(QString::fromStdString(obj.name));
				QFont font = text->font();
				font.setBold(true);
				text->setFont(font);
				text->setDefaultTextColor(QColor("black"));
				text->setTransform(QTransform::fromScale(0.012, -0.012));
				text->setPos(p.x, p.y);
				text->setZValue(120);
			}
		}

		if(not map_data.layout.empty())
		{
			float min_x = std::numeric_limits<float>::max();
			float max_x = std::numeric_limits<float>::lowest();
			float min_y = std::numeric_limits<float>::max();
			float max_y = std::numeric_limits<float>::lowest();

			for(const auto &p : map_data.layout)
			{
				min_x = std::min(min_x, p.x);
				max_x = std::max(max_x, p.x);
				min_y = std::min(min_y, p.y);
				max_y = std::max(max_y, p.y);
			}

			auto *legend = viewer->scene.addText("Room: magenta (thick) | Objects: unique filled colors");
			legend->setDefaultTextColor(QColor("darkBlue"));
			legend->setTransform(QTransform::fromScale(0.02, -0.02));
			legend->setPos(min_x, max_y + 0.15f);

			viewer->fitToScene(QRectF(min_x, min_y, max_x - min_x, max_y - min_y));
		}
	}
	catch(const Ice::Exception &e)
	{
		qWarning() << "Error requesting map from Navigator:" << e.what();
	}

    /////////GET PARAMS, OPEND DEVICES....////////
    //int period = configLoader.get<int>("Period.Compute") //NOTE: If you want get period of compute use getPeriod("compute")
    //std::string device = configLoader.get<std::string>("Device.name") 

}



void SpecificWorker::compute()
{

	if(viewer == nullptr or viewer->robot_poly() == nullptr)
		return;

	try
	{
		const auto pose = navigator_proxy->getRobotPose();
		viewer->robot_poly()->setPos(pose.x, pose.y);
		viewer->robot_poly()->setRotation(qRadiansToDegrees(pose.r));
		if(not planned_path_points.empty())
			redraw_planned_path(RoboCompNavigator::TPoint{pose.x, pose.y});
		label_robotCoordsValue->setText(QString("x=%1  y=%2  θ=%3")
		                               .arg(pose.x, 0, 'f', 2)
		                               .arg(pose.y, 0, 'f', 2)
		                               .arg(pose.r, 0, 'f', 2));

	}
	catch(const Ice::Exception &e) { qWarning() << "Error requesting robot pose from Navigator:" << e.what(); }	
	try
	{
		auto status = navigator_proxy->getStatus();
		QString state_text = "UNKNOWN";
		switch(status.state)
		{
			case RoboCompNavigator::NavigationState::IDLE: 
			case RoboCompNavigator::NavigationState::REACHED: 
				state_text = (status.state == RoboCompNavigator::NavigationState::REACHED) ? "REACHED" : "IDLE"; 
				if(current_mission_state.load() == MissionState::NAVIGATING_TO_OBJECT && navigation_started)
				{
					current_mission_state = MissionState::CENTERING_OBJECT;
					navigation_started = false;
					std::cout << "\n===========================================\n" << std::endl;
					std::cout << "[DEBUG] Destino alcanzado. Iniciando escaneo visual IA directamente..." << std::endl;
				}

				// FASE 2: Captura de imagen y verificación visual con IA
				if (current_mission_state.load() == MissionState::CENTERING_OBJECT)
				{
					current_mission_state = MissionState::WAITING_FOR_VISION;
					std::cout << "[DEBUG] Tomando fotografía para validación de centrado visual..." << std::endl;
					try {
						auto img = camerargbdsimple_proxy->getImage("");
						if(!img.image.empty()) {
							QImage qimage(reinterpret_cast<const uchar*>(img.image.data()), img.width, img.height, QImage::Format_RGB888);
							label_camera->setPixmap(QPixmap::fromImage(qimage).scaled(label_camera->size(), Qt::KeepAspectRatio));
							std::cout << "[DEBUG] Fotografía mostrada en UI." << std::endl;
							
							QByteArray byteArray;
							QBuffer buffer(&byteArray);
							buffer.open(QIODevice::WriteOnly);
							qimage.save(&buffer, "JPEG");
							std::string base64_img = byteArray.toBase64().toStdString();
							
							std::thread([this, base64_img]() {
								std::cout << "[DEBUG] Enviando imagen a Ollama (llava:7b) para verificar centrado..." << std::endl;
								ollama::request req;
								req["model"] = "llava:7b";
								req["system"] = "Eres el controlador visual de un robot. Analiza la imagen de la cámara del robot. "
									"El robot ya está orientado aproximadamente hacia el objeto de interés (" + target_object_name + "). "
									"Tu tarea es verificar si dicho objeto está centrado en la imagen. "
									"Si el objeto está centrado (ocupa la zona central de la imagen), responde ÚNICAMENTE: {\"comando\": \"centrado\"}. "
									"Si necesita un pequeño ajuste, responde ÚNICAMENTE con: {\"comando\": \"rotar\", \"giro_rads\": X} "
									"donde X es un valor PEQUEÑO entre -0.3 y 0.3 radianes (positivo=izquierda, negativo=derecha). "
									"NUNCA des explicaciones. SOLO el JSON.";
								req["prompt"] = "¿Está el objeto centrado en esta imagen? Responde solo con JSON.";
								req["stream"] = false;
								req["images"] = std::vector<std::string>{base64_img};
								try {
									ollama::response res = ollama::generate(req);
									std::string resp = res.as_simple_string();
									std::cout << "\n[OLLAMA VISION]: " << resp << std::endl;
									
									if(resp.find("rotar") != std::string::npos) {
										float angulo = 0.1f;
										QRegularExpression regex(R"("giro_rads"\s*:\s*([\-\d\.]+))");
										QRegularExpressionMatch match = regex.match(QString::fromStdString(resp));
										if (match.hasMatch()) angulo = match.captured(1).toFloat();
										angulo = std::clamp(angulo, -0.3f, 0.3f); // Limitar micro-ajustes

										std::cout << "[DEBUG] Micro-ajuste visual de " << angulo << " rad/s durante 0.4s" << std::endl;
										omnirobot_proxy->setSpeedBase(0.0f, 0.0f, angulo);
										std::this_thread::sleep_for(std::chrono::milliseconds(400));
										omnirobot_proxy->setSpeedBase(0.0f, 0.0f, 0.0f);
										
										std::cout << "[DEBUG] Ajuste completado. Volviendo a comprobar...\n" << std::endl;
										current_mission_state = MissionState::CENTERING_OBJECT;
									} else {
										current_mission_state = MissionState::FINISHED;
										std::cout << "\n[✓ MISSION COMPLETE] El objeto está perfectamente centrado en cámara." << std::endl;
										emit chatMessageReceived("<br><font color='green'><b>MISIÓN SUPERADA: Objeto centrado.</b></font>");
										QImage qimageGuardar = QImage::fromData(QByteArray::fromBase64(QString::fromStdString(base64_img).toUtf8()));
										qimageGuardar.save("captura_objeto_centrado.jpg");
										std::cout << "[DEBUG] Imagen guardada en captura_objeto_centrado.jpg" << std::endl;
									}
								} catch (const std::exception& e) {
									std::cerr << "[ERROR VISION] " << e.what() << "\nAsumiendo finalizado por fallo IA." << std::endl;
									current_mission_state = MissionState::FINISHED;
								}
							}).detach();
						} else {
							std::cerr << "[DEBUG] Error: La cámara devolvió una imagen vacía." << std::endl;
						}
					} catch(const Ice::Exception &e) {
						std::cerr << "[DEBUG] Excepción Ice al leer cámara: " << e.what() << std::endl;
					}
				}
				break;
			case RoboCompNavigator::NavigationState::NAVIGATING: 
				state_text = "NAVIGATING"; 
				if(current_mission_state.load() == MissionState::NAVIGATING_TO_OBJECT) navigation_started = true;
				break;
			case RoboCompNavigator::NavigationState::PAUSED: state_text = "PAUSED"; break;
			case RoboCompNavigator::NavigationState::BLOCKED: state_text = "BLOCKED"; break;
			case RoboCompNavigator::NavigationState::ERROR: state_text = "ERROR"; break;
		}
		if(not status.statusMessage.empty())
		{
			state_text += QString(" (%1)").arg(QString::fromStdString(status.statusMessage));
		}
		label_missionStatusValue->setText(state_text);

		label_distanceToTargetValue->setText(QString::number(status.distanceToTarget, 'f', 2));
		label_etaValue->setText(QString::number(status.estimatedTime, 'f', 2));
		label_currentSpeedValue->setText(QString::number(status.currentSpeed, 'f', 2));
	}
	catch(const Ice::Exception &e)
	{
		qWarning() << "Error requesting mission status from Navigator:" << e.what();
		if(label_missionStatusValue->text().isEmpty())
			label_missionStatusValue->setText("IDLE");
		label_distanceToTargetValue->setText("--");
		label_etaValue->setText("--");
		label_currentSpeedValue->setText("--");
	}
}

////////////////////////////////////////////////////////////////////////////////


void SpecificWorker::slot_new_target(QPointF target)
{
	//obtenerDestinoIA();
	if(viewer == nullptr)
		return;

	try
	{
		const auto pose = navigator_proxy->getRobotPose();
		RoboCompNavigator::TPoint source{pose.x, pose.y};
		RoboCompNavigator::TPoint destination{static_cast<float>(target.x()), static_cast<float>(target.y())};
		last_target = destination;
		has_target = true;

		constexpr float safety = 0.25f;
		const auto result = navigator_proxy->getPath(source, destination, safety);

		if(not result.valid)
		{
			qWarning() << "Path request failed:" << QString::fromStdString(result.errorMsg);
			return;
		}

		if(result.path.size() < 2)
		{
			qWarning() << "Path request returned an empty or single-point path";
			planned_path_points.clear();
			return;
		}

		planned_path_points = result.path;
		redraw_planned_path(source);

		qInfo() << "Path planned with" << result.path.size() << "points";
	}
	catch(const Ice::Exception &e)
	{
		qWarning() << "Error requesting path to clicked target:" << e.what();
	}
}

void SpecificWorker::redraw_planned_path(const RoboCompNavigator::TPoint &current_source)
{
	if(viewer == nullptr)
		return;

	for(auto *item : planned_path_items)
	{
		viewer->scene.removeItem(item);
		delete item;
	}
	planned_path_items.clear();

	if(planned_path_points.size() < 2)
		return;

	QPen path_pen(QColor("orange"), 0.09);
	RoboCompNavigator::TPoint p1 = current_source;
	const auto &p2_first = planned_path_points[1];
	auto *first_segment = viewer->scene.addLine(QLineF(p1.x, p1.y, p2_first.x, p2_first.y), path_pen);
	first_segment->setZValue(200);
	planned_path_items.push_back(first_segment);

	for(size_t i = 1; i < planned_path_points.size() - 1; ++i)
	{
		const auto &p1_mid = planned_path_points[i];
		const auto &p2_mid = planned_path_points[i + 1];
		auto *segment = viewer->scene.addLine(QLineF(p1_mid.x, p1_mid.y, p2_mid.x, p2_mid.y), path_pen);
		segment->setZValue(200);
		planned_path_items.push_back(segment);
	}

	auto *start_marker = viewer->scene.addEllipse(current_source.x - 0.08, current_source.y - 0.08, 0.16, 0.16,
	                                             QPen(QColor("darkOrange"), 0.03),
	                                             QBrush(QColor("darkOrange"), Qt::SolidPattern));
	start_marker->setZValue(210);
	planned_path_items.push_back(start_marker);

	const auto &goal = planned_path_points.back();
	auto *goal_marker = viewer->scene.addEllipse(goal.x - 0.09, goal.y - 0.09, 0.18, 0.18,
	                                            QPen(QColor("red"), 0.03),
	                                            QBrush(QColor("red"), Qt::SolidPattern));
	goal_marker->setZValue(210);
	planned_path_items.push_back(goal_marker);
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

void SpecificWorker::obtenerDestinoIA(const std::string& mision)
{
    std::thread([this, mision]() 
    {
        try 
        {
            // 1. Obtener datos del mundo y del robot
            const auto pose = navigator_proxy->getRobotPose();
            const auto map_data = navigator_proxy->getLayout();

            // 2. Crear el objeto JSON principal
            QJsonObject estadoMundo;

            // Añadir el robot al JSON
            QJsonObject jsonRobot;
            jsonRobot["x"] = pose.x;
            jsonRobot["y"] = pose.y;
            jsonRobot["theta"] = pose.r;
            estadoMundo["robot"] = jsonRobot;

            // Añadir los objetos al JSON
            QJsonArray jsonObjetos;
            for(const auto &obj : map_data.objects)
            {
                QJsonObject jsonObj;
                jsonObj["id"] = QString::fromStdString(obj.name);
                
                // Calculamos el centro aproximado del objeto usando sus puntos (layout)
                if(!obj.layout.empty()) {
                    float sum_x = 0, sum_y = 0;
                    for(const auto &p : obj.layout) {
                        sum_x += p.x;
                        sum_y += p.y;
                    }
                    jsonObj["x"] = sum_x / obj.layout.size();
                    jsonObj["y"] = sum_y / obj.layout.size();
                }
                jsonObjetos.append(jsonObj);
            }
            estadoMundo["objetos"] = jsonObjetos;

            // Convertir el JSON a un std::string
            QJsonDocument doc(estadoMundo);
            std::string contextoJSON = doc.toJson(QJsonDocument::Compact).toStdString();

            // --- AQUÍ EMPIEZA LA CONEXIÓN CON OLLAMA ---
            llamarOllama(contextoJSON, mision); // Separamos la lógica para que quede limpio
            
        }
        catch(const Ice::Exception &e) {
            qCritical() << "Error obteniendo datos del Ice:" << e.what();
        }
    }).detach();
}

void SpecificWorker::llamarOllama(const std::string& contextoJSON, const std::string& mision)
{
    // Las reglas estrictas para el modelo (System Prompt)
    std::string sistema = 
        "Eres el cerebro de navegación de un robot. "
        "Recibirás el estado del mundo en formato JSON (posición del robot y lista de objetos). "
        "Tu misión es decidir a qué objeto debe ir el robot basándote en la orden del usuario. "
        "REGLAS CRÍTICAS:\n"
        "1. Tu respuesta DEBE ser ÚNICAMENTE un objeto JSON válido con esta estructura exacta: "
        "{\"comando\": \"goto\", \"target_id\": \"nombre_del_objeto\"}.\n"
        "2. El valor de \"comando\" SIEMPRE debe ser \"goto\".\n"
        "3. El valor de \"target_id\" DEBE ser EXACTAMENTE el mismo atributo \"id\" del objeto "
        "en el JSON recibido. NUNCA modifiques el nombre (NO reemplaces espacios por guiones bajos, etc).\n"
        "4. No incluyas saludos, explicaciones, ni etiquetas markdown. Devuelve SOLO el JSON.\n"
        "EJEMPLO DE RESPUESTA PERFECTA: {\"comando\": \"goto\", \"target_id\": \"silla 1 beta\"}";

    // La misión específica + el contexto inyectado
    std::string promptCompleto = "Estado del mundo:\n" + contextoJSON + "\n\n" + mision;

    std::cout << "\n===========================================\n";
    std::cout << "ENVIANDO PROMPT A OLLAMA:\n";
    std::cout << promptCompleto << "\n";
    std::cout << "===========================================\n" << std::endl;

    ollama::request req;
    req["model"] = "qwen2.5:0.5b"; // Usa tu modelo local (ej. "llama3" o "mistral")
    req["system"] = sistema;
    req["prompt"] = promptCompleto;
    req["stream"] = true;
    // req["format"] = "json"; // Descomenta esta línea si tu librería y modelo soportan el modo JSON nativo de Ollama

try 
    {
        std::cout << "Ollama pensando y respondiendo en tiempo real..." << std::endl;
        
        std::string respuestaTexto = "";
        std::cout << "\nRespuesta cruda de IA:\n";
        
        // 1. Recibimos la respuesta de la IA (Streaming)
        ollama::generate(req, [this, &respuestaTexto](const ollama::response& r) {
            try {
                if (r.has_error()) {
                    std::cerr << "\n[ERROR DE OLLAMA]: " << r.get_error() << std::endl;
                    emit chatMessageReceived("<br><font color='red'>Error de IA: " + QString::fromStdString(r.get_error()) + "</font><br>");
                    return false; // Stop streaming
                }
                std::string token = r.as_simple_string();
                std::cout << token << std::flush;
                emit chatTokenReceived(QString::fromStdString(token));
                respuestaTexto += token;
            } catch (const std::exception& ex) {
                std::cerr << "\n[Error in stream chunk]: " << ex.what() << std::endl;
            }
            return true; // Devolver true indica que queremos seguir recibiendo tokens
        });
        
        std::cout << "\n\n" << std::endl;

        // 2. Limpiamos la respuesta de etiquetas Markdown (```json ... ```) buscando las llaves
        size_t primerLlave = respuestaTexto.find('{');
        size_t ultimaLlave = respuestaTexto.rfind('}');
        
        if (primerLlave != std::string::npos && ultimaLlave != std::string::npos && ultimaLlave > primerLlave) {
            respuestaTexto = respuestaTexto.substr(primerLlave, ultimaLlave - primerLlave + 1);
        }

        // 3. Convertimos el texto a un objeto JSON de Qt
        QByteArray datosJson = QString::fromStdString(respuestaTexto).toUtf8();
        QJsonDocument docRespuesta = QJsonDocument::fromJson(datosJson);

        // 3. Comprobamos que la IA nos haya hecho caso y sea un JSON válido
        if(!docRespuesta.isNull() && docRespuesta.isObject()) 
        {
            QJsonObject objRespuesta = docRespuesta.object();
            
            // 4. Extraemos el comando y el destino
            QString comandoStr = objRespuesta["comando"].toString().toLower();
            if(objRespuesta.contains("target_id") && 
               (comandoStr.startsWith("go") || comandoStr.startsWith("mov") || comandoStr.contains("ir"))) 
            {
                QString target = objRespuesta["target_id"].toString().replace("_", " ").trimmed();
                std::cout << "¡Orden aceptada! Mandando el robot a:" << target.toStdString() << std::endl;
                
                // 5. ¡Ejecutamos el movimiento real del robot!
                auto obj_pos = navigator_proxy->gotoObject(target.toStdString());
                target_object_pos = obj_pos;
                target_object_name = target.toStdString();
                current_mission_state = MissionState::NAVIGATING_TO_OBJECT;
            }
            else 
            {
                qWarning() << "El JSON es válido, pero no tiene el formato esperado.";
            }
        } 
        else 
        {
            qWarning() << "Aviso: La IA no devolvió un JSON válido por formato, activando extractor de emergencia (RegEx).";
            QRegularExpression regex(R"("target_id"\s*:\s*"([^"]+))");
            QRegularExpressionMatch match = regex.match(QString::fromStdString(respuestaTexto));
            
            if (match.hasMatch()) {
                QString target = match.captured(1).replace("_", " ").trimmed();
                std::cout << "¡Orden recuperada por emergencia! Mandando el robot a:" << target.toStdString() << std::endl;
                auto obj_pos = navigator_proxy->gotoObject(target.toStdString());
                target_object_pos = obj_pos;
                current_mission_state = MissionState::NAVIGATING_TO_OBJECT;
            } else {
                qWarning() << "Error total: No se pudo extraer un JSON ni un parámetro target de: " << QString::fromStdString(respuestaTexto);
            }
        }
    }
    catch (const std::exception& e) {
        qCritical() << "Error en Ollama:" << e.what();
    }
}

/**************************************/
// From the RoboCompNavigator you can call this methods:
// RoboCompNavigator::LayoutData this->navigator_proxy->getLayout()
// RoboCompNavigator::Result this->navigator_proxy->getPath(TPoint source, TPoint target, float safety)
// RoboCompNavigator::TPoint this->navigator_proxy->gotoObject(string object)
// RoboCompNavigator::TPoint this->navigator_proxy->gotoPoint(TPoint target)
// RoboCompNavigator::void this->navigator_proxy->resume()
// RoboCompNavigator::void this->navigator_proxy->stop()

/**************************************/
// From the RoboCompNavigator you can use this types:
// RoboCompNavigator::TPoint
// RoboCompNavigator::Result
// RoboCompNavigator::Pose
// RoboCompNavigator::NavigationStatus
// RoboCompNavigator::TObject
// RoboCompNavigator::LayoutData

