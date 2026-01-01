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
        contains the worker constructor/destructor, lifecycle hooks
   (`initialize`, `compute`) and the behaviour state machine implementations
   used to control the robot based on detected geometry and matches. */
#include "specificworker.h"
#include "common_types.h"
#include <QRect>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/groupby.hpp>
#include <cppitertools/zip.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <qcolor.h>

using namespace std;

const float MIN_DISTANCE_TURN =
    600.0f;                   // Distancia mínima para salir del estado TURN
const float MAX_ADV = 600.0f; // Velocidad máxima de avance en mm/s
const float MIN_THRESHOLD =
    50.0f;                      // Distancia mínima para empezar a frenar en mm
const float MAX_BRAKE = 100.0f; // Velocidad máxima de frenado en mm/s
const float DIST_CHANGE_TO_SPIRAL =
    200000.0f; // Distancia a la que no consideramos que no hay nada cerca para
               // la espiral
// Localisation match error threshold (tunable)
constexpr float LOCALISATION_MATCH_ERROR_THRESHOLD = 3500.0f; // mm

SpecificWorker *SpecificWorker::instance = nullptr;

void stateMessageHandler(QtMsgType type, const QMessageLogContext &context,
                         const QString &msg) {
  // Print to console/stderr as usual
  QByteArray localMsg = msg.toLocal8Bit();
  switch (type) {
  case QtDebugMsg:
    fprintf(stderr, "Debug: %s\n", localMsg.constData());
    break;
  case QtInfoMsg:
    fprintf(stderr, "Info: %s\n", localMsg.constData());
    break;
  case QtWarningMsg:
    fprintf(stderr, "Warning: %s\n", localMsg.constData());
    break;
  case QtCriticalMsg:
    fprintf(stderr, "Critical: %s\n", localMsg.constData());
    break;
  case QtFatalMsg:
    fprintf(stderr, "Fatal: %s\n", localMsg.constData());
    break;
  }

  // Log to file based on current state
  if (SpecificWorker::instance) {
    auto it = SpecificWorker::instance->log_files.find(
        SpecificWorker::instance->state);
    if (it != SpecificWorker::instance->log_files.end() && it->second) {
      *(it->second) << msg.toStdString() << std::endl;
    }
  }
}

SpecificWorker::SpecificWorker(const ConfigLoader &configLoader, TuplePrx tprx,
                               bool startup_check)
    : GenericWorker(configLoader, tprx) {
  SpecificWorker::instance = this;
  this->startup_check_flag = startup_check;
  if (this->startup_check_flag) {
    this->startup_check();
  } else {
#ifdef HIBERNATION_ENABLED
    hibernationChecker.start(500);
#endif

    // Example statemachine:
    /***
    //Your definition for the statesmachine (if you dont want use a execute
    function, use nullptr) states["CustomState"] =
    std::make_unique<GRAFCETStep>("CustomState", period,
                                                                                                            std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
                                                                                                            std::bind(&SpecificWorker::customEnter, this), // On-enter function
                                                                                                            std::bind(&SpecificWorker::customExit, this)); // On-exit function

    //Add your definition of transitions (addTransition(originOfSignal, signal,
    dstState)) states["CustomState"]->addTransition(states["CustomState"].get(),
    SIGNAL(entered()), states["OtherState"].get());
    states["Compute"]->addTransition(this, SIGNAL(customSignal()),
    states["CustomState"].get()); //Define your signal in the .h file under the
    "Signals" section.

    //Add your custom state
    statemachine.addState(states["CustomState"].get());
    ***/

    statemachine.setChildMode(QState::ExclusiveStates);
    statemachine.start();

    auto error = statemachine.errorString();
    if (error.length() > 0) {
      qWarning() << error;
      throw error;
    }
  }
}

SpecificWorker::~SpecificWorker() {
  omnirobot_proxy->setSpeedBase(0, 0, 0);
  std::cout << "Destroying SpecificWorker" << std::endl;
}

/**
 * @brief Initialize the SpecificWorker GUI and internal state.
 *
 * This method sets up both the main viewer (left pane) used for live
 * visualization of the robot and LIDAR points, and the dedicated "room"
 * viewer (right pane) used to display the nominal room and a scaled view.
 */
void SpecificWorker::initialize() {
  if (this->startup_check_flag) {
    this->startup_check();
  } else {
    std::cout << "initialize worker"
              << std::endl; // announce initialisation on stdout

    // Create logs directory if it doesn't exist
    QDir().mkdir("logs");

    // Initialize log files for each state
    log_files[STATE::IDLE] =
        std::make_shared<std::ofstream>("logs/IDLE.log", std::ios::trunc);
    log_files[STATE::LOCALISE] =
        std::make_shared<std::ofstream>("logs/LOCALISE.log", std::ios::trunc);
    log_files[STATE::GOTO_DOOR] =
        std::make_shared<std::ofstream>("logs/GOTO_DOOR.log", std::ios::trunc);
    log_files[STATE::TURN] =
        std::make_shared<std::ofstream>("logs/TURN.log", std::ios::trunc);
    log_files[STATE::ORIENT_TO_DOOR] = std::make_shared<std::ofstream>(
        "logs/ORIENT_TO_DOOR.log", std::ios::trunc);
    log_files[STATE::GOTO_ROOM_CENTER] = std::make_shared<std::ofstream>(
        "logs/GOTO_ROOM_CENTER.log", std::ios::trunc);
    log_files[STATE::CROSS_DOOR] =
        std::make_shared<std::ofstream>("logs/CROSS_DOOR.log", std::ios::trunc);


    qInstallMessageHandler(stateMessageHandler);

    // --- Prepare GUI geometry
    // -------------------------------------------------
    /*
     * "viewer" (left pane) shows the robot at the center with a +/-6m x, +/-3m
     * y scene extent. It will display live LIDAR points and robot movement.
     *
     * "viewer_room" (right pane) shows a fixed-size room with grid and
     * nominal room outline. The robot will move inside this room according to
     * its pose.
     */
    // Setup the auto-generated GUI from Qt Designer
    setupUi(this);

    viewer = new AbstractGraphicViewer(this->frame, params.GRID_MAX_DIM);
    auto [r, e] = viewer->add_robot(params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0,
                                    100, QColor("Blue"));
    robot_draw = r;

    viewer_room =
        new AbstractGraphicViewer(this->frame_room, params.GRID_MAX_DIM);

    auto [rr, re] = viewer_room->add_robot(
        params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
    robot_room_draw = rr;
    // Note: Room walls are NOT drawn here - they appear only after badge recognition

    // Initialize Graph Topology Viewer
    viewer_graph = new AbstractGraphicViewer(this->frame_graph, QRectF(-2000, -2000, 4000, 4000), false);


    // initialise robot pose
    robot_pose.setIdentity();
    robot_pose.translate(Eigen::Vector2d(0.0, 0.0));

    this->show();

    // Connect mouse events from the viewer to a slot that handles new targets
    // (clicks)
    connect(viewer, &AbstractGraphicViewer::new_mouse_coordinates, this,
            &SpecificWorker::new_target_slot);
    connect(pushButton_stop, &QPushButton::clicked,
            []() { QCoreApplication::instance()->quit(); });
    srand(time(NULL)); // Viewer

    // Read runtime debug flag from environment variable if provided
    if (const char *dbg = std::getenv("DEBUG_RUNTIME")) {
      std::string s(dbg);
      for (auto &ch : s)
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
      if (s == "1" || s == "true" || s == "yes")
        debug_runtime = true;
    }

    // time series plotter for match error
    TimeSeriesPlotter::Config plotConfig;
    plotConfig.title = "Maximum Match Error Over Time";
    plotConfig.yAxisLabel = "Error (mm)";
    plotConfig.timeWindowSeconds = 15.0; // Show a 15-second window
    plotConfig.autoScaleY = false;       // We will set a fixed range
    plotConfig.yMin = 0;
    plotConfig.yMax = 5000;

    SpecificWorker::time_series_plotter =
        std::make_unique<TimeSeriesPlotter>(localizationPlot, plotConfig);
    match_error_graph = time_series_plotter->addGraph("", Qt::blue);

    // Initialize room recognition tracking
    room_recognized = std::vector<bool>(nominal_rooms.size(), false);
    
    // Initialize topology graph for room/door memory
    topology_graph = std::make_unique<Graph>("logs/GRAPH.log");

    // stop robot
    move_robot(0, 0, 0);
  }
}

void SpecificWorker::new_target_slot(QPointF target) {
  try {
    RoboCompGenericBase::TBaseState
        bState;                            // variable to receive odometry/state
    omnirobot_proxy->getBaseState(bState); // populate bState with x, z, alpha

    // Update graphic representing robot orientation (Qt rotation uses degrees
    // internally for items)
    robot_draw->setRotation(bState.alpha + M_PI_2);

    // Update graphic position using robot's x,z coordinates
    robot_draw->setPos(bState.x, bState.z);

    // Print state for debugging
    std::cout << bState.alpha << " " << bState.x << " " << bState.z
              << std::endl;
  } catch (const Ice::Exception &e) {
    // Print ICE exception in case of RPC failure
    std::cout << e.what() << std::endl;
    return;
  }
}

void SpecificWorker::draw_mainViewer() {
  data = lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);
  // qInfo() << "full" << data.points.size();

  auto filtered_data = filter_data_basic(data.points);

  filtered_data = door_detector.filter_points(filtered_data, &viewer->scene);

  // ========== CORNER DETECTION AND VISUALIZATION ==========
  // Detect corners from LIDAR points and visualize them in the main viewer
  // This call encapsulates the entire pipeline: RANSAC line extraction,
  // 90° intersection detection, and dynamic circle visualization

  // Cogemos las esquinas detectadas o mesuradas
  const auto &[detected_corners, lines] =
      room_detector.compute_corners(filtered_data, &viewer->scene);
  corners = detected_corners;

  // ========== ROBOT POSE ESTIMATION VIA CORNER MATCHING ==========
  // Estimate and update the robot's pose based on matched corners
  const auto center_opt = room_detector.estimate_center_from_walls(lines);

  // Draw LiDAR points at their correct positions (no offset)
  draw_lidar(filtered_data, std::nullopt, &viewer->scene);

  // Draw the room center as a separate visual indicator if it exists
  if (center_opt.has_value()) {
    target_room_center = center_opt.value(); // Store for controller
    draw_room_center(center_opt.value(), &viewer->scene);
  }

  draw_door_target(target_door_point, &viewer->scene);
}

void SpecificWorker::emergency() {}

// Execute one when exiting to emergencyState
void SpecificWorker::restore() {
  std::cout << "Restore worker" << std::endl;
  // restoreCODE
  // Restore emergency component
}

int SpecificWorker::startup_check() {
  std::cout << "Startup check" << std::endl;
  QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
  return 0;
}

void SpecificWorker::compute() {
  // QThread::msleep(500); // wait for viewer to be ready
  if (debug_runtime)
    qInfo()
        << "--------------------------- Compute ---------------------------";

  // Draw main viewer (LIDAR points, corners, room center)
  draw_mainViewer();

  // Draw axes at center (always visible)
  auto center = nominal_rooms[current_room_index].rect().center();
  viewer_room->scene.addLine(center.x(), center.y(), center.x() + 350,
                             center.y(), QPen(Qt::red, 20));
  viewer_room->scene.addLine(center.x(), center.y(), center.x(),
                             center.y() + 350, QPen(Qt::green, 20));

  // Draw room walls and doors only if room is recognized
  if (room_recognized[current_room_index]) {
    // Draw walls
    viewer_room->scene.addRect(nominal_rooms[current_room_index].rect(),
                               QPen(Qt::black, 30));
    // Draw doors
    draw_room_doors(nominal_rooms[current_room_index].doors, &viewer_room->scene);
  }

  // Draw the topological graph
  draw_topology_graph(&viewer_graph->scene);

  // Run localisation logic
  execute_localiser();

  // Execute state machine using last matching results and command robot
  auto ret_val =
      this->process_state(data.points, corners, last_matched, viewer);
  auto [st, adv, rot] = ret_val;
  state = st;

  // Send velocities (movement independent of localisation as requested)
  move_robot(adv, rot, last_match_error);

  // Get image from camera
  RoboCompCamera360RGB::TImage img;
  try {
    img = camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
  } catch (const Ice::Exception &e) {
    std::cout << e.what() << " Error reading 360 camera " << std::endl;
  }

  if (img.width > 0 && img.height > 0) {
    // convert to cv::Mat
    cv::Mat cv_img(img.height, img.width, CV_8UC3, img.image.data());

    // Convert BGR -> RGB for display
    cv::Mat display_img;
    cv::cvtColor(cv_img, display_img, cv::COLOR_BGR2RGB);

    // optionally update the label with the ROI preview even when not detected
    QImage qimg(display_img.data, display_img.cols, display_img.rows,
                static_cast<int>(display_img.step), QImage::Format_RGB888);
    label_img->setPixmap(QPixmap::fromImage(qimg).scaled(
        label_img->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }

  // Update UI
  RoboCompGenericBase::TBaseState bState;
  omnirobot_proxy->getBaseState(bState);
  lcdNumber_x->display(bState.x);
  lcdNumber_y->display(bState.z);
  lcdNumber_angle->display(qRadiansToDegrees(bState.alpha));
  lcdNumber_adv->display(adv);
  lcdNumber_rot->display(rot);
  lcdNumber_rot->display(rot);
  lcdNumber_room->display(current_room_index);
  label_state->setText(to_string(state));
}

void SpecificWorker::execute_localiser() {
  // Bootstrap: If not yet localized, try multi-hypothesis grid search
  if (!initial_localisation_done && !corners.empty()) {
    // If room is already recognized, use stored heading as constraint
    std::optional<float> ref_heading = std::nullopt;
    if (room_recognized[current_room_index] && topology_graph->is_room_in_graph(current_room_index)) {
      ref_heading = topology_graph->get_room_heading(current_room_index);
    }
    
    robot_pose = find_best_initial_pose(corners, ref_heading);
    initial_localisation_done = true;
    
    // Save entry position for later door matching (proximity-based)
    room_entry_position = robot_pose.translation().cast<float>();
    qInfo() << "[POS_SAVED] Saved entry position for room" << current_room_index
            << ": (" << room_entry_position.value().x() 
            << "," << room_entry_position.value().y() << ")";
    
    // If room already recognized, update entry door immediately
    if (room_recognized[current_room_index]) {
      if (previous_traversed_door_id != -1) {
          qInfo() << "[POS_SAVED] Room" << current_room_index << "recognized, calling set_entry_door_by_proximity to learn connection";
          set_entry_door_by_proximity();
      } else {
          qInfo() << "[POS_SAVED] Room" << current_room_index << "recognized, entry door already set by learned connection (skipping proximity)";
          room_entry_position.reset(); // Clear unused position
      }
    } else {
      qInfo() << "[POS_SAVED] Room" << current_room_index << "not yet recognized, will set entry later";
    }
    
    if (debug_runtime)
      qInfo() << "localiser: Initial pose found via grid search at x="
              << robot_pose.translation().x() << " y=" << robot_pose.translation().y();
  }

  // 1) Transform nominal room corners into the robot frame for matching
  Corners robot_corners =
      nominal_rooms[current_room_index].transform_corners_to(
          robot_pose.inverse());

  // 2) Match detected corners to nominal room corners using Hungarian algorithm
  Match matched = hungarian.match(corners, robot_corners);

  // 3) Compute maximum match error (for plotting and localisation decision)
  float max_match_error = std::numeric_limits<float>::infinity();
  if (!matched.empty()) {
    const auto max_it =
        std::ranges::max_element(matched, [](const auto &a, const auto &b) {
          return std::get<2>(a) < std::get<2>(b);
        });
    max_match_error = static_cast<float>(std::get<2>(*max_it));
  }

  // 4) Update time-series plot (if available)
  if (time_series_plotter) {
    time_series_plotter->addDataPoint(
        match_error_graph,
        (std::isfinite(max_match_error) ? max_match_error : 0.f));
    time_series_plotter->update();
  }

  // Store matching results so compute() can run the state machine
  last_matched = matched;
  last_match_error =
      (std::isfinite(max_match_error) ? max_match_error
                                      : std::numeric_limits<float>::infinity());

  // 5) Localisation decision using configured threshold (badge-independent)
  const bool was_localised = localised;
  if (std::isfinite(max_match_error) &&
      max_match_error < LOCALISATION_MATCH_ERROR_THRESHOLD &&
      matched.size() >= 3) {
    localised = true;
    if (!was_localised && debug_runtime)
      qInfo() << "localiser: Localisation achieved. max_error="
              << max_match_error;
  } else {
    localised = false;
    if (was_localised && debug_runtime)
      qInfo() << "localiser: Localisation lost. max_error=" << max_match_error;
  }

  if (debug_runtime)
    qInfo() << "localiser: Localised=" << (localised ? "true" : "false")
            << " max_error=" << max_match_error << " matches=" << matched.size();

  // 6) Update pose continuously when we have good matches (independent of badge)
  if (matched.size() >= 3 && std::isfinite(max_match_error) &&
      max_match_error < LOCALISATION_MATCH_ERROR_THRESHOLD) {
    Eigen::Vector3d pose = solve_pose(corners, matched);
    if (update_robot_pose(pose)) {


      if (debug_runtime) {
        qInfo() << "localiser: Pose updated: x=" << robot_pose.translation().x()
                << " y=" << robot_pose.translation().y();
        double angle = std::atan2(robot_pose.rotation()(1, 0),
                                  robot_pose.rotation()(0, 0));
        qInfo() << " theta(deg)=" << qRadiansToDegrees(angle);
      }
    } else {
      if (debug_runtime)
        qInfo() << "localiser: Invalid pose (NaN), not updating robot_pose";
    }
  }

  // 7) Update robot graphic in the room viewer regardless of localisation
  robot_room_draw->setPos(robot_pose.translation().x(),
                          robot_pose.translation().y());
  double angle =
      std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
  robot_room_draw->setRotation(qRadiansToDegrees(angle));

  // 8) Debug/trace
  if (debug_runtime)
    qInfo() << "localiser: Updated robot graphic at ("
            << robot_pose.translation().x() << ", "
            << robot_pose.translation().y()
            << ") angle=" << qRadiansToDegrees(angle) << " deg";
}

Eigen::Affine2d SpecificWorker::find_best_initial_pose(const Corners &detected_corners,
                                                        std::optional<float> reference_heading) {
  const auto &room = nominal_rooms[current_room_index];
  const float w = room.width / 2.0f;
  const float l = room.length / 2.0f;

  Eigen::Affine2d best_pose = Eigen::Affine2d::Identity();
  float best_error = std::numeric_limits<float>::infinity();

  // If reference heading is provided, constrain search to ±45° around it
  float theta_start = 0.0f;
  float theta_end = 2 * M_PI;
  float theta_step = M_PI / 4;
  
  if (reference_heading.has_value()) {
    // Search only near the reference heading (±45°)
    theta_start = reference_heading.value() - M_PI / 4;
    theta_end = reference_heading.value() + M_PI / 4;
    theta_step = M_PI / 8;  // Finer steps within constrained range
    if (debug_runtime)
      qInfo() << "Grid search constrained to heading" << reference_heading.value() << "±45°";
  }

  // Grid search: 5 positions × 5 positions × angles
  for (float x = -w * 0.8f; x <= w * 0.8f; x += w * 0.4f) {
    for (float y = -l * 0.8f; y <= l * 0.8f; y += l * 0.4f) {
      for (float theta = theta_start; theta <= theta_end; theta += theta_step) {
        Eigen::Affine2d candidate;
        candidate.setIdentity();
        candidate.translate(Eigen::Vector2d(x, y));
        candidate.rotate(theta);

        // Transform nominal corners to robot frame using this candidate
        Corners robot_corners = room.transform_corners_to(candidate.inverse());

        // Match with detected corners
        Match matched = hungarian.match(detected_corners, robot_corners);

        if (matched.size() < 3)
          continue; // Need at least 3 matches

        // Compute max error
        float max_err = 0.0f;
        for (const auto &m : matched) {
          float err = static_cast<float>(std::get<2>(m));
          if (err > max_err)
            max_err = err;
        }

        if (max_err < best_error) {
          best_error = max_err;
          best_pose = candidate;
        }
      }
    }
  }

  if (debug_runtime)
    qInfo() << "Grid search best error:" << best_error
            << "at x=" << best_pose.translation().x()
            << "y=" << best_pose.translation().y();

  return best_pose;
}

void SpecificWorker::draw_lidar(const RoboCompLidar3D::TPoints &filtered_points,
                                std::optional<Eigen::Vector2d> center,
                                QGraphicsScene *scene) {
  // Vector estático para almacenar los puntos dibujados y poder borrarlos
  // después.
  static std::vector<QGraphicsItem *> draw_points;

  // 1. Limpia los puntos de la iteración anterior
  for (const auto &p : draw_points) {
    scene->removeItem(p);
    delete p;
  }
  draw_points.clear();

  // 2. Define el estilo de los nuevos puntos
  const QColor color("Black");
  const QPen pen(color, 10); // Un trazo rosa de 10px de grosor

  // 3. Determina el desplazamiento (offset) basado en el nuevo parámetro
  // 'center' Si 'center' no tiene valor, el offset es (0, 0).
  double offsetX = 0.0;
  double offsetY = 0.0;

  if (center.has_value()) // Comprueba si se ha pasado un centro
  {
    offsetX = center.value().x(); // Obtiene la coordenada x del centro
    offsetY = center.value().y(); // Obtiene la coordenada y del centro
  }

  // 4. Dibuja los nuevos puntos usando el nombre de variable 'filtered_points'
  for (const auto &p : filtered_points) {
    // Crea un pequeño rectángulo para representar el punto
    const auto dp = scene->addRect(-25, -25, 50, 50, pen);

    // Coloca el punto en su coordenada (p.x, p.y) MÁS el offset
    dp->setPos(offsetX + p.x, offsetY + p.y);

    // Guarda el punto dibujado para borrarlo en la siguiente llamada
    draw_points.push_back(dp);
  }
}

/**
 * @brief Draws the estimated room center as a visual indicator.
 *
 * This function draws a cross marker and circle at the estimated center
 * of the room to help visualize where the room center has been calculated.
 *
 * @param center The estimated center position of the room
 * @param scene  Pointer to the QGraphicsScene where the center should be drawn
 */
void SpecificWorker::draw_room_center(const Eigen::Vector2d &center,
                                      QGraphicsScene *scene) {
  // Static vector to store the drawn center items so we can delete them later
  static std::vector<QGraphicsItem *> center_items;

  // 1. Clear previous center visualization
  for (const auto &item : center_items) {
    scene->removeItem(item);
    delete item;
  }
  center_items.clear();

  // 2. Define style for the center marker
  const QColor centerColor("Red");
  const QPen centerPen(centerColor, 30);
  const QBrush centerBrush(centerColor);

  // 3. Draw a circle at the center
  auto circle = scene->addEllipse(-100, -100, 200, 200, centerPen, Qt::NoBrush);
  circle->setPos(center.x(), center.y());
  center_items.push_back(circle);

  // 4. Draw a cross (+ shape) at the center
  // Horizontal line
  auto hLine = scene->addLine(-150, 0, 150, 0, centerPen);
  hLine->setPos(center.x(), center.y());
  center_items.push_back(hLine);

  // Vertical line
  auto vLine = scene->addLine(0, -150, 0, 150, centerPen);
  vLine->setPos(center.x(), center.y());
  center_items.push_back(vLine);
}

void SpecificWorker::draw_door_target(
    const std::optional<Eigen::Vector2f> &target, QGraphicsScene *scene) {
  static std::vector<QGraphicsItem *> door_target_items;

  for (const auto &item : door_target_items) {
    scene->removeItem(item);
    delete item;
  }
  door_target_items.clear();

  if (!target.has_value())
    return;

  const QColor color("green");
  const QPen pen(color, 30);
  auto circle = scene->addEllipse(-80, -80, 160, 160, pen, Qt::NoBrush);
  circle->setPos(target.value().x(), target.value().y());
  door_target_items.push_back(circle);
}

void SpecificWorker::draw_room_doors(const std::vector<Door> &doors,
                                     QGraphicsScene *scene) {
  for (const auto &item : room_door_items) {
    scene->removeItem(item);
    delete item;
  }
  room_door_items.clear();

  // Cyan line styling
  const QColor lineColor("Cyan");
  const QPen linePen(lineColor, 30);

  // Dark blue endpoint styling
  const QColor pointColor("DarkBlue");
  const QPen pointPen(pointColor, 30);
  const QBrush pointBrush(pointColor);

  for (const auto &door : doors) {
    // Draw line (cyan)
    auto line =
        scene->addLine(door.p1.x(), door.p1.y(), door.p2.x(), door.p2.y(), linePen);
    room_door_items.push_back(line);

    // Draw endpoint circles (dark blue)
    auto c1 = scene->addEllipse(-50, -50, 100, 100, pointPen, pointBrush);
    c1->setPos(door.p1.x(), door.p1.y());
    room_door_items.push_back(c1);

    auto c2 = scene->addEllipse(-50, -50, 100, 100, pointPen, pointBrush);
    c2->setPos(door.p2.x(), door.p2.y());
    room_door_items.push_back(c2);
  }
}

void SpecificWorker::draw_topology_graph(QGraphicsScene *scene) {
  static std::vector<QGraphicsItem *> graph_items;

  for (auto *item : graph_items) {
    scene->removeItem(item);
    delete item;
  }
  graph_items.clear();

  if (!topology_graph) return;

// Force-directed layout parameters
  const float REPULSION_FORCE = 5000000.f;
  const float SPRING_FORCE = 0.05f;
  const float SPRING_LENGTH = 800.f;  // Target distance for connected nodes
  const float CENTER_FORCE = 0.01f;
  const int ITERATIONS = 50; // Run layout multiple times per frame if needed

  // 1. Initialize new nodes
  for (const auto &[id, node] : topology_graph->get_nodes()) {
      if (graph_vis_positions.find(id) == graph_vis_positions.end()) {
          // New node: place near neighbors or at random
          Eigen::Vector2f init_pos(0,0);
          auto neighbors = topology_graph->get_neighbors(id);
          if (!neighbors.empty() && graph_vis_positions.count(neighbors[0])) {
               // Place near first neighbor with random offset
               Eigen::Vector2f offset = Eigen::Vector2f::Random() * 100.f;
               init_pos = graph_vis_positions[neighbors[0]] + offset;
          } else {
               // Random start
               init_pos = Eigen::Vector2f::Random() * 500.f;
          }
          graph_vis_positions[id] = init_pos;
      }
  }

  // 2. Physics Simulation Step
  for (int iter = 0; iter < ITERATIONS; ++iter) {
      std::map<int, Eigen::Vector2f> forces;
      
      // Calculate Forces
      for (const auto &[id1, pos1] : graph_vis_positions) {
          forces[id1] = Eigen::Vector2f::Zero();
          
          // Repulsion (All nodes repel)
          for (const auto &[id2, pos2] : graph_vis_positions) {
              if (id1 == id2) continue;
              Eigen::Vector2f diff = pos1 - pos2;
              float dist_sq = diff.squaredNorm();
              if (dist_sq < 1) dist_sq = 1; // Avoid singularity
              float dist = std::sqrt(dist_sq);
              
              forces[id1] += (diff / dist) * (REPULSION_FORCE / dist_sq);
          }
          
          // Spring (Connected nodes attract)
          for (int neighbor_id : topology_graph->get_neighbors(id1)) {
              if (graph_vis_positions.count(neighbor_id)) {
                  Eigen::Vector2f diff = graph_vis_positions[neighbor_id] - pos1;
                  float dist = diff.norm();
                  float displacement = dist - SPRING_LENGTH;
                  // F = k * x
                  forces[id1] += (diff / dist) * (displacement * SPRING_FORCE);
              }
          }
          
           // Center weak attraction to keep graph in view
          forces[id1] -= pos1 * CENTER_FORCE;
      }
      
      // Apply Forces
      for (auto &[id, pos] : graph_vis_positions) {
          pos += forces[id]; // Assuming unit mass and dt=1 for simplicity
      }
  }

  // 3. Draw Edges
  QPen edgePen(Qt::black, 15);
  for (const auto &[id1, node] : topology_graph->get_nodes()) {
    for (int id2 : topology_graph->get_neighbors(id1)) {
        if (id2 > id1) { // Avoid duplicates
             if (!graph_vis_positions.count(id1) || !graph_vis_positions.count(id2)) continue;
             
             auto p1 = graph_vis_positions[id1];
             auto p2 = graph_vis_positions[id2];
             
             auto line = scene->addLine(p1.x(), p1.y(), p2.x(), p2.y(), edgePen);
             graph_items.push_back(line);
        }
    }
  }

  // 4. Draw Nodes
  QBrush roomBrush(Qt::blue);
  QPen roomPen(Qt::black, 5);
  QBrush doorBrush(Qt::red);

  for (const auto &[id, pos] : graph_vis_positions) {
      if (!topology_graph->has_node(id)) continue;
      auto node = topology_graph->get_node(id);
      
      if (node.type == NodeType::ROOM) {
          QBrush brush = (node.room_index == current_room_index) ? QBrush(Qt::green) : roomBrush;
          auto circle = scene->addEllipse(-150, -150, 300, 300, roomPen, brush);
          circle->setPos(pos.x(), pos.y());
          graph_items.push_back(circle);
          
          auto text = scene->addText(QString::fromStdString(node.name));
          QFont font("Arial", 40, QFont::Bold);
          text->setFont(font);
          
          // Flip text vertically to counter inverted Y-axis in view
          text->setTransform(QTransform::fromScale(1, -1));
          
          // Center text in the node (accounting for flipped bounding rect)
          QRectF textRect = text->boundingRect();
          text->setPos(pos.x() - textRect.width() / 2, pos.y() + textRect.height() / 2);
          
          graph_items.push_back(text);
          
      } else if (node.type == NodeType::DOOR) {
          auto rect = scene->addRect(-100, -50, 200, 100, roomPen, doorBrush);
          rect->setPos(pos.x(), pos.y());
          graph_items.push_back(rect);
          
          if (debug_runtime) {
            auto text = scene->addText(QString::number(id));
            text->setScale(3);
            text->setPos(pos.x(), pos.y());
            graph_items.push_back(text);
          }
      }
  }
}

void SpecificWorker::capture_doors_for_current_room() {
  // Use accumulated doors from TURN phase instead of single snapshot
  nominal_rooms[current_room_index].doors = accumulated_doors_for_room;
  
  if (debug_runtime)
    qInfo() << "[DOORS] Captured" << nominal_rooms[current_room_index].doors.size()
            << "doors for room" << current_room_index << "(from accumulation)";
            
  // Clear accumulation for next room
  accumulated_doors_for_room.clear();
}

int SpecificWorker::select_door_from_graph() {
  // Get room node from graph
  int room_id = topology_graph->get_room_node_id(current_room_index);
  if (room_id < 0) return -1;  // Room not in graph yet
  
  int entry_door = topology_graph->get_entry_door(current_room_index);
  auto unexplored = topology_graph->get_unexplored_doors(current_room_index);
  
  std::vector<int> candidates;
  
  // 1. Prefer unexplored, non-entry doors
  for (int door_id : unexplored) {
    if (door_id != entry_door) {
      candidates.push_back(door_id);
    }
  }
  if (!candidates.empty()) {
    std::uniform_int_distribution<> dist(0, static_cast<int>(candidates.size()) - 1);
    int selected = candidates[dist(rd)];
    if (debug_runtime)
      qInfo() << "[GRAPH_SELECT] Chose unexplored non-entry door" << selected 
              << "(from" << candidates.size() << "candidates)";
    return selected;
  }
  
  // 2. Any unexplored door (even if entry)
  if (!unexplored.empty()) {
    std::uniform_int_distribution<> dist(0, static_cast<int>(unexplored.size()) - 1);
    int selected = unexplored[dist(rd)];
    if (debug_runtime)
      qInfo() << "[GRAPH_SELECT] Chose unexplored door" << selected;
    return selected;
  }
  
  // 3. Any explored non-entry door
  candidates.clear();
  for (int door_id : topology_graph->get_neighbors(room_id)) {
    if (topology_graph->get_node(door_id).type == NodeType::DOOR) {
      // Room 0: True random (include entry door)
      if (current_room_index == 0) {
          candidates.push_back(door_id);
      }
      // Other rooms: Prefer non-entry (traversal)
      else if (door_id != entry_door) {
          candidates.push_back(door_id);
      }
    }
  }
  if (!candidates.empty()) {
    std::uniform_int_distribution<> dist(0, static_cast<int>(candidates.size()) - 1);
    int selected = candidates[dist(rd)];
    if (debug_runtime)
      qInfo() << "[GRAPH_SELECT] Chose explored non-entry door" << selected;
    return selected;
  }
  
  // 4. Backtrack through entry door
  if (debug_runtime)
    qInfo() << "[GRAPH_SELECT] Backtracking via entry door" << entry_door;
  return entry_door;
}

void SpecificWorker::set_entry_door_by_proximity() {
  int room_id = topology_graph->get_room_node_id(current_room_index);
  if (room_id < 0 || !room_entry_position.has_value()) {
    qInfo() << "[ENTRY_DOOR] Skip: room_id=" << room_id 
            << "has_position=" << room_entry_position.has_value();
    return;
  }
  
  qInfo() << "[ENTRY_DOOR] Room" << current_room_index 
          << "robot entry position: (" << room_entry_position.value().x() 
          << "," << room_entry_position.value().y() << ")";
  
  int closest_door = -1;
  float min_dist = std::numeric_limits<float>::max();
  
  for (int neighbor_id : topology_graph->get_neighbors(room_id)) {
    const auto& node = topology_graph->get_node(neighbor_id);
    if (node.type == NodeType::DOOR) {
      // Only consider doors from THIS room (filter out doors from other rooms)
      std::string prefix = "Door_R" + std::to_string(current_room_index) + "_";
      if (node.name.rfind(prefix, 0) != 0) {
        qInfo() << "[ENTRY_DOOR] Skipping door" << neighbor_id 
                << "(" << QString::fromStdString(node.name) << ") - not from this room";
        continue;
      }
      
      auto door_pos = node.position;
      float dist = (door_pos - room_entry_position.value()).norm();
      
      qInfo() << "[ENTRY_DOOR] Door" << neighbor_id 
              << "at (" << door_pos.x() << "," << door_pos.y() << ")"
              << "dist=" << dist;
      
      if (dist < min_dist) {
        min_dist = dist;
        closest_door = neighbor_id;
      }
    }
  }
  
  if (closest_door >= 0) {
    topology_graph->set_entry_door(current_room_index, closest_door);
    qInfo() << "[ENTRY_DOOR] Selected door" << closest_door 
            << "for room" << current_room_index << "(closest)";
            
    // LEARN: Connect previous door to this closest door if valid
    if (previous_traversed_door_id != -1 && topology_graph->has_node(previous_traversed_door_id)) {
        // Validation: Ensure we are not connecting two doors of the SAME room
        bool same_room = false;
        bool door_already_linked = false;

        auto prev_neighbors = topology_graph->get_neighbors(previous_traversed_door_id);
        auto curr_neighbors = topology_graph->get_neighbors(closest_door);
        
        // Check 1: Same Room?
        for (int p_neighbor : prev_neighbors) {
            for (int c_neighbor : curr_neighbors) {
                if (p_neighbor == c_neighbor) {
                    same_room = true; 
                    break;
                }
            }
            if (same_room) break;
        }

        // Check 2: Door Exclusivity (Max degree 2: 1 Room + 1 Door)
        // Check if 'previous' door already has a DOOR neighbor
        for (int p_neighbor : prev_neighbors) {
             if (topology_graph->get_node(p_neighbor).type == NodeType::DOOR) {
                 door_already_linked = true;
                 qInfo() << "[ENTRY_DOOR] Blocked: Previous door" << previous_traversed_door_id 
                         << "already linked to door" << p_neighbor;
                 break;
             }
        }
        // Check if 'current' door already has a DOOR neighbor
        if (!door_already_linked) {
            for (int c_neighbor : curr_neighbors) {
                if (topology_graph->get_node(c_neighbor).type == NodeType::DOOR) {
                    door_already_linked = true;
                     qInfo() << "[ENTRY_DOOR] Blocked: Current door" << closest_door 
                             << "already linked to door" << c_neighbor;
                    break;
                }
            }
        }

        if (!same_room && !door_already_linked) {
            topology_graph->connect(previous_traversed_door_id, closest_door);
            qInfo() << "[ENTRY_DOOR] LEARNED connection:" << previous_traversed_door_id 
                    << "<->" << closest_door;
        } else {
            qInfo() << "[ENTRY_DOOR] Connection blocked. Same Room:" << same_room 
                    << "Already Linked:" << door_already_linked;
        }
        previous_traversed_door_id = -1; // Reset attempt regardless of success
    }
  }
  room_entry_position.reset();
}

// Esta función recibe un conjunto de puntos del LiDAR (cada punto tiene
// coordenadas x, y, r y phi) y devuelve, para cada ángulo 'phi', el punto más
// cercano (menor distancia 'r'). Se usa std::optional para devolver un
// resultado vacío si no hay puntos válidos.
std::optional<RoboCompLidar3D::TPoints>
SpecificWorker::filter_min_distance_cppitertools(
    const RoboCompLidar3D::TPoints &points) {
  // 🧩 1️⃣ Si no hay puntos, se devuelve un optional vacío.
  if (points.empty()) {
    return {};
  }

  // 🧩 2️⃣ Se crea un contenedor para los puntos filtrados.
  // Se reserva memoria para evitar realocaciones (mejora de rendimiento).
  RoboCompLidar3D::TPoints result;
  result.reserve(points.size());

  // 🧩 3️⃣ Se agrupan los puntos por su ángulo 'phi' con una precisión de 2
  // decimales.
  //    Esto significa que todos los puntos que tengan un ángulo muy similar se
  //    procesan juntos.
  for (auto &&[angle, group] : iter::groupby(points, [](const auto &p) {
         // Redondeo de 'phi' a dos decimales.
         float multiplier = std::pow(10.0f, 2);
         return std::floor(p.phi * multiplier) / multiplier;
       })) {
    // 🧩 4️⃣ Dentro de cada grupo de puntos con el mismo ángulo,
    // se busca el punto con menor distancia 'r' (el más cercano al robot).
    auto min_it = std::min_element(
        std::begin(group), std::end(group),
        [](const auto &a, const auto &b) { return a.r < b.r; });

    // 🧩 5️⃣ Se añade ese punto al vector de resultados.
    // Este punto representa el obstáculo más cercano en esa dirección.
    result.emplace_back(*min_it);
  }

  // 🧩 6️⃣ Se devuelve el conjunto de puntos filtrados (uno por ángulo).
  return result;
}

RoboCompLidar3D::TPoints
SpecificWorker::filter_data_basic(const RoboCompLidar3D::TPoints &data) {
  auto filtered_data = filter_min_distance_cppitertools(data);
  if (!filtered_data.has_value() || filtered_data.value().empty()) {
    qWarning() << "No points received";
    return RoboCompLidar3D::TPoints{};
  }
  auto result = filter_isolated_points(filtered_data.value(), 200.0f);
  return result;
}

std::vector<RoboCompLidar3D::TPoint> SpecificWorker::read_data() {
  // 1. Declara 'filter_data' en el alcance de la función (aquí fuera)
  RoboCompLidar3D::TPoints filter_data = {}; // Inicialízala vacía

  try {
    const auto data =
        lidar3d_proxy->getLidarDataWithThreshold2d("helios", 12000, 1);

    // 2. Asigna el valor (sin 'RoboCompLidar3D::TPoints' delante)
    //    Usamos .value_or() para desenvolver el 'optional' que devuelve la
    //    función
    filter_data = filter_min_distance_cppitertools(data.points)
                      .value_or(RoboCompLidar3D::TPoints{});

    if (data.points.empty()) {
      qWarning() << "No points received";
      return {};
    }
  } catch (const Ice::Exception &e) {
    std::cout << e.what() << std::endl;
    return {}; // Devuelve vacío también si hay un error
  }

  // 3. Ahora 'filter_data' SÍ existe aquí, y es un TPoints (un vector),
  //    así que no uses .value()
  return filter_isolated_points(filter_data, 200.0f);
}

std::expected<int, std::string>
SpecificWorker::closest_lidar_index_to_given_angle(const auto &points,
                                                   float angle) {
  return std::expected<int, std::string>();
}

RoboCompLidar3D::TPoints
SpecificWorker::filter_same_phi(const RoboCompLidar3D::TPoints &points) {
  return RoboCompLidar3D::TPoints();
}

RoboCompLidar3D::TPoints
SpecificWorker::filter_isolated_points(const RoboCompLidar3D::TPoints &points,
                                       float d) {
  if (points.empty())
    return {};

  const float d_squared = d * d; // Avoid sqrt by comparing squared distances
  std::vector<bool> hasNeighbor(points.size(), false);

  // Create index vector for parallel iteration
  std::vector<size_t> indices(points.size());
  std::iota(indices.begin(), indices.end(), size_t{0});

  for (size_t i = 0; i < points.size(); ++i) {
    const auto &p1 = points[i];
    for (size_t j = 0; j < points.size(); ++j) {
      if (i == j)
        continue;
      const auto &p2 = points[j];
      float dx = p1.x - p2.x;
      float dy = p1.y - p2.y;
      if (dx * dx + dy * dy <= d_squared) {
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

void SpecificWorker::print_match(const Match &match, const float error) const {}

bool SpecificWorker::update_robot_pose(Eigen::Vector3d pose) {

  if (debug_runtime) {
    qInfo() << "Nueva pose estimada:";
    qInfo() << "X:" << pose(0) << "Y:" << pose(1) << "Theta:" << pose(2);
  }

  if (pose.array().isNaN().any())
    return false;

  robot_pose.translate(Eigen::Vector2d(pose(0), pose(1)));
  robot_pose.rotate(pose(2));

  return true;
}

void SpecificWorker::move_robot(float adv, float rot, float max_match_error) {
  // Movement is decided by the state machine in compute(); do not gate here.
  try {
    if (debug_runtime)
      qInfo() << "move_robot: adv=" << adv << " rot=" << rot
              << " max_match_error=" << max_match_error;
    omnirobot_proxy->setSpeedBase(0, adv, rot);
  } catch (const Ice::Exception &e) {
    qWarning() << "move_robot: exception calling setSpeedBase:" << e.what();
    return;
  }
}

Eigen::Vector3d SpecificWorker::solve_pose(const Corners &corners,
                                           const Match &match) {
  Eigen::MatrixXd W(match.size() * 2, 3);
  Eigen::VectorXd b(match.size() * 2);

  for (auto &&[i, m] : match | iter::enumerate) {
    auto &[meas_c, nom_c, _] = m;
    auto &[p_meas, __, ___] = meas_c;
    auto &[p_nom, ____, _____] = nom_c;

    b(2 * i) = p_nom.x() - p_meas.x();
    b(2 * i + 1) = p_nom.y() - p_meas.y();
    W.block<1, 3>(2 * i, 0) << 1.0, 0.0, -p_meas.y();
    W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
  }
  // estimate new pose with pseudoinverse
  const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
  return r;
}

/**
 * @brief Angle-distance controller to navigate robot to room center.
 *
 * Implements the controller from RoboLab Technical Report RL0021:
 * - PD controller for rotation (angular velocity ω)
 * - Gaussian angle-brake for smooth turns
 * - Sigmoid distance-brake for smooth stopping
 *
 * @param points LiDAR points (unused in current implementation)
 * @return RetVal {next_state, advance_velocity, rotation_velocity}
 */
std::tuple<float, float>
SpecificWorker::robot_controller(const Eigen::Vector2f &target) {
  // 1. Calculate distance and angle to target (robot frame coordinates)
  float d = target.norm();
  if (d < 100.f)
    return {0.f, 0.f};
  float theta_e = std::atan2(target.x(), target.y());

  // 2. Higher rotation gain for faster alignment (was 0.5)
  float theta_dot_e = 0.8f * theta_e;

  // 3. Wider Gaussian falloff for smoother speed curve (was π/6)
  float omega = exp((-theta_e * theta_e) / (M_PI / 4.f));

  // 4. Distance-based speed scaling for smooth approach
  float dist_factor = std::min(1.0f, d / 500.f);

  float v = 1000.f * omega * dist_factor;

  return {v, theta_dot_e};
}

SpecificWorker::RetVal
SpecificWorker::goto_room_center(const RoboCompLidar3D::TPoints &points) {
  // Check if we have a valid target
  if (!target_room_center.has_value()) {
    if (debug_runtime)
      qInfo() << "[GOTO_ROOM_CENTER] No target available, returning to IDLE";
    omnirobot_proxy->setSpeedBase(0, 0, 0);
    return {STATE::IDLE, 0.0f, 0.0f};
  }

  const auto &target = target_room_center.value();

  // 1. Calculate distance
  float d = target.norm();
  // float theta_e = std::atan2(target.y(), target.x());

  // 2. Check if we've arrived at the target
  if (d < 100.f) {
    if (debug_runtime)
      qInfo() << "[GOTO_ROOM_CENTER] Target reached! d=" << d << "mm";
    target_room_center.reset();
    omnirobot_proxy->setSpeedBase(0, 0, 0);
    
    // Skip TURN for recognized rooms - go directly to GOTO_DOOR
    if (room_recognized[current_room_index]) {
      if (debug_runtime)
        qInfo() << "[GOTO_ROOM_CENTER] Room already recognized, skipping TURN";
      return {STATE::GOTO_DOOR, 0.0f, 0.0f};
    }
    return {STATE::TURN, 0.0f, 0.0f};
  }
  auto [v, omega] = robot_controller(target.cast<float>());

  if (debug_runtime)
    qInfo() << "[GOTO_ROOM_CENTER] Target:" << target.x() << "," << target.y()
            << "Dist:" << d << "mm. V:" << v << "mm/s, W:" << omega << "rad/s";
  omnirobot_proxy->setSpeedBase(0, v, omega);

  return {STATE::GOTO_ROOM_CENTER, v, omega};
}
SpecificWorker::RetVal SpecificWorker::turn(const Corners &corners) {
  bool detected = false;
  int direction = 1;
  float rot_speed = 0.8f;
  
  // Accumulate doors during rotation to capture all doors from different angles
  auto current_doors = door_detector.doors();
  for (const auto& door : current_doors) {
    Eigen::Vector2d center_robot(door.center().x(), door.center().y());
    Eigen::Vector2d center_world = robot_pose * center_robot;
    
    // Check if this door is already in accumulated list (by proximity)
    bool is_new = true;
    for (const auto& existing : accumulated_doors_for_room) {
        Eigen::Vector2f existing_center = (existing.p1 + existing.p2) / 2.f;
        if ((existing_center - center_world.cast<float>()).norm() < 500.f) {
            is_new = false;
            break;
        }
    }
    
    if (is_new) {
        // Transform and add to accumulated list
        Eigen::Vector2d p1_world = robot_pose * Eigen::Vector2d(door.p1.x(), door.p1.y());
        Eigen::Vector2d p2_world = robot_pose * Eigen::Vector2d(door.p2.x(), door.p2.y());
        
        // Align to wall
        Eigen::Vector2f center_f = center_world.cast<float>();
        const auto& room = nominal_rooms[current_room_index];
        const auto wall = room.get_closest_wall_to_point(center_f);
        const auto& wall_line = std::get<0>(wall);
        
        Eigen::Vector2f p1_aligned = wall_line.projection(p1_world.cast<float>());
        Eigen::Vector2f p2_aligned = wall_line.projection(p2_world.cast<float>());
        
        Door world_door(p1_aligned, door.p1_angle, p2_aligned, door.p2_angle);
        accumulated_doors_for_room.push_back(world_door);
        
        if (debug_runtime)
          qInfo() << "[TURN] Accumulated new door. Total:" << accumulated_doors_for_room.size();
    }
  }
  
  if (debug_runtime)
    qInfo() << "[TURN] Searching for" << (search_green ? "GREEN" : "RED")
            << "panel. Rot speed:" << rot_speed;

  if (search_green) {
    // Check for Green Panel
    auto [det, dir] = rc::ImageProcessor::check_color_patch_in_image(
        camera360rgb_proxy, rc::ImageProcessor::Color::GREEN, nullptr, 200);
    detected = det;
    direction = dir;
  } else {
    // Check for Red Panel
    auto [det, dir] = rc::ImageProcessor::check_color_patch_in_image(
        camera360rgb_proxy, rc::ImageProcessor::Color::RED, nullptr);
    detected = det;
    direction = dir;
  }

  if (debug_runtime)
    qInfo() << "[TURN] Detected: " << detected;

  // Logic to handle detection
  if (detected) {
    if (debug_runtime)
      qInfo() << "[TURN] Panel centered! Stopping.";
    badge_found = true;
    
    // Recognize room and add to graph if not already done
    if (!room_recognized[current_room_index]) {
      // Capture doors for this room
      capture_doors_for_current_room();
      
      // Add room to topology graph
      auto& room = nominal_rooms[current_room_index];
      auto center = room.rect().center();
      int room_node_id = topology_graph->add_room(
          "Room_" + std::to_string(current_room_index),
          current_room_index,
          room.width, room.length,
          Eigen::Vector2f(center.x(), center.y()));
      
      // Add doors to graph and connect to room
      for (size_t i = 0; i < room.doors.size(); ++i) {
        const auto& door = room.doors[i];
        int door_node_id = topology_graph->add_door(
            "Door_R" + std::to_string(current_room_index) + "_" + std::to_string(i),
            door.p1, door.p2);
        topology_graph->connect(room_node_id, door_node_id);
      }
      
      // Set entry door by proximity (find door closest to where robot entered)
      set_entry_door_by_proximity();
      
      room_recognized[current_room_index] = true;
      
      // Store robot heading for future re-entry (breaks 180° ambiguity)
      float current_heading = std::atan2(robot_pose.rotation()(1,0), robot_pose.rotation()(0,0));
      topology_graph->set_room_heading(current_room_index, current_heading);
      
      if (debug_runtime)
        qInfo() << "[TURN] Room" << current_room_index << "recognized and added to graph";
    }
    
    omnirobot_proxy->setSpeedBase(0, 0, 0);
    return {STATE::IDLE, 0.0f, 0.0f}; // Found and centered
  }
  // If not detected or not centered, rotate
  // omnirobot_proxy->setSpeedBase(0, 0, rot_speed * -direction); // Overridden
  // by compute()
  return {STATE::TURN, 0.0f, rot_speed * direction};
}
SpecificWorker::RetVal
SpecificWorker::goto_door(const RoboCompLidar3D::TPoints &points) {
  auto doors = door_detector.doors();
  if (doors.empty()) {
    if (debug_runtime)
      qInfo() << "[GOTO_DOOR] No door found. Rotating.";
    // Don't reset target_door_point here - keep navigating to last known target if we have one
    omnirobot_proxy->setSpeedBase(0, 0, 0.2);
    return {STATE::GOTO_DOOR, 0.0f, 0.2f};
  }

  // Sort doors by angle (or some consistent metric) to ensure indices are
  // stable
  std::sort(doors.begin(), doors.end(), [](const auto &a, const auto &b) {
    return a.direction() < b.direction();
  });

  int selected_index = 0;

  // Use graph-based selection for recognized rooms
  if (room_recognized[current_room_index]) {
    // Only select once (sticky) - reuse until door is crossed
    if (selected_graph_door_id < 0) {
      selected_graph_door_id = select_door_from_graph();
    }
    
    if (selected_graph_door_id >= 0) {
      // Get door position from graph
      const auto& door_node = topology_graph->get_node(selected_graph_door_id);
      Eigen::Vector2d door_world_pos = door_node.position.cast<double>();
      
      // Find the detected door closest to graph door position
      float min_dist = std::numeric_limits<float>::max();
      for (size_t i = 0; i < doors.size(); ++i) {
        Eigen::Vector2d detected_world = robot_pose * doors[i].center().cast<double>();
        float dist = (detected_world - door_world_pos).norm();
        if (dist < min_dist) {
          min_dist = dist;
          selected_index = static_cast<int>(i);
        }
      }
      
      // Set chosen_door_world_pos for sticky tracking
      chosen_door_world_pos = robot_pose * doors[selected_index].center().cast<double>();
      
      // Explicitly set traversing_door_id for the sticky case
      traversing_door_id = selected_graph_door_id;
      
      if (debug_runtime)
        qInfo() << "[GOTO_DOOR] Graph selected door" << selected_graph_door_id
                << "matched to index" << selected_index;
      
      // Skip the room-specific logic below
      goto navigate_to_door;
    }
  }

  // Fallback: Room 0: Random selection, track by world position
  if (current_room_index == 0) {
    if (!chosen_door_world_pos.has_value()) {
      // First time: randomly select
      if (doors.size() > 1) {
        std::uniform_int_distribution<> dist(0, static_cast<int>(doors.size()) - 1);
        selected_index = dist(rd);
      } else {
        selected_index = 0;
      }
      // Save world position and left/right for Room 1
      chosen_door_world_pos = robot_pose * doors[selected_index].center().cast<double>();
      chosen_door_was_on_left = (doors[selected_index].center().x() < 0);
      if (debug_runtime)
        qInfo() << "[GOTO_DOOR] Room 0: Selected door" << selected_index 
                << "x=" << doors[selected_index].center().x()
                << "world=(" << chosen_door_world_pos.value().x() << "," 
                << chosen_door_world_pos.value().y() << ")";
    } else {
      // Sticky: find door closest to saved world position
      float min_dist = std::numeric_limits<float>::max();
      for (size_t i = 0; i < doors.size(); ++i) {
        Eigen::Vector2d door_world = robot_pose * doors[i].center().cast<double>();
        float dist = (door_world - chosen_door_world_pos.value()).norm();
        if (dist < min_dist) {
          min_dist = dist;
          selected_index = i;
        }
      }
      // Update saved position for next frame
      chosen_door_world_pos = robot_pose * doors[selected_index].center().cast<double>();
      if (debug_runtime)
        qInfo() << "[GOTO_DOOR] Room 0: Sticky selection, door" << selected_index
                << "world_dist=" << min_dist;
    }
  }
  // Room 1: SAME side (coordinates are mirrored because badges are on opposite walls)
  else {
    if (!chosen_door_world_pos_room1.has_value() && chosen_door_was_on_left.has_value()) {
      // First frame: select by min/max x
      bool want_left = chosen_door_was_on_left.value();
      float target_x = want_left ? std::numeric_limits<float>::max() 
                                  : std::numeric_limits<float>::lowest();
      for (size_t i = 0; i < doors.size(); ++i) {
        float x = doors[i].center().x();
        if ((want_left && x < target_x) || (!want_left && x > target_x)) {
          target_x = x;
          selected_index = i;
        }
      }
      // Save world position for sticky tracking
      chosen_door_world_pos_room1 = robot_pose * doors[selected_index].center().cast<double>();
      if (debug_runtime)
        qInfo() << "[GOTO_DOOR] Room 1: Selected door" << selected_index
                << "x=" << doors[selected_index].center().x()
                << "want_left=" << want_left;
    } else if (chosen_door_world_pos_room1.has_value()) {
      // Sticky: find door closest to saved world position
      float min_dist = std::numeric_limits<float>::max();
      for (size_t i = 0; i < doors.size(); ++i) {
        Eigen::Vector2d door_world = robot_pose * doors[i].center().cast<double>();
        float dist = (door_world - chosen_door_world_pos_room1.value()).norm();
        if (dist < min_dist) {
          min_dist = dist;
          selected_index = i;
        }
      }
      // Update saved position
      chosen_door_world_pos_room1 = robot_pose * doors[selected_index].center().cast<double>();
      if (debug_runtime)
        qInfo() << "[GOTO_DOOR] Room 1: Sticky selection, door" << selected_index
                << "world_dist=" << min_dist;
    }
  }

  // Safety check
  if (selected_index >= static_cast<int>(doors.size()))
    selected_index = 0;

  // IMPORTANT: Identify the Graph Node ID for the selected door
  // This ensures 'traversing_door_id' is set correctly for the connection logic in 'switch_room'
  {
      int room_graph_id = topology_graph->get_room_node_id(current_room_index);
      if (room_graph_id >= 0) {
          Eigen::Vector2d selected_world_pos = robot_pose * doors[selected_index].center().cast<double>();
          
          int closest_node_id = -1;
          float best_dist = 2000.0f; // Threshold (mm) - increased to 2m to handle drift
          
          for (int neighbor_id : topology_graph->get_neighbors(room_graph_id)) {
              const auto& node = topology_graph->get_node(neighbor_id);
              if (node.type == NodeType::DOOR) {
                  float d = (node.position.cast<double>() - selected_world_pos).norm();
                  if (d < best_dist) {
                      best_dist = d;
                      closest_node_id = neighbor_id;
                  }
              }
          }
          
          if (closest_node_id != -1) {
              traversing_door_id = closest_node_id;
              if (debug_runtime)
                qInfo() << "[GOTO_DOOR] Identified traversing_door_id =" << traversing_door_id 
                        << "for physical door index" << selected_index;
          }
      }
  }

navigate_to_door:
  const auto &door = doors[selected_index];
  Eigen::Vector2f target =
      door.center_before(Eigen::Vector2d(0, 0), params.DOOR_APPROACH_DISTANCE);
  target_door_point = target;
  auto [v, w] = robot_controller(target);

  if (v == 0.0f && w == 0.0f) {
    if (debug_runtime)
      qInfo() << "[GOTO_DOOR] Reached door approximation point.";

    // Update graph tracking


    auto_nav_sequence_running = false;
    target_door_point.reset();
    omnirobot_proxy->setSpeedBase(0, 0, 0);

    // 1. Calculate vectors:
    //    - door_parallel: used as the TARGET for alignment (system expects
    //    parallel to align perpendicular).
    //    - door_normal: used for the STABLE check against the inward vector.
    Eigen::Vector2f door_parallel = door.p2 - door.p1;
    Eigen::Vector2f door_normal =
        Eigen::Vector2f(-door_parallel.y(), door_parallel.x());
    Eigen::Vector2f target_vector = door_parallel;

    // 2. Determine the correct direction for the vector (it should //    We use
    // the nominal room's center as a stable reference for the "inward"
    // direction.
    QPointF room_center_qpoint =
        nominal_rooms[current_room_index].rect().center();
    Eigen::Vector2d room_center_eigen(room_center_qpoint.x(),
                                      room_center_qpoint.y());
    Eigen::Vector2d world_inward_vector =
        room_center_eigen - robot_pose.translation();
    Eigen::Vector2d robot_inward_vector =
        robot_pose.rotation().inverse() * world_inward_vector;

    if (debug_runtime) {
      qInfo() << "DEBUG: Door p1 = (" << door.p1.x() << ", " << door.p1.y()
              << ")";
      qInfo() << "DEBUG: Door p2 = (" << door.p2.x() << ", " << door.p2.y()
              << ")";
      qInfo() << "DEBUG: door_parallel = (" << door_parallel.x() << ", "
              << door_parallel.y() << ")";
      qInfo() << "DEBUG: door_normal = (" << door_normal.x() << ", "
              << door_normal.y() << ")";
      qInfo()
          << "DEBUG: nominal_rooms[current_room_index].center() (QPointF) = ("
          << room_center_qpoint.x() << ", " << room_center_qpoint.y() << ")";
      qInfo() << "DEBUG: robot_pose.translation() = ("
              << robot_pose.translation().x() << ", "
              << robot_pose.translation().y() << ")";
      qInfo() << "DEBUG: world_inward_vector = (" << world_inward_vector.x()
              << ", " << world_inward_vector.y() << ")";
      qInfo() << "DEBUG: robot_inward_vector = (" << robot_inward_vector.x()
              << ", " << robot_inward_vector.y() << ")";
      qInfo() << "DEBUG: dot product (normal . inward) = "
              << door_normal.cast<double>().dot(robot_inward_vector);
    }
    bool flipped = false;
    // 3. Check stability using NORMAL vector.
    //    We want the normal to point OUTWARD (towards the door), i.e., OPPOSED
    //    to the inward vector. So if it is ALIGNED (dot > 0), we flip it.
    if (door_normal.cast<double>().dot(robot_inward_vector) > 0.0) {
      // If normal is opposed, we flip the TARGET vector.
      target_vector = -target_vector;
      flipped = true;
    }
    if (debug_runtime) {
      qInfo() << "DEBUG: Flipped = " << (flipped ? "true" : "false");
      qInfo() << "DEBUG: Final target_vector = (" << target_vector.x() << ", "
              << target_vector.y() << ")";
    }

    // 4. Calculate the angle of this desired vector in the robot's frame.
    const float target_angle_robot =
        std::atan2(target_vector.y(), target_vector.x());
    // 5. Convert the relative angle to a world angle for the orientation
    // target.
    const double current_robot_angle =
        std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
    const double door_target_angle_world =
        current_robot_angle + target_angle_robot;
    orient_target_angle =
        atan2(sin(door_target_angle_world), cos(door_target_angle_world));
    if (debug_runtime)
      qInfo() << "[GOTO_DOOR] Setting sticky world orientation target:"
              << orient_target_angle.value();

    return {STATE::ORIENT_TO_DOOR, 0.0f, 0.0f};
  }
  if (debug_runtime)
    qInfo() << "[GOTO_DOOR] Door found. Target:" << target.x() << ","
            << target.y() << ". V:" << v << "mm/s, W:" << w << "rad/s";

  omnirobot_proxy->setSpeedBase(v, 0, w);
  return {STATE::GOTO_DOOR, v, w};
}

SpecificWorker::RetVal
SpecificWorker::orient_to_door(const RoboCompLidar3D::TPoints &points) {
  // 1. Check for a sticky target
  if (!orient_target_angle.has_value()) {
    qWarning()
        << "ORIENT_TO_DOOR: No orientation target set. Transitioning to IDLE.";
    return {STATE::IDLE, 0.0f, 0.0f};
  }

  // 2. Calculate current rotational error
  double current_robot_angle =
      std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
  float rot_error = orient_target_angle.value() - current_robot_angle;
  rot_error = atan2(sin(rot_error), cos(rot_error)); // Normalize error

  // 3. State Transition Check
  const float angle_tolerance = 0.1f; // approx 5.7 degrees
  if (std::abs(rot_error) < angle_tolerance) {
    if (debug_runtime)
      qInfo() << "[ORIENT_TO_DOOR] Aligned! Transitioning to CROSS_DOOR.";
    omnirobot_proxy->setSpeedBase(0, 0, 0);
    orient_target_angle.reset(); // Clear the sticky target
    
    // NOTE: traversing_door_id is already set correctly during goto_door() 
    // at lines 1440 (graph-based) or 1552 (proximity-based). Do not overwrite.
    
    return {STATE::CROSS_DOOR, 0.0f, 0.0f};
  }

  // 4. Rotation Control
  const float rot_speed = 0.5f;
  float rot_velocity = -std::copysign(rot_speed, rot_error);

  if (debug_runtime) {
    qInfo() << "[ORIENT_TO_DOOR] World Target:" << orient_target_angle.value()
            << "rad, Current World:" << current_robot_angle
            << "rad, Rotational Error:" << rot_error
            << "rad, Velocity:" << rot_velocity << "rad/s";
    Eigen::Vector2d robot_vector(cos(current_robot_angle),
                                 sin(current_robot_angle));
    Eigen::Vector2d door_vector(cos(orient_target_angle.value()),
                                sin(orient_target_angle.value()));
    qInfo() << "    [VECTORS] Door: (" << door_vector.x() << ","
            << door_vector.y() << ") Robot: (" << robot_vector.x() << ","
            << robot_vector.y() << ") Angle: " << qRadiansToDegrees(rot_error)
            << " deg";
  }

  // Send command to robot (only rotation)
  omnirobot_proxy->setSpeedBase(0, 0, rot_velocity);

  // Remain in the current state
  return {STATE::ORIENT_TO_DOOR, 0.0f, rot_velocity};
}

SpecificWorker::RetVal
SpecificWorker::cross_door(const RoboCompLidar3D::TPoints &points) {
  const float adv_speed = 400.0f; // mm/s

  // 1. Initialize start time if not set
  if (!cross_door_start_time.has_value()) {
    cross_door_start_time = std::chrono::steady_clock::now();
    if (debug_runtime)
      qInfo() << "[CROSS_DOOR] Starting cross door maneuver (Timer-based).";
  }

  // 2. Calculate elapsed time
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - cross_door_start_time.value();

  // 3. Required duration (Hardcoded as per user request)
  double required_duration = 5.0;

  // 4. Check termination condition
  if (elapsed.count() >= required_duration) {
    if (debug_runtime)
      qInfo() << "[CROSS_DOOR] Timer expired (" << elapsed.count()
              << "s). Transitioning to GOTO_ROOM_CENTER.";
    omnirobot_proxy->setSpeedBase(0, 0, 0);
    cross_door_start_time.reset();
    switch_room();
    return {STATE::GOTO_ROOM_CENTER, 0.0f, 0.0f};
  }

  // 5. Move straight forward
  omnirobot_proxy->setSpeedBase(adv_speed, 0, 0);

  if (debug_runtime)
    qInfo() << "[CROSS_DOOR] Crossing... Time: " << elapsed.count() << "/"
            << required_duration << " s";

  return {STATE::CROSS_DOOR, adv_speed, 0.0f};
}

void SpecificWorker::switch_room() {
  // 1. Update room index
  current_room_index = (current_room_index + 1) % nominal_rooms.size();
  if (debug_runtime)
    qInfo() << "[SWITCH_ROOM] Switching to room index: " << current_room_index;

  // 2. Clear and redraw viewer_room
  viewer_room->scene.clear();
  room_door_items
      .clear(); // Clear the vector as items are deleted by scene.clear()

  // Re-add robot to viewer_room (since clear removed it)
  auto [rr, re] = viewer_room->add_robot(
      params.ROBOT_WIDTH, params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
  robot_room_draw = rr;

  // Draw axes at center (always visible)
  auto center = nominal_rooms[current_room_index].rect().center();
  // X-axis (Red)
  viewer_room->scene.addLine(center.x(), center.y(), center.x() + 350,
                             center.y(), QPen(Qt::red, 20));
  // Y-axis (Green)
  viewer_room->scene.addLine(center.x(), center.y(), center.x(),
                             center.y() + 350, QPen(Qt::green, 20));

  // Draw walls only if room is already recognized
  if (room_recognized[current_room_index]) {
    // Draw room walls
    viewer_room->scene.addRect(nominal_rooms[current_room_index].rect(),
                               QPen(Qt::black, 30));
  }

  // 3. Reset robot pose and re-run Ultra-localiser for new entry position
  robot_pose.setIdentity();
  initial_localisation_done = false;  // Force grid search to find correct entry pose

  // 3b. Update graph: store previous door and check for learned connections
  previous_traversed_door_id = traversing_door_id;
  
  if (previous_traversed_door_id != -1 && topology_graph->is_room_in_graph(current_room_index)) {
    // Check if we already have a learned connection to the new room
    int connected_door = -1;
    for (int neighbor : topology_graph->get_neighbors(previous_traversed_door_id)) {
        const auto& node = topology_graph->get_node(neighbor);
        // If neighbor is a door belonging to the NEW room, we found our entry!
        std::string prefix = "Door_R" + std::to_string(current_room_index) + "_";
        if (node.type == NodeType::DOOR && node.name.find(prefix) == 0) {
            connected_door = neighbor;
            break;
        }
    }

    if (connected_door != -1) {
        topology_graph->set_entry_door(current_room_index, connected_door);
        if (debug_runtime)
          qInfo() << "[ENTRY_DOOR] Found learned connection:" << previous_traversed_door_id 
                  << "<->" << connected_door << ". Set entry door immediately.";
        previous_traversed_door_id = -1; // Connection found, no need to learn again
    } else {
        if (debug_runtime)
          qInfo() << "[SWITCH_ROOM] No learned connection from door" << previous_traversed_door_id 
                  << "yet. Will learn via proximity.";
    }
  }
  
  traversing_door_id = -1;  // Reset
  selected_graph_door_id = -1;  // Reset sticky door selection for new room

  // 4. Reset badge_found status for the new room
  badge_found = false;

  // 5. Set search_green based on room index (Room 0 = Red, Room 1 = Green)
  if (current_room_index == 1)
    search_green = true;
  else
    search_green = false;

  if (debug_runtime)
    qInfo() << "[SWITCH_ROOM] search_green set to: "
            << (search_green ? "TRUE" : "FALSE");

  // 6. If we are back to Room 0, reset the door memory so we choose randomly
  // again next time
  if (current_room_index == 0) {
    chosen_door_was_on_left.reset();
    chosen_door_world_pos.reset();
    chosen_door_world_pos_room1.reset();
    if (debug_runtime)
      qInfo() << "[SWITCH_ROOM] Resetting door memory for new cycle.";
  }
}

SpecificWorker::RetVal
SpecificWorker::process_state(const RoboCompLidar3D::TPoints &data,
                              const Corners &corners, const Match &match,
                              AbstractGraphicViewer *viewer) {
  // State machine for robot navigation
  switch (state) {
  case STATE::IDLE:
    if (debug_runtime)
      qInfo() << "[IDLE] In IDLE state. auto_nav:" << auto_nav_sequence_running
              << "has_target:" << target_room_center.has_value()
              << "localised:" << localised;
    // Check if we should transition to GOTO_ROOM_CENTER
    if (auto_nav_sequence_running && target_room_center.has_value()) {
      qInfo() << "State transition: IDLE -> GOTO_ROOM_CENTER";
      return goto_room_center(data);
    } // Stay in IDLE, do nothing
    return {STATE::IDLE, 0.0f, 0.0f};

  case STATE::GOTO_ROOM_CENTER: {
    auto [next_s, v, w] = goto_room_center(data);
    if (next_s == STATE::IDLE &&
        target_room_center.has_value()) // Reached center (and we had a target)
    {
      // If room is already recognized, skip TURN and go to GOTO_DOOR
      if (room_recognized[current_room_index]) {
        qInfo() << "State transition: GOTO_ROOM_CENTER -> GOTO_DOOR (room recognized)";
        return {STATE::GOTO_DOOR, 0.0f, 0.0f};
      }
      qInfo() << "State transition: GOTO_ROOM_CENTER -> TURN";
      return {STATE::TURN, 0.0f, 0.0f};
    }
    return {next_s, v, w};
  }

  case STATE::TURN: {
    auto [next_s, v, w] = turn(corners);
    if (next_s == STATE::IDLE) {
      qInfo() << "State transition: TURN -> GOTO_DOOR";



      return {STATE::GOTO_DOOR, 0.0f, 0.0f};
    }
    return {next_s, v, w};
  }

  case STATE::GOTO_DOOR:
    return goto_door(data);

  case STATE::ORIENT_TO_DOOR:
    return orient_to_door(data);

  case STATE::CROSS_DOOR:
    return cross_door(data);

  // Other states can be added here in the future
  case STATE::LOCALISE:
  default:
    qWarning() << "Unimplemented state:" << to_string(state)
               << ", returning to IDLE";
    return {STATE::IDLE, 0.0f, 0.0f};
  }
}

/**************************************/
// From the RoboCompLidar3D you can call this methods:
// RoboCompLidar3D::TData this->lidar3d_proxy->getLidarData(string name, float
// start, float len, int decimationDegreeFactor) RoboCompLidar3D::TDataImage
// this->lidar3d_proxy->getLidarDataArrayProyectedInImage(string name)
// RoboCompLidar3D::TDataCategory
// this->lidar3d_proxy->getLidarDataByCategory(TCategories categories, long
// timestamp) RoboCompLidar3D::TData
// this->lidar3d_proxy->getLidarDataProyectedInImage(string name)
// RoboCompLidar3D::TData
// this->lidar3d_proxy->getLidarDataWithThreshold2d(string name, float distance,
// int decimationDegreeFactor)

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory

/**************************************/
// From the RoboCompOmniRobot you can call this methods:
// RoboCompOmniRobot::void this->omnirobot_proxy->correctOdometer(int x, int z,
// float alpha) RoboCompOmniRobot::void this->omnirobot_proxy->getBasePose(int
// x, int z, float alpha) RoboCompOmniRobot::void
// this->omnirobot_proxy->getBaseState(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->resetOdometer()
// RoboCompOmniRobot::void
// this->omnirobot_proxy->setOdometer(RoboCompGenericBase::TBaseState state)
// RoboCompOmniRobot::void this->omnirobot_proxy->setOdometerPose(int x, int z,
// float alpha) RoboCompOmniRobot::void
// this->omnirobot_proxy->setSpeedBase(float advx, float advz, float rot)
// RoboCompOmniRobot::void this->omnirobot_proxy->stopBase()

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams


