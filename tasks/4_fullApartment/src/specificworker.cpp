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
    : GenericWorker(configLoader, tprx),
      navigation_sm(std::make_unique<NavigationStateMachine>(this)) {
  // Save singleton instance for global access if necessary
  SpecificWorker::instance = this;
  this->startup_check_flag = startup_check;

  // If startup check is requested, execute it immediately
  if (this->startup_check_flag) {
    this->startup_check();
  } else {
    // Check if hibernation is enabled (power saving)
  #ifdef HIBERNATION_ENABLED
      hibernationChecker.start(500);
  #endif

    // Configure and start the Qt state machine
    statemachine.setChildMode(QState::ExclusiveStates);
    statemachine.start();

    // Check for errors when starting the state machine
    auto error = statemachine.errorString();
    if (error.length() > 0) {
      qWarning() << error;
      throw error;
    }
  }
}

SpecificWorker::~SpecificWorker() {
  // Stop the robot for safety when destroying the worker
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

void SpecificWorker::emergency() {}

void SpecificWorker::restore() {}

int SpecificWorker::startup_check() {
  std::cout << "Startup check" << std::endl;
  QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
  return 0;
}


void SpecificWorker::compute() {
  if (debug_runtime)
    qInfo()
        << "--------------------------- Compute ---------------------------";

  // ----------------------------------------------------------------------
  // 1. VIEWER UPDATE
  // ----------------------------------------------------------------------
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

    localiser.update_pose(robot_room_draw, robot_pose);
  
  }

  // ----------------------------------------------------------------------
  // 2. LOCALISATION AND TOPOLOGICAL MAP
  // ----------------------------------------------------------------------
  // Run localisation logic
  execute_localiser();

  // Draw the topological graph
  draw_topology_graph(&viewer_graph->scene);


  // ----------------------------------------------------------------------
  // 3. STATE MACHINE AND CONTROL
  // ----------------------------------------------------------------------
  // Execute state machine using last matching results and command robot
  auto ret_val =
      navigation_sm->process_state(data.points, corners, last_matched, viewer);
  auto [st, adv, rot] = ret_val;
  // Update local state copy if needed, or rely on SM
  state = st;
  navigation_sm->set_current_state(st); // Sync state

  // Send velocities (movement independent of localisation as requested)
  move_robot(adv, rot, last_match_error);

  // ----------------------------------------------------------------------
  // 4. IMAGE PROCESSING (DIGIT RECOGNITION)
  // ----------------------------------------------------------------------
  // Get image from camera
  RoboCompCamera360RGB::TImage img;
  try {
    img = camera360rgb_proxy->getROI(-1, -1, -1, -1, -1, -1);
  } catch (const Ice::Exception &e) {
    std::cout << e.what() << " Error reading 360 camera " << std::endl;
  }

  if (img.width > 0 && img.height > 0 && !img.image.empty()) {
    // convert to cv::Mat
    cv::Mat cv_img(img.height, img.width, CV_8UC3, img.image.data());

    // Convert BGR -> RGB for display
    cv::Mat display_img;
    cv::cvtColor(cv_img, display_img, cv::COLOR_BGR2RGB);

    // Only perform number detection and remote calls when in TURN state to save CPU
    if (state == STATE::TURN) {
        // Detect the red frame on the number plate
        auto rect_opt = detect_frame(display_img);

        if (rect_opt.has_value()) {
            // Draw the green frame on the number plate
             cv::Rect r = rect_opt.value();
             cv::rectangle(display_img, r, cv::Scalar(0, 255, 0), 2);
             
             // Get number from proxy
             try { 
                 // Draw the number and confidence on the number plate 
                 auto digit = mnist_proxy->getNumber();
                 if (digit.val != -1) {
                      std::string text = "Num: " + std::to_string(digit.val) + " (" + std::to_string(digit.confidence).substr(0,4) + ")";
                      cv::putText(display_img, text, cv::Point(r.x, r.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
                 }
             } catch (...) {}
        }
    }

    QImage qimg(display_img.data, display_img.cols, display_img.rows,
                static_cast<int>(display_img.step), QImage::Format_RGB888);
    label_img->setPixmap(QPixmap::fromImage(qimg).scaled(
        label_img->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }

  // ----------------------------------------------------------------------
  // 5. UI AND ODOMETRY UPDATE
  // ----------------------------------------------------------------------
  // Update UI
  RoboCompGenericBase::TBaseState bState;
  omnirobot_proxy->getBaseState(bState);

  // --- Odometry Integration (Dead Reckoning) ---
  // Always integrate odometry to keep robot_pose alive even when localiser is off.
  if (last_odom_state.has_value()) {
      float dx = bState.x - last_odom_state.value().x;
      float dy = bState.z - last_odom_state.value().z; // Z is Y in 2D
      float da = bState.alpha - last_odom_state.value().alpha;       
      float current_odom_angle = last_odom_state.value().alpha;
      float c = cos(-current_odom_angle);
      float s = sin(-current_odom_angle);
      float dx_r = dx * c - dy * s;
      float dy_r = dx * s + dy * c;
      
      // Apply to our tracking pose
      robot_pose.translate(Eigen::Vector2d(dx_r, dy_r));
      robot_pose.rotate(da);
  }
  last_odom_state = bState;


  // Display Robot Pose (Room Coordinates) instead of raw Odometry
  lcdNumber_x->display(robot_pose.translation().x());
  lcdNumber_y->display(robot_pose.translation().y());
  double da = std::atan2(robot_pose.rotation()(1, 0), robot_pose.rotation()(0, 0));
  lcdNumber_angle->display(qRadiansToDegrees(da));
  
  lcdNumber_adv->display(adv);
  lcdNumber_rot->display(rot);
  lcdNumber_room->display(current_room_index);
  label_state->setText(to_string(state));

  // Update Error Graph
  if (time_series_plotter) {
      float plot_val = last_match_error;
      if (std::isinf(plot_val)) plot_val = 10.0f; // Clamp max for plot
      
      time_series_plotter->addDataPoint(match_error_graph, plot_val);
      time_series_plotter->update();
  }
}


// -------------------------------------------------------------------
// -------------------------------------------------------------------
// ----------------------- STATE MACHINE FUNCTIONS -------------------
// -------------------------------------------------------------------
// -------------------------------------------------------------------


// Methods extracted to NavigationStateMachine
// process_state, goto_room_center, turn, goto_door, orient_to_door, cross_door, switch_room

void SpecificWorker::execute_localiser() {

  // Use graph heading (if available) as hint for grid search
  std::optional<float> ref_heading = std::nullopt;
  if (!initial_localisation_done && !corners.empty() && 
      room_recognized[current_room_index] && topology_graph->is_room_in_graph(current_room_index)) {
      ref_heading = topology_graph->get_room_heading(current_room_index);
  }

  // --- Process Localization via Localiser Class ---
  const auto& room = nominal_rooms[current_room_index];
  bool was_initial_done = initial_localisation_done;
  
  auto result = localiser.process(corners, room, robot_pose, 
                                  initial_localisation_done, 
                                  localised, 
                                  ref_heading, 
                                  expected_restart_position, // New arg: position hint from entry door
                                  debug_runtime);

  // Update State from Result
  localised = result.localised;
  last_matched = result.matches;
  last_match_error = result.max_match_error;

  // Handle Pose Update
  if (result.new_pose.has_value()) {
      robot_pose = result.new_pose.value();
  }
  
  // Capture entry position on first lock
  if (!was_initial_done && initial_localisation_done) {
      if (debug_runtime) qInfo() << "[LOCALISER] Initial localization done. Setting entry position.";
      room_entry_position = Eigen::Vector2f(robot_pose.translation().x(), robot_pose.translation().y());
  }
}
// -------------------------------------------------------------------
// ----------------------- MOVEMENT FUNCTIONS -----------------------
// -------------------------------------------------------------------

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

// -------------------------------------------------------------------
// ----------------------- DOOR CAPTURE -----------------------------
// -------------------------------------------------------------------

void SpecificWorker::capture_doors_for_current_room() {
  
  // Calculate the centroid of all accumulated doors to essentially find the "center" of the door arrangement.
  // This is used as a reference point for angular sorting.
  if (!accumulated_doors_for_room.empty()) {
      Eigen::Vector2f centroid(0.f, 0.f);
      for (const auto &d : accumulated_doors_for_room) {
          centroid += d.center();
      }
      centroid /= static_cast<float>(accumulated_doors_for_room.size());
      
      // Sort the doors based on their angle relative to the calculated centroid.
      // This ensures a deterministic order (e.g., counter-clockwise) for the doors in the list,
      // which allows for consistent identification and traversal order.
      std::sort(accumulated_doors_for_room.begin(), accumulated_doors_for_room.end(),
                [centroid](const Door &a, const Door &b) {
                    float angle_a = std::atan2(a.center().y() - centroid.y(), a.center().x() - centroid.x());
                    float angle_b = std::atan2(b.center().y() - centroid.y(), b.center().x() - centroid.x());
                    return angle_a < angle_b;
                });
  }

  // Store the sorted list of doors into the nominal room structure for the current room.
  nominal_rooms[current_room_index].doors = accumulated_doors_for_room;
  
  if (debug_runtime)
    qInfo() << "[DOORS] Captured" << nominal_rooms[current_room_index].doors.size()
            << "doors for room" << current_room_index << "(from accumulation)";
            
  // Clear the accumulation vector to prepare for detecting doors in the next room or next scan.
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
  
  // 3. Any explored non-entry door (Round Robin)
  candidates.clear();
  for (int door_id : topology_graph->get_neighbors(room_id)) {
    if (topology_graph->get_node(door_id).type == NodeType::DOOR) {
      // Room 0: Include all (including entry if it loops back)
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
    // Round-robin selection
    int selected = candidates[0];
    if (last_exit_door_for_room.count(current_room_index)) {
        int last_id = last_exit_door_for_room[current_room_index];
        // Find index of last_id
        auto it = std::find(candidates.begin(), candidates.end(), last_id);
        if (it != candidates.end()) {
            size_t index = std::distance(candidates.begin(), it);
            selected = candidates[(index + 1) % candidates.size()];
        }
    }
    
    // Update memory
    last_exit_door_for_room[current_room_index] = selected;

    if (debug_runtime)
      qInfo() << "[GRAPH_SELECT] Chose explored non-entry door" << selected << "(Round-Robin)";
    return selected;
  }
  
  // 4. Backtrack through entry door
  if (debug_runtime)
    qInfo() << "[GRAPH_SELECT] Backtracking via entry door" << entry_door;
  last_exit_door_for_room[current_room_index] = entry_door;
  return entry_door;
}

void SpecificWorker::set_entry_door_by_proximity() {
  // 1. Validate that current room is in the graph and we have an entry position recorded
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
  
  // 2. Iterate through neighbors (doors) of the current room to find the one closest to entry position
  for (int neighbor_id : topology_graph->get_neighbors(room_id)) {
    const auto& node = topology_graph->get_node(neighbor_id);
    if (node.type == NodeType::DOOR) {
      // Filter: Only consider doors belonging to THIS room (based on naming convention)
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
      
      // Update closest door
      if (dist < min_dist) {
        min_dist = dist;
        closest_door = neighbor_id;
      }
    }
  }
  
  // 3. If a valid closest door is found, register it and attempt to learn connections
  if (closest_door >= 0) {
    topology_graph->set_entry_door(current_room_index, closest_door);
    
    // Set localization hint: Robot should restart localization near this door
    const auto& dnode = topology_graph->get_node(closest_door);
    expected_restart_position = dnode.position;

    qInfo() << "[ENTRY_DOOR] Selected door" << closest_door 
            << "for room" << current_room_index << "(closest). Hint pos:" 
            << expected_restart_position.value().x() << "," << expected_restart_position.value().y();
            
    // 4. Connect previous room's exit door to this room's entry door
    if (previous_traversed_door_id != -1 && topology_graph->has_node(previous_traversed_door_id)) {
        // Validation: Ensure we are not connecting two doors of the SAME room (loopback)
        bool same_room = false;
        bool door_already_linked = false;

        auto prev_neighbors = topology_graph->get_neighbors(previous_traversed_door_id);
        auto curr_neighbors = topology_graph->get_neighbors(closest_door);
        
        // Check 4a: Are these doors already connected to the SAME room node?
        for (int p_neighbor : prev_neighbors) {
            for (int c_neighbor : curr_neighbors) {
                if (p_neighbor == c_neighbor) {
                    same_room = true; 
                    break;
                }
            }
            if (same_room) break;
        }

        // Check 4b: Door Exclusivity (Max degree 2: 1 Room + 1 Door)
        // Ensure neither door is already connected to ANOTHER door (topology constraint)
        
        // Check 'previous' door
        for (int p_neighbor : prev_neighbors) {
             if (topology_graph->get_node(p_neighbor).type == NodeType::DOOR) {
                 door_already_linked = true;
                 qInfo() << "[ENTRY_DOOR] Blocked: Previous door" << previous_traversed_door_id 
                         << "already linked to door" << p_neighbor;
                 break;
             }
        }
        // Check 'current' door
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

        // If validation passes, create the edge
        if (!same_room && !door_already_linked) {
            topology_graph->connect(previous_traversed_door_id, closest_door);
            qInfo() << "[ENTRY_DOOR] LEARNED connection:" << previous_traversed_door_id 
                    << "<->" << closest_door;
        } else {
            qInfo() << "[ENTRY_DOOR] Connection blocked. Same Room:" << same_room 
                    << "Already Linked:" << door_already_linked;
        }
        // Reset previous ID to prevent duplicate connection attempts
        previous_traversed_door_id = -1; 
    }
  }
  // Clear entry position buffer
  room_entry_position.reset();
}

// -------------------------------------------------------------------
// ----------------------- DRAWING FUNCTIONS -------------------------
// -------------------------------------------------------------------

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
    navigation_sm->set_target_room_center(center_opt.value()); // Store for controller
    draw_room_center(center_opt.value(), &viewer->scene);
  }

  draw_door_target(navigation_sm->get_target_door_point(), &viewer->scene);
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
  const float REPULSION_FORCE = 10000000.f; // Stronger repulsion spreading
  const float SPRING_FORCE = 0.08f;
  const float SPRING_LENGTH = 1000.f;  // Longer edges to allow space for nodes
  const float CENTER_FORCE = 0.02f;
  const int ITERATIONS = 100; // More iterations for stability
  const float MAX_DISPLACEMENT = 100.0f; // Limit movement per frame to prevent explosion

  // 1. Initialize new nodes
  for (const auto &[id, node] : topology_graph->get_nodes()) {
      if (graph_vis_positions.find(id) == graph_vis_positions.end()) {
          // New node
          Eigen::Vector2f init_pos(0,0);
          auto neighbors = topology_graph->get_neighbors(id);
          if (!neighbors.empty() && graph_vis_positions.count(neighbors[0])) {
               // Place near first neighbor with random offset
               Eigen::Vector2f offset = Eigen::Vector2f::Random().normalized() * SPRING_LENGTH;
               init_pos = graph_vis_positions[neighbors[0]] + offset;
          } else {
               // Random circular init
               float angle = (float(rand()) / RAND_MAX) * 2 * M_PI;
               init_pos = Eigen::Vector2f(std::cos(angle), std::sin(angle)) * 500.f;
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
              if (dist_sq < 100) dist_sq = 100; // Avoid singularity
              float dist = std::sqrt(dist_sq);
              
              forces[id1] += (diff / dist) * (REPULSION_FORCE / dist_sq);
          }
          
          // Spring (Connected nodes attract/repel to target length)
          for (int neighbor_id : topology_graph->get_neighbors(id1)) {
              if (graph_vis_positions.count(neighbor_id)) {
                  Eigen::Vector2f diff = graph_vis_positions[neighbor_id] - pos1;
                  float dist = diff.norm();
                  if(dist < 0.1f) dist = 0.1f; 
                  
                  // Use shorter spring for door-to-door connections (merging)
                  float target_length = SPRING_LENGTH;
                  if (topology_graph->get_node(id1).type == NodeType::DOOR && 
                      topology_graph->get_node(neighbor_id).type == NodeType::DOOR) {
                      target_length = SPRING_LENGTH / 4.f; // Much shorter for door pairs
                  }
                  
                  // Hooke's Law with Target Length
                  float displacement = dist - target_length;
                  forces[id1] += (diff / dist) * (displacement * SPRING_FORCE);
              }
          }
          
           // Center weak attraction to keep graph in view
          forces[id1] -= pos1 * CENTER_FORCE;
      }
      
      // Apply Forces with Limit
      for (auto &[id, pos] : graph_vis_positions) {
          Eigen::Vector2f displacement = forces[id];
          float disp_len = displacement.norm();
          if(disp_len > MAX_DISPLACEMENT) {
            displacement = displacement.normalized() * MAX_DISPLACEMENT;
          }
          pos += displacement; 
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


// -------------------------------------------------------------------
// ----------------------- FILTERING FUNCTIONS -----------------------
// -------------------------------------------------------------------


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

// Helper function to detect the red frame in the image 
// We need this method for print the green frame in the principal frame
std::optional<cv::Rect> SpecificWorker::detect_frame(const cv::Mat& img) {
    cv::Mat hsv;
    // Convert RGB to HSV for color-based segmentation
    cv::cvtColor(img, hsv, cv::COLOR_RGB2HSV); 

    cv::Mat mask1, mask2, mask;
    // Define HSV ranges for the red color
    // Red wraps around the hue value of 180, usually requiring two ranges:
    // Range 1: Lower red values
    cv::inRange(hsv, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), mask1);
    // Range 2: Upper red values
    cv::inRange(hsv, cv::Scalar(170, 70, 50), cv::Scalar(180, 255, 255), mask2);
    
    // Combine both masks to capture the full red spectrum
    cv::add(mask1, mask2, mask);
    
    // Apply morphological operations to clean up noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // Opening: Removes small noise spots
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    // Closing: Fills small holes within the detected objects
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    
    // Find contours in the processed mask
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    double max_area = 0;
    std::optional<cv::Rect> best_rect = std::nullopt;
    
    // Evaluate contours to find the most likely candidate for the frame
    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        // Desecard small contours
        if (area < 500) continue;
        
        // Get the bounding rectangle
        cv::Rect r = cv::boundingRect(cnt);
        float aspect = (float)r.width / (float)r.height;
        
        // Filter based on aspect ratio constraints
        if (aspect >= 0.5f && aspect <= 2.0f) {
             // Keep the largest valid contour
             if (area > max_area) {
                 max_area = area;
                 
                 // Apply margin to extract the inner region of interest
                 int mx = (int)(r.width * 0.15);
                 int my = (int)(r.height * 0.15);
                 best_rect = cv::Rect(r.x + mx, r.y + my, r.width - 2*mx, r.height - 2*my);
             }
        }
    }
    // Return the best detected region, or nullopt if none found
    return best_rect;
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



