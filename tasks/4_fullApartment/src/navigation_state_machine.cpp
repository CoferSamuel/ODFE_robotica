#include "navigation_state_machine.h"
#include "specificworker.h"

// Define to avoid circular dependency issues if needed, strictly used in cpp
// Accessing SpecificWorker members requires definition of SpecificWorker

NavigationStateMachine::NavigationStateMachine(SpecificWorker* worker) : worker(worker) {
}

const char* NavigationStateMachine::to_string(const State s) {
    switch(s) {
        case State::IDLE:               return "IDLE";
        case State::LOCALISE:           return "LOCALISE";
        case State::GOTO_DOOR:          return "GOTO_DOOR";
        case State::TURN:               return "TURN";
        case State::ORIENT_TO_DOOR:     return "ORIENT_TO_DOOR";
        case State::GOTO_ROOM_CENTER:   return "GOTO_ROOM_CENTER";
        case State::CROSS_DOOR:         return "CROSS_DOOR";
        default:                        return "UNKNOWN";
    }
}

NavigationStateMachine::RetVal
NavigationStateMachine::process_state(const RoboCompLidar3D::TPoints &data,
                              const Corners &corners, const Match &match,
                              AbstractGraphicViewer *viewer) {
  // State machine for robot navigation
  switch (current_state) {
  case State::IDLE:
    if (worker->debug_runtime)
      qInfo() << "[IDLE] In IDLE state. auto_nav:" << auto_nav_sequence_running
              << "has_target:" << target_room_center.has_value()
              << "localised:" << worker->localised;
    // Check if we should transition to GOTO_ROOM_CENTER
    if (auto_nav_sequence_running && target_room_center.has_value()) {
      qInfo() << "State transition: IDLE -> GOTO_ROOM_CENTER";
      return goto_room_center(data);
    } // Stay in IDLE, do nothing
    return {State::IDLE, 0.0f, 0.0f};

  case State::GOTO_ROOM_CENTER: {
    auto [next_s, v, w] = goto_room_center(data);
    if (next_s == State::IDLE &&
        target_room_center.has_value()) // Reached center (and we had a target)
    {
      // If room is already recognized, skip TURN and go to GOTO_DOOR
      if (worker->room_recognized[worker->current_room_index]) {
        qInfo() << "State transition: GOTO_ROOM_CENTER -> GOTO_DOOR (room recognized)";
        return {State::GOTO_DOOR, 0.0f, 0.0f};
      }
      qInfo() << "State transition: GOTO_ROOM_CENTER -> TURN";
      return {State::TURN, 0.0f, 0.0f};
    }
    return {next_s, v, w};
  }

  case State::TURN: {
    auto [next_s, v, w] = turn(corners);
    if (next_s == State::IDLE) {
      qInfo() << "State transition: TURN -> GOTO_DOOR";
      return {State::GOTO_DOOR, 0.0f, 0.0f};
    }
    return {next_s, v, w};
  }

  case State::GOTO_DOOR:
    return goto_door(data);

  case State::ORIENT_TO_DOOR:
    return orient_to_door(data);

  case State::CROSS_DOOR:
    return cross_door(data);

  // Other states can be added here in the future
  case State::LOCALISE:
  default:
    qWarning() << "Unimplemented state:" << to_string(current_state)
               << ", returning to IDLE";
    return {State::IDLE, 0.0f, 0.0f};
  }
}

NavigationStateMachine::RetVal
NavigationStateMachine::goto_room_center(const RoboCompLidar3D::TPoints &points) {

  // Check if we have a valid target
  if (!target_room_center.has_value()) {
    if (worker->debug_runtime)
      qInfo() << "[GOTO_ROOM_CENTER] No target available, returning to IDLE";
    worker->omnirobot_proxy->setSpeedBase(0, 0, 0);
    return {State::IDLE, 0.0f, 0.0f};
  }

  const auto &target = target_room_center.value();

  // 1. Calculate distance
  float d = target.norm();
  
  // 2. Check if we've arrived at the target
  if (d < 100.f) {
    if (worker->debug_runtime)
      qInfo() << "[GOTO_ROOM_CENTER] Target reached! d=" << d << "mm";
    target_room_center.reset();
    worker->omnirobot_proxy->setSpeedBase(0, 0, 0);
    
    // Skip TURN for recognized rooms - go directly to GOTO_DOOR
    if (worker->room_recognized[worker->current_room_index]) {
      if (worker->debug_runtime)
        qInfo() << "[GOTO_ROOM_CENTER] Room already recognized, skipping TURN";
      return {State::GOTO_DOOR, 0.0f, 0.0f};
    }
    return {State::TURN, 0.0f, 0.0f};
  }
  auto [v, omega] = robot_controller(target.cast<float>());

  if (worker->debug_runtime)
    qInfo() << "[GOTO_ROOM_CENTER] Target:" << target.x() << "," << target.y()
            << "Dist:" << d << "mm. V:" << v << "mm/s, W:" << omega << "rad/s";
  worker->omnirobot_proxy->setSpeedBase(0, v, omega);

  return {State::GOTO_ROOM_CENTER, v, omega};
}

NavigationStateMachine::RetVal NavigationStateMachine::turn(const Corners &corners) {
  bool detected = false;
  int direction = 1;
  float rot_speed = 0.8f;
  
  // Accumulate doors during rotation to capture all doors from different angles
  auto current_doors = worker->door_detector.doors();
  for (const auto& door : current_doors) {
    Eigen::Vector2d center_robot(door.center().x(), door.center().y());
    Eigen::Vector2d center_world = worker->robot_pose * center_robot;
    
    // Check if this door is already in accumulated list (by proximity)
    bool is_new = true;
    for (const auto& existing : worker->accumulated_doors_for_room) {
        Eigen::Vector2f existing_center = (existing.p1 + existing.p2) / 2.f;
        if ((existing_center - center_world.cast<float>()).norm() < 500.f) {
            is_new = false;
            break;
        }
    }
    
    if (is_new) {
        // Transform and add to accumulated list
        Eigen::Vector2d p1_world = worker->robot_pose * Eigen::Vector2d(door.p1.x(), door.p1.y());
        Eigen::Vector2d p2_world = worker->robot_pose * Eigen::Vector2d(door.p2.x(), door.p2.y());
        
        // Align to wall
        Eigen::Vector2f center_f = center_world.cast<float>();
        const auto& room = worker->nominal_rooms[worker->current_room_index];
        const auto wall = room.get_closest_wall_to_point(center_f);
        const auto& wall_line = std::get<0>(wall);
        
        Eigen::Vector2f p1_aligned = wall_line.projection(p1_world.cast<float>());
        Eigen::Vector2f p2_aligned = wall_line.projection(p2_world.cast<float>());
        
        Door world_door(p1_aligned, door.p1_angle, p2_aligned, door.p2_angle);
        worker->accumulated_doors_for_room.push_back(world_door);
        
        if (worker->debug_runtime)
          qInfo() << "[TURN] Accumulated new door. Total:" << worker->accumulated_doors_for_room.size();
    }
  }
  
  if (worker->debug_runtime)
    qInfo() << "[TURN] Searching for digit panel (MNIST)...";
  
  auto [det, dir] = rc::ImageProcessor::check_color_patch_in_image(worker->camera360rgb_proxy, rc::ImageProcessor::Color::RED, nullptr, 200);
  direction = dir;
  
  int detected_digit = -1;
  try {
      // Call MNIST component (no image argument needed, component captures its own)
      auto result = worker->mnist_proxy->getNumber();
      
      if (result.val != -1 && result.confidence > 0.6 && det) {
          detected = true;
          detected_digit = result.val;
          if (worker->debug_runtime)
            qInfo() << "[TURN] MNIST DETECTED! Digit:" << result.val << "Conf:" << result.confidence;
      } else {
          detected = false;
          // Continue scanning
      }
      
  } catch (const Ice::Exception &e) {
      if (worker->debug_runtime) qInfo() << "[TURN] MNIST Proxy Error:" << e.what();
  }

  if (worker->debug_runtime)
    qInfo() << "[TURN] Detected: " << detected;

  // Logic to handle detection
  if (detected) {
    if (worker->debug_runtime)
      qInfo() << "[TURN] Panel centered! Stopping.";
    worker->badge_found = true;
    
    // Update 'current_room_index' to match detected digit immediately.
    // This allows execute_localiser() to use the correct room template in the next frame.
    if (detected_digit >= 0 && detected_digit < static_cast<int>(worker->nominal_rooms.size())) {
        worker->current_room_index = detected_digit;
        if (worker->debug_runtime) qInfo() << "[TURN] Set current_room_index to detected digit:" << worker->current_room_index;
    } else {
        if (worker->debug_runtime) qWarning() << "[TURN] Detected digit" << detected_digit << "out of nominal bounds. Keeping index:" << worker->current_room_index;
    }
    
    // Ensure status vector is large enough
    if (worker->current_room_index >= static_cast<int>(worker->room_recognized.size())) {
        worker->room_recognized.resize(worker->current_room_index + 1, false);
    }
    
    // Recognize room and add to graph if not already done
    if (!worker->room_recognized[worker->current_room_index]) {
      // Capture doors for this room
      worker->capture_doors_for_current_room();
      
      const auto& room = worker->nominal_rooms[worker->current_room_index];
      auto center = room.rect().center();  
      int room_node_id = worker->topology_graph->add_room(
          "Room_" + std::to_string(worker->current_room_index),
          worker->current_room_index,
          room.width, room.length,
          Eigen::Vector2f(center.x(), center.y()));

      // Add doors to graph and connect to room
      for (size_t i = 0; i < room.doors.size(); ++i) {
        const auto& door = room.doors[i];
        int door_node_id = worker->topology_graph->add_door(
            "Door_R" + std::to_string(worker->current_room_index) + "_" + std::to_string(i),
            door.p1, door.p2);
        worker->topology_graph->connect(room_node_id, door_node_id);
      }
      
      // Set entry door by proximity (find door closest to where robot entered)
      worker->set_entry_door_by_proximity();
      
      worker->room_recognized[worker->current_room_index] = true;
      
      // Store robot heading for future re-entry (breaks 180° ambiguity)
      float current_heading = std::atan2(worker->robot_pose.rotation()(1,0), worker->robot_pose.rotation()(0,0));
      worker->topology_graph->set_room_heading(worker->current_room_index, current_heading);
      
      if (worker->debug_runtime)
        qInfo() << "[TURN] Room" << worker->current_room_index << "recognized and added to graph";
    }
    
    worker->omnirobot_proxy->setSpeedBase(0, 0, 0);
    return {State::IDLE, 0.0f, 0.0f}; // Found and centered
  }
  // If not detected or not centered, rotate
  return {State::TURN, 0.0f, rot_speed * direction};
}

NavigationStateMachine::RetVal
NavigationStateMachine::goto_door(const RoboCompLidar3D::TPoints &points) {
  auto doors = worker->door_detector.doors();
  if (doors.empty()) {
    if (worker->debug_runtime)
      qInfo() << "[GOTO_DOOR] No door found. Rotating.";
    // Don't reset target_door_point here - keep navigating to last known target if we have one
    worker->omnirobot_proxy->setSpeedBase(0, 0, 0.2);
    return {State::GOTO_DOOR, 0.0f, 0.2f};
  }

  // Sort doors by angle (or some consistent metric) to ensure indices are
  // stable
  std::sort(doors.begin(), doors.end(), [](const auto &a, const auto &b) {
    return a.direction() < b.direction();
  });

  int selected_index = 0;

  // Use graph-based selection for recognized rooms
  if (worker->room_recognized[worker->current_room_index]) {
    // Only select once (sticky) - reuse until door is crossed
    if (worker->selected_graph_door_id < 0) {
      worker->selected_graph_door_id = worker->select_door_from_graph();
    }
    
    if (worker->selected_graph_door_id >= 0) {
      // Get door position from graph
      const auto& door_node = worker->topology_graph->get_node(worker->selected_graph_door_id);
      Eigen::Vector2d door_world_pos = door_node.position.cast<double>();
      
      // Find the detected door closest to graph door position
      float min_dist = std::numeric_limits<float>::max();
      for (size_t i = 0; i < doors.size(); ++i) {
        Eigen::Vector2d detected_world = worker->robot_pose * doors[i].center().cast<double>();
        float dist = (detected_world - door_world_pos).norm();
        if (dist < min_dist) {
          min_dist = dist;
          selected_index = static_cast<int>(i);
        }
      }
      
      // Set chosen_door_world_pos for sticky tracking
      worker->chosen_door_world_pos = worker->robot_pose * doors[selected_index].center().cast<double>();
      
      // Explicitly set traversing_door_id for the sticky case
      worker->traversing_door_id = worker->selected_graph_door_id;
      
      if (worker->debug_runtime)
        qInfo() << "[GOTO_DOOR] Graph selected door" << worker->selected_graph_door_id
                << "matched to index" << selected_index;
      
      // Skip the room-specific logic below
      goto navigate_to_door;
    }
  }

  // Fallback: Room 0: Random selection, track by world position
  if (worker->current_room_index == 0) {
    if (!worker->chosen_door_world_pos.has_value()) {
      // First time: randomly select
      if (doors.size() > 1) {
        std::uniform_int_distribution<> dist(0, static_cast<int>(doors.size()) - 1);
        selected_index = dist(worker->rd);
      } else {
        selected_index = 0;
      }
      // Save world position and left/right for Room 1
      worker->chosen_door_world_pos = worker->robot_pose * doors[selected_index].center().cast<double>();
      worker->chosen_door_was_on_left = (doors[selected_index].center().x() < 0);
      if (worker->debug_runtime)
        qInfo() << "[GOTO_DOOR] Room 0: Selected door" << selected_index 
                << "x=" << doors[selected_index].center().x()
                << "world=(" << worker->chosen_door_world_pos.value().x() << "," 
                << worker->chosen_door_world_pos.value().y() << ")";
    } else {
      // Sticky: find door closest to saved world position
      float min_dist = std::numeric_limits<float>::max();
      for (size_t i = 0; i < doors.size(); ++i) {
        Eigen::Vector2d door_world = worker->robot_pose * doors[i].center().cast<double>();
        float dist = (door_world - worker->chosen_door_world_pos.value()).norm();
        if (dist < min_dist) {
          min_dist = dist;
          selected_index = i;
        }
      }
      // Update saved position for next frame
      worker->chosen_door_world_pos = worker->robot_pose * doors[selected_index].center().cast<double>();
      if (worker->debug_runtime)
        qInfo() << "[GOTO_DOOR] Room 0: Sticky selection, door" << selected_index
                << "world_dist=" << min_dist;
    }
  }
  // Room 1: SAME side (coordinates are mirrored because badges are on opposite walls)
  else {
    if (!worker->chosen_door_world_pos_room1.has_value()) {
      // First frame: Select a door that is NOT the entry door (Exploration)
      // 1. Identify Entry Door (Closest to origin/robot start)
      int entry_index = -1;
      float min_dist_origin = std::numeric_limits<float>::max();
      
      for (size_t i = 0; i < doors.size(); ++i) {
          float d = doors[i].center().norm(); // Distance to (0,0)
          if (d < min_dist_origin) {
              min_dist_origin = d;
              entry_index = static_cast<int>(i);
          }
      }
      
      // 2. Pick a candidate that is DIFFERENT from entry_index
      std::vector<int> candidates;
      for (size_t i = 0; i < doors.size(); ++i) {
          if (static_cast<int>(i) != entry_index) {
              candidates.push_back(i);
          }
      }
      
      if (!candidates.empty()) {
          // Found exit doors! Pick one (e.g. random or first)
           std::uniform_int_distribution<> dist(0, static_cast<int>(candidates.size()) - 1);
           selected_index = candidates[dist(worker->rd)];
           if (worker->debug_runtime) qInfo() << "[GOTO_DOOR] Exploration: Chose candidate door" << selected_index << "(avoiding entry door" << entry_index << ")";
      } else {
          // Dead end? Go back.
          selected_index = entry_index;
          if (worker->debug_runtime) qInfo() << "[GOTO_DOOR] Dead End: Backtracking via entry door" << selected_index;
      }

      // Save world position for sticky tracking
      worker->chosen_door_world_pos_room1 = worker->robot_pose * doors[selected_index].center().cast<double>();

    } else if (worker->chosen_door_world_pos_room1.has_value()) {
      // Sticky: find door closest to saved world position
      float min_dist = std::numeric_limits<float>::max();
      for (size_t i = 0; i < doors.size(); ++i) {
        Eigen::Vector2d door_world = worker->robot_pose * doors[i].center().cast<double>();
        float dist = (door_world - worker->chosen_door_world_pos_room1.value()).norm();
        if (dist < min_dist) {
          min_dist = dist;
          selected_index = i;
        }
      }
      // Update saved position
      worker->chosen_door_world_pos_room1 = worker->robot_pose * doors[selected_index].center().cast<double>();
      if (worker->debug_runtime)
        qInfo() << "[GOTO_DOOR] Room 1: Sticky selection, door" << selected_index
                << "world_dist=" << min_dist;
    }
  }

  // Safety check
  if (selected_index >= static_cast<int>(doors.size()))
    selected_index = 0;



  navigate_to_door:
    // IMPORTANT: Identify the Graph Node ID for the selected door (Correct Placement)
    {
        int room_graph_id = worker->topology_graph->get_room_node_id(worker->current_room_index);
        if (room_graph_id >= 0 && !worker->nominal_rooms[worker->current_room_index].doors.empty()) {
            
            // 1. Transform selected physical door to World/Map Frame
            Eigen::Vector2d selected_world_pos = worker->robot_pose * doors[selected_index].center().cast<double>();
            
            // 2. Find the corresponding index in 'nominal_rooms' (which aligns with Graph Names)
            // Since we are in the same room, 'robot_pose' is accurate relative to 'nominal_rooms', so this is drift-safe.
            int matched_nominal_index = -1;
            float min_dist_nominal = std::numeric_limits<float>::max();
            
            const auto& nom_doors = worker->nominal_rooms[worker->current_room_index].doors;
            for (size_t k = 0; k < nom_doors.size(); ++k) {
                 Eigen::Vector2f nom_center = nom_doors[k].center();
                 float d = (nom_center.cast<double>() - selected_world_pos).norm();
                 if (d < min_dist_nominal) {
                     min_dist_nominal = d;
                     matched_nominal_index = static_cast<int>(k);
                 }
            }
            
            if (matched_nominal_index != -1) {
                // 3. Generate Name based on the MATCHED Nominal Index
                std::string target_name = "Door_R" + std::to_string(worker->current_room_index) + "_" + std::to_string(matched_nominal_index);
                
                int found_node_id = -1;
                for (int neighbor_id : worker->topology_graph->get_neighbors(room_graph_id)) {
                    const auto& node = worker->topology_graph->get_node(neighbor_id);
                    if (node.type == NodeType::DOOR && node.name == target_name) {
                        found_node_id = neighbor_id;
                        break; 
                    }
                }
                
                if (found_node_id != -1) {
                    worker->traversing_door_id = found_node_id;
                    if (worker->debug_runtime)
                      qInfo() << "[GOTO_DOOR] Mapped visible index" << selected_index 
                              << "to Nominal Index" << matched_nominal_index 
                              << "-> Graph ID" << worker->traversing_door_id;
                } else {
                    if (worker->debug_runtime) qInfo() << "[GOTO_DOOR] Name match failed for " << QString::fromStdString(target_name);
                }
            }
        }
    }

    const auto &door = doors[selected_index];
    Eigen::Vector2f target =
        door.center_before(Eigen::Vector2d(0, 0), worker->params.DOOR_APPROACH_DISTANCE);
    target_door_point = target;
    auto [v, w] = robot_controller(target);

  if (v == 0.0f && w == 0.0f) {
    if (worker->debug_runtime)
      qInfo() << "[GOTO_DOOR] Reached door approximation point.";

    // Update graph tracking


    auto_nav_sequence_running = false;
    target_door_point.reset();
    worker->omnirobot_proxy->setSpeedBase(0, 0, 0);

    // Calculate vectors for orientation
    Eigen::Vector2f door_parallel = door.p2 - door.p1;
    Eigen::Vector2f door_normal =
        Eigen::Vector2f(-door_parallel.y(), door_parallel.x());
    Eigen::Vector2f target_vector = door_parallel;

    // Determine direction
    QPointF room_center_qpoint =
        worker->nominal_rooms[worker->current_room_index].rect().center();
    Eigen::Vector2d room_center_eigen(room_center_qpoint.x(),
                                      room_center_qpoint.y());
    Eigen::Vector2d world_inward_vector =
        room_center_eigen - worker->robot_pose.translation();
    Eigen::Vector2d robot_inward_vector =
        worker->robot_pose.rotation().inverse() * world_inward_vector;

    bool flipped = false;
    // Check stability using NORMAL vector.
    if (door_normal.cast<double>().dot(robot_inward_vector) > 0.0) {
      target_vector = -target_vector;
      flipped = true;
    }
    if (worker->debug_runtime) {
      qInfo() << "DEBUG: Flipped = " << (flipped ? "true" : "false");
      qInfo() << "DEBUG: Final target_vector = (" << target_vector.x() << ", "
              << target_vector.y() << ")";
    }

    // Calculate the angle of this desired vector in the robot's frame.
    const float target_angle_robot =
        std::atan2(target_vector.y(), target_vector.x());
    // Convert relative angle to world angle
    const double current_robot_angle =
        std::atan2(worker->robot_pose.rotation()(1, 0), worker->robot_pose.rotation()(0, 0));
    const double door_target_angle_world =
        current_robot_angle + target_angle_robot;
    orient_target_angle =
        atan2(sin(door_target_angle_world), cos(door_target_angle_world));
    if (worker->debug_runtime)
      qInfo() << "[GOTO_DOOR] Setting sticky world orientation target:"
              << orient_target_angle.value();

    return {State::ORIENT_TO_DOOR, 0.0f, 0.0f};
  }
  if (worker->debug_runtime)
    qInfo() << "[GOTO_DOOR] Door found. Target:" << target.x() << ","
            << target.y() << ". V:" << v << "mm/s, W:" << w << "rad/s";

  worker->omnirobot_proxy->setSpeedBase(v, 0, w);
  return {State::GOTO_DOOR, v, w};
}

NavigationStateMachine::RetVal
NavigationStateMachine::orient_to_door(const RoboCompLidar3D::TPoints &points) {
  // 1. Check for a sticky target
  if (!orient_target_angle.has_value()) {
    qWarning()
        << "ORIENT_TO_DOOR: No orientation target set. Transitioning to IDLE.";
    return {State::IDLE, 0.0f, 0.0f};
  }

  // 2. Calculate current rotational error
  double current_robot_angle =
      std::atan2(worker->robot_pose.rotation()(1, 0), worker->robot_pose.rotation()(0, 0));
  float rot_error = orient_target_angle.value() - current_robot_angle;
  rot_error = atan2(sin(rot_error), cos(rot_error)); // Normalize error

  // 3. State Transition Check
  const float angle_tolerance = 0.1f; // approx 5.7 degrees
  if (std::abs(rot_error) < angle_tolerance) {
    if (worker->debug_runtime)
      qInfo() << "[ORIENT_TO_DOOR] Aligned! Transitioning to CROSS_DOOR.";
    worker->omnirobot_proxy->setSpeedBase(0, 0, 0);
    orient_target_angle.reset(); // Clear the sticky target
    
    return {State::CROSS_DOOR, 0.0f, 0.0f};
  }

  // 4. Rotation Control
  const float rot_speed = 0.5f;
  float rot_velocity = -std::copysign(rot_speed, rot_error);

  if (worker->debug_runtime) {
    qInfo() << "[ORIENT_TO_DOOR] World Target:" << orient_target_angle.value()
            << "rad, Current World:" << current_robot_angle
            << "rad, Rotational Error:" << rot_error
            << "rad, Velocity:" << rot_velocity << "rad/s";
  }

  // Send command to robot (only rotation)
  worker->omnirobot_proxy->setSpeedBase(0, 0, rot_velocity);

  // Remain in the current state
  return {State::ORIENT_TO_DOOR, 0.0f, rot_velocity};
}

NavigationStateMachine::RetVal
NavigationStateMachine::cross_door(const RoboCompLidar3D::TPoints &points) {
  const float adv_speed = 400.0f; // mm/s

  // 1. Initialize start time if not set
  if (!cross_door_start_time.has_value()) {
    cross_door_start_time = std::chrono::steady_clock::now();
    if (worker->debug_runtime)
      qInfo() << "[CROSS_DOOR] Starting cross door maneuver (Timer-based).";
  }

  // 2. Calculate elapsed time
  auto now = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = now - cross_door_start_time.value();

  // 3. Required duration (7 seconds)
  double required_duration = 7.0;

  // 4. Check termination condition
  if (elapsed.count() >= required_duration) {
    if (worker->debug_runtime)
      qInfo() << "[CROSS_DOOR] Timer expired (" << elapsed.count()
              << "s). Transitioning to GOTO_ROOM_CENTER.";
    worker->omnirobot_proxy->setSpeedBase(0, 0, 0);
    cross_door_start_time.reset();
    switch_room();
    return {State::GOTO_ROOM_CENTER, 0.0f, 0.0f};
  }

  // 5. Move straight forward
  worker->omnirobot_proxy->setSpeedBase(adv_speed, 0, 0);

  if (worker->debug_runtime)
    qInfo() << "[CROSS_DOOR] Crossing... Time: " << elapsed.count() << "/"
            << required_duration << " s";

  return {State::CROSS_DOOR, adv_speed, 0.0f};
}

void NavigationStateMachine::switch_room() {
  // 1. Update room index
  worker->current_room_index = (worker->current_room_index + 1) % worker->nominal_rooms.size();
  if (worker->debug_runtime)
    qInfo() << "[SWITCH_ROOM] Switching to room index: " << worker->current_room_index;

  // 2. Clear and redraw viewer_room
  worker->viewer_room->scene.clear();
  worker->room_door_items.clear();

  // Re-add robot to viewer_room (since clear removed it)
  auto [rr, re] = worker->viewer_room->add_robot(
      worker->params.ROBOT_WIDTH, worker->params.ROBOT_LENGTH, 0, 100, QColor("Blue"));
  worker->robot_room_draw = rr;

  // Draw axes at center (always visible)
  auto center = worker->nominal_rooms[worker->current_room_index].rect().center();
  worker->viewer_room->scene.addLine(center.x(), center.y(), center.x() + 350,center.y(), QPen(Qt::red, 20));
  worker->viewer_room->scene.addLine(center.x(), center.y(), center.x(), center.y() + 350, QPen(Qt::green, 20));

  // Draw walls only if room is already recognized
  if (worker->room_recognized[worker->current_room_index]) 
    worker->viewer_room->scene.addRect(worker->nominal_rooms[worker->current_room_index].rect(), QPen(Qt::black, 30));
  

  // 3. Reset robot pose and re-run Ultra-localiser for new entry position
  worker->robot_pose.setIdentity();
  worker->robot_pose.rotate(M_PI); 
  worker->initial_localisation_done = false; 
  
  // Initialize entry position to current pose (origin), so graph builder can find anchor door
  worker->room_entry_position = worker->robot_pose.translation().cast<float>();

  // 3b. Update graph: store previous door and check for learned connections
  worker->previous_traversed_door_id = worker->traversing_door_id;
  
  if (worker->previous_traversed_door_id != -1 && worker->topology_graph->is_room_in_graph(worker->current_room_index)) {
    // Check if we already have a learned connection to the new room
    int connected_door = -1;
    for (int neighbor : worker->topology_graph->get_neighbors(worker->previous_traversed_door_id)) {
        const auto& node = worker->topology_graph->get_node(neighbor);
        // If neighbor is a door belonging to the NEW room, we found our entry!
        std::string prefix = "Door_R" + std::to_string(worker->current_room_index) + "_";
        if (node.type == NodeType::DOOR && node.name.find(prefix) == 0) {
            connected_door = neighbor;
            break;
        }
    }

    if (connected_door != -1) {
        worker->topology_graph->set_entry_door(worker->current_room_index, connected_door);
        
        // Save the position of this door to hint the localiser
        const auto& dnode = worker->topology_graph->get_node(connected_door);
        worker->expected_restart_position = dnode.position;
        
        if (worker->debug_runtime)
          qInfo() << "[ENTRY_DOOR] Found learned connection:" << worker->previous_traversed_door_id 
                  << "<->" << connected_door << ". Set entry door immediately. Hint pos: "
                  << worker->expected_restart_position.value().x() << "," << worker->expected_restart_position.value().y();
        
        worker->previous_traversed_door_id = -1; // Connection found, no need to learn again
    } else {
        worker->expected_restart_position.reset(); // No hint available yet (unless proximity finds one)
        if (worker->debug_runtime)
          qInfo() << "[SWITCH_ROOM] No learned connection from door" << worker->previous_traversed_door_id 
                  << "yet. Trying to learn via proximity now.";
        
        // Try to identify entry door immediately if doors exist in graph
        worker->set_entry_door_by_proximity();
    }
  }
  
  worker->traversing_door_id = -1;  // Reset
  worker->selected_graph_door_id = -1;  // Reset sticky door selection for new room

  // 4. Reset badge_found status for the new room
  worker->badge_found = false;

  // 5. If we are back to Room 0, reset the door memory so we choose randomly
  // again next time
  if (worker->current_room_index == 0) {
    worker->chosen_door_was_on_left.reset();
    worker->chosen_door_world_pos.reset();
    worker->chosen_door_world_pos_room1.reset();
    if (worker->debug_runtime)
      qInfo() << "[SWITCH_ROOM] Resetting door memory for new cycle.";
  }
}

std::tuple<float, float>
NavigationStateMachine::robot_controller(const Eigen::Vector2f &target) {
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
