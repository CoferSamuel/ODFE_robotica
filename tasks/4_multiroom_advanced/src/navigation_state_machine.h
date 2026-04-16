#ifndef NAVIGATION_STATE_MACHINE_H
#define NAVIGATION_STATE_MACHINE_H

#include <tuple>
#include <optional>
#include <chrono>
#include <Eigen/Dense>
#include <memory>
#include <map>
#include <fstream>
#include <iostream>
#include <QDebug>

// RoboComp includes (forward declarations where possible)
#include <Lidar3D.h>

// Project specific types
#include "room_detector.h"
#include "door_detector.h"
#include "localiser/localiser.h"
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"

// Forward declaration
class SpecificWorker;

class NavigationStateMachine {
public:
    enum class State { IDLE, GOTO_DOOR, ORIENT_TO_DOOR, LOCALISE, GOTO_ROOM_CENTER, TURN, CROSS_DOOR };
    
    using RetVal = std::tuple<State, float, float>;

    NavigationStateMachine(SpecificWorker* worker);
    ~NavigationStateMachine() = default;

    RetVal process_state(const RoboCompLidar3D::TPoints &data, const Corners &corners, const Match &match, AbstractGraphicViewer *viewer);
    
    // Helper to convert state to string
    static const char* to_string(const State s);

    // Access current state
    State get_current_state() const { return current_state; }
    void set_current_state(State s) { current_state = s; }

    // Navigation targets management
    void set_target_room_center(std::optional<Eigen::Vector2d> target) { target_room_center = target; }
    std::optional<Eigen::Vector2d> get_target_room_center() const { return target_room_center; }
    
    void set_target_door_point(std::optional<Eigen::Vector2f> target) { target_door_point = target; }
    std::optional<Eigen::Vector2f> get_target_door_point() const { return target_door_point; }
    
    void set_orient_target_angle(std::optional<float> angle) { orient_target_angle = angle; }
    std::optional<float> get_orient_target_angle() const { return orient_target_angle; }
    
    void set_auto_nav_sequence_running(bool val) { auto_nav_sequence_running = val; }
    bool is_auto_nav_sequence_running() const { return auto_nav_sequence_running; }

    std::tuple<float, float> robot_controller(const Eigen::Vector2f &target);

private:
    SpecificWorker* worker;
    State current_state = State::IDLE;

    // Navigation state variables (moved from SpecificWorker)
    std::optional<Eigen::Vector2d> target_room_center;
    std::optional<Eigen::Vector2f> target_door_point;
    std::optional<float> orient_target_angle;
    std::optional<std::chrono::steady_clock::time_point> cross_door_start_time;
    bool auto_nav_sequence_running = true;

    // Internal logic methods (extracted from SpecificWorker)
    RetVal goto_room_center(const RoboCompLidar3D::TPoints &points);
    RetVal turn(const Corners &corners);
    RetVal goto_door(const RoboCompLidar3D::TPoints &points);
    RetVal orient_to_door(const RoboCompLidar3D::TPoints &points);
    RetVal cross_door(const RoboCompLidar3D::TPoints &points);
    void switch_room();
};

#endif // NAVIGATION_STATE_MACHINE_H
