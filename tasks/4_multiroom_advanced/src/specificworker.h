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

/**
    \brief
    @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H


// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
//#define HIBERNATION_ENABLED

#include "navigation_state_machine.h"

#include <genericworker.h>
#include <opencv2/opencv.hpp>
#include "abstract_graphic_viewer/abstract_graphic_viewer.h"
#include <random>
#include "time_series_plotter.h"
#include <QTimer>
#include <QLabel>
#include <memory>


#ifdef emit // To avoid problems with the emit keyword defined in Qt.
#undef emit // To avoid problems with the emit keyword defined in Qt
#endif
#include <tuple>
#include <utility>
#include <vector>
#include <limits>
#include "room_detector.h"
#include "door_detector.h"
#include "image_processor.h"
#include "nominal_room.h"
#include "graph.h"
#include "localiser/localiser.h"

class SpecificWorker final : public GenericWorker
{
    Q_OBJECT
    friend class NavigationStateMachine; // Allow access to private members
    
    public:
        // ===================================
        // LIFECYCLE & CORE
        // ===================================
        SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        static SpecificWorker* instance;

        // ===================================
        // UTILS & CONFIG
        // ===================================
        // Enable/disable runtime debug logging
        void set_debug_runtime(bool val) { debug_runtime = val; }

    public slots:
        // ===================================
        // SLOTS (RoboComp Interface)
        // ===================================
        void initialize();
        void new_target_slot(QPointF target);
        void compute();
        void emergency();
        void restore();
        int startup_check();

    private:
        // ===================================
        // PARAMETERS
        // ===================================
        struct Params
        {
            float ROBOT_WIDTH = 460;  // mm
            float ROBOT_LENGTH = 480;  // mm
            QRectF GRID_MAX_DIM{-5000, 2500, 10000, -5000};
            float DOOR_APPROACH_DISTANCE = 1000.f;
        };
        Params params;

        // ===================================
        // STATE MACHINE
        // ===================================
    public: 
        // Using State from NavigationStateMachine
        using STATE = NavigationStateMachine::State;
        
        inline const char* to_string(const STATE s) const {
            return NavigationStateMachine::to_string(s);
        }
        
        // Log files map
        std::map<STATE, std::shared_ptr<std::ofstream>> log_files;
        
        // Proxy for current state (optional, or access via sm)
        STATE state = STATE::IDLE;
    
    private:
        // Navigation State Machine Instance
        std::unique_ptr<NavigationStateMachine> navigation_sm;

        // State Handlers are now in NavigationStateMachine

        // ===================================
        // NAVIGATION & CONTROL
        // ===================================
        void move_robot(float adv, float rot, float max_match_error);

        // ===================================
        // PERCEPTION (LiDAR, Vision, Features)
        // ===================================
        // Data & Processors
        RoboCompLidar3D::TData data;
        Corners corners;
        rc::Room_Detector room_detector;
        DoorDetector door_detector;
        
        // Filtering Methods
        std::optional<RoboCompLidar3D::TPoints> filter_min_distance_cppitertools(const RoboCompLidar3D::TPoints &points);
        RoboCompLidar3D::TPoints filter_data_basic(const RoboCompLidar3D::TPoints &data);
        RoboCompLidar3D::TPoints filter_isolated_points(const RoboCompLidar3D::TPoints &points, float d);
        std::optional<cv::Rect> detect_frame(const cv::Mat& img);

        // ===================================
        // LOCALIZATION
        // ===================================
        void execute_localiser();
        
        Localiser localiser;
        Eigen::Affine2d robot_pose;
        Match last_matched;
        float last_match_error = std::numeric_limits<float>::infinity();
        
        bool localised = false;
        bool initial_localisation_done = false;
        std::optional<RoboCompGenericBase::TBaseState> last_odom_state; // Odometry integration
        std::vector<bool> room_recognized; // Badge detection status
        bool badge_found = false;
        bool search_green = false;

        // ===================================
        // MAPPING & TOPOLOGY (Rooms & Doors)
        // ===================================
        void capture_doors_for_current_room();
        int  select_door_from_graph();
        void set_entry_door_by_proximity();
        
        std::vector<NominalRoom> nominal_rooms{ NominalRoom{5500.f, 4000.f}, NominalRoom{8000.f, 4000.f}};
        int current_room_index = 0;
        std::unique_ptr<Graph> topology_graph;
        
        // Topology State
        int traversing_door_id = -1;
        int previous_traversed_door_id = -1;
        int selected_graph_door_id = -1;
        std::optional<Eigen::Vector2f> room_entry_position;
        std::optional<Eigen::Vector2f> expected_restart_position;
        std::vector<Door> accumulated_doors_for_room;
        std::map<int, int> last_exit_door_for_room;
        
        // Legacy/Special logic vars
        std::optional<bool> chosen_door_was_on_left;
        std::optional<Eigen::Vector2d> chosen_door_world_pos;
        std::optional<Eigen::Vector2d> chosen_door_world_pos_room1;

        // ===================================
        // VISUALIZATION
        // ===================================
        void draw_mainViewer();
        void draw_lidar(const RoboCompLidar3D::TPoints &filtered_points, std::optional<Eigen::Vector2d> center, QGraphicsScene *scene);
        void draw_topology_graph(QGraphicsScene *scene);
        void draw_room_center(const Eigen::Vector2d &center, QGraphicsScene *scene);
        void draw_room_doors(const std::vector<Door> &doors, QGraphicsScene *scene);
        void draw_door_target(const std::optional<Eigen::Vector2f> &target, QGraphicsScene *scene);
        void draw_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene* scene);

        AbstractGraphicViewer *viewer, *viewer_room, *viewer_graph;
        QGraphicsPolygonItem *robot_draw, *robot_room_draw;
        
        std::vector<QGraphicsItem *> room_door_items;
        std::map<int, Eigen::Vector2f> graph_vis_positions;

        // ===================================
        // INTERNAL UTILS
        // ===================================
        bool startup_check_flag;
        bool debug_runtime = false;
        std::random_device rd;
        
        // Plotting / Logging
        std::unique_ptr<TimeSeriesPlotter> time_series_plotter;
        int match_error_graph;

    signals:
        //void customSignal();
};

#endif