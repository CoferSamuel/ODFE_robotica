//
// Created by pbustos on 2/12/22.
//
/* Purpose: Room feature extraction utilities. `Room_Detector` converts 2D
    points and line segments into room hypotheses: computes corners, filters
    lines, estimates room sizes and provides drawing helpers to visualize
    detected geometry in a `QGraphicsScene`. */

#ifndef FORCEFIELD_ROOM_DETECTOR_H
#define FORCEFIELD_ROOM_DETECTOR_H

#include <vector>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <QtCore>
#include <ranges>
#include "common_types.h"
#include <Lidar3D.h>
#include <QGraphicsScene>
#include "ransac_line_detector.h"

// Room_Detector class is a class that detects rooms in a 2D space. It uses a set of Hough lines to detect the rooms.
// The class has a method detect that receives a set of lines and returns a Room object.
// The class has a method compute_features that receives a set of lines and returns a tuple with the following features:
// 1. A set of lines
// 2. A set of parallel lines
// 3. A set of corners
// 4. A set of rooms

namespace rc   // aka RoboComp
{
    class Room_Detector
    {
        public:
            /**
             * @brief Compute room corners from a 2D polyline or set of edge points.
             * @param line 2D points (mm) describing a wall edge or contour.
             * @param scene Optional scene to visualise intermediate results.
             * @return Corners Ordered list of detected corners.
             */
            Corners compute_corners(const std::vector<Eigen::Vector2d> &line, QGraphicsScene *scene = nullptr);

            /**
             * @brief Compute room corners from 3D points projected on the ground plane.
             * @param line 3D points with (x,y,z) in mm; only x,y are used.
             * @param scene Optional scene to visualise intermediate results.
             * @return Corners Ordered list of detected corners.
             */
            Corners compute_corners(const std::vector<Eigen::Vector3d> &line, QGraphicsScene *scene = nullptr);

            /**
             * @brief Compute room corners directly from RoboComp Lidar2D-style points.
             * @param points Lidar points (x,y,r,phi) with planar coordinates in mm.
             * @param scene Optional scene to visualise intermediate results.
             * @return Corners Ordered list of detected corners.
             */
            Corners compute_corners(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene = nullptr);

            /**
             * @brief Estimate room sizes from a detected center and floor edges.
             * @param room_center Estimated room center (x,y in mm).
             * @param floor_line_cart Floor boundary as 2D points (mm).
             * @return Eigen::Vector3d Width, length, and orientation (rad) estimate.
             */
            Eigen::Vector3d estimate_room_sizes(const Eigen::Vector2d &room_center, std::vector<Eigen::Vector2d> &floor_line_cart);

            /**
             * @brief Extract corner candidates from a set of line segments.
             * @param elines Input line segments.
             * @return Corners List of corners derived from line intersections.
             */
            Corners get_corners(Lines &elines);

            /**
             * @brief Keep only lines whose length exceeds a given threshold.
             * @param lines Input line segments.
             * @param threshold Minimum length in millimetres.
             * @return Lines Filtered line segments.
             */
            Lines filter_lines_by_length(const Lines &lines, float threshold);

            // aux
            /**
             * @brief Euclidean distance between two points.
             * @param p1 First point (mm).
             * @param p2 Second point (mm).
             * @return Distance in millimetres.
             */
            double euc_distance_between_points(const QPointF &p1, const QPointF &p2);

            /**
             * @brief Return the point among p1 and p2 that is farthest to p.
             * @param p Reference point.
             * @param p1 Candidate 1.
             * @param p2 Candidate 2.
             * @return QPointF The farthest candidate from p.
             */
            QPointF get_most_distant_point(const QPointF &p, const QPointF &p1, const QPointF &p2);

            /**
             * @brief Reorder centers counter-clockwise around their centroid.
             * @param points Centers to reorder.
             * @return std::vector<Center> Points ordered CCW.
             */
            [[nodiscard]] std::vector<Center> reorder_points_CCW(const std::vector<Center> &points);

            // draw
            /**
             * @brief Draw line segments onto a QGraphicsScene for debugging.
             * @param lines Line segments to draw.
             * @param scene Target scene.
             */
            void draw_lines_on_2D_tab(const Lines &lines, QGraphicsScene *scene);

            /**
             * @brief Draw detected corners and model corners on a scene.
             * @param corners Detected corners to draw.
             * @param model_corners Model corners in the same frame (optional overlay).
             * @param scene Target scene.
             */
            void draw_corners_on_2D_tab(const Corners &corners, const std::vector<Eigen::Vector2d> &model_corners, QGraphicsScene *scene);

            // local data
            /** Convert Qt point to Eigen 2D vector (mm). */
            Eigen::Vector2d to_eigen(const QPointF &p);
            /** Convert OpenCV 2D point to Eigen 2D vector (mm). */
            Eigen::Vector2d to_eigen(const cv::Point2d &p);
            /** Convert OpenCV 2D point to Qt point (mm). */
            QPointF to_qpointf(const cv::Point2d &p);
    };


} // rc

#endif //FORCEFIELD_ROOM_DETECTOR_H
