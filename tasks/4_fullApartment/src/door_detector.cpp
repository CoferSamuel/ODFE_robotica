//
// Created by pbustos on 11/11/25.
//

#include "door_detector.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/combinations.hpp>
#include <QGraphicsItem>
#include <cmath>

// Local helper to draw peaks on a scene
static void draw_peaks(const Peaks &peaks,
                       std::optional<Eigen::Vector2d> center,
                       QGraphicsScene *scene)
{
    if (!scene) return;
    
    static std::vector<QGraphicsItem*> draw_points;

    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("Yellow");
    const QPen pen(color, 10);

    double offsetX = 0.0;
    double offsetY = 0.0;
    if (center.has_value())
    {
        offsetX = center.value().x();
        offsetY = center.value().y();
    }

    for (const auto &[point, angle] : peaks)
    {
        const auto dp = scene->addRect(-20, -20, 40, 40, pen);
        dp->setPos(offsetX + point.x(), offsetY + point.y());
        draw_points.push_back(dp);
    }
}

// Local helper to draw door points on a scene
static void draw_doors(const Doors &doors,
                       std::optional<Eigen::Vector2d> center,
                       QGraphicsScene *scene)
{
    if (!scene) return;
    
    static std::vector<QGraphicsItem*> draw_points;

    for (const auto &p : draw_points)
    {
        scene->removeItem(p);
        delete p;
    }
    draw_points.clear();

    const QColor color("Red");
    const QPen pen(color, 15);  // Red, thicker to distinguish doors
    const QBrush brush(color);

    double offsetX = 0.0;
    double offsetY = 0.0;
    if (center.has_value())
    {
        offsetX = center.value().x();
        offsetY = center.value().y();
    }

    for (const auto &door : doors)
    {
        // Draw first point of the door as a circle
        const auto dp1 = scene->addEllipse(-25, -25, 50, 50, pen, brush);
        dp1->setPos(offsetX + door.p1.x(), offsetY + door.p1.y());
        draw_points.push_back(dp1);

        // Draw second point of the door as a circle
        const auto dp2 = scene->addEllipse(-25, -25, 50, 50, pen, brush);
        dp2->setPos(offsetX + door.p2.x(), offsetY + door.p2.y());
        draw_points.push_back(dp2);

        // Draw a line connecting the two door points
        const auto line = scene->addLine(door.p1.x(), door.p1.y(),
                                         door.p2.x(), door.p2.y(), pen);
        line->setPos(offsetX, offsetY);
        draw_points.push_back(line);
    }
}

Doors DoorDetector::detect(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    Peaks peaks;
    // 1. Sliding window to find distance jumps
    for (const auto &p : points | iter::sliding_window(2))
    {
        const auto &p1 = p[0];
        const auto &p2 = p[1];
        const float d = std::abs(p2.distance2d - p1.distance2d);
        if (d > 1000.f)
        {
            const auto &shorter = (p1.distance2d < p2.distance2d) ? p1 : p2;
            peaks.emplace_back(Eigen::Vector2f(shorter.x, shorter.y), shorter.phi);
        }
    }

    // Draw peaks to visualize them
    draw_peaks(peaks, std::nullopt, scene);

    // 2. Non-maximum suppression of peaks: remove peaks closer than 500mm
    Peaks nms_peaks;
    for (const auto &[p, a] : peaks)
    {
        if (const bool too_close = std::ranges::any_of(nms_peaks, [&p](const auto &p2)
            { return (p - std::get<0>(p2)).norm() < 500.f; }); not too_close)
        {
            nms_peaks.emplace_back(p, a);
        }
    }
    peaks = nms_peaks;

    // 3. Detect doors from pairs of peaks
    Doors doors;
    for (const auto &c : iter::combinations(peaks, 2))
    {
        const auto &[p1, phi1] = c[0];
        const auto &[p2, phi2] = c[1];
        const float dist = (p1 - p2).norm();
        if (800.f < dist && dist < 1200.f)
        {
            doors.emplace_back(Door(p1, phi1, p2, phi2));
        }
    }

    // Draw detected doors in red
    draw_doors(doors, std::nullopt, scene);

    doors_cache = doors;
    return doors;
}


// Method to use the Doors vector to filter out the LiDAR points that come from a room outside the current one
RoboCompLidar3D::TPoints DoorDetector::filter_points(const RoboCompLidar3D::TPoints &points, QGraphicsScene *scene)
{
    const auto doors = detect(points, scene);
    if (doors.empty()) return points;

    // For each door, check if the distance from the robot to each lidar point is smaller than the distance from the robot to the door
    RoboCompLidar3D::TPoints filtered;
    for (const auto &d : doors)
    {
        const float dist_to_door = d.center().norm();
        // Check if the angular range wraps around the -π/+π boundary
        const bool angle_wraps = d.p2_angle < d.p1_angle;
        for (const auto &p : points)
        {
            // Determine if point is within the door's angular range
            bool point_in_angular_range;
            if (angle_wraps)
            {
                // If the range wraps around, point is in range if it's > p1_angle OR < p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) || (p.phi < d.p2_angle);
            }
            else
            {
                // Normal case: point is in range if it's between p1_angle and p2_angle
                point_in_angular_range = (p.phi > d.p1_angle) && (p.phi < d.p2_angle);
            }

            // Filter out points that are through the door (in angular range and farther than door)
            if (point_in_angular_range && p.distance2d >= dist_to_door)
                continue;

            filtered.emplace_back(p);
        }
    }
    return filtered;
}
