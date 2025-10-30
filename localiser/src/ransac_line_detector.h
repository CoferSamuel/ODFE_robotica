// Created by pbustos on 27/10/25.
//
/* Purpose: Declaration of a RANSAC-based 2D line detector. The detector
    extracts line segments from 2D point clouds (e.g., projected lidar scans)
    and returns `LineSegment` objects describing segment endpoints, direction and inliers. */

#ifndef RANSACLINEDETECTOR_H
#define RANSACLINEDETECTOR_H

#include <vector>
#include <Eigen/Geometry>
#include "common_types.h"

namespace rc {

class RansacLineDetector
{
public:
    struct Params
    {
        double distance_threshold = 50.0;   // mm - points within this are inliers
        int min_points_per_line = 20;       // minimum inliers to accept line
        int max_iterations = 100;           // RANSAC iterations per line
        int max_lines = 10;                 // maximum lines to detect
        double min_line_length = 300.0;     // mm - minimum line length
        double max_line_separation = 100.0; // mm - maximum separation between lines for non-maximum suppression

        Params() = default;
    };

    // Detect line segments from 2D points using RANSAC.
    std::vector<LineSegment> detect_lines(const std::vector<Eigen::Vector2d>& points, const Params& params);
};

} // namespace rc

#endif // RANSACLINEDETECTOR_H
