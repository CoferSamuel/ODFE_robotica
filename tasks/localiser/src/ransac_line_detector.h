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
    /**
     * @brief Adjustable parameters controlling the RANSAC line extraction.
     */
    struct Params
    {
        /** Maximum distance (mm) from a candidate model to consider a point an inlier. */
        double distance_threshold = 50.0;
        /** Minimum number of inliers required to accept a line hypothesis. */
        int min_points_per_line = 20;
        /** Number of RANSAC iterations attempted per extracted line. */
        int max_iterations = 100;
        /** Upper bound on how many lines to extract from a single set of points. */
        int max_lines = 10;
        /** Minimum accepted physical length (mm) of a detected line segment. */
        double min_line_length = 300.0;
        /** Threshold (mm) used to merge/suppress nearly identical neighbouring lines. */
        double max_line_separation = 100.0;

        Params() = default;
    };

    /**
     * @brief Detect line segments from 2D points using RANSAC.
     *
     * Repeatedly samples minimal sets, estimates a line model, counts inliers using
     * @p params.distance_threshold, and extracts the best-supported line segments
     * until either no more valid models can be found or @p params.max_lines is reached.
     *
     * @param points Input 2D points in millimetres, in any consistent frame.
     * @param params Algorithm parameters controlling thresholds and limits.
     * @return std::vector<LineSegment> Detected line segments with endpoints and scores.
     */
    std::vector<LineSegment> detect_lines(const std::vector<Eigen::Vector2d>& points, const Params& params);
    
    /**
     * @brief Detect line segments using default parameters.
     * 
     * This overload allows calling detect_lines without explicitly providing parameters.
     * It internally creates a default Params() object with standard threshold values:
     *   - distance_threshold = 50.0 mm
     *   - min_points_per_line = 20
     *   - max_iterations = 100
     *   - max_lines = 10
     *   - min_line_length = 300.0 mm
     * 
     * @param points Input 2D points in millimetres.
     * @return std::vector<LineSegment> Detected line segments.
     */
    std::vector<LineSegment> detect_lines(const std::vector<Eigen::Vector2d>& points);
};

} // namespace rc

#endif // RANSACLINEDETECTOR_H
