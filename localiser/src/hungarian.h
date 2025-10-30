// Created by robolab on 12/5/24.
//
/* Purpose: High-level wrapper around the Munkres algorithm to match measured
  corners to nominal corners. Produces `Match` results (measurement, nominal, error)
  used by the room-localisation pipeline. */

#ifndef BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H
#define BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H

#include <Eigen/Geometry>
#include "munkres.hpp"
#include "common_types.h"

namespace rc
{
    class Hungarian
    {
        public:
            /**
             * @brief Match measured room corners to nominal corners using the Hungarian algorithm.
             *
             * Given two sets of corners expressed in the same coordinate frame, this method builds a
             * cost matrix (Euclidean distances) and runs the Munkres/Hungarian assignment to obtain
             * the lowest-cost one-to-one correspondence. Pairs whose distance exceeds @p max_corner_diff
             * are discarded from the final result.
             *
             * @param measurement_corners Corners detected from sensor data (measurement frame).
             * @param nominal_corners Nominal model corners expressed in the same frame as measurements.
             * @param max_corner_diff Maximum admissible distance (in mm) to accept a pairing.
             * @return Match Vector of tuples {measured corner, nominal corner, matching error}.
             */
            Match match(const Corners &measurement_corners, const Corners &nominal_corners, double max_corner_diff = std::numeric_limits<double>::max());

            // aux methods
            /**
             * @brief Euclidean distance between two Qt points.
             * @param p1 First point (x,y in mm).
             * @param p2 Second point (x,y in mm).
             * @return Distance in millimetres.
             */
            double euclidean_distance(const QPointF &p1, const QPointF &p2);

            /**
             * @brief Euclidean distance between two corners.
             * @param c1 First corner tuple (QPointF, angle, timestamp).
             * @param c2 Second corner tuple (QPointF, angle, timestamp).
             * @return Distance in millimetres using the embedded QPointF.
             */
            double euclidean_distance(const Corner &c1, const Corner &c2);
    };
} // rc

#endif //BETA_ROBOTICA_CLASS_PRIVATE_HUNGARIAN_H
