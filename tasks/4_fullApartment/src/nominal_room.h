#pragma once
#include <QPointF>
#include <QRectF>
#include <Eigen/Dense>
#include <vector>
#include "common_types.h"
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/enumerate.hpp>

  struct NominalRoom
        {
            float width; //  mm
            float length;
            std::vector<Door> doors;
            explicit NominalRoom(const float width_=10000.f, const float length_=5000.f, Corners  corners_ = {}) :
                width(width_), length(length_)
            {};
            [[nodiscard]] Corners corners() const
            {
                // compute corners from width and length
                return {
                    {QPointF{-width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, -length/2.f}, 0.f, 0.f},
                    {QPointF{width/2.f, length/2.f}, 0.f, 0.f},
                    {QPointF{-width/2.f, length/2.f}, 0.f, 0.f}
                };
            }
            [[nodiscard]] QRectF rect() const
            {
                return QRectF{-width/2.f, -length/2.f, width, length};
            }
            [[nodiscard]] Corners transform_corners_to(const Eigen::Affine2d &transform) const  // for room to robot pass the inverse of robot_pose
            {
                Corners transformed_corners;
                for(const auto &[p, _, __] : corners())
                {
                    auto ep = Eigen::Vector2d{p.x(), p.y()};
                    Eigen::Vector2d tp = transform * ep;
                    transformed_corners.emplace_back(QPointF{static_cast<float>(tp.x()), static_cast<float>(tp.y())}, 0.f, 0.f);
                }
                return transformed_corners;
            }
            [[nodiscard]] Walls get_walls() const
            {
                Walls walls;
                // Get the corners of the room
                auto cs = corners();
                // Duplicate the first corner at the end to close the loop (last point connects to first)
                cs.push_back(cs[0]);
                
                // Iterate over corners using a sliding window of size 2 to get adjacent pairs (segments)
                // iter::enumerate gives us the index 'i' for each segment
                for(auto &&[i, c] : cs | iter::sliding_window(2) | iter::enumerate)
                {
                    const auto &c1 = c[0]; // Start point of the segment
                    const auto &c2 = c[1]; // End point of the segment
                    
                    // Convert to Eigen vectors
                    Eigen::Vector2f p1(std::get<0>(c1).x(), std::get<0>(c1).y());
                    Eigen::Vector2f p2(std::get<0>(c2).x(), std::get<0>(c2).y());
                    
                    // Create a parametrized line passing through p1 and p2
                    auto r = Eigen::ParametrizedLine<float, 2>::Through(p1, p2);
                    
                    // Add the wall to the list: line, index, start corner, end corner
                    walls.emplace_back(r, i, c1, c2);
                }
                return walls;
            }

            [[nodiscard]] Wall get_closest_wall_to_point(const Eigen::Vector2f &p) const
            {
                const auto walls = get_walls();
                auto min_it = std::ranges::min_element(walls, [&p](const Wall &w1, const Wall &w2) {
                    const auto &[line1, i1, c1_start, c1_end] = w1;
                    const auto &[line2, i2, c2_start, c2_end] = w2;
                    return line1.distance(p) < line2.distance(p);
                });
                return *min_it;
            }

            [[nodiscard]] Eigen::Vector2f get_projection_of_point_on_closest_wall(const Eigen::Vector2f &p) const
            {
                const auto w = get_closest_wall_to_point(p);
                const auto &[line, i, c1, c2] = w;
                return line.projection(p);
            }
        };