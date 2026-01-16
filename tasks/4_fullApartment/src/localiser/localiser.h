#ifndef LOCALISER_H
#define LOCALISER_H

#include "../common_types.h"
#include "../nominal_room.h"
#include "../hungarian.h"
#include <Eigen/Dense>
#include <optional>
#include <tuple>
#include <vector>
#include <QDebug>
#include <QGraphicsPolygonItem>

class Localiser {
public:
    struct LocaliserResult {
        bool localised;
        float max_match_error;
        Match matches;
        std::optional<Eigen::Affine2d> new_pose; // Absolute pose if updated
    };

    Localiser();
    ~Localiser() = default;

    /**
     * @brief Core localization logic.
     * 
     * @param detected_corners Corners detected by LiDAR.
     * @param room The current nominal room description.
     * @param current_pose Current robot pose.
     * @param initial_localisation_done Flag to indicate if we need grid search.
     * @param predefined_heading Optional heading to constrain grid search (e.g. from graph).
     * @param debug Enable debug logging.
     * @return LocaliserResult container match info, status and optional new pose.
     */
    LocaliserResult process(const Corners& detected_corners, 
                            const NominalRoom& room, 
                            const Eigen::Affine2d& current_pose,
                            bool& initial_localisation_done, /* updated by ref or we return it? ref is easier for state */
                            bool previously_localised,
                            std::optional<float> predefined_heading = std::nullopt,
                            std::optional<Eigen::Vector2f> reference_position = std::nullopt,
                            bool debug = false);

    /**
     * @brief Initialize pose via grid search.
     */
    Eigen::Affine2d find_best_initial_pose(const Corners& detected_corners,
                                           const NominalRoom& room,
                                           std::optional<float> reference_heading = std::nullopt,
                                           std::optional<Eigen::Vector2f> reference_position = std::nullopt,
                                           bool debug = false);
    
    /**
     * @brief Estimate pose from matched corners.
     */
    Eigen::Vector3d solve_pose(const Corners& detected, const Match& match);

    /**
     * @brief Update pose from matched corners.
     */
    void update_pose(QGraphicsPolygonItem *room, const Eigen::Affine2d& current_pose);

private:
   rc::Hungarian hungarian;
   const float LOCALISATION_MATCH_ERROR_THRESHOLD = 3500.0f;
};

#endif // LOCALISER_H
