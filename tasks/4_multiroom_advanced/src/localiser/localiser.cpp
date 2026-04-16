#include "localiser.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <cppitertools/enumerate.hpp>

Localiser::Localiser() {
}

Localiser::LocaliserResult Localiser::process(const Corners& detected_corners, 
                                              const NominalRoom& room, 
                                              const Eigen::Affine2d& current_pose,
                                              bool& initial_localisation_done,
                                              bool previously_localised,
                                              std::optional<float> predefined_heading,
                                              std::optional<Eigen::Vector2f> reference_position,
                                              bool debug) 
{
    LocaliserResult result;
    result.localised = previously_localised;
    result.max_match_error = std::numeric_limits<float>::infinity();
    result.new_pose = std::nullopt;

    Eigen::Affine2d working_pose = current_pose;

    // Bootstrap: If not yet localized, try multi-hypothesis grid search
    if (!initial_localisation_done && !detected_corners.empty()) {
        working_pose = find_best_initial_pose(detected_corners, room, predefined_heading, reference_position, debug);
        initial_localisation_done = true;
        
        // We might want to return this initial pose immediately
        // But the flow below will refine it using solve_pose
        
        // Convert Affine2d back to x, y, theta for result if needed, 
        // but typically we want the refined one. 
        // Let's let the flow continue to refine it.
        if (debug)
            qInfo() << "localiser: Initial pose found via grid search at x="
                    << working_pose.translation().x() << " y=" << working_pose.translation().y();
         
         // Force update of returned pose so worker knows we jumped
         result.new_pose = working_pose;
    }

    // 1) Transform nominal room corners into the robot frame for matching
    Corners robot_corners = room.transform_corners_to(working_pose.inverse());

    // 2) Match detected corners to nominal room corners using Hungarian algorithm
    result.matches = hungarian.match(detected_corners, robot_corners);

    // 3) Compute maximum match error
    float max_match_error = std::numeric_limits<float>::infinity();
    if (!result.matches.empty()) {
        const auto max_it = std::ranges::max_element(result.matches, [](const auto &a, const auto &b) {
          return std::get<2>(a) < std::get<2>(b);
        });
        max_match_error = static_cast<float>(std::get<2>(*max_it));
    }
    result.max_match_error = max_match_error;

    // 5) Localisation decision
    const bool was_localised = previously_localised;
    if (std::isfinite(max_match_error) &&
        max_match_error < LOCALISATION_MATCH_ERROR_THRESHOLD &&
        result.matches.size() >= 3) {
        result.localised = true;
        if (!was_localised && debug)
            qInfo() << "localiser: Localisation achieved. max_error=" << max_match_error;
    } else {
        result.localised = false;
        if (was_localised && debug)
            qInfo() << "localiser: Localisation lost. max_error=" << max_match_error;
    }

    if (debug)
        qInfo() << "localiser: Localised=" << (result.localised ? "true" : "false")
                << " max_error=" << max_match_error << " matches=" << result.matches.size();

    // 6) Update pose continuously when we have good matches
    if (result.matches.size() >= 3 && std::isfinite(max_match_error) &&
        max_match_error < LOCALISATION_MATCH_ERROR_THRESHOLD) {
        
        Eigen::Vector3d pose_update = solve_pose(detected_corners, result.matches);
        
        if (!pose_update.array().isNaN().any()) {
             // Apply correction to working_pose
             working_pose.translate(Eigen::Vector2d(pose_update(0), pose_update(1)));
             working_pose.rotate(pose_update(2));
             
             result.new_pose = working_pose;
             
             if (debug) {
                qInfo() << "localiser: Pose updated: x=" << working_pose.translation().x()
                        << " y=" << working_pose.translation().y();
             }
        } else {
            if (debug)
                qInfo() << "localiser: Invalid pose (NaN)";
        }
    }

    return result;
}

Eigen::Affine2d Localiser::find_best_initial_pose(const Corners &detected_corners,
                                                  const NominalRoom& room,
                                                  std::optional<float> reference_heading,
                                                  std::optional<Eigen::Vector2f> reference_position,
                                                  bool debug) {
  const float w = room.width / 2.0f;
  const float l = room.length / 2.0f;

  Eigen::Affine2d best_pose = Eigen::Affine2d::Identity();
  float best_error = std::numeric_limits<float>::infinity();

  // If reference heading is provided, constrain search to ±45° around it
  float theta_start = 0.0f;
  float theta_end = 2 * M_PI;
  float theta_step = M_PI / 4;
  
  if (reference_heading.has_value()) {
    theta_start = reference_heading.value() - M_PI / 4;
    theta_end = reference_heading.value() + M_PI / 4;
    theta_step = M_PI / 8;
    if (debug)
      qInfo() << "Grid search constrained to heading" << reference_heading.value() << "±45°";
  }

  for (float x = -w * 0.8f; x <= w * 0.8f; x += w * 0.4f) {
    for (float y = -l * 0.8f; y <= l * 0.8f; y += l * 0.4f) {
      
      // If reference position is provided, filter candidates that are too far
      if (reference_position.has_value()) {
          float dist = (Eigen::Vector2f(x, y) - reference_position.value()).norm();
          if (dist > 1500.0f) { // 1.5m tolerance
              continue;
          }
      }

      for (float theta = theta_start; theta <= theta_end; theta += theta_step) {
        Eigen::Affine2d candidate;
        candidate.setIdentity();
        candidate.translate(Eigen::Vector2d(x, y));
        candidate.rotate(theta);

        Corners robot_corners = room.transform_corners_to(candidate.inverse());

        Match matched = hungarian.match(detected_corners, robot_corners);

        if (matched.size() < 3)
          continue;

        float max_err = 0.0f;
        for (const auto &m : matched) {
          float err = static_cast<float>(std::get<2>(m));
          if (err > max_err)
            max_err = err;
        }

        if (max_err < best_error) {
          best_error = max_err;
          best_pose = candidate;
        }
      }
    }
  }

  if (debug)
    qInfo() << "Grid search best error:" << best_error
            << "at x=" << best_pose.translation().x()
            << "y=" << best_pose.translation().y();

  return best_pose;
}

Eigen::Vector3d Localiser::solve_pose(const Corners &corners, const Match &match) {
  Eigen::MatrixXd W(match.size() * 2, 3);
  Eigen::VectorXd b(match.size() * 2);

  for (auto &&[i, m] : iter::enumerate(match)) {
    auto &[meas_c, nom_c, _] = m;
    auto &[p_meas, __, ___] = meas_c;
    auto &[p_nom, ____, _____] = nom_c;

    b(2 * i) = p_nom.x() - p_meas.x();
    b(2 * i + 1) = p_nom.y() - p_meas.y();
    W.block<1, 3>(2 * i, 0) << 1.0, 0.0, -p_meas.y();
    W.block<1, 3>(2 * i + 1, 0) << 0.0, 1.0, p_meas.x();
  }
  // estimate new pose with pseudoinverse
  const Eigen::Vector3d r = (W.transpose() * W).inverse() * W.transpose() * b;
  return r;
}

void Localiser::update_pose(QGraphicsPolygonItem *room, const Eigen::Affine2d& current_pose) {
    // Update robot graphic in room viewer
    room->setPos(current_pose.translation().x(), current_pose.translation().y());
    double current_angle = std::atan2(current_pose.rotation()(1, 0), current_pose.rotation()(0, 0));
    // Convert to degrees for Qt
    room->setRotation(current_angle * 180.0 / M_PI); 
}

