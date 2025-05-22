#pragma once

#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>

#include <g2o/core/base_unary_edge.h>
#include <cmath>

namespace teb_local_planner
{

/**
 * @class EdgeCooperation
 * @brief Edge defining a cost that includes both position and yaw alignment to another robot
 *
 * Error is 2D:
 *  - error[0] = Euclidean distance to the target robot position
 *  - error[1] = normalized angle difference to target yaw
 */
class EdgeCooperation : public BaseTebUnaryEdge<2, const Eigen::Vector3d*, VertexPose>
{
public:
  EdgeCooperation()
  {
    _measurement = nullptr;  // target pose (x, y, theta)
  }

  void computeError() override
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig() and setTargetPose() on EdgeCooperation");

    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);

    // Extract current pose
    const Eigen::Vector2d current_pos = pose->position();
    double current_theta = pose->theta();

    // Extract target pose
    const Eigen::Vector2d target_pos = _measurement->head<2>();
    double target_theta = (*_measurement)[2];

    // Compute Euclidean distance error
    _error[0] = (current_pos - target_pos).norm();

    // Compute yaw error, normalized to [-pi, pi]
    double yaw_diff = current_theta - target_theta;
    _error[1] = g2o::normalize_theta(yaw_diff);

    ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), 
                   "EdgeCooperation::computeError() produced non-finite error values.");
  }

  void setTargetPose(const Eigen::Vector3d* other_robot_pose)
  {
    _measurement = other_robot_pose;
  }

  void setParameters(const TebConfig& cfg, const Eigen::Vector3d* other_robot_pose)
  {
    cfg_ = &cfg;
    _measurement = other_robot_pose;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace teb_local_planner
