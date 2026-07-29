// SPDX-License-Identifier: MIT
// Construct an M3T Body from ROS parameters instead of an M3T-specific YAML.

#ifndef M3T_ROS2_BODY_FACTORY_HPP_
#define M3T_ROS2_BODY_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include <m3t/body.h>

#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace m3t_ros2 {

inline m3t::Transform3fA TransformFromRowMajor(
    const std::vector<double> &values, const std::string &parameter_name) {
  if (values.size() != 16) {
    throw std::runtime_error(parameter_name +
                             " must contain 16 row-major values");
  }
  Eigen::Matrix4f matrix;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      matrix(row, column) =
          static_cast<float>(values[static_cast<size_t>(row * 4 + column)]);
    }
  }
  return m3t::Transform3fA{matrix};
}

inline std::shared_ptr<m3t::Body> DeclareAndCreateBody(
    rclcpp::Node *node, bool force_disable_culling = false) {
  const auto object_name =
      node->declare_parameter<std::string>("object_name", "object");
  const auto geometry_path =
      node->declare_parameter<std::string>("geometry_path", "");
  const float geometry_unit = static_cast<float>(
      node->declare_parameter<double>("geometry_unit_in_meter", 1.0));
  const bool counterclockwise =
      node->declare_parameter<bool>("geometry_counterclockwise", true);
  bool enable_culling =
      node->declare_parameter<bool>("geometry_enable_culling", false);
  const auto geometry2body_values = node->declare_parameter<std::vector<double>>(
      "geometry2body_pose",
      {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0});
  const int64_t body_id = node->declare_parameter<int64_t>("body_id", 1);
  const int64_t region_id = node->declare_parameter<int64_t>("region_id", 1);

  if (geometry_path.empty()) {
    throw std::runtime_error("geometry_path is required");
  }
  if (!std::filesystem::exists(geometry_path)) {
    throw std::runtime_error("geometry_path does not exist: " + geometry_path);
  }
  if (body_id < 1 || body_id > 255 || region_id < 1 || region_id > 255) {
    throw std::runtime_error("body_id and region_id must be within [1, 255]");
  }
  if (force_disable_culling) enable_culling = false;

  auto body = std::make_shared<m3t::Body>(
      object_name, geometry_path, geometry_unit, counterclockwise,
      enable_culling,
      TransformFromRowMajor(geometry2body_values, "geometry2body_pose"));
  body->set_body_id(static_cast<uchar>(body_id));
  body->set_region_id(static_cast<uchar>(region_id));
  return body;
}

}  // namespace m3t_ros2

#endif  // M3T_ROS2_BODY_FACTORY_HPP_
