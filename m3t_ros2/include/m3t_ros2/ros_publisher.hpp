// SPDX-License-Identifier: MIT
// RosPublisher: snapshot-driven ROS output, decoupled from the tracking solve.
// It is called by a wall-timer (not by the tracker), so the tracking thread is
// never blocked by ROS serialization. Optional image publishers and their
// processing are created only when requested. TF and the estimate marker remain
// available independently of image output. No GUI windows.

#ifndef M3T_ROS2_ROS_PUBLISHER_HPP_
#define M3T_ROS2_ROS_PUBLISHER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/features2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <vector>

#include <m3t/common.h>

namespace m3t_ros2 {

struct RosPublisherConfig {
  std::string world_frame{"camera"};
  std::string mesh_resource;
  float mesh_scale{1.0f};
  bool mesh_use_embedded_materials{false};
  bool publish_color{false};
  bool publish_depth{false};
  bool publish_overlay{false};
  bool publish_gt{false};
  bool publish_keypoints{false};
};

// One consistent frame handed from the tracking thread to the publisher.
struct Snapshot {
  bool valid{false};
  bool has_depth{false};
  bool has_gt{false};
  cv::Mat color, depth, overlay;
  m3t::Transform3fA body2world_est, geometry2world_est;
  m3t::Transform3fA body2world_gt, geometry2world_gt;
};

class RosPublisher {
 public:
  RosPublisher(rclcpp::Node *node, const RosPublisherConfig &cfg)
      : node_{node}, cfg_{cfg} {
    auto qos = rclcpp::SensorDataQoS();
    if (cfg_.publish_color)
      pub_color_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/color/image_raw", qos);
    if (cfg_.publish_depth)
      pub_depth_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/depth/image_raw", qos);
    if (cfg_.publish_overlay)
      pub_overlay_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/overlay/image", qos);
    if (cfg_.publish_keypoints) {
      pub_keypoints_ = node_->create_publisher<sensor_msgs::msg::Image>(
          "~/keypoints/image", qos);
      orb_ = cv::ORB::create(500);
    }
    pub_marker_est_ = node_->create_publisher<visualization_msgs::msg::Marker>("~/marker_est", 1);
    if (cfg_.publish_gt)
      pub_marker_gt_ = node_->create_publisher<visualization_msgs::msg::Marker>(
          "~/marker_gt", 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  void Publish(const Snapshot &s) {
    if (!s.valid) return;
    const rclcpp::Time stamp = node_->now();
    std_msgs::msg::Header h;
    h.stamp = stamp;
    h.frame_id = cfg_.world_frame;

    if (cfg_.publish_color && !s.color.empty())
      pub_color_->publish(*cv_bridge::CvImage(h, "bgr8", s.color).toImageMsg());
    if (cfg_.publish_depth && s.has_depth && !s.depth.empty())
      pub_depth_->publish(*cv_bridge::CvImage(h, "16UC1", s.depth).toImageMsg());
    if (cfg_.publish_overlay && !s.overlay.empty())
      pub_overlay_->publish(*cv_bridge::CvImage(h, "bgr8", s.overlay).toImageMsg());
    if (cfg_.publish_keypoints && !s.color.empty()) {
      std::vector<cv::KeyPoint> kp;
      orb_->detect(s.color, kp);
      cv::Mat kimg;
      cv::drawKeypoints(s.color, kp, kimg, cv::Scalar(0, 255, 0));
      pub_keypoints_->publish(*cv_bridge::CvImage(h, "bgr8", kimg).toImageMsg());
    }

    BroadcastTf(stamp, "object_est", s.body2world_est);
    PublishMarker(pub_marker_est_, stamp, "est", s.geometry2world_est, 1.0f, 0.1f, 0.1f, 0.9f);
    if (cfg_.publish_gt && s.has_gt) {
      BroadcastTf(stamp, "object_gt", s.body2world_gt);
      PublishMarker(pub_marker_gt_, stamp, "gt", s.geometry2world_gt, 0.55f, 0.55f, 0.55f, 0.7f);
    }
  }

 private:
  static geometry_msgs::msg::Pose ToPose(const m3t::Transform3fA &t) {
    geometry_msgs::msg::Pose p;
    const Eigen::Vector3f tr = t.translation();
    const Eigen::Quaternionf q{t.rotation()};
    p.position.x = tr.x(); p.position.y = tr.y(); p.position.z = tr.z();
    p.orientation.x = q.x(); p.orientation.y = q.y();
    p.orientation.z = q.z(); p.orientation.w = q.w();
    return p;
  }

  void BroadcastTf(const rclcpp::Time &stamp, const std::string &child,
                   const m3t::Transform3fA &t) {
    geometry_msgs::msg::TransformStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = cfg_.world_frame;
    m.child_frame_id = child;
    const auto p = ToPose(t);
    m.transform.translation.x = p.position.x;
    m.transform.translation.y = p.position.y;
    m.transform.translation.z = p.position.z;
    m.transform.rotation = p.orientation;
    tf_->sendTransform(m);
  }

  void PublishMarker(
      const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &pub,
      const rclcpp::Time &stamp, const std::string &ns,
      const m3t::Transform3fA &pose, float r, float g, float b, float a) {
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = cfg_.world_frame;
    mk.header.stamp = stamp;
    mk.ns = ns;
    mk.id = 0;
    mk.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mk.action = visualization_msgs::msg::Marker::ADD;
    mk.mesh_resource = cfg_.mesh_resource;
    mk.mesh_use_embedded_materials = cfg_.mesh_use_embedded_materials;
    mk.pose = ToPose(pose);
    mk.scale.x = mk.scale.y = mk.scale.z = cfg_.mesh_scale;
    mk.color.r = r; mk.color.g = g; mk.color.b = b; mk.color.a = a;
    pub->publish(mk);
  }

  rclcpp::Node *node_;
  RosPublisherConfig cfg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_, pub_depth_,
      pub_overlay_, pub_keypoints_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_est_,
      pub_marker_gt_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  cv::Ptr<cv::ORB> orb_;
};

}  // namespace m3t_ros2

#endif  // M3T_ROS2_ROS_PUBLISHER_HPP_
