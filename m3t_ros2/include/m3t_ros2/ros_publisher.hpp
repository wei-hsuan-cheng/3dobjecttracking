// SPDX-License-Identifier: MIT
// RosPublisher: an m3t::Publisher that emits all per-frame ROS output for the
// M3T tracker — raw color/depth images, an ORB keypoint image, a mesh-overlay
// image (normals blended over color), and TF + mesh markers for both the
// estimate and (optionally) ground truth. No GUI windows.

#ifndef M3T_ROS2_ROS_PUBLISHER_HPP_
#define M3T_ROS2_ROS_PUBLISHER_HPP_

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <m3t/body.h>
#include <m3t/loader_camera.h>
#include <m3t/normal_renderer.h>
#include <m3t/publisher.h>

namespace m3t_ros2 {

struct RosPublisherConfig {
  std::string world_frame{"camera"};
  std::string mesh_resource;           // file:// or package:// URI
  float mesh_scale{1.0f};              // = geometry_unit_in_meter
  bool mesh_use_embedded_materials{false};
  bool publish_gt{true};
  bool publish_keypoints{true};
  bool publish_overlay{true};
};

class RosPublisher : public m3t::Publisher {
 public:
  RosPublisher(const std::string &name, rclcpp::Node *node,
               std::shared_ptr<m3t::LoaderColorCamera> color_camera,
               std::shared_ptr<m3t::LoaderDepthCamera> depth_camera,
               std::shared_ptr<m3t::Body> body,
               std::shared_ptr<m3t::FullNormalRenderer> overlay_renderer,
               std::vector<Eigen::Matrix4f> gt_poses,
               const RosPublisherConfig &cfg)
      : m3t::Publisher{name},
        node_{node},
        color_camera_{std::move(color_camera)},
        depth_camera_{std::move(depth_camera)},
        body_{std::move(body)},
        overlay_renderer_{std::move(overlay_renderer)},
        gt_poses_{std::move(gt_poses)},
        cfg_{cfg} {
    auto qos = rclcpp::SensorDataQoS();
    pub_color_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "~/color/image_raw", qos);
    pub_depth_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "~/depth/image_raw", qos);
    pub_overlay_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "~/overlay/image", qos);
    pub_keypoints_ = node_->create_publisher<sensor_msgs::msg::Image>(
        "~/keypoints/image", qos);
    pub_marker_est_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "~/marker_est", 1);
    pub_marker_gt_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "~/marker_gt", 1);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    orb_ = cv::ORB::create(500);
  }

  bool SetUp() override {
    set_up_ = true;
    return true;
  }

  bool UpdatePublisher(int /*iteration*/) override {
    const rclcpp::Time stamp = node_->now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = cfg_.world_frame;

    const cv::Mat color = color_camera_->image();
    if (!color.empty())
      pub_color_->publish(*cv_bridge::CvImage(header, "bgr8", color).toImageMsg());

    if (depth_camera_) {
      const cv::Mat depth = depth_camera_->image();
      if (!depth.empty())
        pub_depth_->publish(
            *cv_bridge::CvImage(header, "16UC1", depth).toImageMsg());
    }

    // Mesh-overlay image: render normals at the estimated pose, blend on color.
    if (cfg_.publish_overlay && overlay_renderer_ && !color.empty()) {
      if (overlay_renderer_->StartRendering() &&
          overlay_renderer_->FetchNormalImage()) {
        cv::Mat nb;
        cv::cvtColor(overlay_renderer_->normal_image(), nb,
                     cv::COLOR_BGRA2BGR);
        cv::Mat ov = color.clone();
        for (int y = 0; y < ov.rows; ++y)
          for (int x = 0; x < ov.cols; ++x) {
            const cv::Vec3b n = nb.at<cv::Vec3b>(y, x);
            if (n[0] || n[1] || n[2]) {
              cv::Vec3b &o = ov.at<cv::Vec3b>(y, x);
              for (int c = 0; c < 3; ++c)
                o[c] = cv::saturate_cast<uchar>(0.5 * o[c] + 0.5 * n[c]);
            }
          }
        pub_overlay_->publish(
            *cv_bridge::CvImage(header, "bgr8", ov).toImageMsg());
      }
    }

    // ORB keypoint image.
    if (cfg_.publish_keypoints && !color.empty()) {
      std::vector<cv::KeyPoint> kp;
      orb_->detect(color, kp);
      cv::Mat kimg;
      cv::drawKeypoints(color, kp, kimg, cv::Scalar(0, 255, 0));
      pub_keypoints_->publish(
          *cv_bridge::CvImage(header, "bgr8", kimg).toImageMsg());
    }

    // Estimate: TF + marker.
    BroadcastTf(stamp, "object_est", body_->body2world_pose());
    PublishMarker(pub_marker_est_, stamp, "est", body_->geometry2world_pose(),
                  1.0f, 0.1f, 0.1f, 0.9f);

    // Ground truth: TF + marker.
    if (cfg_.publish_gt && !gt_poses_.empty()) {
      const int f = std::clamp(color_camera_->load_index() - 1, 0,
                               static_cast<int>(gt_poses_.size()) - 1);
      const m3t::Transform3fA gt_b2w{gt_poses_[f]};
      const m3t::Transform3fA gt_g2w = gt_b2w * body_->geometry2body_pose();
      BroadcastTf(stamp, "object_gt", gt_b2w);
      PublishMarker(pub_marker_gt_, stamp, "gt", gt_g2w, 0.1f, 0.9f, 0.1f, 0.5f);
    }
    return true;
  }

 private:
  static geometry_msgs::msg::Pose ToPose(const m3t::Transform3fA &t) {
    geometry_msgs::msg::Pose p;
    const Eigen::Vector3f tr = t.translation();
    const Eigen::Quaternionf q{t.rotation()};
    p.position.x = tr.x();
    p.position.y = tr.y();
    p.position.z = tr.z();
    p.orientation.x = q.x();
    p.orientation.y = q.y();
    p.orientation.z = q.z();
    p.orientation.w = q.w();
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
    mk.color.r = r;
    mk.color.g = g;
    mk.color.b = b;
    mk.color.a = a;
    pub->publish(mk);
  }

  rclcpp::Node *node_;
  std::shared_ptr<m3t::LoaderColorCamera> color_camera_;
  std::shared_ptr<m3t::LoaderDepthCamera> depth_camera_;
  std::shared_ptr<m3t::Body> body_;
  std::shared_ptr<m3t::FullNormalRenderer> overlay_renderer_;
  std::vector<Eigen::Matrix4f> gt_poses_;
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
