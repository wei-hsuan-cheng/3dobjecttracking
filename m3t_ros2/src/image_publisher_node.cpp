// SPDX-License-Identifier: MIT
// m3t_ros2 image publisher — node 1 (the "camera").
//
// Reads a recorded RGB-D sequence from disk (M3T LoaderCameras) and publishes
// color + depth images, their CameraInfo, and the ground-truth TF + mesh marker
// at a fixed rate, looping. Replace this node with a real camera driver by just
// matching the topic names; the tracker node needs no change.

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <m3t/body.h>
#include <m3t/loader_camera.h>

namespace fs = std::filesystem;

static std::vector<Eigen::Matrix4f> ReadGtPoses(const fs::path &path) {
  std::vector<Eigen::Matrix4f> poses;
  std::ifstream ifs{path.string()};
  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss{line};
    Eigen::Matrix4f m; int i = 0; float v;
    while (i < 16 && (iss >> v)) { m(i / 4, i % 4) = v; ++i; }
    if (i == 16) poses.push_back(m);
  }
  return poses;
}

static sensor_msgs::msg::CameraInfo MakeInfo(const m3t::Intrinsics &in,
                                             const std::string &frame) {
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame;
  info.width = in.width; info.height = in.height;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {in.fu, 0, in.ppu, 0, in.fv, in.ppv, 0, 0, 1};
  info.p = {in.fu, 0, in.ppu, 0, 0, in.fv, in.ppv, 0, 0, 0, 1, 0};
  return info;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("m3t_image_publisher");

  const auto seq = node->declare_parameter<std::string>("sequence_dir", "");
  const auto body_metafile = node->declare_parameter<std::string>("body_metafile", "");
  const double rate = node->declare_parameter<double>("publish_rate", 30.0);
  const bool loop = node->declare_parameter<bool>("loop", true);
  const auto frame = node->declare_parameter<std::string>("world_frame", "camera");
  const bool publish_gt = node->declare_parameter<bool>("publish_gt", true);
  const auto color_topic = node->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
  const auto depth_topic = node->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  const auto color_info_topic = node->declare_parameter<std::string>("color_info_topic", "/camera/color/camera_info");
  const auto depth_info_topic = node->declare_parameter<std::string>("depth_info_topic", "/camera/depth/camera_info");
  const auto mesh_resource = node->declare_parameter<std::string>("mesh_resource", "");
  const double mesh_scale = node->declare_parameter<double>("mesh_scale", 1.0);
  const bool mesh_embedded = node->declare_parameter<bool>("mesh_use_embedded_materials", false);

  if (seq.empty()) { RCLCPP_FATAL(node->get_logger(), "sequence_dir required"); return 1; }

  auto color_cam = std::make_shared<m3t::LoaderColorCamera>(
      "color", fs::path{seq} / "color_camera.yaml");
  std::shared_ptr<m3t::LoaderDepthCamera> depth_cam;
  if (fs::exists(fs::path{seq} / "depth_camera.yaml"))
    depth_cam = std::make_shared<m3t::LoaderDepthCamera>("depth", fs::path{seq} / "depth_camera.yaml");
  if (!color_cam->SetUp() || (depth_cam && !depth_cam->SetUp())) {
    RCLCPP_FATAL(node->get_logger(), "camera SetUp failed"); return 1; }

  m3t::Transform3fA geometry2body{m3t::Transform3fA::Identity()};
  if (!body_metafile.empty()) {
    auto body = std::make_shared<m3t::Body>("body", body_metafile);
    if (body->SetUp()) geometry2body = body->geometry2body_pose();
  }
  std::vector<Eigen::Matrix4f> gt = publish_gt ? ReadGtPoses(fs::path{seq} / "poses_gt_matrix.txt")
                                               : std::vector<Eigen::Matrix4f>{};

  auto qos = rclcpp::SensorDataQoS();
  auto pub_color = node->create_publisher<sensor_msgs::msg::Image>(color_topic, qos);
  auto pub_cinfo = node->create_publisher<sensor_msgs::msg::CameraInfo>(color_info_topic, qos);
  auto pub_depth = depth_cam ? node->create_publisher<sensor_msgs::msg::Image>(depth_topic, qos) : nullptr;
  auto pub_dinfo = depth_cam ? node->create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic, qos) : nullptr;
  auto pub_marker_gt = node->create_publisher<visualization_msgs::msg::Marker>("~/marker_gt", 1);
  tf2_ros::TransformBroadcaster tf{node};

  auto cinfo = MakeInfo(color_cam->intrinsics(), frame);
  auto dinfo = depth_cam ? MakeInfo(depth_cam->intrinsics(), frame) : sensor_msgs::msg::CameraInfo{};

  auto advance = [&]() {
    bool c = color_cam->UpdateImage(true);
    bool d = depth_cam ? depth_cam->UpdateImage(true) : true;
    return c && d;
  };

  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>{rate > 0 ? 1.0 / rate : 1.0 / 30.0});
  auto timer = node->create_wall_timer(period, [&]() {
    if (!advance()) {
      if (!loop) { rclcpp::shutdown(); return; }
      color_cam->SetUp(); if (depth_cam) depth_cam->SetUp();
      advance();
    }
    const rclcpp::Time stamp = node->now();
    std_msgs::msg::Header h; h.stamp = stamp; h.frame_id = frame;

    cinfo.header.stamp = stamp;
    pub_color->publish(*cv_bridge::CvImage(h, "bgr8", color_cam->image()).toImageMsg());
    pub_cinfo->publish(cinfo);
    if (depth_cam) {
      dinfo.header.stamp = stamp;
      pub_depth->publish(*cv_bridge::CvImage(h, "16UC1", depth_cam->image()).toImageMsg());
      pub_dinfo->publish(dinfo);
    }

    if (!gt.empty()) {
      const int f = std::max(0, std::min(color_cam->load_index() - 1, (int)gt.size() - 1));
      const m3t::Transform3fA b2w{gt[f]};
      const m3t::Transform3fA g2w = b2w * geometry2body;
      const Eigen::Vector3f t = b2w.translation();
      const Eigen::Quaternionf q{b2w.rotation()};
      geometry_msgs::msg::TransformStamped tfm;
      tfm.header.stamp = stamp; tfm.header.frame_id = frame; tfm.child_frame_id = "object_gt";
      tfm.transform.translation.x = t.x(); tfm.transform.translation.y = t.y();
      tfm.transform.translation.z = t.z();
      tfm.transform.rotation.x = q.x(); tfm.transform.rotation.y = q.y();
      tfm.transform.rotation.z = q.z(); tfm.transform.rotation.w = q.w();
      tf.sendTransform(tfm);

      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = frame; mk.header.stamp = stamp; mk.ns = "gt"; mk.id = 0;
      mk.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.mesh_resource = mesh_resource; mk.mesh_use_embedded_materials = mesh_embedded;
      const Eigen::Vector3f gt2 = g2w.translation();
      const Eigen::Quaternionf gq{g2w.rotation()};
      mk.pose.position.x = gt2.x(); mk.pose.position.y = gt2.y(); mk.pose.position.z = gt2.z();
      mk.pose.orientation.x = gq.x(); mk.pose.orientation.y = gq.y();
      mk.pose.orientation.z = gq.z(); mk.pose.orientation.w = gq.w();
      mk.scale.x = mk.scale.y = mk.scale.z = mesh_scale;
      mk.color.r = 0.1; mk.color.g = 0.9; mk.color.b = 0.1; mk.color.a = 0.5;
      pub_marker_gt->publish(mk);
    }
  });

  RCLCPP_INFO(node->get_logger(), "publishing %s at %.0f Hz on %s / %s",
              seq.c_str(), rate, color_topic.c_str(), depth_topic.c_str());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
