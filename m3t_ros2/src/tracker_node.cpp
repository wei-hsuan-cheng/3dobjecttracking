// SPDX-License-Identifier: MIT
// m3t_ros2 tracker node — single in-process node that runs the M3T tracker on a
// recorded (synthetic) RGB-D sequence read from disk (no inter-node topics for
// the images), with object / initial pose / modality combination selected via
// parameters, and publishes color/depth/keypoint/overlay images + TF and mesh
// markers for estimate and ground truth. No GUI window.

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <m3t/body.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/link.h>
#include <m3t/loader_camera.h>
#include <m3t/normal_renderer.h>
#include <m3t/optimizer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>

#include "m3t_ros2/ros_publisher.hpp"

namespace fs = std::filesystem;

static bool HasModality(const std::string &list, const std::string &m) {
  return ("," + list + ",").find("," + m + ",") != std::string::npos;
}

static std::vector<Eigen::Matrix4f> ReadGtPoses(const fs::path &path) {
  std::vector<Eigen::Matrix4f> poses;
  std::ifstream ifs{path.string()};
  std::string line;
  while (std::getline(ifs, line)) {
    std::istringstream iss{line};
    Eigen::Matrix4f m;
    int i = 0;
    float v;
    while (i < 16 && (iss >> v)) m(i / 4, i % 4) = v, ++i;
    if (i == 16) poses.push_back(m);
  }
  return poses;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("m3t_tracker_node");

  // ---- Parameters ------------------------------------------------------------
  const auto sequence_dir =
      node->declare_parameter<std::string>("sequence_dir", "");
  const auto body_metafile =
      node->declare_parameter<std::string>("body_metafile", "");
  const auto modalities =
      node->declare_parameter<std::string>("modalities", "region");
  const int fps = static_cast<int>(node->declare_parameter<int>("fps", 20));
  const bool loop = node->declare_parameter<bool>("loop", true);
  auto temp_dir = node->declare_parameter<std::string>("temp_dir", "");
  if (temp_dir.empty()) temp_dir = sequence_dir;

  m3t_ros2::RosPublisherConfig cfg;
  cfg.world_frame = node->declare_parameter<std::string>("world_frame", "camera");
  cfg.mesh_resource = node->declare_parameter<std::string>("mesh_resource", "");
  cfg.mesh_scale = node->declare_parameter<double>("mesh_scale", 1.0);
  cfg.mesh_use_embedded_materials =
      node->declare_parameter<bool>("mesh_use_embedded_materials", false);
  cfg.publish_gt = node->declare_parameter<bool>("publish_gt", true);
  cfg.publish_keypoints = node->declare_parameter<bool>("publish_keypoints", true);
  cfg.publish_overlay = node->declare_parameter<bool>("publish_overlay", true);

  if (sequence_dir.empty() || body_metafile.empty()) {
    RCLCPP_FATAL(node->get_logger(),
                 "sequence_dir and body_metafile parameters are required");
    return 1;
  }
  const bool use_region = HasModality(modalities, "region");
  const bool use_depth = HasModality(modalities, "depth");
  const bool use_texture = HasModality(modalities, "texture");
  RCLCPP_INFO(node->get_logger(), "modalities: region=%d depth=%d texture=%d",
              use_region, use_depth, use_texture);

  // ---- Build the tracker -----------------------------------------------------
  auto tracker = std::make_shared<m3t::Tracker>(
      "tracker", 5, 2, /*synchronize_cameras=*/false,
      /*start_tracking_after_detection=*/true,
      std::chrono::milliseconds{fps > 0 ? 1000 / fps : 33});

  auto color_camera = std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", fs::path{sequence_dir} / "color_camera.yaml");
  std::shared_ptr<m3t::LoaderDepthCamera> depth_camera;
  if (use_depth)
    depth_camera = std::make_shared<m3t::LoaderDepthCamera>(
        "depth_camera", fs::path{sequence_dir} / "depth_camera.yaml");

  auto body = std::make_shared<m3t::Body>("body", body_metafile);
  auto link = std::make_shared<m3t::Link>("link", body);

  if (use_region) {
    auto rm = std::make_shared<m3t::RegionModel>(
        "region_model", body, fs::path{temp_dir} / "region_model.bin");
    link->AddModality(std::make_shared<m3t::RegionModality>(
        "region_modality", body, color_camera, rm));
  }
  if (use_depth) {
    auto dm = std::make_shared<m3t::DepthModel>(
        "depth_model", body, fs::path{temp_dir} / "depth_model.bin");
    link->AddModality(std::make_shared<m3t::DepthModality>(
        "depth_modality", body, depth_camera, dm));
  }
  std::shared_ptr<m3t::RendererGeometry> rg_texture;
  if (use_texture) {
    rg_texture = std::make_shared<m3t::RendererGeometry>("rg_texture");
    rg_texture->AddBody(body);
    auto sr = std::make_shared<m3t::FocusedSilhouetteRenderer>(
        "silhouette_renderer", rg_texture, color_camera);
    sr->AddReferencedBody(body);
    link->AddModality(std::make_shared<m3t::TextureModality>(
        "texture_modality", body, color_camera, sr));
  }

  auto optimizer = std::make_shared<m3t::Optimizer>("optimizer", link);
  tracker->AddOptimizer(optimizer);
  tracker->AddDetector(std::make_shared<m3t::StaticDetector>(
      "detector", fs::path{sequence_dir} / "static_detector.yaml", optimizer));

  // Overlay renderer (its own geometry so it is independent of any modality).
  auto rg_overlay = std::make_shared<m3t::RendererGeometry>("rg_overlay");
  rg_overlay->AddBody(body);
  auto overlay_renderer = std::make_shared<m3t::FullNormalRenderer>(
      "overlay_renderer", rg_overlay, color_camera, 0.01f, 10.0f);

  // Ground-truth poses (full 4x4 per frame), if present.
  std::vector<Eigen::Matrix4f> gt_poses;
  if (cfg.publish_gt) {
    gt_poses = ReadGtPoses(fs::path{sequence_dir} / "poses_gt_matrix.txt");
    if (gt_poses.empty()) {
      RCLCPP_WARN(node->get_logger(),
                  "poses_gt_matrix.txt not found/empty — GT disabled");
      cfg.publish_gt = false;
    }
  }

  auto ros_publisher = std::make_shared<m3t_ros2::RosPublisher>(
      "ros_publisher", node.get(), color_camera, depth_camera, body,
      overlay_renderer, gt_poses, cfg);
  tracker->AddPublisher(ros_publisher);

  if (!tracker->SetUp()) {
    RCLCPP_FATAL(node->get_logger(), "tracker SetUp failed");
    return 1;
  }
  // Overlay geometry/renderer set up after the tracker (body + camera ready).
  if (!rg_overlay->SetUp() || !overlay_renderer->SetUp()) {
    RCLCPP_FATAL(node->get_logger(), "overlay renderer SetUp failed");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "tracking %s (fps=%d, loop=%d, gt=%d)",
              body_metafile.c_str(), fps, loop, cfg.publish_gt);

  // ---- Run the tracker in a thread; spin ROS in the main thread --------------
  std::atomic<bool> running{true};
  std::thread worker([&]() {
    while (running && rclcpp::ok()) {
      color_camera->SetUp();               // reset to first frame
      if (depth_camera) depth_camera->SetUp();
      tracker->RunTrackerProcess(/*execute_detection=*/true,
                                 /*start_tracking=*/false);
      if (!loop) break;
    }
  });

  rclcpp::spin(node);
  running = false;
  tracker->QuitTrackerProcess();
  worker.join();
  rclcpp::shutdown();
  return 0;
}
