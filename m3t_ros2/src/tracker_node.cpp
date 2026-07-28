// SPDX-License-Identifier: MIT
// m3t_ros2 tracker node.
//
// Threading: a dedicated worker thread drives the M3T tracker step by step and
// measures the pure pose-solve time (ExecuteTrackingStep). Publishing is fully
// decoupled — a ROS wall-timer reads the latest snapshot and does all ROS
// serialization + the ORB keypoint image on the spin thread, so it never blocks
// the solve loop. All OpenGL (tracker renderers + overlay) stays on the worker
// thread. Solve-time statistics are logged every `log_period` seconds.

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgproc.hpp>

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
using clk = std::chrono::steady_clock;
using dsec = std::chrono::duration<double>;

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
    int i = 0; float v;
    while (i < 16 && (iss >> v)) { m(i / 4, i % 4) = v; ++i; }
    if (i == 16) poses.push_back(m);
  }
  return poses;
}

static cv::Mat Composite(const cv::Mat &color, const cv::Mat &normal_bgra) {
  cv::Mat nb, ov = color.clone();
  cv::cvtColor(normal_bgra, nb, cv::COLOR_BGRA2BGR);
  for (int y = 0; y < ov.rows; ++y)
    for (int x = 0; x < ov.cols; ++x) {
      const cv::Vec3b n = nb.at<cv::Vec3b>(y, x);
      if (n[0] || n[1] || n[2]) {
        cv::Vec3b &o = ov.at<cv::Vec3b>(y, x);
        for (int c = 0; c < 3; ++c)
          o[c] = cv::saturate_cast<uchar>(0.5 * o[c] + 0.5 * n[c]);
      }
    }
  return ov;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("m3t_tracker_node");

  const auto sequence_dir = node->declare_parameter<std::string>("sequence_dir", "");
  const auto body_metafile = node->declare_parameter<std::string>("body_metafile", "");
  const auto modalities = node->declare_parameter<std::string>("modalities", "region");
  auto temp_dir = node->declare_parameter<std::string>("temp_dir", "");
  if (temp_dir.empty()) temp_dir = sequence_dir;
  const double track_rate = node->declare_parameter<double>("track_rate", 0.0);   // 0 = as fast as possible
  const double publish_rate = node->declare_parameter<double>("publish_rate", 30.0);
  const double log_period = node->declare_parameter<double>("log_period", 2.0);

  m3t_ros2::RosPublisherConfig cfg;
  cfg.world_frame = node->declare_parameter<std::string>("world_frame", "camera");
  cfg.mesh_resource = node->declare_parameter<std::string>("mesh_resource", "");
  cfg.mesh_scale = node->declare_parameter<double>("mesh_scale", 1.0);
  cfg.mesh_use_embedded_materials = node->declare_parameter<bool>("mesh_use_embedded_materials", false);
  cfg.publish_keypoints = node->declare_parameter<bool>("publish_keypoints", true);
  const bool publish_overlay = node->declare_parameter<bool>("publish_overlay", true);
  bool publish_gt = node->declare_parameter<bool>("publish_gt", true);

  if (sequence_dir.empty() || body_metafile.empty()) {
    RCLCPP_FATAL(node->get_logger(), "sequence_dir and body_metafile are required");
    return 1;
  }
  const bool use_region = HasModality(modalities, "region");
  const bool use_depth = HasModality(modalities, "depth");
  const bool use_texture = HasModality(modalities, "texture");
  RCLCPP_INFO(node->get_logger(), "modalities: region=%d depth=%d texture=%d",
              use_region, use_depth, use_texture);

  auto tracker = std::make_shared<m3t::Tracker>(
      "tracker", 5, 2, /*synchronize_cameras=*/false,
      /*start_tracking_after_detection=*/true);

  auto color_camera = std::make_shared<m3t::LoaderColorCamera>(
      "color_camera", fs::path{sequence_dir} / "color_camera.yaml");
  std::shared_ptr<m3t::LoaderDepthCamera> depth_camera;
  if (use_depth)
    depth_camera = std::make_shared<m3t::LoaderDepthCamera>(
        "depth_camera", fs::path{sequence_dir} / "depth_camera.yaml");

  auto body = std::make_shared<m3t::Body>("body", body_metafile);
  auto link = std::make_shared<m3t::Link>("link", body);
  if (use_region) {
    auto rm = std::make_shared<m3t::RegionModel>("region_model", body,
                                                 fs::path{temp_dir} / "region_model.bin");
    link->AddModality(std::make_shared<m3t::RegionModality>("region_modality", body, color_camera, rm));
  }
  if (use_depth) {
    auto dm = std::make_shared<m3t::DepthModel>("depth_model", body,
                                                fs::path{temp_dir} / "depth_model.bin");
    link->AddModality(std::make_shared<m3t::DepthModality>("depth_modality", body, depth_camera, dm));
  }
  std::shared_ptr<m3t::RendererGeometry> rg_texture;
  if (use_texture) {
    rg_texture = std::make_shared<m3t::RendererGeometry>("rg_texture");
    rg_texture->AddBody(body);
    auto sr = std::make_shared<m3t::FocusedSilhouetteRenderer>("silhouette_renderer", rg_texture, color_camera);
    sr->AddReferencedBody(body);
    link->AddModality(std::make_shared<m3t::TextureModality>("texture_modality", body, color_camera, sr));
  }
  auto optimizer = std::make_shared<m3t::Optimizer>("optimizer", link);
  tracker->AddOptimizer(optimizer);
  tracker->AddDetector(std::make_shared<m3t::StaticDetector>(
      "detector", fs::path{sequence_dir} / "static_detector.yaml", optimizer));

  auto rg_overlay = std::make_shared<m3t::RendererGeometry>("rg_overlay");
  rg_overlay->AddBody(body);
  auto overlay_renderer = std::make_shared<m3t::FullNormalRenderer>(
      "overlay_renderer", rg_overlay, color_camera, 0.01f, 10.0f);

  std::vector<Eigen::Matrix4f> gt_poses;
  if (publish_gt) {
    gt_poses = ReadGtPoses(fs::path{sequence_dir} / "poses_gt_matrix.txt");
    if (gt_poses.empty()) { publish_gt = false;
      RCLCPP_WARN(node->get_logger(), "no poses_gt_matrix.txt — GT disabled"); }
  }

  if (!tracker->SetUp()) { RCLCPP_FATAL(node->get_logger(), "tracker SetUp failed"); return 1; }
  if (!rg_overlay->SetUp() || !overlay_renderer->SetUp()) {
    RCLCPP_FATAL(node->get_logger(), "overlay SetUp failed"); return 1; }
  const m3t::Transform3fA geometry2body = body->geometry2body_pose();

  // ---- Decoupled publisher on a wall-timer -----------------------------------
  auto ros_publisher = std::make_shared<m3t_ros2::RosPublisher>(node.get(), cfg);
  std::mutex snap_mutex;
  m3t_ros2::Snapshot snap;
  const auto pub_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      dsec{publish_rate > 0 ? 1.0 / publish_rate : 1.0 / 30.0});
  auto pub_timer = node->create_wall_timer(pub_period, [&]() {
    m3t_ros2::Snapshot local;
    { std::lock_guard<std::mutex> lk{snap_mutex}; if (!snap.valid) return; local = snap; }
    ros_publisher->Publish(local);
  });

  // ---- Worker thread: pure solve + timing (GL lives here) --------------------
  std::atomic<bool> running{true};
  std::thread worker([&]() {
    auto last_log = clk::now();
    auto last_snap = clk::now();
    auto win_start = clk::now();
    int solve_n = 0, loop_n = 0;
    double solve_sum = 0, solve_min = 1e9, solve_max = 0;
    const dsec track_period{track_rate > 0 ? 1.0 / track_rate : 0.0};
    const dsec snap_period{publish_rate > 0 ? 1.0 / publish_rate : 1.0 / 30.0};

    while (running && rclcpp::ok()) {
      color_camera->SetUp();
      if (depth_camera) depth_camera->SetUp();
      tracker->ExecuteDetection(false);          // re-detect from the static pose
      for (int it = 0; running && rclcpp::ok(); ++it) {
        const auto iter_t0 = clk::now();
        if (!tracker->UpdateCameras(it)) break;  // end of sequence -> re-detect
        tracker->UpdateSubscribers(it);
        tracker->CalculateConsistentPoses();
        tracker->ExecuteDetectingStep(it);
        tracker->ExecuteStartingStep(it);

        const auto s0 = clk::now();
        tracker->ExecuteTrackingStep(it);        // *** pure pose solve ***
        const double solve_ms = dsec(clk::now() - s0).count() * 1e3;
        solve_sum += solve_ms; solve_min = std::min(solve_min, solve_ms);
        solve_max = std::max(solve_max, solve_ms); ++solve_n; ++loop_n;

        // Snapshot (+ overlay render) at the publish rate, not every solve.
        if (clk::now() - last_snap >= snap_period) {
          cv::Mat overlay;
          if (publish_overlay && overlay_renderer->StartRendering() &&
              overlay_renderer->FetchNormalImage())
            overlay = Composite(color_camera->image(), overlay_renderer->normal_image());
          const int frame = std::max(0, color_camera->load_index() - 1);
          std::lock_guard<std::mutex> lk{snap_mutex};
          snap.color = color_camera->image().clone();
          if (depth_camera) { snap.depth = depth_camera->image().clone(); snap.has_depth = true; }
          snap.overlay = overlay;
          snap.body2world_est = body->body2world_pose();
          snap.geometry2world_est = body->geometry2world_pose();
          if (publish_gt && frame < (int)gt_poses.size()) {
            snap.body2world_gt = m3t::Transform3fA{gt_poses[frame]};
            snap.geometry2world_gt = snap.body2world_gt * geometry2body;
            snap.has_gt = true;
          }
          snap.valid = true;
          last_snap = clk::now();
        }

        if (clk::now() - last_log >= dsec{log_period}) {
          const double win = dsec(clk::now() - win_start).count();
          const double mean = solve_sum / std::max(1, solve_n);
          RCLCPP_INFO(node->get_logger(),
                      "solve: %d frames | mean %.2f ms (%.0f Hz) | min %.2f max %.2f ms | loop %.0f Hz",
                      solve_n, mean, mean > 0 ? 1000.0 / mean : 0.0, solve_min, solve_max,
                      loop_n / std::max(1e-6, win));
          solve_n = loop_n = 0; solve_sum = 0; solve_min = 1e9; solve_max = 0;
          win_start = last_log = clk::now();
        }

        if (track_period.count() > 0.0) {
          const auto target = iter_t0 + std::chrono::duration_cast<clk::duration>(track_period);
          std::this_thread::sleep_until(target);
        }
      }
    }
  });

  RCLCPP_INFO(node->get_logger(),
              "tracking %s | track_rate=%.0f publish_rate=%.0f log_period=%.1fs",
              body_metafile.c_str(), track_rate, publish_rate, log_period);
  rclcpp::spin(node);
  running = false;
  worker.join();
  rclcpp::shutdown();
  return 0;
}
