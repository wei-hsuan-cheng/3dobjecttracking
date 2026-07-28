// SPDX-License-Identifier: MIT
// m3t_ros2 tracker node — node 2.
//
// Subscribes to color + depth image topics (+ CameraInfo) from any source (the
// sequence publisher, a bag, or a real camera driver — only topic names differ)
// via ROS-backed M3T cameras, and runs the tracker. Same two-thread structure
// as before: a worker thread does the pure pose solve (timed, logged every
// log_period) and all OpenGL; a decoupled wall-timer publishes the estimate
// overlay/keypoints/marker/TF from a mutex-guarded snapshot. It does NOT decode
// images from disk, so its loop is bounded by the solve, not by PNG decoding.

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/imgproc.hpp>

#include <m3t/body.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/link.h>
#include <m3t/normal_renderer.h>
#include <m3t/optimizer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
#include <m3t/tracker.h>

#include "m3t_ros2/ros_camera.hpp"
#include "m3t_ros2/ros_publisher.hpp"

namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;
using dsec = std::chrono::duration<double>;

static bool HasModality(const std::string &l, const std::string &m) {
  return ("," + l + ",").find("," + m + ",") != std::string::npos;
}
static m3t::Intrinsics FromInfo(const sensor_msgs::msg::CameraInfo &i) {
  return {static_cast<float>(i.k[0]), static_cast<float>(i.k[4]),
          static_cast<float>(i.k[2]), static_cast<float>(i.k[5]),
          static_cast<int>(i.width), static_cast<int>(i.height)};
}
static m3t::Transform3fA TfToTransform(const geometry_msgs::msg::Transform &tr) {
  m3t::Transform3fA t{m3t::Transform3fA::Identity()};
  t.translation() = Eigen::Vector3f(tr.translation.x, tr.translation.y, tr.translation.z);
  t.linear() = Eigen::Quaternionf(tr.rotation.w, tr.rotation.x, tr.rotation.y,
                                  tr.rotation.z).toRotationMatrix();
  return t;
}
static float RotErrorDeg(const Eigen::Matrix3f &a, const Eigen::Matrix3f &b) {
  const float c = 0.5f * ((a.transpose() * b).trace() - 1.0f);
  return std::acos(std::max(-1.0f, std::min(1.0f, c))) * 180.0f / float(M_PI);
}
static cv::Mat Composite(const cv::Mat &color, const cv::Mat &nrm) {
  cv::Mat nb, ov = color.clone();
  cv::cvtColor(nrm, nb, cv::COLOR_BGRA2BGR);
  for (int y = 0; y < ov.rows; ++y)
    for (int x = 0; x < ov.cols; ++x) {
      const cv::Vec3b n = nb.at<cv::Vec3b>(y, x);
      if (n[0] || n[1] || n[2]) {
        cv::Vec3b &o = ov.at<cv::Vec3b>(y, x);
        for (int c = 0; c < 3; ++c) o[c] = cv::saturate_cast<uchar>(0.5 * o[c] + 0.5 * n[c]);
      }
    }
  return ov;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("m3t_tracker_node");

  const auto body_metafile = node->declare_parameter<std::string>("body_metafile", "");
  const auto modalities = node->declare_parameter<std::string>("modalities", "region");
  auto temp_dir = node->declare_parameter<std::string>("temp_dir", "");
  const double track_rate = node->declare_parameter<double>("track_rate", 0.0);
  const double publish_rate = node->declare_parameter<double>("publish_rate", 30.0);
  const double log_period = node->declare_parameter<double>("log_period", 2.0);
  // true  = solve once per NEW frame (honest per-frame solve time; loop = source rate)
  // false = free-run, re-solving the latest frame (shows compute headroom, dilutes solve time)
  const bool event_driven = node->declare_parameter<bool>("event_driven", true);
  const float depth_scale = node->declare_parameter<double>("depth_scale", 0.001);
  const auto color_topic = node->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
  const auto depth_topic = node->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  const auto color_info_topic = node->declare_parameter<std::string>("color_info_topic", "/camera/color/camera_info");
  const auto depth_info_topic = node->declare_parameter<std::string>("depth_info_topic", "/camera/depth/camera_info");
  const auto gt_frame = node->declare_parameter<std::string>("gt_frame", "object_gt");
  const double lost_threshold = node->declare_parameter<double>("lost_threshold", 0.05);  // m
  // true  = seed the initial pose from the (image-aligned) GT/detector pose topic.
  // false = seed from a fixed static_detector.yaml (real cameras: a known start
  //         pose or a real detector publishing on init_pose_topic).
  const bool use_gt_initial_pose = node->declare_parameter<bool>("use_gt_initial_pose", true);
  const bool publish_overlay = node->declare_parameter<bool>("publish_overlay", true);

  m3t_ros2::RosPublisherConfig cfg;
  cfg.world_frame = node->declare_parameter<std::string>("world_frame", "camera");
  cfg.mesh_resource = node->declare_parameter<std::string>("mesh_resource", "");
  cfg.mesh_scale = node->declare_parameter<double>("mesh_scale", 1.0);
  cfg.mesh_use_embedded_materials = node->declare_parameter<bool>("mesh_use_embedded_materials", false);
  cfg.publish_keypoints = node->declare_parameter<bool>("publish_keypoints", true);
  cfg.publish_color = false;  // node 1 publishes raw color/depth + GT
  cfg.publish_depth = false;
  cfg.publish_gt = false;

  if (body_metafile.empty()) { RCLCPP_FATAL(node->get_logger(), "body_metafile required"); return 1; }
  if (temp_dir.empty()) temp_dir = fs::path{body_metafile}.parent_path().string();
  const bool use_region = HasModality(modalities, "region");
  const bool use_depth = HasModality(modalities, "depth");
  const bool use_texture = HasModality(modalities, "texture");

  auto color_camera = std::make_shared<m3t_ros2::RosColorCamera>("color_camera");
  std::shared_ptr<m3t_ros2::RosDepthCamera> depth_camera;
  if (use_depth) {
    depth_camera = std::make_shared<m3t_ros2::RosDepthCamera>("depth_camera");
    depth_camera->SetDepthScale(depth_scale);
  }

  auto qos = rclcpp::SensorDataQoS();
  auto sub_color = node->create_subscription<sensor_msgs::msg::Image>(
      color_topic, qos, [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
        color_camera->SetLatest(cv_bridge::toCvCopy(m, "bgr8")->image);
      });
  auto sub_cinfo = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      color_info_topic, qos, [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        color_camera->SetIntrinsics(FromInfo(*m));
      });
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_dinfo;
  if (use_depth) {
    sub_depth = node->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, qos, [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          depth_camera->SetLatest(cv_bridge::toCvCopy(m, "16UC1")->image);
        });
    sub_dinfo = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_topic, qos, [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          depth_camera->SetIntrinsics(FromInfo(*m));
        });
  }

  // GT / detector pose via TF lookup (init seed + tracking-error reference).
  // node 1 broadcasts world_frame -> gt_frame; a real detector could publish it.
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf2_ros::TransformListener tf_listener{*tf_buffer};
  auto lookup_gt = [&](m3t::Transform3fA &out) -> bool {
    try {
      out = TfToTransform(
          tf_buffer->lookupTransform(cfg.world_frame, gt_frame, tf2::TimePointZero).transform);
      return true;
    } catch (const tf2::TransformException &) { return false; }
  };

  auto body = std::make_shared<m3t::Body>("body", body_metafile);
  auto link = std::make_shared<m3t::Link>("link", body);
  if (use_region) {
    auto rm = std::make_shared<m3t::RegionModel>("region_model", body, fs::path{temp_dir} / "region_model.bin");
    link->AddModality(std::make_shared<m3t::RegionModality>("region_modality", body, color_camera, rm));
  }
  if (use_depth) {
    auto dm = std::make_shared<m3t::DepthModel>("depth_model", body, fs::path{temp_dir} / "depth_model.bin");
    link->AddModality(std::make_shared<m3t::DepthModality>("depth_modality", body, depth_camera, dm));
  }
  if (use_texture) {
    auto rg = std::make_shared<m3t::RendererGeometry>("rg_texture");
    rg->AddBody(body);
    auto sr = std::make_shared<m3t::FocusedSilhouetteRenderer>("silhouette_renderer", rg, color_camera);
    sr->AddReferencedBody(body);
    link->AddModality(std::make_shared<m3t::TextureModality>("texture_modality", body, color_camera, sr));
  }
  auto optimizer = std::make_shared<m3t::Optimizer>("optimizer", link);
  auto tracker = std::make_shared<m3t::Tracker>("tracker", 5, 2, false, true);
  tracker->AddOptimizer(optimizer);
  // use_gt: seed pose is set at runtime from the pose topic (image-aligned).
  // else:  fixed pose from static_detector.yaml.
  std::shared_ptr<m3t::StaticDetector> detector =
      use_gt_initial_pose
          ? std::make_shared<m3t::StaticDetector>("detector", optimizer,
                                                  m3t::Transform3fA::Identity(), true)
          : std::make_shared<m3t::StaticDetector>(
                "detector", fs::path{temp_dir} / "static_detector.yaml", optimizer);
  tracker->AddDetector(detector);

  // Service to (re-)initialize from the latest pose (e.g. after a loss or a loop).
  std::atomic<bool> redetect_req{false};
  auto redetect_srv = node->create_service<std_srvs::srv::Trigger>(
      "~/redetect", [&](std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        redetect_req = true; resp->success = true; resp->message = "re-detect queued";
      });

  auto rg_overlay = std::make_shared<m3t::RendererGeometry>("rg_overlay");
  rg_overlay->AddBody(body);
  auto overlay_renderer = std::make_shared<m3t::FullNormalRenderer>(
      "overlay_renderer", rg_overlay, color_camera, 0.01f, 10.0f);

  // Wait for the first frames + intrinsics (+ init pose if seeding from GT).
  RCLCPP_INFO(node->get_logger(), "waiting for images on %s ...", color_topic.c_str());
  m3t::Transform3fA gt_tmp;
  while (rclcpp::ok() &&
         !(color_camera->HasImage() && color_camera->HasIntrinsics() &&
           (!use_depth || (depth_camera->HasImage() && depth_camera->HasIntrinsics())) &&
           (!use_gt_initial_pose || lookup_gt(gt_tmp))))
    rclcpp::spin_some(node);
  if (!rclcpp::ok()) return 0;
  RCLCPP_INFO(node->get_logger(), "got first frames — setting up tracker (region=%d depth=%d texture=%d)",
              use_region, use_depth, use_texture);

  if (!tracker->SetUp() || !rg_overlay->SetUp() || !overlay_renderer->SetUp()) {
    RCLCPP_FATAL(node->get_logger(), "SetUp failed"); return 1; }

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

  std::atomic<bool> running{true};
  std::thread worker([&]() {
    // Seed the initial guess from the GT/detector TF (image-aligned), then detect.
    if (use_gt_initial_pose) {
      m3t::Transform3fA g;
      if (lookup_gt(g)) detector->set_link2world_pose(g);
    }
    tracker->ExecuteDetection(false);
    auto last_log = clk::now(), last_snap = clk::now(), win = clk::now();
    int solve_n = 0, loop_n = 0, err_n = 0, lost_n = 0;
    double solve_sum = 0, solve_min = 1e9, solve_max = 0;
    double perr_sum = 0, perr_max = 0, rerr_sum = 0, rerr_max = 0;
    const dsec track_period{track_rate > 0 ? 1.0 / track_rate : 0.0};
    const dsec snap_period{publish_rate > 0 ? 1.0 / publish_rate : 1.0 / 30.0};
    uint64_t last_seq = 0;
    for (int it = 0; running && rclcpp::ok(); ++it) {
      // Re-initialize on request (service) from the latest GT/detector pose.
      if (redetect_req.exchange(false)) {
        m3t::Transform3fA g;
        if (lookup_gt(g)) detector->set_link2world_pose(g);
        tracker->ExecuteDetection(false);
      }
      // Event-driven: process each new frame exactly once (honest solve time).
      if (event_driven) {
        while (running && rclcpp::ok() && color_camera->seq() == last_seq)
          std::this_thread::sleep_for(std::chrono::microseconds{200});
        last_seq = color_camera->seq();
      }
      const auto iter_t0 = clk::now();
      if (!tracker->UpdateCameras(it)) { std::this_thread::sleep_for(std::chrono::milliseconds{1}); continue; }
      tracker->UpdateSubscribers(it);
      tracker->CalculateConsistentPoses();
      tracker->ExecuteDetectingStep(it);
      tracker->ExecuteStartingStep(it);
      const auto s0 = clk::now();
      tracker->ExecuteTrackingStep(it);  // *** pure pose solve ***
      const double solve_ms = dsec(clk::now() - s0).count() * 1e3;
      solve_sum += solve_ms; solve_min = std::min(solve_min, solve_ms);
      solve_max = std::max(solve_max, solve_ms); ++solve_n; ++loop_n;

      // Tracking-error monitor: estimate vs GT (from TF). Tells on-track vs lost,
      // so the solve time above is only trusted while OK.
      m3t::Transform3fA gtp;
      if (lookup_gt(gtp)) {
        const auto est = body->body2world_pose();
        const double perr = (est.translation() - gtp.translation()).norm();
        perr_sum += perr; perr_max = std::max(perr_max, perr);
        const double rerr = RotErrorDeg(est.rotation(), gtp.rotation());
        rerr_sum += rerr; rerr_max = std::max(rerr_max, rerr);
        if (perr > lost_threshold) ++lost_n;
        ++err_n;
      }

      if (clk::now() - last_snap >= snap_period) {
        cv::Mat overlay;
        if (publish_overlay && overlay_renderer->StartRendering() && overlay_renderer->FetchNormalImage())
          overlay = Composite(color_camera->image(), overlay_renderer->normal_image());
        std::lock_guard<std::mutex> lk{snap_mutex};
        snap.color = color_camera->image();
        snap.overlay = overlay;
        snap.body2world_est = body->body2world_pose();
        snap.geometry2world_est = body->geometry2world_pose();
        snap.valid = true;
        last_snap = clk::now();
      }
      if (clk::now() - last_log >= dsec{log_period}) {
        const double w = dsec(clk::now() - win).count();
        const double mean = solve_sum / std::max(1, solve_n);
        const double pmean = err_n ? 1e3 * perr_sum / err_n : -1.0;   // mm
        const double rmean = err_n ? rerr_sum / err_n : -1.0;         // deg
        const bool tracked = err_n > 0 && lost_n * 2 <= err_n;        // majority within threshold
        RCLCPP_INFO(node->get_logger(),
                    "solve: %d | mean %.2f ms (%.0f Hz) | min %.2f max %.2f | loop %.0f Hz"
                    " | err pos %.1f/%.1f mm rot %.1f/%.1f deg | %s",
                    solve_n, mean, mean > 0 ? 1000.0 / mean : 0.0, solve_min, solve_max,
                    loop_n / std::max(1e-6, w), pmean, 1e3 * perr_max, rmean, rerr_max,
                    err_n == 0 ? "no-GT" : (tracked ? "TRACKED" : "LOST"));
        solve_n = loop_n = err_n = lost_n = 0;
        solve_sum = 0; solve_min = 1e9; solve_max = 0;
        perr_sum = perr_max = rerr_sum = rerr_max = 0;
        win = last_log = clk::now();
      }
      if (track_period.count() > 0.0)
        std::this_thread::sleep_until(iter_t0 + std::chrono::duration_cast<clk::duration>(track_period));
    }
  });

  RCLCPP_INFO(node->get_logger(), "tracking | track_rate=%.0f publish_rate=%.0f log_period=%.1fs",
              track_rate, publish_rate, log_period);
  rclcpp::spin(node);
  running = false;
  worker.join();
  rclcpp::shutdown();
  return 0;
}
