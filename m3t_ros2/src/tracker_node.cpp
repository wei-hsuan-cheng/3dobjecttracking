// SPDX-License-Identifier: MIT
// m3t_ros2 tracker node — node 2.
//
// Subscribes to color + depth image topics (+ CameraInfo) from any source (the
// sequence publisher, a bag, or a real camera driver — only topic names differ)
// via ROS-backed M3T cameras, and runs the tracker. Same two-thread structure
// as before: a worker thread does the pure pose solve (timed, logged every
// log_period) and optional overlay rendering; a decoupled wall-timer publishes
// the enabled outputs from a mutex-guarded snapshot. It does NOT decode images
// from disk.

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

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
#include "m3t_ros2/body_factory.hpp"
#include "m3t_ros2/runtime_paths.hpp"

namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;
using dsec = std::chrono::duration<double>;

static bool HasOption(const std::string &l, const std::string &m) {
  return ("," + l + ",").find("," + m + ",") != std::string::npos;
}
static bool HasOnlyOptions(const std::string &list,
                           const std::vector<std::string> &allowed) {
  if (list.empty() || list == "none") return true;
  size_t begin = 0;
  while (begin <= list.size()) {
    const size_t end = list.find(',', begin);
    const auto option = list.substr(begin, end - begin);
    if (option.empty() ||
        std::find(allowed.begin(), allowed.end(), option) == allowed.end()) {
      return false;
    }
    if (end == std::string::npos) break;
    begin = end + 1;
  }
  return true;
}
static m3t::Intrinsics FromInfo(const sensor_msgs::msg::CameraInfo &i) {
  return {static_cast<float>(i.k[0]), static_cast<float>(i.k[4]),
          static_cast<float>(i.k[2]), static_cast<float>(i.k[5]),
          static_cast<int>(i.width), static_cast<int>(i.height)};
}
static bool HasValidIntrinsics(const sensor_msgs::msg::CameraInfo &i) {
  return i.width > 0 && i.height > 0 && i.k[0] > 0.0 && i.k[4] > 0.0;
}
static m3t::Transform3fA TfToTransform(const geometry_msgs::msg::Transform &tr) {
  m3t::Transform3fA t{m3t::Transform3fA::Identity()};
  t.translation() = Eigen::Vector3f(tr.translation.x, tr.translation.y, tr.translation.z);
  t.linear() = Eigen::Quaternionf(tr.rotation.w, tr.rotation.x, tr.rotation.y,
                                  tr.rotation.z).toRotationMatrix();
  return t;
}
static float RotationDistanceDeg(const Eigen::Matrix3f &a,
                                 const Eigen::Matrix3f &b) {
  const float c = 0.5f * ((a.transpose() * b).trace() - 1.0f);
  return std::acos(std::max(-1.0f, std::min(1.0f, c))) * 180.0f / float(M_PI);
}
static std::vector<Eigen::Matrix3f> ParseRotationSymmetries(
    const std::vector<double> &values) {
  if (values.size() % 9 != 0) {
    throw std::invalid_argument(
        "rotation_symmetries must contain row-major 3x3 matrices");
  }
  std::vector<Eigen::Matrix3f> symmetries{Eigen::Matrix3f::Identity()};
  for (size_t offset = 0; offset < values.size(); offset += 9) {
    Eigen::Matrix3f symmetry;
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        symmetry(row, column) =
            static_cast<float>(values[offset + 3 * row + column]);
      }
    }
    const float orthogonality_error =
        (symmetry.transpose() * symmetry - Eigen::Matrix3f::Identity()).norm();
    if (!symmetry.allFinite() || orthogonality_error > 1.0e-4f ||
        std::abs(symmetry.determinant() - 1.0f) > 1.0e-4f) {
      throw std::invalid_argument(
          "rotation_symmetries entries must be valid rotation matrices");
    }
    symmetries.push_back(symmetry);
  }
  return symmetries;
}
static float SymmetricRotationErrorDeg(
    const Eigen::Matrix3f &estimate, const Eigen::Matrix3f &reference,
    const std::vector<Eigen::Matrix3f> &symmetries) {
  float best_error = 180.0f;
  for (const auto &symmetry : symmetries) {
    best_error = std::min(
        best_error,
        RotationDistanceDeg(estimate * symmetry, reference));
  }
  return best_error;
}
static m3t::Transform3fA ClosestSymmetricPose(
    const m3t::Transform3fA &pose, const m3t::Transform3fA &reference,
    const std::vector<Eigen::Matrix3f> &symmetries) {
  m3t::Transform3fA closest = pose;
  float best_error =
      RotationDistanceDeg(pose.rotation(), reference.rotation());
  for (const auto &symmetry : symmetries) {
    m3t::Transform3fA candidate = pose;
    candidate.linear() = pose.rotation() * symmetry;
    const float error =
        RotationDistanceDeg(candidate.rotation(), reference.rotation());
    if (error < best_error) {
      best_error = error;
      closest = candidate;
    }
  }
  return closest;
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

  const auto modalities = node->declare_parameter<std::string>(
      "modalities", "region,depth");
  auto model_cache_dir =
      node->declare_parameter<std::string>("model_cache_dir", "");
  const auto initial_pose_values =
      node->declare_parameter<std::vector<double>>(
          "initial_pose",
          {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
           0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 1.0});
  const auto rotation_symmetry_values =
      node->declare_parameter<std::vector<double>>(
          "rotation_symmetries", std::vector<double>{});
  const double track_rate = node->declare_parameter<double>("track_rate", 0.0);
  const double publish_rate = node->declare_parameter<double>("publish_rate", 60.0);
  const double log_period = node->declare_parameter<double>("log_period", 2.0);
  // true  = solve once per NEW frame (honest per-frame solve time; loop = source rate)
  // false = benchmark mode that repeatedly solves the latest frame; this can
  // over-update stateful modalities and is not intended for normal tracking.
  const bool event_driven = node->declare_parameter<bool>("event_driven", true);
  const auto image_outputs = node->declare_parameter<std::string>(
      "image_outputs", "none");
  const float depth_scale = node->declare_parameter<double>("depth_scale", 0.001);
  const double sync_tolerance =
      node->declare_parameter<double>("sync_tolerance", 0.02);
  const auto color_topic = node->declare_parameter<std::string>("color_topic", "/camera/color/image_raw");
  const auto depth_topic = node->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
  const auto color_info_topic = node->declare_parameter<std::string>("color_info_topic", "/camera/color/camera_info");
  const auto depth_info_topic = node->declare_parameter<std::string>("depth_info_topic", "/camera/depth/camera_info");
  const auto gt_frame = node->declare_parameter<std::string>("gt_frame", "object_gt");
  const double lost_threshold = node->declare_parameter<double>("lost_threshold", 0.05);  // m
  const double lost_rotation_threshold = node->declare_parameter<double>(
      "lost_rotation_threshold", 45.0);  // degrees; <= 0 disables the test
  // true  = seed the initial pose from the (image-aligned) GT/detector pose topic.
  // false = seed from the initial_pose ROS parameter in the object YAML.
  const bool use_gt_initial_pose = node->declare_parameter<bool>("use_gt_initial_pose", true);
  if (!HasOnlyOptions(image_outputs, {"overlay", "keypoints"})) {
    RCLCPP_FATAL(node->get_logger(),
                 "image_outputs must be none, overlay, keypoints, or "
                 "overlay,keypoints; got '%s'",
                 image_outputs.c_str());
    return 1;
  }
  const bool publish_overlay = HasOption(image_outputs, "overlay");
  const bool publish_keypoints = HasOption(image_outputs, "keypoints");
  if (!event_driven) {
    RCLCPP_WARN(
        node->get_logger(),
        "event_driven=false repeatedly updates stateful modalities with the "
        "same image; use only for short compute benchmarks");
  }

  m3t_ros2::RosPublisherConfig cfg;
  cfg.world_frame = node->declare_parameter<std::string>("world_frame", "camera");
  cfg.mesh_resource = node->declare_parameter<std::string>("mesh_resource", "");
  cfg.mesh_scale = node->declare_parameter<double>("mesh_scale", 1.0);
  cfg.mesh_use_embedded_materials = node->declare_parameter<bool>("mesh_use_embedded_materials", false);
  cfg.publish_overlay = publish_overlay;
  cfg.publish_keypoints = publish_keypoints;
  cfg.publish_color = false;  // node 1 publishes raw color/depth + GT
  cfg.publish_depth = false;
  cfg.publish_gt = false;

  std::shared_ptr<m3t::Body> body;
  m3t::Transform3fA initial_pose;
  std::vector<Eigen::Matrix3f> rotation_symmetries;
  try {
    body = m3t_ros2::DeclareAndCreateBody(node.get());
    initial_pose =
        m3t_ros2::TransformFromRowMajor(initial_pose_values, "initial_pose");
    rotation_symmetries =
        ParseRotationSymmetries(rotation_symmetry_values);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(node->get_logger(), "object parameter error: %s",
                 error.what());
    return 1;
  }
  const std::string object_name = body->name();
  if (model_cache_dir.empty()) {
    model_cache_dir =
        (m3t_ros2::DefaultRuntimeRoot() / "cache" /
         m3t_ros2::SanitizePathComponent(object_name))
            .string();
  }
  std::string cache_error;
  if (!m3t_ros2::EnsureWritableDirectory(model_cache_dir, &cache_error)) {
    RCLCPP_FATAL(node->get_logger(), "model cache error: %s",
                 cache_error.c_str());
    return 1;
  }
  const bool use_region = HasOption(modalities, "region");
  const bool use_depth = HasOption(modalities, "depth");
  const bool use_texture = HasOption(modalities, "texture");
  if (!use_region && !use_depth && !use_texture) {
    RCLCPP_FATAL(node->get_logger(),
                 "modalities must contain region, depth, and/or texture; got '%s'",
                 modalities.c_str());
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "model cache: %s",
              fs::absolute(model_cache_dir).lexically_normal().c_str());

  auto color_camera = std::make_shared<m3t_ros2::RosColorCamera>("color_camera");
  color_camera->SetUp();  // overlay renderer uses it even when no modality does
  std::shared_ptr<m3t_ros2::RosDepthCamera> depth_camera;
  if (use_depth) {
    depth_camera = std::make_shared<m3t_ros2::RosDepthCamera>("depth_camera");
    depth_camera->SetDepthScale(depth_scale);
    depth_camera->SetUp();
  }

  auto qos = rclcpp::SensorDataQoS();
  std::mutex input_sync_mutex;
  cv::Mat pending_color;
  cv::Mat pending_depth;
  int64_t pending_color_stamp = 0;
  int64_t pending_depth_stamp = 0;
  bool has_pending_color = false;
  bool has_pending_depth = false;
  const int64_t sync_tolerance_ns =
      static_cast<int64_t>(std::max(0.0, sync_tolerance) * 1.0e9);
  // Called with input_sync_mutex held.  Committing depth first and color last
  // makes the color sequence counter represent a complete RGB-D frame.
  auto commit_synchronized_frame = [&]() {
    if (!has_pending_color || !has_pending_depth) return;
    const int64_t delta =
        std::llabs(pending_color_stamp - pending_depth_stamp);
    if (delta <= sync_tolerance_ns) {
      depth_camera->SetLatest(pending_depth);
      color_camera->SetLatest(pending_color);
      has_pending_color = false;
      has_pending_depth = false;
    } else if (pending_color_stamp < pending_depth_stamp) {
      has_pending_color = false;
    } else {
      has_pending_depth = false;
    }
  };
  auto sub_color = node->create_subscription<sensor_msgs::msg::Image>(
      color_topic, qos, [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
        cv::Mat image = cv_bridge::toCvCopy(m, "bgr8")->image;
        if (!use_depth) {
          color_camera->SetLatest(image);
          return;
        }
        std::lock_guard<std::mutex> lock{input_sync_mutex};
        pending_color = image;
        pending_color_stamp = rclcpp::Time{m->header.stamp}.nanoseconds();
        has_pending_color = true;
        commit_synchronized_frame();
      });
  auto sub_cinfo = node->create_subscription<sensor_msgs::msg::CameraInfo>(
      color_info_topic, qos, [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        if (!HasValidIntrinsics(*m)) {
          RCLCPP_WARN_THROTTLE(
              node->get_logger(), *node->get_clock(), 5000,
              "ignoring invalid RGB CameraInfo (positive width, height, fx, "
              "and fy are required)");
          return;
        }
        color_camera->SetIntrinsics(FromInfo(*m));
      });
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_dinfo;
  if (use_depth) {
    sub_depth = node->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, qos, [&](sensor_msgs::msg::Image::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lock{input_sync_mutex};
          pending_depth = cv_bridge::toCvCopy(m, "16UC1")->image;
          pending_depth_stamp = rclcpp::Time{m->header.stamp}.nanoseconds();
          has_pending_depth = true;
          commit_synchronized_frame();
        });
    sub_dinfo = node->create_subscription<sensor_msgs::msg::CameraInfo>(
        depth_info_topic, qos, [&](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
          if (!HasValidIntrinsics(*m)) {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(), *node->get_clock(), 5000,
                "ignoring invalid depth CameraInfo (positive width, height, "
                "fx, and fy are required)");
            return;
          }
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

  auto link = std::make_shared<m3t::Link>("link", body);
  if (use_region) {
    auto rm = std::make_shared<m3t::RegionModel>(
        "region_model", body,
        fs::path{model_cache_dir} / "region_model.bin");
    link->AddModality(std::make_shared<m3t::RegionModality>("region_modality", body, color_camera, rm));
  }
  if (use_depth) {
    auto dm = std::make_shared<m3t::DepthModel>(
        "depth_model", body,
        fs::path{model_cache_dir} / "depth_model.bin");
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
  // else: fixed pose from the initial_pose ROS parameter.
  std::shared_ptr<m3t::StaticDetector> detector =
      use_gt_initial_pose
          ? std::make_shared<m3t::StaticDetector>("detector", optimizer,
                                                  m3t::Transform3fA::Identity(), true)
          : std::make_shared<m3t::StaticDetector>("detector", optimizer,
                                                  initial_pose, true);
  tracker->AddDetector(detector);

  // Service to (re-)initialize from the latest pose (e.g. after a loss or a loop).
  std::atomic<bool> redetect_req{false};
  auto redetect_srv = node->create_service<std_srvs::srv::Trigger>(
      "~/redetect", [&](std_srvs::srv::Trigger::Request::SharedPtr,
                        std_srvs::srv::Trigger::Response::SharedPtr resp) {
        redetect_req = true; resp->success = true; resp->message = "re-detect queued";
      });

  std::shared_ptr<m3t::RendererGeometry> rg_overlay;
  std::shared_ptr<m3t::FullNormalRenderer> overlay_renderer;
  if (publish_overlay) {
    rg_overlay = std::make_shared<m3t::RendererGeometry>("rg_overlay");
    rg_overlay->AddBody(body);
    overlay_renderer = std::make_shared<m3t::FullNormalRenderer>(
        "overlay_renderer", rg_overlay, color_camera, 0.01f, 10.0f);
  }

  // Camera calibration is intentionally not loaded from a tracker config file.
  // A camera driver must publish CameraInfo, and setup stays blocked until both
  // the pixels and matching intrinsics have arrived.
  if (use_depth) {
    RCLCPP_INFO(
        node->get_logger(),
        "waiting for RGB image + CameraInfo on %s and %s, and depth image + "
        "CameraInfo on %s and %s ...",
        color_topic.c_str(), color_info_topic.c_str(), depth_topic.c_str(),
        depth_info_topic.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(),
                "waiting for RGB image + CameraInfo on %s and %s ...",
                color_topic.c_str(), color_info_topic.c_str());
  }
  m3t::Transform3fA gt_tmp;
  while (rclcpp::ok() &&
         !(color_camera->HasImage() && color_camera->HasIntrinsics() &&
           (!use_depth || (depth_camera->HasImage() && depth_camera->HasIntrinsics())) &&
           (!use_gt_initial_pose || lookup_gt(gt_tmp))))
    rclcpp::spin_some(node);
  if (!rclcpp::ok()) return 0;
  RCLCPP_INFO(node->get_logger(), "got first frames — setting up tracker (region=%d depth=%d texture=%d)",
              use_region, use_depth, use_texture);

  if (!tracker->SetUp() ||
      (publish_overlay &&
       (!rg_overlay->SetUp() || !overlay_renderer->SetUp()))) {
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
        if (!running || !rclcpp::ok()) break;
        last_seq = color_camera->seq();
      }
      const auto iter_t0 = clk::now();
      if (!tracker->UpdateCameras(it)) { std::this_thread::sleep_for(std::chrono::milliseconds{1}); continue; }
      const auto pose_before_update = body->body2world_pose();
      if (!tracker->UpdateSubscribers(it) ||
          !tracker->CalculateConsistentPoses() ||
          !tracker->ExecuteDetectingStep(it) ||
          !tracker->ExecuteStartingStep(it)) {
        body->set_body2world_pose(pose_before_update);
        redetect_req.store(true);
        RCLCPP_ERROR_THROTTLE(
            node->get_logger(), *node->get_clock(), 2000,
            "M3T frame preparation failed; restored the last pose and queued "
            "re-detection");
        continue;
      }
      const auto s0 = clk::now();
      if (!tracker->ExecuteTrackingStep(it)) {
        body->set_body2world_pose(pose_before_update);
        redetect_req.store(true);
        RCLCPP_ERROR_THROTTLE(
            node->get_logger(), *node->get_clock(), 2000,
            "M3T tracking step failed; restored the last pose and queued "
            "re-detection");
        continue;
      }
      const double solve_ms = dsec(clk::now() - s0).count() * 1e3;
      if (!body->body2world_pose().matrix().allFinite()) {
        body->set_body2world_pose(pose_before_update);
        redetect_req.store(true);
        RCLCPP_ERROR_THROTTLE(
            node->get_logger(), *node->get_clock(), 2000,
            "M3T produced a non-finite pose; restored the last pose and queued "
            "re-detection");
        continue;
      }
      body->set_body2world_pose(ClosestSymmetricPose(
          body->body2world_pose(), pose_before_update,
          rotation_symmetries));
      solve_sum += solve_ms; solve_min = std::min(solve_min, solve_ms);
      solve_max = std::max(solve_max, solve_ms); ++solve_n; ++loop_n;

      // Tracking-error monitor: estimate vs GT (from TF). Tells on-track vs lost,
      // so the solve time above is only trusted while OK.
      m3t::Transform3fA gtp;
      if (lookup_gt(gtp)) {
        const auto est = body->body2world_pose();
        const double perr = (est.translation() - gtp.translation()).norm();
        perr_sum += perr; perr_max = std::max(perr_max, perr);
        const double rerr = SymmetricRotationErrorDeg(
            est.rotation(), gtp.rotation(), rotation_symmetries);
        rerr_sum += rerr; rerr_max = std::max(rerr_max, rerr);
        if (perr > lost_threshold ||
            (lost_rotation_threshold > 0.0 &&
             rerr > lost_rotation_threshold))
          ++lost_n;
        ++err_n;
      }

      // Every solved camera frame must replace the estimate snapshot. In
      // free-run benchmark mode only, rate-limit snapshot preparation to the
      // publication rate.
      if (event_driven || clk::now() - last_snap >= snap_period) {
        cv::Mat overlay;
        if (publish_overlay && overlay_renderer->StartRendering() &&
            overlay_renderer->FetchNormalImage())
          overlay = Composite(color_camera->image(), overlay_renderer->normal_image());
        std::lock_guard<std::mutex> lk{snap_mutex};
        if (publish_keypoints) {
          snap.color = color_camera->image();
        }
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

  RCLCPP_INFO(node->get_logger(),
              "tracking | mode=%s track_rate=%.0f (0=max) publish_rate=%.0f "
              "image_outputs=%s log_period=%.1fs",
              event_driven ? "new-frame" : "free-run", track_rate,
              publish_rate, image_outputs.c_str(), log_period);
  rclcpp::spin(node);
  running = false;
  worker.join();
  rclcpp::shutdown();
  return 0;
}
