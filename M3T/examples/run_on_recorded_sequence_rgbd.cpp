// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// Multi-modality (RGB-D) headless demo: tracks a single body on a recorded
// color + depth sequence using BOTH a RegionModality (color) and a
// DepthModality (depth). No physical camera or driver is required -- the frames
// are read from disk (generate them with generate_orbit_sequence, which now
// also writes depth PNGs + depth_camera.yaml). A StaticDetector supplies the
// initial pose and a Publisher prints the estimated pose per frame.
//
// Usage:
//   ./run_on_recorded_sequence_rgbd \
//       <color_camera_metafile> <depth_camera_metafile> <body_metafile> \
//       <static_detector_metafile> <temp_directory> [n_frames]

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/optimizer.h>
#include <m3t/publisher.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/static_detector.h>
#include <m3t/tracker.h>

#include <Eigen/Geometry>
#include <iostream>
#include <memory>

#include "looping_loader_camera.h"

class PosePrinter : public m3t::Publisher {
 public:
  PosePrinter(const std::string &name, std::shared_ptr<m3t::Body> body_ptr,
              m3t::Tracker *tracker_ptr, int n_frames)
      : m3t::Publisher{name},
        body_ptr_{std::move(body_ptr)},
        tracker_ptr_{tracker_ptr},
        n_frames_{n_frames} {}

  bool SetUp() override {
    set_up_ = true;
    return true;
  }

  bool UpdatePublisher(int iteration) override {
    const m3t::Transform3fA &body2world = body_ptr_->body2world_pose();
    const Eigen::Vector3f t = body2world.translation();
    const Eigen::Quaternionf q{body2world.rotation()};
    std::cout << "[frame " << iteration << "] " << body_ptr_->name()
              << "  translation(m)=[" << t.x() << ", " << t.y() << ", " << t.z()
              << "]  quaternion(wxyz)=[" << q.w() << ", " << q.x() << ", "
              << q.y() << ", " << q.z() << "]" << std::endl;
    if (iteration + 1 >= n_frames_) tracker_ptr_->QuitTrackerProcess();
    return true;
  }

 private:
  std::shared_ptr<m3t::Body> body_ptr_;
  m3t::Tracker *tracker_ptr_;
  int n_frames_;
};

int main(int argc, char *argv[]) {
  if (argc < 6 || argc > 7) {
    std::cerr << "Usage: color metafile, depth metafile, body metafile, static "
                 "detector metafile, temp directory, [n_frames]"
              << std::endl;
    return -1;
  }
  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path depth_camera_metafile_path{argv[2]};
  const std::filesystem::path body_metafile_path{argv[3]};
  const std::filesystem::path detector_metafile_path{argv[4]};
  const std::filesystem::path temp_directory{argv[5]};
  const int n_frames{argc == 7 ? std::stoi(argv[6]) : 10};

  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};

  // Color + depth cameras, both read from disk (no physical camera).
  auto color_camera_ptr{std::make_shared<LoopingLoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};
  auto depth_camera_ptr{std::make_shared<LoopingLoaderDepthCamera>(
      "depth_camera", depth_camera_metafile_path)};

  auto body_ptr{std::make_shared<m3t::Body>("body", body_metafile_path)};

  // Region modality (color).
  auto region_model_ptr{std::make_shared<m3t::RegionModel>(
      "region_model", body_ptr, temp_directory / "region_model.bin")};
  auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
      "region_modality", body_ptr, color_camera_ptr, region_model_ptr)};

  // Depth modality (depth).
  auto depth_model_ptr{std::make_shared<m3t::DepthModel>(
      "depth_model", body_ptr, temp_directory / "depth_model.bin")};
  auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
      "depth_modality", body_ptr, depth_camera_ptr, depth_model_ptr)};

  // One link carrying BOTH modalities.
  auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
  link_ptr->AddModality(region_modality_ptr);
  link_ptr->AddModality(depth_modality_ptr);
  auto optimizer_ptr{std::make_shared<m3t::Optimizer>("optimizer", link_ptr)};
  tracker_ptr->AddOptimizer(optimizer_ptr);

  auto detector_ptr{std::make_shared<m3t::StaticDetector>(
      "detector", detector_metafile_path, optimizer_ptr)};
  tracker_ptr->AddDetector(detector_ptr);

  tracker_ptr->AddPublisher(std::make_shared<PosePrinter>(
      "pose_printer", body_ptr, tracker_ptr.get(), n_frames));

  tracker_ptr->set_start_tracking_after_detection(true);
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;

  std::cout << "Done (" << n_frames << " frames, region + depth)." << std::endl;
  return 0;
}
