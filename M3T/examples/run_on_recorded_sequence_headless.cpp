// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// Headless variant of run_on_recorded_sequence.cpp.
// It tracks a single body on a recorded image sequence WITHOUT any GUI viewer
// and WITHOUT manual clicking: a StaticDetector supplies the initial pose, and
// a small Publisher prints the estimated pose of the body for every frame.
// This makes it runnable over SSH / in a plain terminal.
//
// The camera loops the sequence (see LoopingLoaderColorCamera), and the run
// stops cleanly after a fixed number of frames, so short clips like the
// 2-frame data/_sequence produce continuous output without end-of-sequence
// errors.
//
// Usage:
//   ./run_on_recorded_sequence_headless \
//       <color_camera_metafile> <body_metafile> <static_detector_metafile> \
//       <temp_directory> [n_frames]

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
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

// Publisher that prints the pose of a body once per tracker iteration and quits
// the tracker process after n_frames iterations.
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
  if (argc < 5 || argc > 6) {
    std::cerr << "Usage: provide camera metafile, body metafile, static "
                 "detector metafile, temp directory, [n_frames]"
              << std::endl;
    return -1;
  }
  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path body_metafile_path{argv[2]};
  const std::filesystem::path detector_metafile_path{argv[3]};
  const std::filesystem::path temp_directory{argv[4]};
  const int n_frames{argc == 6 ? std::stoi(argv[5]) : 10};

  // Set up tracker
  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};

  // Set up camera (reads PNG frames from disk, loops when the clip ends)
  auto camera_ptr{std::make_shared<LoopingLoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};

  // Set up body
  auto body_ptr{std::make_shared<m3t::Body>("triangle", body_metafile_path)};

  // Set up region model + modality
  auto region_model_ptr{std::make_shared<m3t::RegionModel>(
      "region_model", body_ptr, temp_directory / "region_model.bin")};
  auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
      "region_modality", body_ptr, camera_ptr, region_model_ptr)};

  // Set up link + optimizer
  auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
  link_ptr->AddModality(region_modality_ptr);
  auto optimizer_ptr{std::make_shared<m3t::Optimizer>("optimizer", link_ptr)};
  tracker_ptr->AddOptimizer(optimizer_ptr);

  // Set up static detector (supplies the initial pose, no clicking required)
  auto detector_ptr{std::make_shared<m3t::StaticDetector>(
      "detector", detector_metafile_path, optimizer_ptr)};
  tracker_ptr->AddDetector(detector_ptr);

  // Print the estimated pose every frame and stop after n_frames
  tracker_ptr->AddPublisher(std::make_shared<PosePrinter>(
      "pose_printer", body_ptr, tracker_ptr.get(), n_frames));

  // Set up and run. RunTrackerProcess(true, false) executes the detection on
  // the first frame, then tracks. The PosePrinter quits the process after
  // n_frames, so this returns cleanly.
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;

  std::cout << "Done (" << n_frames << " frames)." << std::endl;
  return 0;
}
