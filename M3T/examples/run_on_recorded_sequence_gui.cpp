// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// GUI variant of run_on_recorded_sequence.cpp that loops the recorded sequence
// (see LoopingLoaderColorCamera) so the viewer window stays open on short clips
// like the 2-frame data/_sequence. Otherwise identical: a NormalColorViewer
// window is shown and a ManualDetector is used.
//
// Controls (with the viewer window focused):
//   D = run manual detection (click the 4 reference points), T = start tracking,
//   X = detect + track, S = stop tracking, Q = quit.
//
// Needs a display (e.g. `export DISPLAY=:0`).
//
// Usage:
//   ./run_on_recorded_sequence_gui \
//       <color_camera_metafile> <body_metafile> <manual_detector_metafile> \
//       <temp_directory>

#include <filesystem/filesystem.h>
#include <m3t/basic_depth_renderer.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/manual_detector.h>
#include <m3t/normal_viewer.h>
#include <m3t/optimizer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/tracker.h>

#include <Eigen/Geometry>
#include <memory>

#include "looping_loader_camera.h"

int main(int argc, char *argv[]) {
  if (argc != 5) {
    std::cerr << "Not enough arguments: Provide camera metafile, body "
                 "metafile, manual detector metafile, temp directory"
              << std::endl;
    return -1;
  }
  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path body_metafile_path{argv[2]};
  const std::filesystem::path detector_metafile_path{argv[3]};
  const std::filesystem::path temp_directory{argv[4]};

  // Set up tracker and renderer geometry
  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  // Set up camera (loops the sequence when it ends)
  auto camera_ptr{std::make_shared<LoopingLoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};

  // Set up viewer
  auto viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
      "viewer", camera_ptr, renderer_geometry_ptr)};
  tracker_ptr->AddViewer(viewer_ptr);

  // Set up body
  auto body_ptr{std::make_shared<m3t::Body>("triangle", body_metafile_path)};
  renderer_geometry_ptr->AddBody(body_ptr);

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

  // Set up manual detector
  auto detector_ptr{std::make_shared<m3t::ManualDetector>(
      "detector", detector_metafile_path, optimizer_ptr, camera_ptr)};
  tracker_ptr->AddDetector(detector_ptr);

  // Start tracking
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(false, false)) return -1;
  return 0;
}
