// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// Auto-tracking GUI demo: like run_on_recorded_sequence_gui.cpp but uses a
// StaticDetector (no manual clicking) and starts detecting + tracking
// automatically, so you can just watch the NormalColorViewer overlay track the
// body. The sequence loops (LoopingLoaderColorCamera) so the window stays open.
//
// Pass a 5th argument "viz" to additionally open the RegionModality debug
// windows (correspondence lines and result points).
//
// Controls (viewer focused): S = stop, D = detect, T/X = track, Q = quit.
// Needs a display (e.g. `export DISPLAY=:0`).
//
// Usage:
//   ./run_on_recorded_sequence_gui_auto \
//       <color_camera_metafile> <body_metafile> <static_detector_metafile> \
//       <temp_directory> [viz]

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/normal_viewer.h>
#include <m3t/optimizer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/static_detector.h>
#include <m3t/tracker.h>

#include <Eigen/Geometry>
#include <memory>
#include <string>

#include "looping_loader_camera.h"

int main(int argc, char *argv[]) {
  if (argc < 5 || argc > 6) {
    std::cerr << "Usage: provide camera metafile, body metafile, static "
                 "detector metafile, temp directory, [viz]"
              << std::endl;
    return -1;
  }
  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path body_metafile_path{argv[2]};
  const std::filesystem::path detector_metafile_path{argv[3]};
  const std::filesystem::path temp_directory{argv[4]};
  const bool visualize{argc == 6 && std::string(argv[5]) == "viz"};

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
  if (visualize) {
    // Open extra debug windows showing the tracker internals.
    region_modality_ptr->set_visualize_lines_correspondence(true);
    region_modality_ptr->set_visualize_points_result(true);
  }

  // Set up link + optimizer
  auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
  link_ptr->AddModality(region_modality_ptr);
  auto optimizer_ptr{std::make_shared<m3t::Optimizer>("optimizer", link_ptr)};
  tracker_ptr->AddOptimizer(optimizer_ptr);

  // Set up static detector (no clicking) and start detecting + tracking
  auto detector_ptr{std::make_shared<m3t::StaticDetector>(
      "detector", detector_metafile_path, optimizer_ptr)};
  tracker_ptr->AddDetector(detector_ptr);

  // After the static detector sets the initial pose, automatically start
  // tracking so the modality refines it (off by default).
  tracker_ptr->set_start_tracking_after_detection(true);

  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;
  return 0;
}
