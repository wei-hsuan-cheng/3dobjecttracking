// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// Live GUI version of the RGB-D (Region + Depth) tracker. Opens a
// NormalColorViewer so you can watch the tracked model overlaid on the color
// frames while both a RegionModality (color) and a DepthModality (depth)
// refine the pose. A StaticDetector auto-starts tracking; the sequence loops.
// No physical camera required. Needs a display (export DISPLAY=:0).
//
// Usage:
//   ./run_on_recorded_sequence_gui_rgbd \
//       <color_camera_metafile> <depth_camera_metafile> <body_metafile> \
//       <static_detector_metafile> <temp_directory>

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/depth_modality.h>
#include <m3t/depth_model.h>
#include <m3t/normal_viewer.h>
#include <m3t/optimizer.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/renderer_geometry.h>
#include <m3t/static_detector.h>
#include <m3t/tracker.h>

#include <Eigen/Geometry>
#include <memory>

#include "looping_loader_camera.h"

int main(int argc, char *argv[]) {
  if (argc != 6) {
    std::cerr << "Usage: color metafile, depth metafile, body metafile, static "
                 "detector metafile, temp directory"
              << std::endl;
    return -1;
  }
  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path depth_camera_metafile_path{argv[2]};
  const std::filesystem::path body_metafile_path{argv[3]};
  const std::filesystem::path detector_metafile_path{argv[4]};
  const std::filesystem::path temp_directory{argv[5]};

  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  auto color_camera_ptr{std::make_shared<LoopingLoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};
  auto depth_camera_ptr{std::make_shared<LoopingLoaderDepthCamera>(
      "depth_camera", depth_camera_metafile_path)};

  auto body_ptr{std::make_shared<m3t::Body>("body", body_metafile_path)};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Color overlay viewer.
  auto viewer_ptr{std::make_shared<m3t::NormalColorViewer>(
      "viewer", color_camera_ptr, renderer_geometry_ptr)};
  tracker_ptr->AddViewer(viewer_ptr);

  auto region_model_ptr{std::make_shared<m3t::RegionModel>(
      "region_model", body_ptr, temp_directory / "region_model.bin")};
  auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
      "region_modality", body_ptr, color_camera_ptr, region_model_ptr)};

  auto depth_model_ptr{std::make_shared<m3t::DepthModel>(
      "depth_model", body_ptr, temp_directory / "depth_model.bin")};
  auto depth_modality_ptr{std::make_shared<m3t::DepthModality>(
      "depth_modality", body_ptr, depth_camera_ptr, depth_model_ptr)};

  auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
  link_ptr->AddModality(region_modality_ptr);
  link_ptr->AddModality(depth_modality_ptr);
  auto optimizer_ptr{std::make_shared<m3t::Optimizer>("optimizer", link_ptr)};
  tracker_ptr->AddOptimizer(optimizer_ptr);

  auto detector_ptr{std::make_shared<m3t::StaticDetector>(
      "detector", detector_metafile_path, optimizer_ptr)};
  tracker_ptr->AddDetector(detector_ptr);

  tracker_ptr->set_start_tracking_after_detection(true);
  if (!tracker_ptr->SetUp()) return -1;
  if (!tracker_ptr->RunTrackerProcess(true, false)) return -1;
  return 0;
}
