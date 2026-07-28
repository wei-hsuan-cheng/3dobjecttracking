// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// Multi-modality headless demo using the THIRD modality: RegionModality (color)
// + TextureModality (color keypoints). TextureModality detects image keypoints
// on the object and validates them against a FocusedSilhouetteRenderer, adding
// keypoint constraints to the pose optimisation. No physical camera required.
//
// NOTE: TextureModality needs trackable image features on the object surface.
// On the synthetic normal-rendered sequences it finds features at the shading
// edges/gradients (tens of keypoints), which is enough to run; a real textured
// object (or a textured render) gives it far more to work with.
//
// Usage:
//   ./run_on_recorded_sequence_texture \
//       <color_camera_metafile> <body_metafile> <static_detector_metafile> \
//       <temp_directory> [n_frames]

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/optimizer.h>
#include <m3t/publisher.h>
#include <m3t/region_modality.h>
#include <m3t/region_model.h>
#include <m3t/silhouette_renderer.h>
#include <m3t/static_detector.h>
#include <m3t/texture_modality.h>
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
    std::cout << "[frame " << iteration << "] " << body_ptr_->name()
              << "  translation(m)=[" << t.x() << ", " << t.y() << ", " << t.z()
              << "]" << std::endl;
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
    std::cerr << "Usage: color metafile, body metafile, static detector "
                 "metafile, temp directory, [n_frames]"
              << std::endl;
    return -1;
  }
  const std::filesystem::path color_camera_metafile_path{argv[1]};
  const std::filesystem::path body_metafile_path{argv[2]};
  const std::filesystem::path detector_metafile_path{argv[3]};
  const std::filesystem::path temp_directory{argv[4]};
  const int n_frames{argc == 6 ? std::stoi(argv[5]) : 10};

  auto tracker_ptr{std::make_shared<m3t::Tracker>("tracker")};
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};

  auto color_camera_ptr{std::make_shared<LoopingLoaderColorCamera>(
      "color_camera", color_camera_metafile_path)};

  auto body_ptr{std::make_shared<m3t::Body>("body", body_metafile_path)};
  renderer_geometry_ptr->AddBody(body_ptr);

  // Region modality (color).
  auto region_model_ptr{std::make_shared<m3t::RegionModel>(
      "region_model", body_ptr, temp_directory / "region_model.bin")};
  auto region_modality_ptr{std::make_shared<m3t::RegionModality>(
      "region_modality", body_ptr, color_camera_ptr, region_model_ptr)};

  // Texture modality (color keypoints validated against the silhouette).
  auto silhouette_renderer_ptr{std::make_shared<m3t::FocusedSilhouetteRenderer>(
      "silhouette_renderer", renderer_geometry_ptr, color_camera_ptr)};
  silhouette_renderer_ptr->AddReferencedBody(body_ptr);
  auto texture_modality_ptr{std::make_shared<m3t::TextureModality>(
      "texture_modality", body_ptr, color_camera_ptr, silhouette_renderer_ptr)};

  auto link_ptr{std::make_shared<m3t::Link>("link", body_ptr)};
  link_ptr->AddModality(region_modality_ptr);
  link_ptr->AddModality(texture_modality_ptr);
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

  std::cout << "Done (" << n_frames << " frames, region + texture)."
            << std::endl;
  return 0;
}
