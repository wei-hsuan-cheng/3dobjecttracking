// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// Helper for the demo examples: a LoaderColorCamera that loops back to the
// first frame when the recorded sequence is exhausted, instead of failing.
// This makes the short 2-frame test clip in data/_sequence play continuously,
// which is needed so the GUI viewer stays open long enough to interact with and
// so the headless demo can run for a fixed number of frames.

#ifndef M3T_EXAMPLES_LOOPING_LOADER_CAMERA_H_
#define M3T_EXAMPLES_LOOPING_LOADER_CAMERA_H_

#include <m3t/loader_camera.h>

#include <filesystem>
#include <string>

class LoopingLoaderColorCamera : public m3t::LoaderColorCamera {
 public:
  using m3t::LoaderColorCamera::LoaderColorCamera;

  bool UpdateImage(bool synchronized) override {
    // Remember the sequence's first index on the very first call.
    if (first_index_ < 0) first_index_ = load_index();

    // If the next frame does not exist, wrap back to the first index. We reset
    // the index directly (set_load_index() would invalidate set_up_, and SetUp()
    // recursively calls UpdateImage()), so no end-of-sequence error is printed
    // and no frame is skipped.
    if (!NextImageExists()) {
      set_load_index(first_index_);
      set_up_ = true;  // restore: set_load_index() cleared it
    }
    return m3t::LoaderColorCamera::UpdateImage(synchronized);
  }

 private:
  // Reproduces the path LoaderColorCamera::UpdateImage will read next.
  bool NextImageExists() const {
    const int idx = load_index();
    const int n_zeros = std::max(
        n_leading_zeros() - static_cast<int>(std::to_string(idx).length()), 0);
    const std::filesystem::path path{
        load_directory() / (image_name_pre() + std::string(n_zeros, '0') +
                            std::to_string(idx) + image_name_post() + "." +
                            load_image_type())};
    return std::filesystem::exists(path);
  }

  int first_index_ = -1;
};

// Same looping behaviour for a depth camera.
class LoopingLoaderDepthCamera : public m3t::LoaderDepthCamera {
 public:
  using m3t::LoaderDepthCamera::LoaderDepthCamera;

  bool UpdateImage(bool synchronized) override {
    if (first_index_ < 0) first_index_ = load_index();
    if (!NextImageExists()) {
      set_load_index(first_index_);
      set_up_ = true;
    }
    return m3t::LoaderDepthCamera::UpdateImage(synchronized);
  }

 private:
  bool NextImageExists() const {
    const int idx = load_index();
    const int n_zeros = std::max(
        n_leading_zeros() - static_cast<int>(std::to_string(idx).length()), 0);
    const std::filesystem::path path{
        load_directory() / (image_name_pre() + std::string(n_zeros, '0') +
                            std::to_string(idx) + image_name_post() + "." +
                            load_image_type())};
    return std::filesystem::exists(path);
  }

  int first_index_ = -1;
};

#endif  // M3T_EXAMPLES_LOOPING_LOADER_CAMERA_H_
