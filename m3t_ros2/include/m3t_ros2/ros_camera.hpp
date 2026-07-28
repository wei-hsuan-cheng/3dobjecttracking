// SPDX-License-Identifier: MIT
// M3T Camera implementations backed by ROS subscriptions. They let the tracker
// consume images from topics (a real camera node, a bag, or the sequence
// publisher) with no code change in the tracker — only topic names differ.
// The latest frame is stored under a mutex; UpdateImage() (called on the worker
// thread) publishes it into Camera::image_. Reassignment of the ref-counted
// cv::Mat makes this race-free without copying pixels.

#ifndef M3T_ROS2_ROS_CAMERA_HPP_
#define M3T_ROS2_ROS_CAMERA_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <opencv2/core.hpp>

#include <m3t/camera.h>

namespace m3t_ros2 {

class RosColorCamera : public m3t::ColorCamera {
 public:
  explicit RosColorCamera(const std::string &name) : m3t::ColorCamera(name) {}
  bool SetUp() override { set_up_ = true; return true; }
  bool UpdateImage(bool) override {
    std::lock_guard<std::mutex> lk{m_};
    if (latest_.empty()) return false;
    image_ = latest_;
    return true;
  }
  void SetLatest(const cv::Mat &img) {
    { std::lock_guard<std::mutex> lk{m_}; latest_ = img; }
    seq_.fetch_add(1);
  }
  void SetIntrinsics(const m3t::Intrinsics &in) { intrinsics_ = in; }
  bool HasImage() { std::lock_guard<std::mutex> lk{m_}; return !latest_.empty(); }
  bool HasIntrinsics() const { return intrinsics_.width > 0; }
  uint64_t seq() const { return seq_.load(); }  // increments on each new frame

 private:
  std::mutex m_;
  cv::Mat latest_;
  std::atomic<uint64_t> seq_{0};
};

class RosDepthCamera : public m3t::DepthCamera {
 public:
  explicit RosDepthCamera(const std::string &name) : m3t::DepthCamera(name) {}
  bool SetUp() override { set_up_ = true; return true; }
  bool UpdateImage(bool) override {
    std::lock_guard<std::mutex> lk{m_};
    if (latest_.empty()) return false;
    image_ = latest_;
    return true;
  }
  void SetLatest(const cv::Mat &img) { std::lock_guard<std::mutex> lk{m_}; latest_ = img; }
  void SetIntrinsics(const m3t::Intrinsics &in) { intrinsics_ = in; }
  void SetDepthScale(float s) { depth_scale_ = s; }
  bool HasImage() { std::lock_guard<std::mutex> lk{m_}; return !latest_.empty(); }
  bool HasIntrinsics() const { return intrinsics_.width > 0; }

 private:
  std::mutex m_;
  cv::Mat latest_;
};

}  // namespace m3t_ros2

#endif  // M3T_ROS2_ROS_CAMERA_HPP_
