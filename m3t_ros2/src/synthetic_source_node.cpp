// SPDX-License-Identifier: MIT
// Online synthetic RGB-D source for M3T.
//
// This is the ROS-native replacement for M3T/examples/generate_orbit_sequence:
// frames, CameraInfo, and ground truth are published directly and no sequence
// files are created in the source tree.

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include <m3t/body.h>
#include <m3t/normal_renderer.h>
#include <m3t/renderer_geometry.h>

#include "m3t_ros2/body_factory.hpp"

namespace {

geometry_msgs::msg::Pose ToPose(const m3t::Transform3fA &transform) {
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3f translation = transform.translation();
  const Eigen::Quaternionf rotation{transform.rotation()};
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

sensor_msgs::msg::CameraInfo MakeCameraInfo(const m3t::Intrinsics &intrinsics,
                                            const std::string &frame_id) {
  sensor_msgs::msg::CameraInfo info;
  info.header.frame_id = frame_id;
  info.width = intrinsics.width;
  info.height = intrinsics.height;
  info.distortion_model = "plumb_bob";
  info.d = {0, 0, 0, 0, 0};
  info.k = {intrinsics.fu, 0, intrinsics.ppu, 0, intrinsics.fv,
            intrinsics.ppv, 0, 0, 1};
  info.p = {intrinsics.fu, 0, intrinsics.ppu, 0, 0, intrinsics.fv,
            intrinsics.ppv, 0, 0, 0, 1, 0};
  return info;
}

}  // namespace

class SyntheticSourceNode : public rclcpp::Node {
 public:
  SyntheticSourceNode() : Node{"m3t_synthetic_source"}, rng_{12345} {
    publish_rate_ = declare_parameter<double>("publish_rate", 30.0);
    n_frames_ = declare_parameter<int>("n_frames", 240);
    loop_ = declare_parameter<bool>("loop", true);
    depth_noise_ = declare_parameter<double>("depth_noise", 0.0);
    distortion_ = declare_parameter<double>("distortion", 0.0);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    spin_turns_ = declare_parameter<double>("spin_turns", 1.0);
    nod_degrees_ = declare_parameter<double>("nod_degrees", 25.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "camera");
    camera_frame_ =
        declare_parameter<std::string>("camera_frame", world_frame_);
    gt_frame_ = declare_parameter<std::string>("gt_frame", "object_gt");
    mesh_resource_ =
        declare_parameter<std::string>("mesh_resource", std::string{});
    mesh_scale_ = declare_parameter<double>("mesh_scale", 1.0);
    mesh_embedded_ =
        declare_parameter<bool>("mesh_use_embedded_materials", false);
    const auto camera_intrinsics =
        declare_parameter<std::vector<double>>(
            "camera_intrinsics",
            {698.128, 698.617, 478.459, 274.426, 960.0, 540.0});
    if (camera_intrinsics.size() != 6 || camera_intrinsics[0] <= 0.0 ||
        camera_intrinsics[1] <= 0.0 || camera_intrinsics[4] <= 0.0 ||
        camera_intrinsics[5] <= 0.0) {
      throw std::runtime_error(
          "camera_intrinsics must be positive "
          "[fu, fv, ppu, ppv, width, height]");
    }
    intrinsics_ = {
        static_cast<float>(camera_intrinsics[0]),
        static_cast<float>(camera_intrinsics[1]),
        static_cast<float>(camera_intrinsics[2]),
        static_cast<float>(camera_intrinsics[3]),
        static_cast<int>(camera_intrinsics[4]),
        static_cast<int>(camera_intrinsics[5])};

    const auto color_topic = declare_parameter<std::string>(
        "color_topic", "/camera/color/image_raw");
    const auto depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image_raw");
    const auto color_info_topic = declare_parameter<std::string>(
        "color_info_topic", "/camera/color/camera_info");
    const auto depth_info_topic = declare_parameter<std::string>(
        "depth_info_topic", "/camera/depth/camera_info");

    if (publish_rate_ <= 0.0 || n_frames_ <= 1 || depth_scale_ <= 0.0) {
      throw std::runtime_error(
          "publish_rate and depth_scale must be positive; n_frames must be > 1");
    }

    auto qos = rclcpp::SensorDataQoS();
    pub_color_ =
        create_publisher<sensor_msgs::msg::Image>(color_topic, qos);
    pub_depth_ =
        create_publisher<sensor_msgs::msg::Image>(depth_topic, qos);
    pub_color_info_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(color_info_topic, qos);
    pub_depth_info_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(depth_info_topic, qos);
    pub_gt_pose_ =
        create_publisher<geometry_msgs::msg::PoseStamped>("~/pose_gt", 10);
    pub_gt_marker_ =
        create_publisher<visualization_msgs::msg::Marker>("~/marker_gt", 1);
    tf_broadcaster_ =
        std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    SetUpRenderer();
    color_info_ = MakeCameraInfo(intrinsics_, camera_frame_);
    depth_info_ = MakeCameraInfo(intrinsics_, camera_frame_);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>{1.0 / publish_rate_});
    timer_ = create_wall_timer(period, [this]() { PublishFrame(); });
    RCLCPP_INFO(get_logger(),
                "online synthetic RGB-D | body=%s frames=%d rate=%.1f Hz "
                "depth_noise=%.4f distortion=%.3f",
                body_->name().c_str(), n_frames_, publish_rate_, depth_noise_,
                distortion_);
  }

 private:
  void SetUpRenderer() {
    body_ = m3t_ros2::DeclareAndCreateBody(this, true);
    renderer_geometry_ =
        std::make_shared<m3t::RendererGeometry>("synthetic_renderer_geometry");
    renderer_geometry_->AddBody(body_);
    renderer_ = std::make_shared<m3t::FullNormalRenderer>(
        "synthetic_renderer", renderer_geometry_,
        m3t::Transform3fA::Identity(), intrinsics_, 0.1f, 5.0f);
    if (!body_->SetUp() || !renderer_geometry_->SetUp() ||
        !renderer_->SetUp()) {
      throw std::runtime_error("failed to set up synthetic M3T renderer");
    }
    if (body_->vertices().empty()) {
      throw std::runtime_error("body mesh contains no vertices");
    }

    Eigen::Vector3f lower = body_->vertices().front();
    Eigen::Vector3f upper = lower;
    for (const auto &vertex : body_->vertices()) {
      lower = lower.cwiseMin(vertex);
      upper = upper.cwiseMax(vertex);
    }
    mesh_center_ = 0.5f * (lower + upper);
    const float diagonal = (upper - lower).norm();
    viewing_distance_ =
        intrinsics_.fu * diagonal / (0.45f * intrinsics_.height);

    depth_lut_.resize(65536);
    for (int value = 0; value < 65536; ++value) {
      depth_lut_[value] = renderer_->Depth(static_cast<ushort>(value));
    }

    if (distortion_ != 0.0) {
      map_x_.create(intrinsics_.height, intrinsics_.width, CV_32F);
      map_y_.create(intrinsics_.height, intrinsics_.width, CV_32F);
      for (int y = 0; y < intrinsics_.height; ++y) {
        for (int x = 0; x < intrinsics_.width; ++x) {
          const float xn = (x - intrinsics_.ppu) / intrinsics_.fu;
          const float yn = (y - intrinsics_.ppv) / intrinsics_.fv;
          const float scale =
              1.0f - static_cast<float>(distortion_) * (xn * xn + yn * yn);
          map_x_.at<float>(y, x) =
              xn * scale * intrinsics_.fu + intrinsics_.ppu;
          map_y_.at<float>(y, x) =
              yn * scale * intrinsics_.fv + intrinsics_.ppv;
        }
      }
    }

    RCLCPP_INFO(get_logger(),
                "mesh diagonal %.4f m, synthetic viewing distance %.4f m",
                diagonal, viewing_distance_);
  }

  m3t::Transform3fA PoseForFrame(int frame) const {
    constexpr float kPi = 3.14159265358979323846f;
    const float phase =
        2.0f * kPi * static_cast<float>(frame) /
        static_cast<float>(n_frames_);
    const float spin =
        phase * static_cast<float>(spin_turns_);
    const float nod =
        static_cast<float>(nod_degrees_) * kPi / 180.0f * std::sin(phase);
    const Eigen::Matrix3f rotation =
        (Eigen::AngleAxisf(nod, Eigen::Vector3f::UnitX()) *
         Eigen::AngleAxisf(spin, Eigen::Vector3f::UnitZ()))
            .toRotationMatrix();
    const Eigen::Vector3f center_in_view{
        viewing_distance_ * 0.15f * std::sin(phase),
        viewing_distance_ * 0.10f * std::cos(phase),
        viewing_distance_ * (1.0f + 0.10f * std::sin(phase))};

    m3t::Transform3fA pose{m3t::Transform3fA::Identity()};
    pose.linear() = rotation;
    pose.translation() = center_in_view - rotation * mesh_center_;
    return pose;
  }

  void PublishGroundTruth(const rclcpp::Time &stamp,
                          const m3t::Transform3fA &body2world) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = world_frame_;
    pose.pose = ToPose(body2world);
    pub_gt_pose_->publish(pose);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = gt_frame_;
    transform.transform.translation.x = pose.pose.position.x;
    transform.transform.translation.y = pose.pose.position.y;
    transform.transform.translation.z = pose.pose.position.z;
    transform.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);

    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "gt";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.mesh_resource = mesh_resource_;
    marker.mesh_use_embedded_materials = mesh_embedded_;
    marker.pose = ToPose(body2world * body_->geometry2body_pose());
    marker.scale.x = marker.scale.y = marker.scale.z = mesh_scale_;
    marker.color.r = 0.1f;
    marker.color.g = 0.9f;
    marker.color.b = 0.1f;
    marker.color.a = 0.5f;
    pub_gt_marker_->publish(marker);
  }

  void PublishFrame() {
    if (frame_index_ >= n_frames_) {
      if (loop_) {
        frame_index_ = 0;
      } else {
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "synthetic sequence complete");
        return;
      }
    }

    const m3t::Transform3fA pose = PoseForFrame(frame_index_);
    body_->set_body2world_pose(pose);
    if (!renderer_->StartRendering() || !renderer_->FetchNormalImage() ||
        !renderer_->FetchDepthImage()) {
      RCLCPP_ERROR(get_logger(), "rendering failed at frame %d", frame_index_);
      return;
    }

    cv::Mat color;
    cv::cvtColor(renderer_->normal_image(), color, cv::COLOR_BGRA2BGR);
    const cv::Mat &raw_depth = renderer_->depth_image();
    cv::Mat depth(intrinsics_.height, intrinsics_.width, CV_16U,
                  cv::Scalar{0});
    for (int y = 0; y < intrinsics_.height; ++y) {
      for (int x = 0; x < intrinsics_.width; ++x) {
        const cv::Vec3b pixel = color.at<cv::Vec3b>(y, x);
        if (!(pixel[0] || pixel[1] || pixel[2])) continue;
        float z = depth_lut_[raw_depth.at<ushort>(y, x)];
        if (depth_noise_ > 0.0) {
          z += static_cast<float>(depth_noise_) * z * z * gaussian_(rng_);
        }
        const int scaled =
            static_cast<int>(std::lround(z / depth_scale_));
        depth.at<ushort>(y, x) = static_cast<ushort>(
            std::min(std::max(scaled, 0), 65535));
      }
    }

    if (distortion_ != 0.0) {
      cv::Mat distorted_color;
      cv::Mat distorted_depth;
      cv::remap(color, distorted_color, map_x_, map_y_, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar{0, 0, 0});
      cv::remap(depth, distorted_depth, map_x_, map_y_, cv::INTER_NEAREST,
                cv::BORDER_CONSTANT, cv::Scalar{0});
      color = distorted_color;
      depth = distorted_depth;
    }

    const rclcpp::Time stamp = now();
    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = camera_frame_;
    color_info_.header.stamp = stamp;
    depth_info_.header.stamp = stamp;

    // Publish GT and depth before color.  The tracker treats color as the frame
    // trigger, so this ordering also gives non-synchronizing subscribers the
    // newest depth/pose before the corresponding color image arrives.
    PublishGroundTruth(stamp, pose);
    pub_color_info_->publish(color_info_);
    pub_depth_info_->publish(depth_info_);
    pub_depth_->publish(
        *cv_bridge::CvImage(header, "16UC1", depth).toImageMsg());
    pub_color_->publish(
        *cv_bridge::CvImage(header, "bgr8", color).toImageMsg());
    ++frame_index_;
  }

  std::string world_frame_;
  std::string camera_frame_;
  std::string gt_frame_;
  std::string mesh_resource_;
  double publish_rate_{30.0};
  int n_frames_{240};
  bool loop_{true};
  double depth_noise_{0.0};
  double distortion_{0.0};
  double depth_scale_{0.001};
  double spin_turns_{1.0};
  double nod_degrees_{25.0};
  double mesh_scale_{1.0};
  bool mesh_embedded_{false};
  int frame_index_{0};

  m3t::Intrinsics intrinsics_{};
  Eigen::Vector3f mesh_center_{Eigen::Vector3f::Zero()};
  float viewing_distance_{0.5f};
  std::shared_ptr<m3t::Body> body_;
  std::shared_ptr<m3t::RendererGeometry> renderer_geometry_;
  std::shared_ptr<m3t::FullNormalRenderer> renderer_;
  std::vector<float> depth_lut_;
  cv::Mat map_x_;
  cv::Mat map_y_;
  std::mt19937 rng_;
  std::normal_distribution<float> gaussian_{0.0f, 1.0f};

  sensor_msgs::msg::CameraInfo color_info_;
  sensor_msgs::msg::CameraInfo depth_info_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_color_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_color_info_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_info_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_gt_pose_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_gt_marker_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SyntheticSourceNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("m3t_synthetic_source"), "%s",
                 error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
