// SPDX-License-Identifier: MIT
// Copyright (c) 2023 Manuel Stoiber, German Aerospace Center (DLR)

// Generates a synthetic recorded sequence with LARGE camera/object motion so the
// tracker can be watched following the object over many frames (the shipped
// data/_sequence is only 2 frames). It renders the body's normals from a moving
// viewpoint into frameNNNN.png and writes ready-to-use metafiles:
//   <out>/frameNNNN.png       rendered color frames (object on black)
//   <out>/color_camera.yaml   LoaderColorCamera metafile for the frames
//   <out>/static_detector.yaml StaticDetector with the frame-0 ground-truth pose
//   <out>/poses_gt.txt        ground-truth translation per frame (for reference)
//
// Then track it with, e.g.:
//   ./run_on_recorded_sequence_gui_auto <out>/color_camera.yaml \
//       ../../data/_body/triangle.yaml <out>/static_detector.yaml <out>
//
// Usage:
//   ./generate_orbit_sequence <body_metafile> <out_directory> [n_frames]

#include <filesystem/filesystem.h>
#include <m3t/body.h>
#include <m3t/common.h>
#include <m3t/normal_renderer.h>
#include <m3t/renderer_geometry.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <vector>

namespace {

void WriteOpenCvMatrix(std::ofstream &ofs, const std::string &key,
                       const Eigen::Matrix4f &m, const std::string &indent) {
  ofs << key << ": !!opencv-matrix\n";
  ofs << indent << "rows: 4\n" << indent << "cols: 4\n" << indent << "dt: d\n";
  ofs << indent << "data: [ ";
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      ofs << m(r, c);
      if (!(r == 3 && c == 3)) ofs << ", ";
    }
  ofs << " ]\n";
}

// Body-to-world pose for frame i (camera sits at the world origin looking down
// +z). Combines an in-image spin, a gentle nod, and a translation sweep so the
// motion is large over the sequence but small between consecutive frames. All
// translations are scaled by the base viewing distance so the motion is
// proportional to the object regardless of its size.
m3t::Transform3fA PoseForFrame(int i, int n_frames, float distance,
                               const Eigen::Vector3f &center) {
  const float phase = 2.0f * float(M_PI) * float(i) / float(n_frames);
  const float spin = 1.5f * float(M_PI) / 180.0f * float(i);  // ~1.5 deg/frame
  const float nod = 25.0f * float(M_PI) / 180.0f * std::sin(phase);

  Eigen::Matrix3f R =
      (Eigen::AngleAxisf(nod, Eigen::Vector3f::UnitX()) *
       Eigen::AngleAxisf(spin, Eigen::Vector3f::UnitZ()))
          .toRotationMatrix();
  // Where the object CENTER should appear (in front of the camera).
  Eigen::Vector3f t_view{distance * 0.15f * std::sin(phase),
                         distance * 0.10f * std::cos(phase),
                         distance * (1.0f + 0.1f * std::sin(phase))};

  m3t::Transform3fA pose{m3t::Transform3fA::Identity()};
  pose.linear() = R;
  // Offset so the mesh center (not its possibly off-origin frame) lands at
  // t_view: origin = t_view - R * center.
  pose.translation() = t_view - R * center;
  return pose;
}

}  // namespace

int main(int argc, char *argv[]) {
  if (argc < 3 || argc > 6) {
    std::cerr << "Usage: body metafile, out directory, [n_frames=180], "
                 "[depth_noise=0], [distortion=0]\n"
                 "  depth_noise: axial depth-noise sigma at 1 m, in metres "
                 "(sigma scales with z^2). Try 0.002.\n"
                 "  distortion: radial lens-distortion coefficient k1 applied to "
                 "color+depth. Try 0.15."
              << std::endl;
    return -1;
  }
  const std::filesystem::path body_metafile_path{argv[1]};
  const std::filesystem::path out_directory{argv[2]};
  const int n_frames{argc >= 4 ? std::stoi(argv[3]) : 180};
  const float depth_noise{argc >= 5 ? std::stof(argv[4]) : 0.0f};
  const float distortion{argc >= 6 ? std::stof(argv[5]) : 0.0f};
  const float kDepthScale = 0.001f;  // depth PNG unit: millimetres
  std::filesystem::create_directories(out_directory);

  // Camera intrinsics (matches the shipped triangle sequence).
  const m3t::Intrinsics intrinsics{698.128f, 698.617f, 478.459f,
                                   274.426f, 960,      540};

  // Body (culling off so it is visible from either side while spinning).
  auto body_ptr{std::make_shared<m3t::Body>("body", body_metafile_path)};
  body_ptr->set_geometry_enable_culling(false);

  // Renderer with the camera at the world origin.
  auto renderer_geometry_ptr{
      std::make_shared<m3t::RendererGeometry>("renderer_geometry")};
  renderer_geometry_ptr->AddBody(body_ptr);
  auto renderer_ptr{std::make_shared<m3t::FullNormalRenderer>(
      "renderer", renderer_geometry_ptr, m3t::Transform3fA::Identity(),
      intrinsics, 0.1f, 5.0f)};
  if (!body_ptr->SetUp() || !renderer_geometry_ptr->SetUp() ||
      !renderer_ptr->SetUp()) {
    std::cerr << "Set up failed" << std::endl;
    return -1;
  }

  // Compute the mesh's true axis-aligned bounding box (in the body frame) to
  // auto-fit the viewing distance and to recenter possibly off-origin meshes.
  Eigen::Vector3f lo = body_ptr->vertices().front();
  Eigen::Vector3f hi = lo;
  for (const auto &v : body_ptr->vertices()) {
    lo = lo.cwiseMin(v);
    hi = hi.cwiseMax(v);
  }
  const Eigen::Vector3f center = 0.5f * (lo + hi);
  const float diagonal = (hi - lo).norm();
  const float distance = intrinsics.fu * diagonal / (0.45f * intrinsics.height);
  std::cout << "mesh bbox diagonal=" << diagonal << " m, viewing distance="
            << distance << " m" << std::endl;

  std::cout << "depth_noise=" << depth_noise << " (sigma at 1 m), distortion k1="
            << distortion << std::endl;

  // Lookup table: renderer depth-buffer value (ushort) -> metric depth [m].
  std::vector<float> depth_lut(65536);
  for (int v = 0; v < 65536; ++v)
    depth_lut[v] = renderer_ptr->Depth(static_cast<ushort>(v));

  // Radial-distortion remap: for each pixel of the (distorted) output, sample
  // the ideal pinhole render at ideal ~= distorted * (1 - k1 * r^2). Applied to
  // color and depth alike so they stay registered.
  const int W = intrinsics.width, H = intrinsics.height;
  cv::Mat map_x, map_y;
  if (distortion != 0.0f) {
    map_x.create(H, W, CV_32F);
    map_y.create(H, W, CV_32F);
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        const float xn = (x - intrinsics.ppu) / intrinsics.fu;
        const float yn = (y - intrinsics.ppv) / intrinsics.fv;
        const float s = 1.0f - distortion * (xn * xn + yn * yn);
        map_x.at<float>(y, x) = xn * s * intrinsics.fu + intrinsics.ppu;
        map_y.at<float>(y, x) = yn * s * intrinsics.fv + intrinsics.ppv;
      }
  }

  std::mt19937 rng{12345};
  std::normal_distribution<float> gauss{0.0f, 1.0f};

  std::ofstream poses_ofs{(out_directory / "poses_gt.txt").string()};
  // Full 4x4 body2world per frame (row-major, 16 values/line) for consumers
  // that need orientation too (e.g. the ROS node's GT marker/TF).
  std::ofstream poses_mat_ofs{(out_directory / "poses_gt_matrix.txt").string()};
  m3t::Transform3fA pose0;

  for (int i = 0; i < n_frames; ++i) {
    const m3t::Transform3fA pose = PoseForFrame(i, n_frames, distance, center);
    if (i == 0) pose0 = pose;
    body_ptr->set_body2world_pose(pose);

    if (!renderer_ptr->StartRendering() || !renderer_ptr->FetchNormalImage() ||
        !renderer_ptr->FetchDepthImage()) {
      std::cerr << "Rendering failed at frame " << i << std::endl;
      return -1;
    }
    cv::Mat bgr;
    cv::cvtColor(renderer_ptr->normal_image(), bgr, cv::COLOR_BGRA2BGR);

    // Build a metric depth image (millimetres, 0 = no measurement) with noise.
    const cv::Mat &raw = renderer_ptr->depth_image();  // CV_16U buffer values
    cv::Mat depth_mm(H, W, CV_16U, cv::Scalar(0));
    for (int y = 0; y < H; ++y)
      for (int x = 0; x < W; ++x) {
        const cv::Vec3b c = bgr.at<cv::Vec3b>(y, x);
        if (!(c[0] || c[1] || c[2])) continue;  // background stays 0
        float z = depth_lut[raw.at<ushort>(y, x)];
        if (depth_noise > 0.0f) z += depth_noise * z * z * gauss(rng);
        const int mm = int(std::lround(z / kDepthScale));
        depth_mm.at<ushort>(y, x) =
            static_cast<ushort>(std::min(std::max(mm, 0), 65535));
      }

    if (distortion != 0.0f) {
      cv::Mat bgr_d, depth_d;
      cv::remap(bgr, bgr_d, map_x, map_y, cv::INTER_LINEAR,
                cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
      cv::remap(depth_mm, depth_d, map_x, map_y, cv::INTER_NEAREST,
                cv::BORDER_CONSTANT, cv::Scalar(0));
      bgr = bgr_d;
      depth_mm = depth_d;
    }

    std::ostringstream cname, dname;
    cname << "frame" << std::setw(4) << std::setfill('0') << i << ".png";
    dname << "depth" << std::setw(4) << std::setfill('0') << i << ".png";
    cv::imwrite((out_directory / cname.str()).string(), bgr);
    cv::imwrite((out_directory / dname.str()).string(), depth_mm);

    const Eigen::Vector3f t = pose.translation();
    poses_ofs << t.x() << " " << t.y() << " " << t.z() << "\n";
    const Eigen::Matrix4f m = pose.matrix();
    for (int r = 0; r < 4; ++r)
      for (int c = 0; c < 4; ++c)
        poses_mat_ofs << m(r, c) << (r == 3 && c == 3 ? '\n' : ' ');
  }
  poses_ofs.close();
  poses_mat_ofs.close();

  // Write the LoaderColorCamera metafile.
  {
    std::ofstream ofs{(out_directory / "color_camera.yaml").string()};
    ofs << "%YAML:1.2\n";
    ofs << "load_directory: \"./\"\n";
    ofs << "intrinsics:\n";
    ofs << "   f_u: " << intrinsics.fu << "\n";
    ofs << "   f_v: " << intrinsics.fv << "\n";
    ofs << "   pp_x: " << intrinsics.ppu << "\n";
    ofs << "   pp_y: " << intrinsics.ppv << "\n";
    ofs << "   width: " << intrinsics.width << "\n";
    ofs << "   height: " << intrinsics.height << "\n";
    WriteOpenCvMatrix(ofs, "camera2world_pose",
                      Eigen::Matrix4f::Identity(), "   ");
    ofs << "image_name_pre: \"frame\"\n";
    ofs << "load_index: 0\n";
    ofs << "n_leading_zeros: 4\n";
    ofs << "image_name_post: \"\"\n";
    ofs << "load_image_type: \"png\"\n";
  }

  // Write the LoaderDepthCamera metafile (registered with the color camera).
  {
    std::ofstream ofs{(out_directory / "depth_camera.yaml").string()};
    ofs << "%YAML:1.2\n";
    ofs << "load_directory: \"./\"\n";
    ofs << "intrinsics:\n";
    ofs << "   f_u: " << intrinsics.fu << "\n";
    ofs << "   f_v: " << intrinsics.fv << "\n";
    ofs << "   pp_x: " << intrinsics.ppu << "\n";
    ofs << "   pp_y: " << intrinsics.ppv << "\n";
    ofs << "   width: " << intrinsics.width << "\n";
    ofs << "   height: " << intrinsics.height << "\n";
    ofs << "depth_scale: " << kDepthScale << "\n";
    WriteOpenCvMatrix(ofs, "camera2world_pose",
                      Eigen::Matrix4f::Identity(), "   ");
    ofs << "image_name_pre: \"depth\"\n";
    ofs << "load_index: 0\n";
    ofs << "n_leading_zeros: 4\n";
    ofs << "image_name_post: \"\"\n";
    ofs << "load_image_type: \"png\"\n";
  }

  // Write the StaticDetector metafile with the frame-0 ground-truth pose.
  {
    std::ofstream ofs{(out_directory / "static_detector.yaml").string()};
    ofs << "%YAML:1.2\n";
    WriteOpenCvMatrix(ofs, "link2world_pose", pose0.matrix(), "  ");
  }

  std::cout << "Wrote " << n_frames << " frames + metafiles to "
            << out_directory.string() << std::endl;
  return 0;
}
