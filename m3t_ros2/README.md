# m3t_ros2

A ROS 2 (Humble) wrapper around the [M3T](../M3T) 3D object tracker. A single
in-process node runs the tracker on a recorded **RGB-D** sequence read from disk
(no inter-node image topics), with the **object, initial pose, and modality
combination selectable from launch args / parameters**, and publishes everything
needed to visualize tracking in RViz. No GUI window (the M3T viewer is replaced
by ROS topics).

## Design

- **In-process, no topic round-trip for images.** The tracker talks to M3T
  `Camera` objects by direct C++ calls. For a recorded sequence it uses M3T's
  `LoaderColorCamera` / `LoaderDepthCamera` (disk). Swapping to a live camera
  later means only providing ROS-backed `Camera` subclasses — the rest is
  unchanged.
- **`m3t` is a colcon package.** `colcon build --packages-up-to m3t_ros2` builds
  the M3T library first (it exports `M3TConfig.cmake`) and links it into the
  node — no manual `make` in `M3T/build`.
- **Modality selection** builds the `Link` with exactly the chosen modalities:
  `region` (RegionModality), `depth` (DepthModality), `texture`
  (TextureModality + FocusedSilhouetteRenderer).
- **Ground truth** is read from the sequence's `poses_gt_matrix.txt` and
  published alongside the estimate for comparison.

## Build

```bash
cd ~/ocs2_ros2_ws
colcon build --packages-up-to m3t_ros2
source install/setup.bash
```

## Generate a sequence (once)

The node consumes a folder produced by M3T's `generate_orbit_sequence` (color +
depth PNGs, `color_camera.yaml`, `depth_camera.yaml`, `static_detector.yaml`,
`poses_gt_matrix.txt`). See [../M3T/../m3t_instruction.md](../m3t_instruction.md)
§4d–4f. Example:

```bash
cd ~/ocs2_ros2_ws/src/3dobjecttracking/M3T/build/examples
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root
./generate_orbit_sequence ../../data/_body/cylinder.yaml ../../temp/rgbd_cylinder 180 0.002 0.15
```

## Run

```bash
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root   # offscreen GL for the renderer

ros2 launch m3t_ros2 m3t.launch.py object:=cylinder modalities:=region,depth rviz:=true
ros2 launch m3t_ros2 m3t.launch.py object:=mustard  modalities:=region,depth,texture rviz:=true
```

Launch args: `object` (triangle|box|cylinder|mustard), `modalities`
(comma list of region,depth,texture), `sequence_dir`, `fps`, `rviz`, `m3t_root`.

## Published interfaces

| Topic | Type | Content |
|-------|------|---------|
| `~/color/image_raw` | Image (bgr8) | raw color frame |
| `~/depth/image_raw` | Image (16UC1) | raw depth frame (mm) |
| `~/overlay/image` | Image (bgr8) | model normals blended over color (the old GUI view) |
| `~/keypoints/image` | Image (bgr8) | ORB keypoints drawn on color |
| `~/marker_est`, `~/marker_gt` | Marker (MESH_RESOURCE) | mesh at estimate (red) / GT (green) |
| TF `world_frame→object_est`, `→object_gt` | tf2 | estimate / GT pose |

## Verified

- `colcon build --packages-up-to m3t_ros2` builds `m3t` then `m3t_ros2`.
- Node runs region+depth and region+depth+texture, publishing all topics at ~20 Hz.
- Per-frame estimate vs ground-truth (matched by stamp): **mean 4.9 mm, max 13.8 mm**
  on the noisy+distorted cylinder (region+depth). The cylinder's spin about its
  symmetry axis is not observable, so only the orientation about that axis diverges.

## Notes

- `libm3t` is currently a static library; it is linked into the node. Making it a
  shared object would require relaxing M3T's hidden-visibility preset.
- The renderer needs an X server for its offscreen GL context — set `DISPLAY`.
