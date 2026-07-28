# m3t_ros2

A ROS 2 (Humble) wrapper around the [M3T](../M3T) 3D object tracker. 

A single in-process node runs the tracker on a recorded **RGB-D** sequence read from disk (no inter-node image topics), with the **object, initial pose, and modality combination selectable from launch args / parameters**, and publishes everything needed to visualize tracking in RViz.

## Design

- **In-process, no topic round-trip for images.** The tracker talks to M3T `Camera` objects by direct C++ calls (M3T's `LoaderColorCamera` / `LoaderDepthCamera` read the sequence from disk). Swapping to a live camera later only means providing ROS-backed `Camera` subclasses.
- **Two threads, decoupled.** A dedicated **worker thread** drives the tracker step by step and runs *only* the pose solve (`ExecuteTrackingStep`) plus all OpenGL. A separate **publisher** (a ROS wall-timer on the spin thread) reads the latest **snapshot** and does all ROS serialization + the ORB image. They share one mutex-guarded `Snapshot` (pose + cloned, ref-counted `cv::Mat`), so the publisher never blocks or corrupts the solve loop — no data race, no segfault. Publish rate is independent of solve rate.
- **Modality selection** builds the `Link` with exactly the chosen modalities: `region`, `depth`, `texture`.
- **Ground truth** from `poses_gt_matrix.txt` is published alongside the estimate.

## Build

```bash
cd <workspace_dir>
NUM_JOBS=2 && \
export CMAKE_BUILD_PARALLEL_LEVEL=${NUM_JOBS} && \
export MAKEFLAGS=-j${NUM_JOBS} && \
export NINJAFLAGS=-j${NUM_JOBS} && \
colcon build --symlink-install \
  --packages-up-to m3t_ros2 \
  --executor sequential --parallel-workers ${NUM_JOBS} \
  --cmake-force-configure \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && \
  . install/setup.bash
```

## Generate a sequence (once)

The node consumes a folder produced by M3T's `generate_orbit_sequence` (see
[../m3t_instruction.md](../m3t_instruction.md) §4d–4f):

```bash
cd <workspace_dir>/src/3dobjecttracking/M3T/build/examples
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root
./generate_orbit_sequence ../../data/_body/cylinder.yaml ../../temp/rgbd_cylinder 180 0.002 0.15
```

## Run

```bash
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root   # offscreen GL for the renderer

ros2 launch m3t_ros2 m3t.launch.py \
  object:=cylinder \
  modalities:=region,depth \
  rviz:=true

ros2 launch m3t_ros2 m3t.launch.py \
  object:=mustard  \
  modalities:=region,depth,texture \
  rviz:=true
```

Launch / node args: `object` (triangle|box|cylinder|mustard), `modalities`
(comma list of region,depth,texture), `sequence_dir`, `m3t_root`, `rviz`, and:

| Param | Default | Meaning |
|-------|---------|---------|
| `track_rate` | `0.0` | solve-loop Hz; **0 = as fast as possible** (for benchmarking) |
| `publish_rate` | `30.0` | Hz of the decoupled publisher / snapshot |
| `log_period` | `2.0` | seconds between solve-time log lines |

## Solve-time logging

Every `log_period` seconds the worker logs the **pure pose-solve time** (`ExecuteTrackingStep` only — no image I/O, no rendering-for-viz, no ROS):

```
solve: 93 frames | mean 15.87 ms (63 Hz) | min 14.99 max 18.66 ms | loop 46 Hz
```

Measured on this arm64 Parallels VM (`track_rate:=0`):

| Modalities | mean solve | ≈ solve Hz |
|------------|-----------|-----------|
| region | ~16–19 ms | ~55 Hz |
| region + depth | ~31–41 ms | ~28 Hz |
| region + depth + texture | ~35 ms | ~28 Hz |

> The M3T papers report **300+ Hz**, but on **x86 + a real GPU**. Each solve does several OpenGL silhouette/depth renders (5 correspondence iterations); on Parallels' virtualized GL a render is ~3 ms, so region-only is ~15 ms/solve here. On real GPU hardware those renders drop to <1 ms and the CPU optimization dominates → hundreds of Hz. The node measures the pure solve time correctly; the absolute number reflects the VM's GL, not the tracker.

## Published interfaces

| Topic | Type | Content |
|-------|------|---------|
| `~/color/image_raw` | Image (bgr8) | raw color frame |
| `~/depth/image_raw` | Image (16UC1) | raw depth frame (mm) |
| `~/overlay/image` | Image (bgr8) | model normals blended over color (the old GUI view) |
| `~/keypoints/image` | Image (bgr8) | ORB keypoints on color |
| `~/marker_est`, `~/marker_gt` | Marker (MESH_RESOURCE) | mesh at estimate (red) / GT (green) |
| TF `world→object_est`, `→object_gt` | tf2 | estimate / GT pose |

## Verified

- `colcon build --packages-up-to m3t_ros2` builds `m3t` then `m3t_ros2`.
- Node runs region, region+depth, and region+depth+texture; publishing decoupled from solve (topics ~22–48 Hz while the solve loop runs independently).
- Per-frame estimate vs ground-truth (matched by stamp): **mean 4.9 mm, max 13.8 mm** on the noisy+distorted cylinder (region+depth). The cylinder's spin about its symmetry axis is not observable, so only that DoF diverges.

## Notes

- `libm3t` is currently linked **statically**; making it a `.so` would need relaxing M3T's hidden-visibility preset.
- The renderer needs an X server for its offscreen GL context — set `DISPLAY`.
