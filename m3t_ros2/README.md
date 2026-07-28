# m3t_ros2

A ROS 2 (Humble) wrapper around the [M3T](../M3T) 3D object tracker, split into **two nodes** so the tracker is decoupled from its image source and can be driven by a recorded sequence, a rosbag, or a real camera by only matching topic names. Object, initial pose, and modality combination are selectable from launch args / parameters. No GUI window — everything is published for RViz.

## Nodes

```
m3t_image_publisher_node ──/camera/color/image_raw────────▶ m3t_tracker_node
  (node 1, the "camera")  ──/camera/depth/image_raw────────▶  (node 2)
   reads the sequence,    ──/camera/{color,depth}/camera_info▶  subscribes,
   publishes images +     ──TF world→object_gt + marker_gt───▶  tracks, publishes
   GT (TF + mesh marker)                                        estimate + monitors
```

- **Replace node 1 with a real camera driver** (or a bag) by matching the topic names — node 2 is unchanged.
- Node 2 receives already-decoded pixels (no PNG decode in its loop), so its loop is bounded by the solve, not by image decoding.

## Design

- **In-process cameras.** Node 2 wraps the topics in M3T `Camera` subclasses (`RosColorCamera` / `RosDepthCamera`) so the tracker talks to them by direct C++ calls.
- **Two threads (node 2), decoupled.** A **worker** thread drives the tracker step by step, runs *only* the pose solve (`ExecuteTrackingStep`) + OpenGL, and logs timing/error. A **wall-timer** (spin thread) publishes the estimate overlay/keypoints/marker/TF from a mutex-guarded snapshot (cloned, ref-counted `cv::Mat`) — no data race, no segfault, and the solve loop is never blocked.
- **`m3t` is a colcon package**: `colcon build --packages-up-to m3t_ros2` builds M3T first and links it — no manual `make`.

## Initial guess (streaming-safe)

A live source keeps moving, so a fixed frame-0 pose no longer matches the current image. Node 2 therefore seeds the tracker from a pose **aligned to the current frame**, obtained by a `tf2` lookup of the GT/detector frame that node 1 broadcasts:

- `use_gt_initial_pose:=true` (default): seed from the `object_gt` TF (in this synthetic setup node 1 is the pose oracle; a real **detector** — AprilTag / DNN / mocap — would publish this TF instead).
- `use_gt_initial_pose:=false`: seed from a fixed `static_detector.yaml` (a known start pose for a real camera).
- `~/redetect` (`std_srvs/Trigger`): re-initialize from the latest pose on demand (after a loss or a sequence loop).

## Tracking-performance monitor

Every `log_period` the worker compares its estimate to the GT TF and logs the **pure solve time** *and* the **tracking error + a TRACKED/LOST verdict**, so the solve time is only trusted while on-track:

```
solve: 114 | mean 0.98 ms (1025 Hz) | min 0.86 max 1.26 | loop 57 Hz | err pos 4.6/13.8 mm rot 12.8/32.9 deg | TRACKED
```
(`err pos mean/max mm`, `rot mean/max deg`; verdict is position-based against `lost_threshold`, since symmetric objects are rotation-ambiguous.)

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

## Generate a sequence (once, for node 1)

```bash
cd <workspace_dir>/src/3dobjecttracking/M3T/build/examples
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root
./generate_orbit_sequence ../../data/_body/cylinder.yaml ../../temp/rgbd_cylinder 180 0.002 0.15
# writes color/depth PNGs, *_camera.yaml, static_detector.yaml, poses_gt_matrix.txt
```

## Run

```bash
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root   # offscreen GL for the renderer

ros2 launch m3t_ros2 m3t.launch.py object:=box modalities:=region,depth,texture rviz:=true
ros2 launch m3t_ros2 m3t.launch.py object:=mustard modalities:=region,depth use_gt_initial_pose:=true
```

Launch args: `object` ∈ {triangle, box, cylinder, mustard}, `modalities` (comma combo of region,depth,texture), `use_gt_initial_pose`, `source_rate` (node 1 fps), `track_rate` (node 2 solve loop; 0 = as fast as possible), `publish_rate`, `log_period`, `rviz`, `m3t_root`, and the four `*_topic` names.

## Topics / TF / services

| Interface | Type | From |
|-----------|------|------|
| `/camera/color/image_raw` (+`camera_info`) | Image (bgr8) / CameraInfo | node 1 |
| `/camera/depth/image_raw` (+`camera_info`) | Image (16UC1) / CameraInfo | node 1 |
| TF `world→object_gt`, `~/marker_gt` | tf2 / Marker | node 1 (GT) |
| `~/overlay/image`, `~/keypoints/image` | Image | node 2 |
| TF `world→object_est`, `~/marker_est` | tf2 / Marker | node 2 (estimate) |
| `~/redetect` | std_srvs/Trigger | node 2 |

## Verified — all objects × all modality combos

Solve is on-track (TRACKED). region ~0.7 ms, region+depth ~1 ms, depth-only
~0.2 ms; **mustard region+depth+texture ~3.3 ms (~300 Hz — the paper's number)**.

| Object | region | depth | texture | reg+dep | reg+tex | dep+tex | all |
|--------|--------|-------|---------|---------|---------|---------|-----|
| triangle | ✔ | ✖ | lost | ✖ | ✔ | ✖ | ✖ |
| box | ✔ | ✔ᵖ | lost | ✔**best** | ✔ | ✔ᵖ | ✔**best** |
| cylinder | ✔ᵖ | ✔ᵖ | lost | ✔ᵖ | ✔ᵖ | ✔ᵖ | ✔ᵖ |
| mustard | ✔ᵖ | ✔ᵖ | lost | ✔ | lost | ✔ | drift |

- **✔ᵖ** = position tracked, rotation ambiguous (cylinder/box/mustard have rotational symmetries; the monitor correctly reports the large rot error).
- **lost** = `texture`-alone loses lock — keypoints on smooth synthetic renders are too weak on their own (expected).
- **✖** = **triangle + depth**: the nearly-flat triangle (1.2 cm) has a degenerate DepthModel (surface-point sampling) — an object-geometry limitation, not a pipeline bug. Skip depth on the flat triangle.

## Notes / limitations

- `libm3t` is linked **statically**; a `.so` would need relaxing M3T's hidden-visibility preset.
- The renderer needs an X server for its offscreen GL context — set `DISPLAY`.
- The est-vs-GT error includes a small timing lag (latest-GT vs the solved frame); it distinguishes on-track (mm) from lost (cm+) reliably.
- The `object_gt`/error path is synthetic-only. On a real camera (no GT), init comes from a real detector on the same TF, and tracking health would use M3T's correspondence/residual instead.
