# m3t_ros2

`m3t_ros2` is the unified ROS 2 interface for M3T. A normal colcon build compiles both M3T and this package; no source-local `M3T/build`, example generator, `sudo`, or generated files below `M3T/temp` are needed.

The tracker is independent of its image source:

```text
camera/image publisher                       m3t_tracker_node
  RGB Image -------------------------------> color camera
  RGB CameraInfo --------------------------> color intrinsics
  depth Image (when depth is enabled) -----> depth camera
  depth CameraInfo ------------------------> depth intrinsics
```

The tracker never reads camera intrinsics from an object or tracker config. It waits until the external camera has published both `Image` and `CameraInfo` before it initializes M3T. The optional synthetic and sequence nodes in this package are camera publishers for development only, so their own intrinsics are configured in their ROS parameter sections.

## Build

From the ROS 2 workspace:

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

M3T's library is a normal colcon package dependency. Its examples are disabled by default.

If `install/` was deleted after the workspace had been sourced, start a fresh container shell before rebuilding; the old shell still contains paths to the deleted prefixes and colcon will warn that they do not exist.

## Package layout

```text
m3t_ros2/
  assets/<object>/model.obj       mesh and future object assets
  config/m3t.yaml                 common node parameters
  config/objects/<object>.yaml    geometry and initial-pose parameters
  launch/m3t.launch.py            one launch interface
```

Object YAML files are ROS parameter files. They replace M3T body metafiles, static-detector YAML, and text GT pose files. A recorded sequence puts its directory, intrinsics, and all frame poses in another ROS parameter YAML passed with `sequence_config:=...`; copy `config/sequence_example.yaml` as a starting point.

Generated region/depth models are runtime cache files, not source assets. The launch default follows the OCS2 convention:

```text
<launch working directory>/auto_generated/m3t/<object>/
  region_model.bin
  depth_model.bin
```

Override it with `model_cache_dir:=/writable/path`. The tracker creates the directory and checks it is writable before setup. Direct `ros2 run` usage without that parameter falls back to `$ROS_HOME/m3t/cache/<object>` (normally `~/.ros/m3t/cache/<object>`).

## Run with the online synthetic source

This replaces `generate_orbit_sequence`: RGB, depth, CameraInfo, and GT are rendered and published online without writing an image sequence.

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic object:=box rviz:=true
```

Built-in objects are `triangle`, `box`, and `cylinder`. `modalities` accepts any comma-separated combination of `region`, `depth`, and `texture`; the default enables all three.

## Run only the tracker with an external camera

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=topics \
  object:=box \
  modalities:=region,depth \
  init_mode:=tf \
  color_topic:=/camera/color/image_raw \
  color_info_topic:=/camera/color/camera_info \
  depth_topic:=/camera/depth/image_raw \
  depth_info_topic:=/camera/depth/camera_info
```

`source:=topics` launches only `m3t_tracker_node`. No camera process is launched and no camera calibration is loaded by the tracker.

Initialization modes:

- `init_mode:=tf`: wait for `world_frame -> gt_frame`; in practice the named frame can be supplied by a detector, mocap system, or another ROS node.
- `init_mode:=static`: use the `initial_pose` 4x4 row-major matrix from the object ROS parameter YAML.
- `init_mode:=gt`: same TF mechanism, named explicitly for synthetic/recorded development sources.

The camera driver must publish valid dimensions and the pinhole matrix `K` in `sensor_msgs/msg/CameraInfo`. With the depth modality enabled, RGB and depth timestamps must fall within `sync_tolerance` (default 0.02 seconds).

## Run a recorded sequence

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=sequence \
  sequence_config:=/data/box_sequence/sequence.yaml \
  object:=box modalities:=region,depth
```

The sequence node is read-only. File patterns, frame count, camera intrinsics, and optional `gt_poses` are ROS parameters. It never creates files in the dataset or package tree.

## Custom object

Keep the same ROS YAML schema as `config/objects/box.yaml`, and point `geometry_path` at `m3t_ros2/assets/<object>/model.obj` (or an absolute mesh path):

```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=topics \
  object_config:=/absolute/path/to/my_object.yaml \
  modalities:=region
```

For an external config, relative `geometry_path` is resolved relative to that YAML file.

## Outputs and control

| Interface | Type |
|---|---|
| `~/overlay/image` | estimated-pose overlay |
| `~/keypoints/image` | texture-modality debug image |
| `~/marker_est` | estimated mesh marker |
| TF `world_frame -> object_est` | estimated object pose |
| `~/redetect` | `std_srvs/srv/Trigger` |

GT topics/TF exist only when the chosen development source publishes them.

## Automated smoke tests

After building and sourcing the workspace:

```bash
ros2 run m3t_ros2 m3t_smoke_test synthetic
ros2 run m3t_ros2 m3t_smoke_test external-camera
```

The second test verifies that tracker-only mode does not initialize before CameraInfo arrives, starts a separate camera publisher process, and then waits for a `TRACKED` result. Logs go to `${ROS_HOME:-$HOME/.ros}/m3t/smoke_logs`; set `M3T_SMOKE_TIMEOUT`, `M3T_SMOKE_OBJECT`, `M3T_SMOKE_MODALITIES`, or `M3T_SMOKE_INIT_MODE` to override test settings. For example, this verifies the object YAML's `initial_pose` path:

```bash
M3T_SMOKE_INIT_MODE=static \
  ros2 run m3t_ros2 m3t_smoke_test external-camera
```
