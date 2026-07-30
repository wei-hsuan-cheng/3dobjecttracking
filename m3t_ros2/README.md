# m3t_ros2

`m3t_ros2` is the unified ROS 2 interface for M3T.

The tracker is independent of its image source:

```text
camera/image publisher                       m3t_tracker_node
  RGB Image -------------------------------> color camera
  RGB CameraInfo --------------------------> color intrinsics
  depth Image (when depth is enabled) -----> depth camera
  depth CameraInfo ------------------------> depth intrinsics
```

- Built-in objects: `triangle`, `box`, `cylinder`, and `mustard`.
- Supported modalities: combination of `region`, `depth`, and `texture`.


The tracker never reads camera intrinsics from an object or tracker config. It waits until the external camera has published both `Image` and `CameraInfo` before it initializes M3T.

The optional synthetic and sequence nodes in this package are camera publishers for development only, so their own intrinsics are configured in their ROS parameter sections.

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


## Package layout

```text
m3t_ros2/
  assets/<object>/               mesh, material, and texture assets
  config/m3t.yaml                 common node parameters
  config/objects/<object>.yaml    geometry and initial-pose parameters
  launch/m3t.launch.py            one launch interface
```


## Run with the online synthetic source


```bash
ros2 launch m3t_ros2 m3t.launch.py \
  source:=synthetic object:=box rviz:=true

# object:=box | triangle | cylinder | mustard | etc.
```

Per-frame refinement is configured under `m3t_tracker_node.ros__parameters` in `config/m3t.yaml`. The tracker runs at least `min_corr_iterations` and at most `max_corr_iterations`, with `n_update_iterations` pose updates per correspondence round. It stops early after both pose-change thresholds remain satisfied for `convergence_required_rounds` consecutive rounds. Set `adaptive_iterations: false` to always run `max_corr_iterations`.

```yaml
adaptive_iterations: true
min_corr_iterations: 2
max_corr_iterations: 7
n_update_iterations: 2
convergence_translation_threshold: 0.0001  # meter
convergence_rotation_threshold_deg: 0.05
convergence_required_rounds: 2
```

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

TBD
