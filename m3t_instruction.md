# M3T — Build & Run Instructions

How to build the [M3T](M3T/readme.md) 3D object tracker and run an example.
Verified on arm64, Ubuntu 22.04, ROS 2 Humble.

> **Note:** M3T is a **plain CMake library**, *not* a ROS 2 package (there is no
> `package.xml`). Build it with `cmake`, **not** `colcon`.

---

## Quick start (TL;DR)

```bash
# 1. Build
cd 3dobjecttracking/M3T && mkdir -p build temp && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DUSE_GTEST=ON \
      -DUSE_AZURE_KINECT=OFF -DUSE_REALSENSE=OFF ..
cmake --build . -j$(nproc)

# 2. Run — HEADLESS (prints pose, no window)
cd examples
./run_on_recorded_sequence_headless \
    ../../data/_sequence/color_camera.yaml ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_static_detector.yaml ../../temp 10

# 3. Run — GUI (viewer window, needs a display)
export DISPLAY=:0
./run_on_recorded_sequence_gui \
    ../../data/_sequence/color_camera.yaml ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_manual_detector.yaml ../../temp
```

Details below.

---

## 1. Dependencies

Required: Eigen 3, GLEW, GLFW 3, OpenCV 4, and (for the tests) GTest.

```bash
apt-get update && apt-get install -y \
    libeigen3-dev libglew-dev libglfw3-dev \
    libopencv-dev libgtest-dev cmake build-essential
```

> **OpenCV `xfeatures2d` caveat.** Upstream M3T requires OpenCV's `xfeatures2d`
> module (only used for the DAISY/FREAK descriptors in the texture modality;
> the default descriptor is ORB). Ubuntu's OpenCV package **excludes**
> `xfeatures2d` for patent reasons. This repo has been patched to make it
> **optional** (see `M3T/CMakeLists.txt`, `include/m3t/texture_modality.h`,
> `src/texture_modality.cpp` — guarded by `USE_XFEATURES2D`; DAISY/FREAK fall
> back to ORB when it is absent). No action needed. If you ever build OpenCV
> from source *with* `opencv_contrib`, the macro is auto-detected and the
> original behaviour is restored.

## 2. Build

```bash
cd 3dobjecttracking/M3T
mkdir -p build && cd build

cmake -DCMAKE_BUILD_TYPE=Release \
      -DUSE_GTEST=ON \
      -DUSE_AZURE_KINECT=OFF \
      -DUSE_REALSENSE=OFF \
      ..

cmake --build . -j$(nproc)
```

- `USE_AZURE_KINECT` / `USE_REALSENSE` are OFF because no physical depth camera
  is attached. Turn them ON only if the K4A / librealsense SDKs are installed
  and a camera is connected.
- Use `Release` for real tracking performance.

## 3. Run the unit tests (optional sanity check)

The readme defines a correct build as being able to run `./gtest_run`:

```bash
cd 3dobjecttracking/M3T
mkdir -p temp                     # tests write here
cd build/test
./gtest_run
```

Expected on arm64: **208 / 213 pass**. The 5 failures
(`TrackerTest.OptimizePoseMatrix` ×2, `RegionModalityTest` gradient/hessian ×2,
`NormalColorViewerTest.UpdateAndSaveImage`) are **not defects** — they are tiny
floating-point / rendered-pixel differences (~1e-3) against golden reference
values the DLR authors recorded on **x86** hardware, checked with a strict
`1e-5` tolerance. Expected cross-platform behaviour.

> Running the tests regenerates the committed model fixtures in
> `data/model_test/*.bin`, so they will show as modified in git. Revert them —
> do **not** commit the machine-specific versions:
> ```bash
> git checkout -- M3T/data/model_test/*.bin
> ```

## 4. Run an example

Both examples below track the `triangle` body on the recorded image sequence in
`data/_sequence` — no physical camera needed. Run from `build/examples`.

The first run generates the region model (`temp/region_model.bin`, 2562
viewpoints — takes a bit); later runs reuse it.

> The shipped `data/_sequence` is only a **2-frame** unit-test clip, so both demo
> binaries below use a looping camera (`looping_loader_camera.h`) that repeats
> the clip — otherwise the sequence is exhausted instantly and the GUI window
> would close before you could interact with it.

### 4a. Headless mode (no window, prints pose)

`run_on_recorded_sequence_headless` (added in this repo) uses a `StaticDetector`
for the initial pose and prints the estimated pose per frame — no viewer window,
no clicking. It runs for `n_frames` (optional 5th arg, default 10) then exits.

```bash
cd 3dobjecttracking/M3T
mkdir -p temp
cd build/examples

./run_on_recorded_sequence_headless \
    ../../data/_sequence/color_camera.yaml \
    ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_static_detector.yaml \
    ../../temp 10
```

Expected output:

```text
[frame 0] triangle  translation(m)=[-0.081876, -0.00546736, 0.618302]  quaternion(wxyz)=[...]
[frame 1] triangle  translation(m)=[-0.081876, -0.00546736, 0.618302]  quaternion(wxyz)=[...]
...
Done (10 frames).
```

(The pose is constant because the 2-frame fixture has no real object motion.)

> If you have no display at all, the offscreen renderer still needs one; wrap
> the command with a virtual framebuffer: `xvfb-run -a ./run_on_recorded_sequence_headless ...`
> (`apt-get install -y xvfb`).

### 4b. GUI mode (viewer window + manual clicks)

`run_on_recorded_sequence_gui` (added in this repo) opens a `NormalColorViewer`
window and uses a `ManualDetector`. With the window focused: click the 4
reference points on `D`, then use keys `D`=detect, `T`=track, `X`=detect+track,
`S`=stop, `Q`=quit. Needs a display (`export DISPLAY=:0`, adjust to your setup).

```bash
cd build/examples

export DISPLAY=:0
./run_on_recorded_sequence_gui \
    ../../data/_sequence/color_camera.yaml \
    ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_manual_detector.yaml \
    ../../temp
```

A 960×540 "viewer" window appears (on the VM desktop) and loops the clip until
you press `Q`. The `dbind-WARNING`, `canberra-gtk-module`, and `XDG_RUNTIME_DIR`
messages are harmless. (Stock `run_on_recorded_sequence` is the non-looping
original; on this 2-frame clip it would exit immediately.)

### 4c. GUI mode, auto-track (viewer window, no clicking)

`run_on_recorded_sequence_gui_auto` (added in this repo) is the easiest one to
watch: it uses a `StaticDetector` (no clicking) and starts detecting + tracking
automatically, so the `NormalColorViewer` overlay appears immediately. The clip
loops until you press `Q` (`S`=stop, `D`=detect, `T`/`X`=track).

```bash
cd build/examples

export DISPLAY=:0
./run_on_recorded_sequence_gui_auto \
    ../../data/_sequence/color_camera.yaml \
    ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_static_detector.yaml \
    ../../temp
```

Add a `viz` argument at the end to also open the region-modality **debug
window(s)** — the correspondence lines and result points overlaid on the object:

```bash
./run_on_recorded_sequence_gui_auto \
    ../../data/_sequence/color_camera.yaml \
    ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_static_detector.yaml \
    ../../temp viz
```

### Changing the initial pose (auto examples)

The auto examples get their initial pose from the **`StaticDetector` YAML**:
`data/_body/triangle_static_detector.yaml`, the `link2world_pose` 4×4 matrix
(rotation 3×3 + translation in the last column, metres). Edit that matrix to
move the starting guess. The tracker then **refines** this guess frame by frame
(the examples call `set_start_tracking_after_detection(true)` so tracking begins
right after detection). The shipped value is already an approximate pose, close
to but not exactly the true object pose — a small offset the tracker corrects.

> On the 2-frame looping clip, refinement with only a color `RegionModality` is
> under-constrained in depth (z), so the estimate can drift. Real multi-frame
> sequences (and adding a depth modality) constrain it properly. The manual
> example (§4b) instead derives the initial pose from your 4 clicks, not a YAML.

### Summary of runnable examples

| Binary | Window? | Interaction | What to look at |
|--------|---------|-------------|-----------------|
| `run_on_recorded_sequence_headless` | no | none | pose printed per frame |
| `run_on_recorded_sequence_gui_auto` | yes | none (auto) | tracking overlay (add `viz` for internals) |
| `run_on_recorded_sequence_gui` | yes | click 4 pts + keys | manual-detection workflow |

---

## 5. Notes on generated files & git

- `build/` and `temp/` are already covered by `M3T/.gitignore`, so
  runtime-generated `.bin` model files and the build tree are **not** tracked.
- The only `.bin` files under version control are the golden fixtures in
  `data/model_test/` — leave those to git; revert if the tests dirty them
  (see §3).

## 6. Local modifications in this repo (branch `test_m3t`)

| File | Change |
|------|--------|
| `M3T/CMakeLists.txt` | `xfeatures2d` moved to `OPTIONAL_COMPONENTS`; defines `USE_XFEATURES2D` only when found |
| `M3T/include/m3t/texture_modality.h` | `xfeatures2d.hpp` include guarded by `USE_XFEATURES2D` |
| `M3T/src/texture_modality.cpp` | DAISY/FREAK guarded, fall back to ORB when `xfeatures2d` absent; SIFT namespace guarded |
| `M3T/examples/looping_loader_camera.h` | Helper: `LoaderColorCamera` that loops a short clip instead of stopping at its end |
| `M3T/examples/run_on_recorded_sequence_headless.cpp` | New headless example (StaticDetector + pose-printing Publisher, looping, runs `n_frames` then exits) |
| `M3T/examples/run_on_recorded_sequence_gui.cpp` | New GUI example (NormalColorViewer + ManualDetector, looping so the window stays open) |
| `M3T/examples/run_on_recorded_sequence_gui_auto.cpp` | New GUI example (NormalColorViewer + StaticDetector, auto-track, optional `viz` debug windows) |
| `M3T/examples/CMakeLists.txt` | Registers the new example targets |
