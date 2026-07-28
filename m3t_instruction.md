# M3T — Build & Run Instructions

How to build the [M3T](M3T/readme.md) 3D object tracker and run an example.
Verified on arm64, Ubuntu 22.04, ROS 2 Humble.

> **Note:** M3T is a **plain CMake library**, *not* a ROS 2 package (there is no
> `package.xml`). Build it with `cmake`, **not** `colcon`.

---

## Quick start (TL;DR)

```bash
# 1. Build
cd 3dobjecttracking/M3T && sudo mkdir -p build temp && cd build
sudo cmake -DCMAKE_BUILD_TYPE=Release -DUSE_GTEST=ON \
      -DUSE_AZURE_KINECT=OFF -DUSE_REALSENSE=OFF ..
sudo cmake --build . -j$(nproc)

# 2. Run — HEADLESS (prints pose, no window)
cd examples
sudo  ./run_on_recorded_sequence_headless \
    ../../data/_sequence/color_camera.yaml ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_static_detector.yaml ../../temp 10

# 3. Run — GUI (viewer window, needs a display)
export DISPLAY=:0
sudo  ./run_on_recorded_sequence_gui \
    ../../data/_sequence/color_camera.yaml ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_manual_detector.yaml ../../temp
```

Details below.

---

## 1. Dependencies

Required: Eigen 3, GLEW, GLFW 3, OpenCV 4, and (for the tests) GTest.

```bash
sudo  apt-get update && sudo apt-get install -y \
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

sudo cmake -DCMAKE_BUILD_TYPE=Release \
      -DUSE_GTEST=ON \
      -DUSE_AZURE_KINECT=OFF \
      -DUSE_REALSENSE=OFF \
      ..

sudo cmake --build . -j$(nproc)
```

- `USE_AZURE_KINECT` / `USE_REALSENSE` are OFF because no physical depth camera
  is attached. Turn them ON only if the K4A / librealsense SDKs are installed
  and a camera is connected.
- Use `Release` for real tracking performance.

## 3. Run the unit tests (optional sanity check)

The readme defines a correct build as being able to run `./gtest_run`:

```bash
cd 3dobjecttracking/M3T
sudo mkdir -p temp                     # tests write here
cd build/test
sudo ./gtest_run
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
sudo mkdir -p temp
cd build/examples

sudo ./run_on_recorded_sequence_headless \
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
sudo ./run_on_recorded_sequence_gui \
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
sudo ./run_on_recorded_sequence_gui_auto \
    ../../data/_sequence/color_camera.yaml \
    ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_static_detector.yaml \
    ../../temp
```

Add a `viz` argument at the end to also open the region-modality **debug
window(s)** — the correspondence lines and result points overlaid on the object:

```bash
sudo ./run_on_recorded_sequence_gui_auto \
    ../../data/_sequence/color_camera.yaml \
    ../../data/_body/triangle.yaml \
    ../../data/_body/triangle_static_detector.yaml \
    ../../temp viz
```

### 4d. Long sequence with large motion (synthetic)

The shipped `data/_sequence` is only 2 frames, so you can't really *see*
tracking. `generate_orbit_sequence` (added in this repo) renders the body from a
moving viewpoint into a long sequence with large motion (a spin + nod +
translation sweep), and writes ready-to-use metafiles. This lets you watch the
overlay follow the object over many frames, with exact ground truth.

**Step 1 — generate the sequence** (writes `frameNNNN.png`, `color_camera.yaml`,
`static_detector.yaml`, `poses_gt.txt` into the out dir; 180 frames by default):

```bash
cd build/examples
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root
sudo ./generate_orbit_sequence ../../data/_body/triangle.yaml ../../temp/orbit 180
```

**Step 2 — track it** (headless to print, or GUI to watch). Note the camera and
detector metafiles now come from the generated `temp/orbit` folder:

```bash
# headless (prints tracked pose per frame; compare with temp/orbit/poses_gt.txt)
sudo ./run_on_recorded_sequence_headless \
    ../../temp/orbit/color_camera.yaml ../../data/_body/triangle.yaml \
    ../../temp/orbit/static_detector.yaml ../../temp/orbit 180

# GUI (watch the overlay track the moving triangle; press Q to quit)
sudo ./run_on_recorded_sequence_gui_auto \
    ../../temp/orbit/color_camera.yaml ../../data/_body/triangle.yaml \
    ../../temp/orbit/static_detector.yaml ../../temp/orbit
```

The generator **auto-fits** the viewing distance to the mesh's bounding box (so
the object fills ~45% of the frame) and **recenters** off-origin meshes, so any
object is framed correctly. The motion (spin + nod + translation sweep) scales
with the object. This is a *synthetic* sequence rendered from the model (object
on black) — a controlled way to confirm tracking under large motion.

Verified: the triangle tracks to within a few mm of ground truth across the
whole 180-frame motion.

**Other shapes.** The generator works with any body metafile. Two more
primitives ship in `data/_body/` — a **box** and a **cylinder** — so you can
compare object types. Generate and track them the same way, just swapping the
body:

```bash
# box (0.08 x 0.05 x 0.03 m)
sudo ./generate_orbit_sequence ../../data/_body/box.yaml ../../temp/orbit_box 180
sudo ./run_on_recorded_sequence_gui_auto ../../temp/orbit_box/color_camera.yaml \
    ../../data/_body/box.yaml ../../temp/orbit_box/static_detector.yaml ../../temp/orbit_box

# cylinder (r=0.028, h=0.08 m)
sudo ./generate_orbit_sequence ../../data/_body/cylinder.yaml ../../temp/orbit_cyl 180
sudo ./run_on_recorded_sequence_gui_auto ../../temp/orbit_cyl/color_camera.yaml \
    ../../data/_body/cylinder.yaml ../../temp/orbit_cyl/static_detector.yaml ../../temp/orbit_cyl
```

The box tracks to ~1 mm like the triangle. The **cylinder drifts more** (a few mm
up to ~1 cm): it is rotationally symmetric, so a single-camera region-only
tracker has a pose ambiguity about its axis and a weak depth constraint — a
representative failure mode. Adding a `DepthModality` (depth camera) would
constrain it. (Note: the headless printout labels the body `triangle` — that is
just the fixed internal body name in the example, not the object being tracked.)

### 4e. Any mesh — the full pipeline (e.g. a YCB object)

Because the generator auto-fits distance and recenters, you can drop in **any
triangulated wavefront `.obj`** (M3T loads it via tinyobjloader, so `v/vt/vn`
textured meshes are fine — only the geometry is used). The pipeline is:

1. **Get a mesh** (`.obj`, triangulated). Note its unit (m vs mm).
2. **Write a body YAML** with `geometry_path` (relative to the YAML) and
   `geometry_unit_in_meter` (`1.0` if the mesh is in metres, `0.001` if mm);
   keep `geometry_enable_culling: 0`.
3. **Generate** the sequence from that YAML.
4. **Track** it with the headless or GUI auto example.

**Worked example — YCB `006_mustard_bottle`.** The body YAML is committed
(`data/_body/ycb_mustard_bottle.yaml`); the mesh is third-party, so download it
into `temp/ycb_mustard/` (where the YAML points):

```bash
cd 3dobjecttracking/M3T
sudo mkdir -p temp/ycb_mustard && (cd temp/ycb_mustard && \
  sudo curl -sL https://ycb-benchmarks.s3.amazonaws.com/data/google/006_mustard_bottle_google_16k.tgz | tar xz)

cd build/examples
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root
# generate (auto-detects the ~19 cm bottle size and frames it)
sudo ./generate_orbit_sequence ../../data/_body/ycb_mustard_bottle.yaml ../../temp/orbit_mustard 180
# watch it track
sudo ./run_on_recorded_sequence_gui_auto ../../temp/orbit_mustard/color_camera.yaml \
    ../../data/_body/ycb_mustard_bottle.yaml \
    ../../temp/orbit_mustard/static_detector.yaml ../../temp/orbit_mustard
```

Verified: it follows the large motion (x sweeps ≈ +0.02 → −0.12 m); like the
cylinder it drifts a few mm–cm in depth (roughly symmetric, region-only, single
camera). To try other YCB objects, swap the object name in the download URL and
in the YAML's `geometry_path` (browse them at
`http://ycb-benchmarks.s3-website-us-east-1.amazonaws.com/`).

### 4f. Multi-modality RGB-D (Region + Depth) — no camera needed

The shipped multi-modality examples need a physical RGB-D camera
(`run_on_camera_sequence`, pen-paper demo) or a dataset download (the
evaluators). To get a **self-contained** multi-modality run, the generator also
renders a **depth** image per frame and writes `depth_camera.yaml`, and
`run_on_recorded_sequence_rgbd` tracks with **RegionModality (color) +
DepthModality (depth)** together — all from disk.

The generator takes two optional knobs to make the depth realistic, both tunable:

```text
generate_orbit_sequence <body> <out> [n_frames=180] [depth_noise=0] [distortion=0]
  depth_noise : axial depth-noise sigma at 1 m, in metres; sigma scales with z^2.
                e.g. 0.01 ~ 1 mm at 0.3 m. 0 = clean.
  distortion  : radial lens-distortion coefficient k1, applied to color AND depth
                (they stay registered). e.g. 0.15 = slight barrel. 0 = none.
```

```bash
cd build/examples
export DISPLAY=:0 XDG_RUNTIME_DIR=/tmp/runtime-root
# generate an RGB-D cylinder sequence with depth noise + slight distortion
sudo ./generate_orbit_sequence ../../data/_body/cylinder.yaml ../../temp/rgbd_cyl 180 0.002 0.15

# track with Region + Depth
sudo ./run_on_recorded_sequence_rgbd \
    ../../temp/rgbd_cyl/color_camera.yaml ../../temp/rgbd_cyl/depth_camera.yaml \
    ../../data/_body/cylinder.yaml ../../temp/rgbd_cyl/static_detector.yaml \
    ../../temp/rgbd_cyl 180
```

The generator now always writes `depthNNNN.png` (16-bit millimetres, 0 = no
measurement) and `depth_camera.yaml` alongside the color frames; with
`depth_noise` / `distortion` left at 0 the color frames are identical to before,
so the region-only examples are unaffected.

**Depth helps.** On the noisy + distorted cylinder, at the hardest frame the
region-only tracker drifts ~15 mm while **Region + Depth** stays ~1.5 mm — the
depth modality constrains what a single color-region view leaves ambiguous
(depth and the axis of symmetric objects). Compare yourself by running
`run_on_recorded_sequence_headless` (region only) vs `run_on_recorded_sequence_rgbd`
on the same generated sequence.

**Verified on all four bodies** (180-frame RGB-D sequences, `depth_noise=0.002`,
`distortion=0.15`, Region + Depth): every one tracked the full sequence without
losing lock. Mean translation error stays low; the ~15 mm peaks come from the
uncorrected lens distortion (the pinhole tracker cannot compensate `k1`):

| Body | mean err | max err |
|------|----------|---------|
| triangle | 1.6 mm | 14.9 mm |
| box | 2.5 mm | 15.5 mm |
| cylinder | 3.1 mm | 15.0 mm |
| mustard (YCB) | 6.4 mm | 18.6 mm |

**Watch it live (GUI).** `run_on_recorded_sequence_gui_rgbd` runs the same
Region + Depth tracker but with a `NormalColorViewer`, so you see the model
overlaid on the color frames while depth refines the pose (needs a display):

```bash
export DISPLAY=:0
sudo ./run_on_recorded_sequence_gui_rgbd \
    ../../temp/rgbd_cyl/color_camera.yaml ../../temp/rgbd_cyl/depth_camera.yaml \
    ../../data/_body/cylinder.yaml ../../temp/rgbd_cyl/static_detector.yaml \
    ../../temp/rgbd_cyl
```

**The third modality — texture.** `run_on_recorded_sequence_texture` tracks with
Region + **TextureModality** (image keypoints validated against a
`FocusedSilhouetteRenderer`). It uses the color sequence only:

```bash
sudo ./run_on_recorded_sequence_texture \
    ../../temp/rgbd_box/color_camera.yaml ../../data/_body/box.yaml \
    ../../temp/rgbd_box/static_detector.yaml ../../temp/rgbd_box 180
```

> TextureModality needs trackable image features **on the object surface**. On
> the synthetic normal-rendered frames it finds tens of keypoints at the shading
> edges — enough to run and track — but it shines on **real textured objects**
> (or a textured render), where hundreds of stable surface features are
> available. M3T's own renderer only produces geometry (normals/depth/
> silhouette), not a textured RGB image, so for a strong texture demo use a real
> dataset or an externally textured render.

### Changing / testing the initial pose (auto examples)

The initial guess is the **`StaticDetector` YAML → `link2world_pose`** 4×4 matrix
(rotation 3×3 + translation in the last column, metres). For the shipped triangle
that is `data/_body/triangle_static_detector.yaml`; **for a generated sequence it
is `temp/<seq>/static_detector.yaml`** (e.g. `temp/orbit_box/static_detector.yaml`).
The tracker refines this guess frame by frame (the examples call
`set_start_tracking_after_detection(true)`).

**To test a different initial guess — the correct procedure:**

1. Generate the sequence **once** (do not touch the body YAML).
2. Edit `link2world_pose` in `temp/<seq>/static_detector.yaml` (or pass a copy as
   the detector argument). Perturb the translation by a few mm/cm or the rotation
   by a few degrees.
3. Re-run **only the tracker** on the same images — **do NOT regenerate**. The
   images are the true motion; you are only changing where the tracker starts.
4. Compare the tracked pose with `temp/<seq>/poses_gt.txt`.

Keep the perturbation **small**. A region tracker only recovers if the guessed
silhouette *overlaps* the real object: a small offset (≈1 cm here) snaps back to
truth on the first frame, but an offset of roughly half the object size leaves no
overlap and the tracker locks onto a wrong local minimum and never recovers.

> **Do NOT change the initial guess via the body YAML's `geometry2body_pose`.**
> That is the geometry↔body transform — it *redefines the object model* and is
> baked into the rendered images (the mesh is drawn at `body2world · geometry2body`).
> Editing it desynchronises the images from the tracker's model, so tracking
> breaks — and stays broken even after you set it back, until you **regenerate**
> the sequence (so images, `static_detector.yaml`, and the model all agree again).
>
> On the tiny 2-frame clip, refinement with only a color `RegionModality` is
> under-constrained in depth (z), so the estimate can drift. Real multi-frame
> sequences (and adding a depth modality) constrain it properly. The manual
> example (§4b) instead derives the initial pose from your 4 clicks, not a YAML.

### Summary of runnable examples

| Binary | Window? | Interaction | What to look at |
|--------|---------|-------------|-----------------|
| `run_on_recorded_sequence_headless` | no | none | pose printed per frame (region only) |
| `run_on_recorded_sequence_rgbd` | no | none | pose per frame, **Region + Depth** (needs a depth sequence) |
| `run_on_recorded_sequence_texture` | no | none | pose per frame, **Region + Texture** |
| `run_on_recorded_sequence_gui_auto` | yes | none (auto) | tracking overlay (add `viz` for internals) |
| `run_on_recorded_sequence_gui_rgbd` | yes | none (auto) | **Region + Depth** live overlay |
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
| `M3T/examples/generate_orbit_sequence.cpp` | New tool: renders a long synthetic large-motion **RGB-D** sequence + metafiles; auto-fits distance, recenters any mesh, writes depth PNGs + `depth_camera.yaml`, with configurable depth noise + lens distortion |
| `M3T/examples/run_on_recorded_sequence_rgbd.cpp` | New multi-modality example: Region (color) + Depth modality, from disk, no physical camera |
| `M3T/examples/run_on_recorded_sequence_gui_rgbd.cpp` | New: Region + Depth with a live `NormalColorViewer` overlay |
| `M3T/examples/run_on_recorded_sequence_texture.cpp` | New: Region + Texture modality (the third modality) |
| `M3T/examples/looping_loader_camera.h` (depth) | Adds `LoopingLoaderDepthCamera` alongside the color one |
| `M3T/data/_body/box.{obj,yaml}` | Box primitive (0.08×0.05×0.03 m) for the generator |
| `M3T/data/_body/cylinder.{obj,yaml}` | Cylinder primitive (r=0.028, h=0.08 m) for the generator |
| `M3T/data/_body/ycb_mustard_bottle.yaml` | Body YAML for the YCB `006_mustard_bottle` example (mesh downloaded, not committed) |
| `M3T/examples/CMakeLists.txt` | Registers the new example targets |
