# Robot Codebase — HERO

This is the codebase for our robot. We are testing a segmentation algorithm that uses `yolo_onnx_ros`, `sam_onnx_ros`, and `bmm_ros`. All three packages run inside `ed_sensor_integration`.

## Key Files

All actively worked on files are in:
```
/home/amigo/ros/noetic/system/src/ed_sensor_integration/src/kinect/
```
- `segmenter.cpp`
- `updater.cpp`
- `sam_seg_module.cpp`

## Previous Bug

Objects close to a surface (e.g. `on_top_of dinner_table`) are not appearing correctly in the 3D point cloud. Even when the YOLO bounding box and SAM segmentation are correct, the resulting point cloud has only 3–4 points — the rest are filtered out.

## Bug Solved:

comment the removeBackground fixed the bug
```cpp
//segmenter_->removeBackground(filtered_depth_image, world_updated, cam_model, sensor_pose, req.background_padding);
```

## Bug 2 (Solved): Objects appear lower than their actual height

Objects were placed in the world map lower than their actual height. A fork or knife's convex hull appeared inside the table instead of on top of it. People's convex hull lowest point (feet) appeared inside the ground.

**Root cause — two issues in `updater.cpp`:**

1. **Legacy extend-and-shrink logic** — the pipeline was extending `chull.z_min` by 4 cm downward and calling `refitConvexHull` to "snap" the object to its supporting surface. With `removeBackground` disabled, this captured actual table/floor depth points in the extended search volume, pulling the estimated centre z below the surface.
2. **5th-percentile z_min** — inside `refitConvexHull`, using the 5th percentile for `z_min` allowed background/noise points to drag the entity's z further down.

**Fix applied in `updater.cpp`:**

- Removed the three legacy functions that were no longer needed with the new YOLO+SAM+BMM pipeline:
  - `refitConvexHull` — re-fitted hull to raw depth image to compensate for inaccurate old clustering
  - `mergeConvexHulls` — merged overlapping hulls from old clustering
  - `mergeOverlappingConvexHulls` — detected and merged colliding clusters
- Removed the commented-out `mergeOverlappingConvexHulls` call and the extend-and-shrink block that called `refitConvexHull`
- Removed the now-unused `#include <geolib/shapes.h>`

**Why these functions are no longer needed:**
The old depth-clustering could produce inaccurate or overlapping clusters, requiring post-hoc correction. With YOLO+SAM, each YOLO detection gets exactly one SAM mask → one cluster per object, no overlap. BMM then denoises the point cloud in sensor frame. The convex hull built directly in `cluster()` from the clean filtered world-frame points is already accurate — no refitting needed.

## Build & Run

**1. Compile on terminal 1:**
```bash
tue-make ed_sensor_integration
```
**2. Open new bash - terminal 2:**
```bash
hero-free-mode
```
**CRITICAL:** hero-free-mode needs to run on a seperated terminal as long as you run ed_sensor_integration tests. When you finish running the experiments stop this command.

**2. Trigger the segmentation pipeline on a terminal 1:**
```bash
rosservice call /hero/ed/kinect/update "area_description: 'on_top_off dinner_table'
background_padding: 0.0
fit_supporting_entity: false"
```

**3. Inspect output** in the `hero-free-mode` terminal 2. Check how many points each cluster has — if too few, that's the problem.

**Example output:**
```
[WARN][/hero/ed]: We reject cluster 5 because it has only 0 points
[WARN][/hero/ed]: We reject cluster 2 because it has only 0 points
[WARN][/hero/ed]: Cluster 1: 16 points
[WARN][/hero/ed]: We reject cluster 3 because it has only 0 points
[WARN][/hero/ed]: Cluster 4: 70 points
[ERROR][/hero/ed]: Cluster 1 classified as 'apple' with confidence 0.30
[ERROR][/hero/ed]: Cluster 4 classified as 'dining table' with confidence 0.11
```

**4. Close terminal processes:** Stop `hero-free-mode` in terminal 2.

## Table Model Reference

**Table model file:**
```
/home/amigo/ros/noetic/repos/github.com/tue-robotics/ed_object_models/models/table_120x80x76/model.sdf
```

**World model entity definitions (pose, type, image per environment):**
```
/home/amigo/ros/noetic/repos/github.com/tue-robotics/hero_bringup/parameters/world_modeling/models_impuls.yaml
```
Note: `dinner_table` has no fixed pose in this file — its pose is set at runtime by localization.

**Correct table calibration (DO NOT CHANGE):**
- Tabletop link: center at z=0.75m, thickness 0.02m → top surface at **z=0.76m**
- `on_top_of` volume: center at z=0.96m, height 0.42m → **bottom at 0.75m, top at 1.17m**
- The volume bottom (0.75m) is 1cm *below* the table surface (0.76m) — this is intentional so flat objects (knife, spoon at z≈0.76m) are included in the segmentation volume
- **Do not raise the volume bottom** — raising it by even 2cm causes flat objects like knives and spoons to fall outside the volume and not be detected at all
- Attempted raising pose z from `0.96` to `0.97` (bottom from 0.75m to 0.77m) → **FAILED**, reverted

## Validator Agent

A reusable validation agent is available at `.github/agents/ed-sensor-validator.agent.md`.
Run it any time after making changes to `ed_sensor_integration` to verify correctness.

### Last validation run (2026-02-26) — PASS

All checks passed:
- Build: 18 packages, 0 warnings
- Coordinate frame consistency: PASS
- Object height sanity: PASS (no z-shift code paths remain)
- BMM denoising logic: PASS
- Legacy functions absent (`refitConvexHull`, `mergeConvexHulls`, `mergeOverlappingConvexHulls`): PASS
- `removeBackground` still commented out: PASS

### Known pre-existing warnings (non-blocking)

**W1 — YOLO+SAM models re-initialized every sensor update** (`sam_seg_module.cpp:20-34`)
Both `Initialize()` calls (ONNX runtime + weights) happen on every call to `SegmentationPipeline` instead of once at startup. This causes large per-frame overhead and risks silent "nothing seen" failures if init returns null. Should be moved to constructor or `configure()`.

**W2 — Possible race condition on `config_`** (`segmenter.cpp:219-345`)
`config_` (a `tue::Configuration`) is read inside `#pragma omp parallel for` without a mutex. Safe only if the reader is purely stateless; risky if it maintains internal cursor/traversal state. Fix: read `psi0`/`nu0`/`alpha`/`kappa0` once before the parallel loop and capture by value.
