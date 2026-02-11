# jpg_depth_node — Progress & TODOs

Last updated: 2026-02-10

## Status
✅ `jpg_depth_node` runs successfully:
- Reads JPG frames from a folder
- Runs Depth Anything v3 (TensorRT) to produce depth
- Publishes depth image + RGB pointcloud (ROS)
- Reads NanoOWL detections JSON (per frame)
- Computes object XYZ using bbox + histogram depth
- Writes per-frame `*.objects.json` for a non-ROS client

---

## TODOs

### 1) Support tiled depth inference
**Goal:** Run inference on large images by splitting into tiles to reduce memory and improve detail, then stitch results back.

**Notes / Design choices:**
- Choose a fixed tile size (e.g. 640×360, 768×432, 1024×576) that matches the engine’s expected input.
- Tiles should overlap (e.g. 32–96 px) to reduce seams.
- Stitch strategy:
  - Prefer center region of each tile (ignore borders), or
  - Blend overlaps with weighted averaging (feathering).

**Implementation outline:**
- Add params:
  - `tiling_enabled` (bool, default: false)
  - `tile_w`, `tile_h` (int)
  - `tile_overlap` (int)
  - `tile_blend` (string: `center_crop` | `feather`)
- If enabled:
  1. Resize or keep full image at target size (decide policy).
  2. Generate tile ROIs with overlap.
  3. For each tile: run `doInference()` and get tile depth.
  4. Stitch into a full-size depth map using chosen blend policy.
  5. Continue pipeline (bbox depth + pointcloud) using stitched depth.

**Acceptance criteria:**
- No visible seams on typical scenes.
- Object depth stable near tile boundaries.

---

### 2) Add debug sleep / step-through between frames
**Goal:** Make debugging easier (slow playback or manual stepping).

**Params:**
- `debug_sleep_ms` (int, default: 0)
- Optional: `debug_step` (bool, default: false) — wait for keypress/Enter each frame (only if you want interactive mode).

**Implementation:**
- At end of `tick()`:
  - If `debug_sleep_ms > 0`: `std::this_thread::sleep_for(...)`
  - If `debug_step == true`: print prompt and block on stdin (optional).

**Acceptance criteria:**
- When enabled, the node clearly slows between frames without affecting output correctness.

---

### 3) Depth calibration
**Goal:** Correct systematic scale/bias error in predicted depth.

**Model options:**
- Scale only: `Z_cal = s * Z_pred`
- Scale + bias: `Z_cal = s * Z_pred + b`

**Params:**
- `depth_scale` (double, default: 1.0)
- `depth_bias`  (double, default: 0.0)

**Where to apply:**
- After tile stitching (if tiling enabled) / after depth inference:
  - Apply to entire `depth32f` before pointcloud generation.
- Also applies automatically to object `Z` (because it’s sampled from calibrated depth map).

**Calibration workflow:**
1. Collect a small set of frames with known distances to a flat surface/object.
2. Record pairs `(Z_pred, Z_gt)` from depth at a chosen pixel/ROI.
3. Fit `s` (and optionally `b`) with least squares.
4. Set `depth_scale` / `depth_bias` as node parameters.

**Acceptance criteria:**
- Measured depth error reduced on calibration scenes.
- No negative depths after calibration (clamp if needed).

---

## Nice-to-haves (later)
- Export both camera-frame and world-frame XYZ using the per-frame pose in JSON (`pose.yaw` + translation).
- Add an overlay debug image (bboxes + labels + depth stats).
- Add JSONL output option (single streaming file).
- Improve object depth sampling (shrink ROI / pick nearest dominant mode).

