# jpg_depth_node README
## What it does

jpg_depth_node runs Depth Anything v3 (TensorRT) on a folder of JPG frames, optionally reads per-frame detection JSON (NanoOWL format), and produces:

1. Depth map (ROS topic)

2. RGB point cloud (ROS topic)

3. Client-friendly JSON output per frame containing estimated 3D object positions from the detections.

This is useful for offline pipelines where your downstream “client” is not ROS2.

## Inputs
### 1) Frames
A folder with images:
* *.jpg / *.jpeg
### 2) Detections JSON (optional)
For each frame, an optional JSON with the same basename:
* R1_20260127_133752.jpg
* R1_20260127_133752.json

Expected JSON layout (NanoOWL):
```json
{
  "pose": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "yaw": 1.23811
  },
  "image": "R1_20260127_133757.jpg",
  "entries": [
    {
      "timestamp": 1770730679,
      "prompt": "Describe the objects in the image",
      "response": "- A rifle\n- A poster\n- A switch\n- A cable\n- A frame"
    }
  ],
  "nanoowl": {
    "ts": 1770730679.6203315,
    "iso_time": "2026-02-10T13:37:59Z",
    "endpoint": "http://172.16.17.15:5060/infer",
    "status": 200,
    "prompts": [
      "a rifle",
      "a poster",
      "a switch",
      "a cable",
      "a frame"
    ],
    "annotate": 0,
    "result": {
      "detections": [
        {
          "bbox": [
            80,
            150,
            204,
            197
          ],
          "label": "a rifle",
          "score": 0.5627567768096924
        }
      ],
      "image": {
        "height": 360,
        "width": 640
      },
      "latency_sec": 0.0789,
      "prompts": [
        "a rifle",
        "a poster",
        "a switch",
        "a cable",
        "a frame"
      ]
    }
  }
}
```

### 3) Camera intrinsics YAML (required)

A calibration YAML containing:

* image_width, image_height
* camera_matrix.data = [fx,0,cx, 0,fy,cy, 0,0,1]

```yaml

# Image resolution
image_width: 640
image_height: 480

# Camera intrinsic parameters from calibration
# fx, fy: focal length
# cx, cy: principal point (optical center)
fx: 465.321303
fy: 466.557484
cx: 303.131049
cy: 276.938277
```
Important: The node resizes each JPG to the YAML’s width/height to keep intrinsics consistent.

## Outputs
### ROS topics
* ~/output/depth_image (sensor_msgs/Image, encoding 32FC1, meters)
* ~/output/point_cloud (sensor_msgs/PointCloud2, fields x y z rgb)
### Per-frame JSON file (for non-ROS client)
In output_dir, for each frame:
*<basename>.objects.json
Example output:
```json
{
  "camera": {
    "cx": 303.131049,
    "cy": 276.938277,
    "fx": 465.321303,
    "fy": 466.557484,
    "height": 480,
    "width": 640
  },
  "image": "R1_20260127_133757.jpg",
  "objects": [
    {
      "bbox": [
        80,
        200,
        204,
        263
      ],
      "depth_m": 2.3233333333333337,
      "label": "a rifle",
      "score": 0.5627567768096924,
      "xyz_m": [
        -0.8045218105491868,
        -0.22627064656267185,
        2.3233333333333337
      ]
    }
  ],
  "stamp_unix_ms": 1770735987231
}
```
Coordinates xyz_m are in the camera optical frame implied by the intrinsics (same frame as the depth).

## Parameters
Required:

* jpg_dir (string): directory of JPGs
* output_dir (string): where *.objects.json will be written
* camera_yaml (string): intrinsics yaml path
* model_path (string): DepthAnything v3 onnx / engine path * expected by your TRT wrapper

Optional:

* json_dir (string): directory containing detection JSONs (default: jpg_dir)
* fps (double): playback rate (default: 10.0)
* frame_id (string): output frame id for ROS headers (default: camera_link)
* min_depth (double): min valid depth in meters (default: 0.2)
* max_depth (double): max valid depth in meters (default: 10.0)
* min_score (double): detection score threshold (default: 0.2)
* hist_bins (int): histogram bins for depth mode (default: 30)
* hist_min_frac (double): required dominance of selected depth bin (default: 0.06)

## Running
```bash 
ros2 run depth_anything_v3 jpg_depth_node --ros-args \
  -p jpg_dir:=/home/user1/Pictures/ExpoTLV/2026_02_10___15_37_31 \
  -p json_dir:=/home/user1/Pictures/ExpoTLV/2026_02_10___15_37_31 \
  -p output_dir:=/tmp/da3_objects \
  -p camera_yaml:=/home/user1/depth_anything_ws/src/ros2-depth-anything-v3-trt/camera_info_example.yaml \
  -p model_path:=/home/user1/depth_anything_ws/src/ros2-depth-anything-v3-trt/onnx/DA3METRIC-LARGE/DA3METRIC-LARGE.onnx \
  -p fps:=10.0 \
  -p frame_id:=camera_optical_frame

```


## Build (colcon)
From your workspace root:
```bash
cd ~/depth_anything_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select depth_anything_v3 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## HTTP service (bbox + depth)
This binary runs an HTTP server with a `/bbox_depth` endpoint that accepts:
- multipart form file `image` (JPEG/PNG)
- optional multipart form file `detections` (JSON)
- optional multipart form field `image_dir` (directory path used for saving colormap images)

### Run the HTTP server
```bash
ros2 run depth_anything_v3 depth_anything_http_server \
  --model /home/user/depth_anything_ws/src/ros2-depth-anything-v3-trt/onnx/DA3-SMALL/DA3-SMALL.onnx \
  --camera-yaml /home/user/depth_anything_ws/src/ros2-depth-anything-v3-trt/camera_info_4k.yaml \
  --host 0.0.0.0 --port 5070 \
  --save-depth 1 \
  --save-every 1 \
  --save-max-depth 10.0
```

Notes:
- Saving happens **per request** (not per second). `--save-every N` saves every N-th request.
- The save directory is derived from the request's `image_dir`:
  if `image_dir=/a/b/c` then depth images go under `/a/b/c_depth/`.

### Python request example (with saving)
When sending `files=...` in `requests`, also send `image_dir` as a multipart field:

```python
from pathlib import Path
import os, json, requests

with open(current_img_path, "rb") as f_img:
    files = {
        "image": (os.path.basename(current_img_path), f_img, "image/jpeg"),
        "detections": ("detections.json", json.dumps(body), "application/json"),
        # IMPORTANT: send image_dir as multipart form field
        "image_dir": (None, str(Path(current_img_path).parent)),
    }
    r_depth = requests.post(depth_endpoint, files=files, timeout=30)
```
