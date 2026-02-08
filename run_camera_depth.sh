#!/bin/bash
# Test single camera with lower resolution for faster inference
# Optional: Set USE_CALIBRATION=1 to use calibrated camera parameters
# Optional: Set CAMERA_INFO_FILE=/path/to/camera_info.yaml to load from file
# Optional: Set ENABLE_UNDISTORTION=1 to enable fisheye undistortion
# Optional: Set UNDISTORTION_BALANCE=0.0 to 1.0 (0.0=crop, 1.0=full FOV)

echo "=========================================="
echo "Single Camera Test (with Fisheye Undistortion Support)"
echo "=========================================="
echo ""

# Kill existing
echo "Cleaning up..."
pkill -9 -f camera_depth_node 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 2

# Source workspace install/setup.bash reliably even if script is run from another cwd
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "$WORKSPACE_ROOT/install/setup.bash"
else
    echo "Warning: workspace install/setup.bash not found at $WORKSPACE_ROOT/install/setup.bash"
    echo "Falling back to system ROS 2 setup (/opt/ros/humble/setup.bash)"
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi

echo ""
echo "Configuration:"
echo "  - Camera: 0"
echo "  - Camera resolution: 1920x1536"
echo "  - Downsample factor: 2x (960x768 for inference)"
echo "  - Depth estimation: 518x518 (TensorRT model input)"

# Initialize parameters
CALIB_PARAMS=""
UNDISTORT_PARAMS=""

# Check if using calibration file
if [ -n "${CAMERA_INFO_FILE}" ] && [ -f "${CAMERA_INFO_FILE}" ]; then
    echo "  - Camera parameters: FROM FILE"
    echo "    File: ${CAMERA_INFO_FILE}"
    CALIB_PARAMS="camera_info_file:=${CAMERA_INFO_FILE}"
elif [ "${USE_CALIBRATION}" = "1" ]; then
    echo "  - Camera parameters: CALIBRATED (FISHEYE)"
    echo "    fx=824.15, fy=823.66, cx=958.28, cy=767.39"
    echo "    Fisheye distortion: D=[1.49, -13.39, 21.41, 3.82]"
    CALIB_PARAMS="use_calibration:=true fx:=824.147361 fy:=823.660879 cx:=958.275200 cy:=767.389372 k1:=1.486308 k2:=-13.386609 p1:=21.409334 p2:=3.817858 k3:=0.0"
else
    echo "  - Camera parameters: ESTIMATED (60° FOV)"
    echo "    Tip: Set USE_CALIBRATION=1 to use calibrated fisheye parameters"
    echo "    Or: Set CAMERA_INFO_FILE=/path/to/camera_info.yaml to load from file"
    CALIB_PARAMS=""
fi

# Check if undistortion is enabled
if [ "${ENABLE_UNDISTORTION}" = "1" ]; then
    # Default balance to 0.0 if not set
    BALANCE="${UNDISTORTION_BALANCE:-0.0}"
    echo "  - Fisheye undistortion: ENABLED"
    echo "    Balance: ${BALANCE} (0.0=crop, 1.0=full FOV)"
    UNDISTORT_PARAMS="enable_undistortion:=true undistortion_balance:=${BALANCE}"
    
    # Warn if no calibration
    if [ -z "${CALIB_PARAMS}" ]; then
        echo ""
        echo "  ⚠️  WARNING: Undistortion requires calibration parameters!"
        echo "     Please set USE_CALIBRATION=1 or CAMERA_INFO_FILE"
        echo ""
    fi
else
    echo "  - Fisheye undistortion: DISABLED"
    echo "    Tip: Set ENABLE_UNDISTORTION=1 to enable fisheye undistortion"
    UNDISTORT_PARAMS=""
fi

echo "  - RViz: enabled"
echo ""
echo "Expected: 2-4x faster inference than full resolution"
echo ""
echo "Starting..."
echo "=========================================="
echo ""

# Run single camera with RViz and downsampling
ros2 launch depth_anything_v3 camera_depth_rviz.launch.py \
    camera_type:=standard \
    camera_id:=0 \
    camera_width:=640 \
    camera_height:=480 \
    model_path:=src/ros2-depth-anything-v3-trt/onnx/DA3-SMALL/DA3-SMALL.fp16.engine \
    publish_rate:=10.0 \
    downsample_factor:=2 \
    ${CALIB_PARAMS} \
    ${UNDISTORT_PARAMS}
