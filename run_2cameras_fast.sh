#!/bin/bash
# Two-camera depth estimation system with point cloud fusion
# Usage: USE_CALIBRATION=1 ./run_2cameras_fast.sh

echo "=========================================="
echo "Two-Camera Depth Estimation System"
echo "=========================================="
echo ""

# Kill existing processes
echo "Cleaning up existing processes..."
pkill -9 -f camera_depth_node 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
sleep 2

# Source ROS environment
source install/setup.bash

echo ""
echo "Configuration:"
echo "  - Cameras: /dev/video0, /dev/video1"
echo "  - Resolution: 1920x1536"
echo "  - Downsample: 2x (960x768 for inference)"
echo "  - Model input: 518x518"
echo "  - Point cloud fusion: enabled"

# Check calibration mode
if [ "${USE_CALIBRATION}" = "1" ]; then
    echo "  - Calibration: FISHEYE (fx=824.15, fy=823.66)"
    CALIB_PARAMS="use_calibration:=true fx:=824.147361 fy:=823.660879 cx:=958.275200 cy:=767.389372 k1:=1.486308 k2:=-13.386609 p1:=21.409334 p2:=3.817858 k3:=0.0"
else
    echo "  - Calibration: ESTIMATED (60° FOV)"
    echo "    Tip: Set USE_CALIBRATION=1 for fisheye calibration"
    CALIB_PARAMS=""
fi

echo ""
echo "GPU Memory: ~1.3 GB (stable on Jetson Orin)"
echo ""
echo "Starting..."
echo "=========================================="
echo ""

# Launch 2-camera system with RViz
ros2 launch depth_anything_v3 gmsl_multi_camera_fusion.launch.py \
    camera_type:=standard \
    model_path:=onnx/DA3METRIC-LARGE.onnx \
    downsample_factor:=2 \
    config_file:=depth_anything_v3/config/2camera_config.yaml \
    launch_rviz:=true \
    ${CALIB_PARAMS}
