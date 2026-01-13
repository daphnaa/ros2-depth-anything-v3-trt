# Implementation Plan: Fisheye Undistortion (鱼眼畸变矫正)

## Overview

This implementation plan breaks down the fisheye undistortion feature into discrete coding tasks. The approach follows an incremental development strategy: first creating the undistortion utility class, then integrating it into the camera depth node, and finally adding configuration and testing.

## Tasks

- [x] 1. Create FisheyeUndistorter utility class
  - [x] 1.1 Create fisheye_undistorter.hpp header file
    - Define FisheyeUndistorter class with Config struct
    - Declare initialize(), undistort(), getNewCameraMatrix(), getUndistortedSize(), isReady() methods
    - Include necessary OpenCV headers
    - _Requirements: 1.1, 1.2, 1.5_

  - [x] 1.2 Implement fisheye_undistorter.cpp
    - Implement initialize() using cv::fisheye::estimateNewCameraMatrixForUndistortRectify and cv::fisheye::initUndistortRectifyMap
    - Implement undistort() using cv::remap with INTER_LINEAR interpolation
    - Implement getNewCameraMatrix() and getUndistortedSize() accessors
    - Add validation for calibration parameters
    - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5, 7.4_

  - [x] 1.3 Write property test for undistortion map generation
    - **Property 1: Undistortion Map Generation**
    - **Validates: Requirements 1.1**

  - [x] 1.4 Write property test for new camera matrix validity
    - **Property 2: New Camera Matrix Validity**
    - **Validates: Requirements 1.5**

- [x] 2. Integrate undistortion into camera_depth_node
  - [x] 2.1 Add undistortion parameters to camera_depth_node.cpp
    - Add enable_undistortion parameter (default: false)
    - Add undistortion_balance parameter (default: 0.0)
    - _Requirements: 2.1, 2.4_

  - [x] 2.2 Initialize FisheyeUndistorter in camera_depth_node
    - Create FisheyeUndistorter instance when undistortion is enabled
    - Pass calibration parameters from YAML or ROS parameters
    - Handle initialization failures gracefully
    - _Requirements: 1.1, 7.1, 7.2_

  - [x] 2.3 Apply undistortion in frame processing loop
    - Call undistort() on captured frame when enabled
    - Use undistorted frame for depth estimation
    - Update camera_info with new camera matrix and zero distortion
    - _Requirements: 1.2, 3.1, 3.2, 3.3, 3.4, 4.1_

  - [x] 2.4 Write property test for undistortion toggle behavior
    - **Property 3: Undistortion Toggle Behavior**
    - **Validates: Requirements 2.2, 2.3**

- [x] 3. Update point cloud generation for undistorted images
  - [x] 3.1 Modify createPointCloudWithCameraInfo to use new camera matrix
    - Use undistorted camera intrinsics for 3D projection
    - Ensure RGB colors come from undistorted image
    - _Requirements: 4.2, 5.1, 5.2, 5.3_

  - [x] 3.2 Write property test for point cloud projection consistency
    - **Property 6: Point Cloud Projection Consistency**
    - **Validates: Requirements 4.2, 5.3**

- [x] 4. Checkpoint - Verify basic undistortion works
  - Test with fisheye camera and calibration file
  - Verify undistorted image appears in RViz
  - Verify point cloud is generated correctly
  - Ask the user if questions arise

- [x] 5. Add CameraInfo publishing for undistorted images
  - [x] 5.1 Create undistorted CameraInfo message
    - Set distortion coefficients to zero
    - Set intrinsics from new camera matrix
    - Preserve frame_id and timestamp
    - _Requirements: 3.2, 3.3, 3.4_

  - [x] 5.2 Write property test for CameraInfo correctness
    - **Property 5: CameraInfo Correctness After Undistortion**
    - **Validates: Requirements 3.3, 3.4**

- [x] 6. Add error handling and validation
  - [x] 6.1 Implement calibration parameter validation
    - Check for positive focal lengths
    - Check principal point within image bounds
    - Validate distortion coefficients are finite and reasonable
    - _Requirements: 7.1, 7.4_

  - [x] 6.2 Add graceful fallback when undistortion fails
    - Log warning and disable undistortion if initialization fails
    - Continue processing with original image
    - _Requirements: 7.1, 7.2_

  - [x] 6.3 Write property test for missing calibration handling
    - **Property 10: Missing Calibration Handling**
    - **Validates: Requirements 7.1**

- [x] 7. Update launch files and configuration
  - [x] 7.1 Update camera_depth_rviz.launch.py
    - Add enable_undistortion parameter
    - Add undistortion_balance parameter
    - _Requirements: 2.1, 2.4_

  - [x] 7.2 Update run_camera_depth.sh script
    - Add ENABLE_UNDISTORTION environment variable support
    - Add UNDISTORTION_BALANCE environment variable support
    - Update help text and configuration display
    - _Requirements: 2.1, 2.4_

  - [x] 7.3 Update camera_info_example.yaml
    - Add distortion_model field to clarify fisheye model
    - Add comments explaining undistortion usage
    - _Requirements: 1.1_

- [ ] 8. Checkpoint - Full integration test
  - Test with CAMERA_INFO_FILE and ENABLE_UNDISTORTION=1
  - Verify undistorted image in RViz shows straight lines
  - Verify point cloud geometry is correct
  - Verify depth estimation quality
  - Ask the user if questions arise

- [x] 9. Add CMakeLists.txt updates
  - [x] 9.1 Add fisheye_undistorter source files to build
    - Add header to include directory
    - Add source to library or executable
    - _Requirements: 1.1_

- [x] 10. Performance optimization
  - [x] 10.1 Add timing logging for undistortion
    - Log undistortion time when debug logging enabled
    - Verify < 5ms latency at 1920x1536
    - _Requirements: 6.3, 6.4_

  - [x] 10.2 Handle resolution changes
    - Detect if input resolution changes
    - Recompute undistortion maps when needed
    - _Requirements: 6.2_

  - [x] 10.3 Write property test for resolution change handling
    - **Property 9: Resolution Change Handling**
    - **Validates: Requirements 6.2**

- [x] 11. Documentation
  - [x] 11.1 Update CAMERA_CALIBRATION_FILE.md
    - Add section on undistortion usage
    - Document enable_undistortion and undistortion_balance parameters
    - Add examples for running with undistortion
    - _Requirements: 2.1, 2.4, 2.5, 2.6_

- [ ] 12. Final checkpoint - Complete validation
  - Run full system with undistortion enabled
  - Verify all ROS topics publish correctly
  - Verify RViz displays undistorted image and point cloud
  - Ensure all tests pass
  - Ask the user if questions arise

## Notes

- All tasks including property-based tests are required
- The implementation uses OpenCV's fisheye camera model (cv::fisheye namespace)
- Undistortion maps are pre-computed during initialization for performance
- The balance parameter controls the trade-off between cropping and black borders
- All existing functionality should continue to work when undistortion is disabled

