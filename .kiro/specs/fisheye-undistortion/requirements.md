# Requirements Document

## Introduction

This document specifies the requirements for adding fisheye lens undistortion (畸变矫正) capability to the ROS2 Depth Anything V3 system. The current system supports fisheye camera calibration parameters but does not perform image undistortion before depth estimation. This feature will enable the system to:

1. Undistort fisheye camera images using calibration parameters
2. Display undistorted images in RViz
3. Perform depth estimation on undistorted images
4. Generate accurate point clouds using undistorted images and corrected camera intrinsics

## Glossary

- **Fisheye_Camera**: A camera with a wide-angle lens that produces significant barrel distortion
- **Undistortion**: The process of removing lens distortion from an image to produce a rectilinear (straight-line preserving) image
- **Camera_Intrinsics**: Internal camera parameters including focal length (fx, fy) and principal point (cx, cy)
- **Distortion_Coefficients**: Parameters describing lens distortion (k1, k2, p1, p2, k3 for fisheye model)
- **Camera_Node**: The ROS2 node responsible for capturing camera frames and performing depth estimation
- **Undistortion_Map**: Pre-computed lookup tables for efficient image undistortion
- **New_Camera_Matrix**: The camera intrinsic matrix for the undistorted image

## Requirements

### Requirement 1: Fisheye Image Undistortion

**User Story:** As a robotics developer, I want to undistort fisheye camera images before processing, so that depth estimation and point cloud generation are more accurate.

#### Acceptance Criteria

1. WHEN the Camera_Node starts with fisheye calibration parameters and undistortion enabled, THE System SHALL compute undistortion maps using OpenCV's fisheye model
2. WHEN processing each frame, THE System SHALL apply the undistortion maps to produce a rectified image
3. WHEN undistorting images, THE System SHALL use cv::fisheye::initUndistortRectifyMap for map generation
4. WHEN undistorting images, THE System SHALL use cv::remap with INTER_LINEAR interpolation for efficient processing
5. THE System SHALL compute a new camera matrix for the undistorted image that preserves the field of view

### Requirement 2: Undistortion Configuration

**User Story:** As a system operator, I want to enable or disable undistortion via configuration, so that I can compare results with and without undistortion.

#### Acceptance Criteria

1. THE Camera_Node SHALL accept a parameter "enable_undistortion" with default value false
2. WHEN enable_undistortion is true and calibration parameters are available, THE System SHALL perform undistortion
3. WHEN enable_undistortion is false, THE System SHALL process the original distorted image
4. THE Camera_Node SHALL accept a parameter "undistortion_balance" (0.0 to 1.0) to control field of view preservation
5. WHEN undistortion_balance is 0.0, THE System SHALL crop to show only valid pixels (no black borders)
6. WHEN undistortion_balance is 1.0, THE System SHALL preserve the full field of view (may have black borders)

### Requirement 3: Undistorted Image Publishing

**User Story:** As a robotics developer, I want to see the undistorted image in RViz, so that I can verify the undistortion quality.

#### Acceptance Criteria

1. WHEN undistortion is enabled, THE Camera_Node SHALL publish the undistorted image on the "~/input/image" topic
2. WHEN undistortion is enabled, THE Camera_Node SHALL publish an updated CameraInfo message with the new camera matrix
3. THE published CameraInfo SHALL have distortion coefficients set to zero (since the image is already undistorted)
4. THE published CameraInfo SHALL contain the correct new camera intrinsics for the undistorted image

### Requirement 4: Depth Estimation with Undistorted Images

**User Story:** As a robotics developer, I want depth estimation to use undistorted images, so that depth values are more accurate.

#### Acceptance Criteria

1. WHEN undistortion is enabled, THE System SHALL perform depth estimation on the undistorted image
2. WHEN undistortion is enabled, THE System SHALL use the new camera matrix for depth-to-3D projection
3. THE depth estimation output SHALL have the same resolution as the undistorted image

### Requirement 5: Point Cloud Generation with Undistorted Data

**User Story:** As a robotics developer, I want point clouds generated from undistorted images, so that 3D reconstruction is geometrically accurate.

#### Acceptance Criteria

1. WHEN undistortion is enabled, THE System SHALL generate point clouds using the undistorted image and new camera matrix
2. WHEN generating point clouds, THE System SHALL use the undistorted RGB image for coloring
3. THE point cloud projection SHALL use the new camera intrinsics (fx, fy, cx, cy) from the undistorted camera matrix
4. THE generated point cloud SHALL have straight lines appear straight (no barrel distortion artifacts)

### Requirement 6: Performance Optimization

**User Story:** As a robotics developer, I want undistortion to be efficient, so that real-time performance is maintained.

#### Acceptance Criteria

1. THE System SHALL pre-compute undistortion maps during initialization (not per-frame)
2. WHEN the camera resolution changes, THE System SHALL recompute the undistortion maps
3. THE undistortion process SHALL add no more than 5ms latency per frame at 1920x1536 resolution
4. THE System SHALL log timing information for undistortion when debug logging is enabled

### Requirement 7: Error Handling

**User Story:** As a system operator, I want clear error messages when undistortion fails, so that I can diagnose configuration issues.

#### Acceptance Criteria

1. IF undistortion is enabled but calibration parameters are not available, THEN THE System SHALL log a warning and disable undistortion
2. IF undistortion map computation fails, THEN THE System SHALL log an error with diagnostic information
3. IF undistortion is enabled, THE System SHALL log the undistortion configuration at startup
4. THE System SHALL validate that distortion coefficients are within reasonable ranges

