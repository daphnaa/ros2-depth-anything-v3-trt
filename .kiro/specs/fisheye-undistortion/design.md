# Design Document: Fisheye Undistortion (鱼眼畸变矫正)

## Overview

This design adds fisheye lens undistortion capability to the existing ROS2 Depth Anything V3 camera depth node. The implementation will:

1. Pre-compute undistortion maps during initialization using OpenCV's fisheye camera model
2. Apply undistortion to each captured frame before depth estimation
3. Compute a new camera intrinsic matrix for the undistorted image
4. Publish undistorted images and updated CameraInfo to ROS topics
5. Use the undistorted image and new camera matrix for depth estimation and point cloud generation

The undistortion process transforms fisheye images with barrel distortion into rectilinear images where straight lines in the 3D world appear straight in the image.

## Architecture

### Processing Pipeline

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Camera Depth Node                                 │
│                                                                          │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐      │
│  │   Camera     │    │   Undistortion   │    │     Depth        │      │
│  │   Capture    │───▶│   (if enabled)   │───▶│   Estimation     │      │
│  │              │    │                  │    │                  │      │
│  └──────────────┘    └──────────────────┘    └──────────────────┘      │
│         │                    │                       │                  │
│         │                    │                       │                  │
│         ▼                    ▼                       ▼                  │
│  ┌──────────────┐    ┌──────────────────┐    ┌──────────────────┐      │
│  │  Raw Image   │    │ Undistorted Img  │    │   Depth Image    │      │
│  │  (internal)  │    │ + New CameraInfo │    │                  │      │
│  └──────────────┘    └──────────────────┘    └──────────────────┘      │
│                              │                       │                  │
│                              │                       │                  │
│                              ▼                       ▼                  │
│                      ┌──────────────────────────────────┐              │
│                      │      Point Cloud Generation      │              │
│                      │  (uses new camera matrix)        │              │
│                      └──────────────────────────────────┘              │
│                                      │                                  │
│                                      ▼                                  │
│                              ┌──────────────┐                          │
│                              │  ROS Topics  │                          │
│                              │  - image     │                          │
│                              │  - depth     │                          │
│                              │  - pointcloud│                          │
│                              │  - camera_info│                         │
│                              └──────────────┘                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### Undistortion Mathematics

The fisheye camera model in OpenCV uses the following projection:

```
θ = atan(r)
θ_d = θ(1 + k1*θ² + k2*θ⁴ + k3*θ⁶ + k4*θ⁸)

where:
- r = sqrt(x² + y²) is the distance from optical axis
- θ is the angle of incidence
- θ_d is the distorted angle
- k1, k2, k3, k4 are distortion coefficients
```

The undistortion process inverts this mapping to produce a rectilinear image.

## Components and Interfaces

### 1. FisheyeUndistorter Class

**Purpose**: Encapsulate all undistortion logic in a reusable class

**Interface**:
```cpp
class FisheyeUndistorter {
public:
    struct Config {
        bool enabled = false;
        double balance = 0.0;  // 0.0 = crop, 1.0 = full FOV
        cv::Size image_size;
        cv::Mat camera_matrix;      // 3x3 intrinsic matrix K
        cv::Mat distortion_coeffs;  // 4x1 fisheye distortion [k1, k2, k3, k4]
    };
    
    // Initialize with configuration
    bool initialize(const Config& config);
    
    // Apply undistortion to an image
    bool undistort(const cv::Mat& input, cv::Mat& output);
    
    // Get the new camera matrix for undistorted images
    cv::Mat getNewCameraMatrix() const;
    
    // Get undistorted image size
    cv::Size getUndistortedSize() const;
    
    // Check if undistortion is enabled and ready
    bool isReady() const;
    
    // Recompute maps for new image size
    bool updateImageSize(const cv::Size& new_size);
    
private:
    Config config_;
    cv::Mat map1_, map2_;           // Undistortion maps
    cv::Mat new_camera_matrix_;     // Camera matrix for undistorted image
    cv::Size undistorted_size_;
    bool initialized_ = false;
};
```

### 2. Enhanced Camera Depth Node

**New Parameters**:
```yaml
# Undistortion parameters
enable_undistortion: false      # Enable/disable fisheye undistortion
undistortion_balance: 0.0       # 0.0=crop to valid pixels, 1.0=full FOV with black borders
```

**Modified Initialization Flow**:
```
1. Read parameters (existing)
2. Load calibration (existing)
3. Initialize camera capture (existing)
4. IF enable_undistortion AND calibration available:
   a. Create FisheyeUndistorter with calibration params
   b. Initialize undistortion maps
   c. Get new camera matrix for undistorted images
   d. Update camera_info with new matrix and zero distortion
5. Initialize depth estimator (existing)
6. Start publishing loop (existing)
```

**Modified Frame Processing Flow**:
```
1. Capture frame from camera
2. IF undistortion enabled:
   a. Apply undistortion to frame
   b. Use undistorted frame for all subsequent processing
3. Downsample if requested (existing)
4. Run depth estimation (existing)
5. Generate point cloud using appropriate camera matrix
6. Publish all outputs
```

## Data Models

### Undistortion Configuration
```cpp
struct UndistortionConfig {
    bool enabled;
    double balance;           // 0.0 to 1.0
    int image_width;
    int image_height;
    
    // Original camera intrinsics
    double fx, fy, cx, cy;
    
    // Fisheye distortion coefficients
    double k1, k2, k3, k4;    // Note: OpenCV fisheye uses 4 coefficients
};
```

### Camera Matrix Structures
```cpp
// Original fisheye camera matrix (3x3)
cv::Mat K = (cv::Mat_<double>(3, 3) <<
    fx, 0,  cx,
    0,  fy, cy,
    0,  0,  1);

// Fisheye distortion coefficients (4x1)
cv::Mat D = (cv::Mat_<double>(4, 1) << k1, k2, k3, k4);

// New camera matrix for undistorted image (computed by OpenCV)
cv::Mat new_K;  // Will have different fx, fy, cx, cy
```

### Undistortion Map Types
```cpp
cv::Mat map1;  // CV_16SC2 - x,y coordinates as 16-bit signed integers
cv::Mat map2;  // CV_16UC1 - interpolation coefficients
```

## Correctness Properties

*A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.*

### Property 1: Undistortion Map Generation

*For any* valid fisheye calibration parameters (positive focal lengths, principal point within image bounds, finite distortion coefficients), the system shall generate non-empty undistortion maps (map1 and map2).

**Validates: Requirements 1.1**

### Property 2: New Camera Matrix Validity

*For any* valid original camera matrix and balance parameter in [0.0, 1.0], the computed new camera matrix shall have:
- Positive focal lengths (fx > 0, fy > 0)
- Principal point within the undistorted image bounds (0 ≤ cx < width, 0 ≤ cy < height)

**Validates: Requirements 1.5**

### Property 3: Undistortion Toggle Behavior

*For any* input image:
- When undistortion is enabled with valid calibration, the output image shall differ from the input (undistortion was applied)
- When undistortion is disabled, the output image shall be identical to the input

**Validates: Requirements 2.2, 2.3**

### Property 4: Balance Parameter Effect on Black Borders

*For any* fisheye image with significant distortion, when undistortion_balance is 0.0, the undistorted image shall have no black border pixels (all pixels contain valid image data).

**Validates: Requirements 2.5**

### Property 5: CameraInfo Correctness After Undistortion

*For any* undistorted output, the published CameraInfo shall:
- Have all distortion coefficients set to zero
- Have intrinsic parameters (fx, fy, cx, cy) matching the computed new camera matrix

**Validates: Requirements 3.3, 3.4**

### Property 6: Point Cloud Projection Consistency

*For any* 3D point P projected to pixel (u, v) using the new camera matrix, back-projecting (u, v) with depth z shall recover a point P' where ||P - P'|| < ε (small error tolerance).

**Validates: Requirements 4.2, 5.3**

### Property 7: Resolution Consistency

*For any* undistorted image, the depth estimation output shall have the same width and height as the undistorted image.

**Validates: Requirements 4.3**

### Property 8: Geometric Correctness - Straight Lines

*For any* set of 3D collinear points, when projected to the undistorted image and then to a point cloud, the resulting 3D points shall be collinear (within numerical tolerance).

**Validates: Requirements 5.4**

### Property 9: Resolution Change Handling

*For any* change in input image resolution, the system shall recompute undistortion maps to match the new resolution, and the new maps shall be valid (non-empty).

**Validates: Requirements 6.2**

### Property 10: Missing Calibration Handling

*For any* configuration where undistortion is enabled but calibration parameters are missing or invalid, the system shall disable undistortion and continue processing with the original image.

**Validates: Requirements 7.1**

### Property 11: Distortion Coefficient Validation

*For any* distortion coefficients, the system shall reject values that would cause numerical instability (e.g., |k1| > 100, |k2| > 1000) and log a warning.

**Validates: Requirements 7.4**

## Error Handling

### Initialization Errors

1. **Missing Calibration Parameters**
   - Check if fx, fy, cx, cy are valid (positive, finite)
   - Log warning: "Undistortion enabled but calibration not available, disabling"
   - Set enable_undistortion_ = false
   - Continue with normal processing

2. **Invalid Distortion Coefficients**
   - Check if k1, k2, k3, k4 are finite and within reasonable ranges
   - Log warning with specific invalid values
   - Disable undistortion

3. **Map Computation Failure**
   - Catch OpenCV exceptions from initUndistortRectifyMap
   - Log error with exception message
   - Disable undistortion

### Runtime Errors

1. **Undistortion Application Failure**
   - Catch exceptions from cv::remap
   - Log warning
   - Fall back to original image for this frame
   - Continue processing

2. **Resolution Mismatch**
   - Detect if input frame size differs from expected
   - Recompute undistortion maps
   - Log info about resolution change

## Testing Strategy

### Unit Tests

1. **FisheyeUndistorter Initialization**
   - Test with valid calibration parameters
   - Test with invalid parameters (negative focal length, etc.)
   - Test with missing parameters

2. **Undistortion Map Generation**
   - Verify maps are non-empty for valid inputs
   - Verify map dimensions match expected output size

3. **New Camera Matrix Computation**
   - Verify positive focal lengths
   - Verify principal point within bounds
   - Test with different balance values

4. **Image Undistortion**
   - Test with synthetic distorted images
   - Verify output dimensions
   - Verify no crashes with edge cases (empty image, etc.)

### Property-Based Tests

Each property test should run a minimum of 100 iterations with randomized inputs.

1. **Property 1: Undistortion Map Generation**
   - Generate random valid calibration parameters
   - Initialize undistorter
   - Verify maps are non-empty
   - **Tag**: Feature: fisheye-undistortion, Property 1: Undistortion Map Generation

2. **Property 2: New Camera Matrix Validity**
   - Generate random camera matrices and balance values
   - Compute new camera matrix
   - Verify validity constraints
   - **Tag**: Feature: fisheye-undistortion, Property 2: New Camera Matrix Validity

3. **Property 3: Undistortion Toggle Behavior**
   - Generate random images
   - Test with undistortion enabled/disabled
   - Verify output differs/matches appropriately
   - **Tag**: Feature: fisheye-undistortion, Property 3: Undistortion Toggle Behavior

4. **Property 5: CameraInfo Correctness**
   - Generate random calibration parameters
   - Compute undistorted CameraInfo
   - Verify zero distortion and correct intrinsics
   - **Tag**: Feature: fisheye-undistortion, Property 5: CameraInfo Correctness

5. **Property 6: Point Cloud Projection Consistency**
   - Generate random 3D points
   - Project to 2D, then back-project to 3D
   - Verify round-trip error is small
   - **Tag**: Feature: fisheye-undistortion, Property 6: Point Cloud Projection Consistency

6. **Property 10: Missing Calibration Handling**
   - Test with various missing/invalid calibration scenarios
   - Verify undistortion is disabled gracefully
   - **Tag**: Feature: fisheye-undistortion, Property 10: Missing Calibration Handling

### Integration Tests

1. **End-to-End Undistortion Pipeline**
   - Launch camera node with undistortion enabled
   - Verify undistorted image is published
   - Verify CameraInfo has zero distortion
   - Verify point cloud is generated correctly

2. **Visual Verification**
   - Use checkerboard pattern image
   - Verify straight lines appear straight after undistortion
   - Compare with OpenCV's standalone undistortion

## Implementation Notes

### OpenCV Fisheye Functions

```cpp
// Compute undistortion maps
cv::fisheye::initUndistortRectifyMap(
    K,                    // Original camera matrix
    D,                    // Distortion coefficients (4x1)
    cv::Mat::eye(3, 3, CV_64F),  // Rectification (identity for monocular)
    new_K,                // New camera matrix
    undistorted_size,     // Output image size
    CV_16SC2,             // Map type
    map1, map2            // Output maps
);

// Estimate new camera matrix
cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
    K, D, image_size,
    cv::Mat::eye(3, 3, CV_64F),
    new_K,
    balance,              // 0.0 = crop, 1.0 = full FOV
    undistorted_size,
    fov_scale             // Optional FOV scaling
);

// Apply undistortion
cv::remap(
    input, output,
    map1, map2,
    cv::INTER_LINEAR,     // Interpolation method
    cv::BORDER_CONSTANT,  // Border handling
    cv::Scalar(0, 0, 0)   // Border color (black)
);
```

### Distortion Coefficient Mapping

The current system uses 5 coefficients (k1, k2, p1, p2, k3) which is the standard OpenCV model. For fisheye undistortion, we need to map to the 4-coefficient fisheye model:

```cpp
// Standard model: k1, k2, p1, p2, k3 (radial + tangential)
// Fisheye model: k1, k2, k3, k4 (equidistant projection)

// For fisheye cameras, the calibration should use fisheye model directly
// The YAML file should specify distortion_model: "fisheye"
cv::Mat D_fisheye = (cv::Mat_<double>(4, 1) << k1, k2, p1, p2);
```

### Performance Considerations

1. **Map Pre-computation**: Maps are computed once during initialization, not per-frame
2. **Map Type**: Using CV_16SC2 for map1 provides good balance of speed and accuracy
3. **Interpolation**: INTER_LINEAR is faster than INTER_CUBIC with acceptable quality
4. **Memory**: Maps require approximately 2 * width * height * 2 bytes

### Thread Safety

The FisheyeUndistorter class should be thread-safe for the undistort() method since maps are read-only after initialization. The initialize() and updateImageSize() methods should not be called concurrently with undistort().

