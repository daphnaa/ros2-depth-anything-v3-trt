// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DEPTH_ANYTHING_V3__FISHEYE_UNDISTORTER_HPP_
#define DEPTH_ANYTHING_V3__FISHEYE_UNDISTORTER_HPP_

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <memory>
#include <string>

namespace depth_anything_v3
{

/**
 * @class FisheyeUndistorter
 * @brief Utility class for camera lens undistortion (畸变矫正)
 * 
 * This class encapsulates all undistortion logic, including:
 * - Pre-computing undistortion maps during initialization
 * - Computing new camera intrinsic matrix for undistorted images
 * - Efficient image undistortion using pre-computed maps
 * 
 * Supports two distortion models:
 * - "fisheye": OpenCV fisheye model (equidistant, 4 coefficients)
 * - "plumb_bob" or "rational_polynomial": Standard pinhole model (Brown-Conrady, 5 coefficients)
 * 
 * The undistortion process transforms distorted images into rectilinear images
 * where straight lines in the 3D world appear straight.
 */
class FisheyeUndistorter
{
public:
  /**
   * @struct Config
   * @brief Configuration for undistortion
   */
  struct Config
  {
    bool enabled = false;           ///< Enable/disable undistortion
    double balance = 0.0;           ///< Balance between cropping (0.0) and full FOV (1.0)
    cv::Size image_size;            ///< Original image size
    cv::Mat camera_matrix;          ///< 3x3 intrinsic matrix K [fx, 0, cx; 0, fy, cy; 0, 0, 1]
    cv::Mat distortion_coeffs;      ///< Distortion coefficients (4 for fisheye, 5 for plumb_bob)
    std::string distortion_model = "fisheye";  ///< "fisheye" or "plumb_bob"
  };

  /**
   * @brief Default constructor
   */
  FisheyeUndistorter() = default;

  /**
   * @brief Destructor
   */
  ~FisheyeUndistorter() = default;

  /**
   * @brief Initialize the undistorter with configuration
   * 
   * This method pre-computes the undistortion maps using OpenCV's fisheye model.
   * The maps are computed once and reused for each frame.
   * 
   * @param config Configuration containing calibration parameters
   * @return true if initialization successful, false otherwise
   */
  bool initialize(const Config& config);

  /**
   * @brief Apply undistortion to an image
   * 
   * Uses pre-computed maps for efficient undistortion.
   * 
   * @param[in] input Input distorted image
   * @param[out] output Output undistorted image
   * @return true if undistortion successful, false otherwise
   */
  bool undistort(const cv::Mat& input, cv::Mat& output);

  /**
   * @brief Get the new camera matrix for undistorted images
   * 
   * The new camera matrix has different intrinsics than the original
   * because the undistortion process changes the effective focal length
   * and principal point.
   * 
   * @return 3x3 camera matrix for undistorted images
   */
  cv::Mat getNewCameraMatrix() const;

  /**
   * @brief Get the size of undistorted images
   * @return Size of undistorted images
   */
  cv::Size getUndistortedSize() const;

  /**
   * @brief Get the new focal length (fx) for undistorted images
   * @return Focal length in x direction
   */
  double getNewFx() const;

  /**
   * @brief Get the new focal length (fy) for undistorted images
   * @return Focal length in y direction
   */
  double getNewFy() const;

  /**
   * @brief Get the new principal point x coordinate
   * @return Principal point x coordinate
   */
  double getNewCx() const;

  /**
   * @brief Get the new principal point y coordinate
   * @return Principal point y coordinate
   */
  double getNewCy() const;

  /**
   * @brief Check if undistortion is enabled and ready
   * @return true if undistortion is ready to use
   */
  bool isReady() const;

  /**
   * @brief Check if undistortion is enabled in configuration
   * @return true if undistortion is enabled
   */
  bool isEnabled() const;

  /**
   * @brief Update undistortion maps for a new image size
   * 
   * Call this method if the input image resolution changes.
   * 
   * @param new_size New image size
   * @return true if maps updated successfully
   */
  bool updateImageSize(const cv::Size& new_size);

  /**
   * @brief Validate calibration parameters
   * 
   * Checks that calibration parameters are valid:
   * - Positive focal lengths
   * - Principal point within image bounds
   * - Finite distortion coefficients
   * 
   * @param config Configuration to validate
   * @return true if parameters are valid
   */
  static bool validateConfig(const Config& config);

private:
  /**
   * @brief Compute undistortion maps
   * @return true if maps computed successfully
   */
  bool computeMaps();

  Config config_;                   ///< Current configuration
  cv::Mat map1_;                    ///< Undistortion map 1 (x coordinates)
  cv::Mat map2_;                    ///< Undistortion map 2 (interpolation coefficients)
  cv::Mat new_camera_matrix_;       ///< Camera matrix for undistorted images
  cv::Size undistorted_size_;       ///< Size of undistorted images
  bool initialized_ = false;        ///< Initialization status
};

}  // namespace depth_anything_v3

#endif  // DEPTH_ANYTHING_V3__FISHEYE_UNDISTORTER_HPP_
