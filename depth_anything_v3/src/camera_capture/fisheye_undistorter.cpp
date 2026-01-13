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

#include "depth_anything_v3/fisheye_undistorter.hpp"
#include <cmath>
#include <iostream>

namespace depth_anything_v3
{

bool FisheyeUndistorter::initialize(const Config& config)
{
  // Validate configuration
  if (!validateConfig(config)) {
    std::cerr << "[FisheyeUndistorter] Invalid configuration, initialization failed" << std::endl;
    initialized_ = false;
    return false;
  }

  config_ = config;

  if (!config_.enabled) {
    std::cout << "[FisheyeUndistorter] Undistortion disabled" << std::endl;
    initialized_ = false;
    return true;  // Not an error, just disabled
  }

  // Compute undistortion maps
  if (!computeMaps()) {
    std::cerr << "[FisheyeUndistorter] Failed to compute undistortion maps" << std::endl;
    initialized_ = false;
    return false;
  }

  initialized_ = true;
  std::cout << "[FisheyeUndistorter] Initialized successfully" << std::endl;
  std::cout << "[FisheyeUndistorter]   Original size: " << config_.image_size.width 
            << "x" << config_.image_size.height << std::endl;
  std::cout << "[FisheyeUndistorter]   Undistorted size: " << undistorted_size_.width 
            << "x" << undistorted_size_.height << std::endl;
  std::cout << "[FisheyeUndistorter]   Balance: " << config_.balance << std::endl;
  std::cout << "[FisheyeUndistorter]   New camera matrix:" << std::endl;
  std::cout << "[FisheyeUndistorter]     fx=" << getNewFx() << ", fy=" << getNewFy() << std::endl;
  std::cout << "[FisheyeUndistorter]     cx=" << getNewCx() << ", cy=" << getNewCy() << std::endl;

  return true;
}

bool FisheyeUndistorter::computeMaps()
{
  try {
    // Ensure camera matrix is CV_64F
    cv::Mat K;
    config_.camera_matrix.convertTo(K, CV_64F);

    // Ensure distortion coefficients are CV_64F
    cv::Mat D;
    config_.distortion_coeffs.convertTo(D, CV_64F);
    if (D.cols > D.rows) {
      D = D.t();  // Transpose to column vector
    }

    // Identity rectification matrix (for monocular camera)
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);

    // Keep same size for undistorted image
    undistorted_size_ = config_.image_size;

    // Check which distortion model to use
    bool use_fisheye = (config_.distortion_model == "fisheye");
    
    std::cout << "[FisheyeUndistorter] Using distortion model: " << config_.distortion_model << std::endl;

    if (use_fisheye) {
      // OpenCV Fisheye model (equidistant projection)
      // Ensure we have exactly 4 coefficients for fisheye model
      if (D.rows < 4) {
        cv::Mat D_padded = cv::Mat::zeros(4, 1, CV_64F);
        D.copyTo(D_padded(cv::Rect(0, 0, 1, D.rows)));
        D = D_padded;
      } else if (D.rows > 4) {
        D = D(cv::Rect(0, 0, 1, 4)).clone();
      }

      // For fisheye, use the original camera matrix as the new camera matrix
      // This preserves the field of view better
      new_camera_matrix_ = K.clone();
      
      // Optionally scale based on balance
      // balance=0: crop to valid pixels (smaller FOV, no black borders)
      // balance=1: keep full FOV (may have black borders)
      if (config_.balance < 1.0) {
        // Scale focal length to crop more
        double scale = 1.0 + (1.0 - config_.balance) * 0.5;
        new_camera_matrix_.at<double>(0, 0) *= scale;  // fx
        new_camera_matrix_.at<double>(1, 1) *= scale;  // fy
      }

      // Compute undistortion maps using fisheye model
      cv::fisheye::initUndistortRectifyMap(
        K, D, R, new_camera_matrix_,
        undistorted_size_,
        CV_16SC2,  // Map type for efficient remap
        map1_, map2_
      );
    } else {
      // Standard pinhole model (Brown-Conrady / plumb_bob)
      // This model uses k1, k2, p1, p2, k3 coefficients
      
      // Get optimal new camera matrix
      // alpha=0 means crop to valid pixels only, alpha=1 means keep all pixels
      new_camera_matrix_ = cv::getOptimalNewCameraMatrix(
        K, D, config_.image_size, 
        config_.balance,  // alpha: 0=crop, 1=full
        undistorted_size_,
        nullptr,  // validPixROI
        false     // centerPrincipalPoint
      );

      // Compute undistortion maps using standard pinhole model
      cv::initUndistortRectifyMap(
        K, D, R, new_camera_matrix_,
        undistorted_size_,
        CV_16SC2,  // Map type for efficient remap
        map1_, map2_
      );
    }

    // Validate new camera matrix has finite values
    double new_fx = new_camera_matrix_.at<double>(0, 0);
    double new_fy = new_camera_matrix_.at<double>(1, 1);
    double new_cx = new_camera_matrix_.at<double>(0, 2);
    double new_cy = new_camera_matrix_.at<double>(1, 2);
    
    if (!std::isfinite(new_fx) || !std::isfinite(new_fy) ||
        !std::isfinite(new_cx) || !std::isfinite(new_cy)) {
      std::cerr << "[FisheyeUndistorter] New camera matrix has non-finite values" << std::endl;
      std::cerr << "[FisheyeUndistorter]   fx=" << new_fx << ", fy=" << new_fy 
                << ", cx=" << new_cx << ", cy=" << new_cy << std::endl;
      return false;
    }
    
    // Validate focal lengths are positive and reasonable
    if (new_fx <= 0 || new_fy <= 0) {
      std::cerr << "[FisheyeUndistorter] New camera matrix has non-positive focal lengths" << std::endl;
      return false;
    }
    
    // Validate principal point is within reasonable bounds (allow some margin for balance adjustments)
    // For fisheye lenses with balance=0, the principal point can be significantly shifted
    double margin = std::max(undistorted_size_.width, undistorted_size_.height) * 2.0;
    if (new_cx < -margin || new_cx >= undistorted_size_.width + margin ||
        new_cy < -margin || new_cy >= undistorted_size_.height + margin) {
      std::cerr << "[FisheyeUndistorter] New principal point too far outside image bounds: cx=" 
                << new_cx << ", cy=" << new_cy << std::endl;
      std::cerr << "[FisheyeUndistorter] Try increasing undistortion_balance (e.g., 0.5 or 1.0)" << std::endl;
      return false;
    }
    
    // Warn if principal point is outside image but within margin
    if (new_cx < 0 || new_cx >= undistorted_size_.width ||
        new_cy < 0 || new_cy >= undistorted_size_.height) {
      std::cout << "[FisheyeUndistorter] Warning: Principal point outside image: cx=" 
                << new_cx << ", cy=" << new_cy << std::endl;
      std::cout << "[FisheyeUndistorter] This may cause significant black borders or cropping" << std::endl;
      std::cout << "[FisheyeUndistorter] Consider increasing undistortion_balance parameter" << std::endl;
    }

    // Verify maps are valid
    if (map1_.empty() || map2_.empty()) {
      std::cerr << "[FisheyeUndistorter] Generated maps are empty" << std::endl;
      return false;
    }

    return true;
  } catch (const cv::Exception& e) {
    std::cerr << "[FisheyeUndistorter] OpenCV exception: " << e.what() << std::endl;
    return false;
  } catch (const std::exception& e) {
    std::cerr << "[FisheyeUndistorter] Exception: " << e.what() << std::endl;
    return false;
  }
}

bool FisheyeUndistorter::undistort(const cv::Mat& input, cv::Mat& output)
{
  if (!initialized_ || !config_.enabled) {
    // If not initialized or disabled, just copy input to output
    output = input.clone();
    return true;
  }

  if (input.empty()) {
    std::cerr << "[FisheyeUndistorter] Input image is empty" << std::endl;
    return false;
  }

  // Check if input size matches expected size
  if (input.size() != config_.image_size) {
    std::cerr << "[FisheyeUndistorter] Input size mismatch: expected " 
              << config_.image_size.width << "x" << config_.image_size.height
              << ", got " << input.cols << "x" << input.rows << std::endl;
    // Try to update maps for new size
    if (!updateImageSize(input.size())) {
      output = input.clone();
      return false;
    }
  }

  try {
    // Apply undistortion using pre-computed maps
    cv::remap(
      input, output,
      map1_, map2_,
      cv::INTER_LINEAR,      // Interpolation method
      cv::BORDER_CONSTANT,   // Border handling
      cv::Scalar(0, 0, 0)    // Border color (black)
    );
    return true;
  } catch (const cv::Exception& e) {
    std::cerr << "[FisheyeUndistorter] Remap failed: " << e.what() << std::endl;
    output = input.clone();
    return false;
  }
}

cv::Mat FisheyeUndistorter::getNewCameraMatrix() const
{
  return new_camera_matrix_.clone();
}

cv::Size FisheyeUndistorter::getUndistortedSize() const
{
  return undistorted_size_;
}

double FisheyeUndistorter::getNewFx() const
{
  if (new_camera_matrix_.empty()) {
    return 0.0;
  }
  return new_camera_matrix_.at<double>(0, 0);
}

double FisheyeUndistorter::getNewFy() const
{
  if (new_camera_matrix_.empty()) {
    return 0.0;
  }
  return new_camera_matrix_.at<double>(1, 1);
}

double FisheyeUndistorter::getNewCx() const
{
  if (new_camera_matrix_.empty()) {
    return 0.0;
  }
  return new_camera_matrix_.at<double>(0, 2);
}

double FisheyeUndistorter::getNewCy() const
{
  if (new_camera_matrix_.empty()) {
    return 0.0;
  }
  return new_camera_matrix_.at<double>(1, 2);
}

bool FisheyeUndistorter::isReady() const
{
  return initialized_ && config_.enabled && !map1_.empty() && !map2_.empty();
}

bool FisheyeUndistorter::isEnabled() const
{
  return config_.enabled;
}

bool FisheyeUndistorter::updateImageSize(const cv::Size& new_size)
{
  if (new_size == config_.image_size) {
    return true;  // No change needed
  }

  std::cout << "[FisheyeUndistorter] Updating image size from " 
            << config_.image_size.width << "x" << config_.image_size.height
            << " to " << new_size.width << "x" << new_size.height << std::endl;

  // Scale camera matrix for new size
  double scale_x = static_cast<double>(new_size.width) / config_.image_size.width;
  double scale_y = static_cast<double>(new_size.height) / config_.image_size.height;

  cv::Mat new_K = config_.camera_matrix.clone();
  new_K.at<double>(0, 0) *= scale_x;  // fx
  new_K.at<double>(0, 2) *= scale_x;  // cx
  new_K.at<double>(1, 1) *= scale_y;  // fy
  new_K.at<double>(1, 2) *= scale_y;  // cy

  config_.camera_matrix = new_K;
  config_.image_size = new_size;

  return computeMaps();
}

bool FisheyeUndistorter::validateConfig(const Config& config)
{
  // Check if enabled - if not, no need to validate other params
  if (!config.enabled) {
    return true;
  }

  // Check image size
  if (config.image_size.width <= 0 || config.image_size.height <= 0) {
    std::cerr << "[FisheyeUndistorter] Invalid image size: " 
              << config.image_size.width << "x" << config.image_size.height << std::endl;
    return false;
  }

  // Check camera matrix
  if (config.camera_matrix.empty() || 
      config.camera_matrix.rows != 3 || 
      config.camera_matrix.cols != 3) {
    std::cerr << "[FisheyeUndistorter] Invalid camera matrix size" << std::endl;
    return false;
  }

  // Get camera matrix values
  cv::Mat K;
  config.camera_matrix.convertTo(K, CV_64F);
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  // Check focal lengths are positive
  if (fx <= 0 || fy <= 0) {
    std::cerr << "[FisheyeUndistorter] Invalid focal lengths: fx=" << fx << ", fy=" << fy << std::endl;
    return false;
  }

  // Check principal point is within image bounds
  if (cx < 0 || cx >= config.image_size.width ||
      cy < 0 || cy >= config.image_size.height) {
    std::cerr << "[FisheyeUndistorter] Principal point outside image bounds: cx=" 
              << cx << ", cy=" << cy << std::endl;
    return false;
  }

  // Check distortion coefficients
  if (config.distortion_coeffs.empty()) {
    std::cerr << "[FisheyeUndistorter] Distortion coefficients are empty" << std::endl;
    return false;
  }

  // Check distortion coefficients are finite and reasonable
  cv::Mat D;
  config.distortion_coeffs.convertTo(D, CV_64F);
  for (int i = 0; i < D.total(); ++i) {
    double val = D.at<double>(i);
    if (!std::isfinite(val)) {
      std::cerr << "[FisheyeUndistorter] Distortion coefficient " << i << " is not finite" << std::endl;
      return false;
    }
    // Check for extreme values that could cause numerical issues
    if (std::abs(val) > 1000.0) {
      std::cerr << "[FisheyeUndistorter] Warning: Distortion coefficient " << i 
                << " has extreme value: " << val << std::endl;
      // Don't fail, just warn
    }
  }

  // Check balance is in valid range
  if (config.balance < 0.0 || config.balance > 1.0) {
    std::cerr << "[FisheyeUndistorter] Balance must be in [0.0, 1.0], got: " 
              << config.balance << std::endl;
    return false;
  }

  return true;
}

}  // namespace depth_anything_v3
