#pragma once

#include <string>
#include <vector>
#include <limits>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace depth_anything_helpers {

using json = nlohmann::json;

// YAML intrinsics
bool loadIntrinsicsFromYaml(
    const std::string& path, int& width, int& height,
    double& fx, double& fy, double& cx, double& cy);

sensor_msgs::msg::CameraInfo makeCameraInfoFromIntrinsics(
    int width, int height, double fx, double fy, double cx, double cy,
    const std::string& frame_id);

// ROI helpers
cv::Rect clampRoi(int x1, int y1, int x2, int y2, int w, int h);

bool depthHistModeInRoi(
    const cv::Mat& depth32f, const cv::Rect& roi,
    double min_d, double max_d,
    int bins, double min_frac,
    double& out_z);

// Detection parsing (NanoOWL-ish)
// Returns array of objects: {label, score, bbox:[x1,y1,x2,y2]}
json parseDetectionsFromNanoowlLikeJson(const json& det_or_sidecar);

// Compute xyz from pixel + depth using CameraInfo K
json projectUvDepthToXyz(double u, double v, double z, const sensor_msgs::msg::CameraInfo& cam);

// Wall probes: assumes 3 bboxes in top third
json computeWallProbesTopThird(
    const cv::Mat& depth32f,
    const sensor_msgs::msg::CameraInfo& cam,
    double min_depth, double max_depth,
    int hist_bins, double hist_min_frac);


void saveDepthColormap(const cv::Mat& depth32f_m,
                       const std::string& out_path_jpg,
                       float vmin = -1.0f,
                       float vmax = -1.0f);

std::string dirnameOf(const std::string& p);
std::string addDepthSuffix(const std::string& dir);
void ensureDir(const std::string& d);
std::string stemOf(const std::string& p);

}  // namespace depth_anything_helpers
