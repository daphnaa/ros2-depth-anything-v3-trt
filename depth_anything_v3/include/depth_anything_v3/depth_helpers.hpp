#pragma once

#include <string>
#include <vector>
#include <limits>
#include <algorithm>   // std::max

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

// Forward-declare the class so this header doesn't need to include heavy TRT headers
namespace depth_anything_v3 {
class TensorRTDepthAnything;
}

namespace depth_anything_helpers {

using json = nlohmann::json;

// -------------------- Intrinsics / CameraInfo --------------------
bool loadIntrinsicsFromYaml(
    const std::string& path,
    int& width, int& height,
    double& fx, double& fy, double& cx, double& cy);

sensor_msgs::msg::CameraInfo makeCameraInfoFromIntrinsics(
    int width, int height,
    double fx, double fy, double cx, double cy,
    const std::string& frame_id);

// -------------------- ROI / Depth helpers --------------------
cv::Rect clampRoi(int x1, int y1, int x2, int y2, int w, int h);

bool depthHistModeInRoi(
    const cv::Mat& depth32f,
    const cv::Rect& roi,
    double min_d, double max_d,
    int bins, double min_frac,
    double& out_z);

// Detection parsing (NanoOWL-ish)
// Returns array of objects: {label, score, bbox:[x1,y1,x2,y2]}
json parseDetectionsFromNanoowlLikeJson(const json& det_or_sidecar);

// Compute xyz from pixel + depth using CameraInfo K
json projectUvDepthToXyz(
    double u, double v, double z,
    const sensor_msgs::msg::CameraInfo& cam);

// Save pretty depth image
void saveDepthColormap(
    const cv::Mat& depth32f_m,
    const std::string& out_path_jpg,
    float vmin = -1.0f,
    float vmax = -1.0f);

// Per-column max depth (old simple signature you had)
bool maxDepthInColumns(
    const cv::Mat& depth32f,
    int x1, int x2,
    double min_d, double max_d,
    double& out_max_depth);


// Path helpers
std::string dirnameOf(const std::string& p);
std::string addDepthSuffix(const std::string& dir);
void ensureDir(const std::string& d);
std::string stemOf(const std::string& p);

// -------------------- Tiling --------------------
struct TileCfg {
  int tile_w = 960;
  int tile_h = 768;
  int overlap_div = 10;  // overlap ~= tile / overlap_div (10 => ~10%)

  int overlap_x() const { return tile_w / std::max(1, overlap_div); }
  int overlap_y() const { return tile_h / std::max(1, overlap_div); }
  int step_x() const    { return std::max(1, tile_w - overlap_x()); }
  int step_y() const    { return std::max(1, tile_h - overlap_y()); }
};

// Return crop rects that cover full image with overlap
std::vector<cv::Rect> iterTiles(int full_w, int full_h, const TileCfg& cfg);

// Weight mask for blending overlaps (CV_32F, size h x w)
cv::Mat blendWeights(int h, int w, int border_px);

// CameraInfo adjusted to tile crop (cx/cy shifted by x0/y0)
sensor_msgs::msg::CameraInfo croppedCamInfo(
    const sensor_msgs::msg::CameraInfo& full,
    int x0, int y0, int tile_w, int tile_h);

// Run DA3 on tiles + blend back into full-res depth (returns CV_32FC1 meters)
cv::Mat inferDepthTiled(
        depth_anything_v3::TensorRTDepthAnything& da,
        const cv::Mat& rgb_full,
        const sensor_msgs::msg::CameraInfo& cam_full,
        const TileCfg& cfg,
        const cv::Mat& tile_weights,
        int tile_fade_px);

}  // namespace depth_anything_helpers
