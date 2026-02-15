#include "depth_anything_v3/depth_helpers.hpp"
#include "depth_anything_v3/tensorrt_depth_anything.hpp"   
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>


namespace depth_anything_helpers {

bool loadIntrinsicsFromYaml(
    const std::string& path, int& width, int& height,
    double& fx, double& fy, double& cx, double& cy)
{
    YAML::Node root = YAML::LoadFile(path);
    if (!root["camera_matrix"] || !root["camera_matrix"]["data"]) return false;

    auto k = root["camera_matrix"]["data"];
    if (!k.IsSequence() || k.size() != 9) return false;

    width  = root["image_width"]  ? root["image_width"].as<int>()  : -1;
    height = root["image_height"] ? root["image_height"].as<int>() : -1;

    fx = k[0].as<double>();
    cx = k[2].as<double>();
    fy = k[4].as<double>();
    cy = k[5].as<double>();
    return (width > 0 && height > 0 && fx > 0.0 && fy > 0.0);
}

sensor_msgs::msg::CameraInfo makeCameraInfoFromIntrinsics(
    int width, int height, double fx, double fy, double cx, double cy,
    const std::string& frame_id)
{
    sensor_msgs::msg::CameraInfo ci;
    ci.width = width;
    ci.height = height;

    ci.k = {fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0};

    ci.d = std::vector<double>(5, 0.0);
    ci.r = {1.0,0.0,0.0,
            0.0,1.0,0.0,
            0.0,0.0,1.0};

    ci.p = {fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0};

    ci.header.frame_id = frame_id;
    return ci;
}

cv::Rect clampRoi(int x1, int y1, int x2, int y2, int w, int h)
{
    x1 = std::max(0, std::min(x1, w - 1));
    x2 = std::max(0, std::min(x2, w - 1));
    y1 = std::max(0, std::min(y1, h - 1));
    y2 = std::max(0, std::min(y2, h - 1));
    if (x2 <= x1 || y2 <= y1) return cv::Rect();
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

bool depthHistModeInRoi(
    const cv::Mat& depth32f, const cv::Rect& roi,
    double min_d, double max_d,
    int bins, double min_frac,
    double& out_z)
{

    (void)bins;
    (void)min_frac;

    std::vector<float> vals;
    vals.reserve(static_cast<size_t>(roi.area()));

    for (int y = roi.y; y < roi.y + roi.height; ++y) {
        const float* row = depth32f.ptr<float>(y);
        for (int x = roi.x; x < roi.x + roi.width; ++x) {
            float d = row[x];
            if (std::isfinite(d) && d >= min_d && d <= max_d) vals.push_back(d);
        }
    }
    if (vals.size() < 30) return false;
    auto [mn_it, mx_it] = std::minmax_element(vals.begin(), vals.end());
    std::cerr << "[DBG] roi depth mn=" << *mn_it << " mx=" << *mx_it << "\n";


    // std::vector<int> hist(std::max(1, bins), 0);
    // const double inv = 1.0 / (max_d - min_d);

    // for (float d : vals) {
    //     int b = int((d - min_d) * inv * bins);
    //     if (b < 0) b = 0;
    //     if (b >= bins) b = bins - 1;
    //     hist[b]++;
    // }

    // int best_b = 0;
    // for (int i = 1; i < bins; ++i) {
    //     if (hist[i] > hist[best_b]) best_b = i;
    // }

    // const double frac = double(hist[best_b]) / double(vals.size());
    // if (frac < min_frac) return false;

    // const double bin_w = (max_d - min_d) / double(bins);
    // out_z = min_d + (best_b + 0.5) * bin_w;

    // std::cerr << "[DBG] roi=" << roi.x << "," << roi.y << " " << roi.width << "x" << roi.height
    //         << " vals=" << vals.size()
    //         << " best_b=" << best_b
    //         << " frac=" << frac
    //         << " out_z=" << out_z
    //         << " min_d=" << min_d << " max_d=" << max_d << " bins=" << bins << "\n";
    
    
    std::nth_element(vals.begin(), vals.begin() + vals.size()/2, vals.end());
    out_z = vals[vals.size()/2];
    return true;
}

// Accept either:
// 1) sidecar JSON that contains nanoowl.result.detections
// 2) direct detections array
json parseDetectionsFromNanoowlLikeJson(const json& det_or_sidecar)
{
    json dets = json::array();

    try {
        if (det_or_sidecar.is_array()) {
            dets = det_or_sidecar;
        } else if (det_or_sidecar.contains("nanoowl")) {
            const auto& n = det_or_sidecar["nanoowl"];
            if (n.contains("result") && n["result"].contains("detections")) {
                dets = n["result"]["detections"];
            }
        } else if (det_or_sidecar.contains("result") && det_or_sidecar["result"].contains("detections")) {
            dets = det_or_sidecar["result"]["detections"];
        } else if (det_or_sidecar.contains("detections")) {
            dets = det_or_sidecar["detections"];
        }
    } catch (...) {
        dets = json::array();
    }

    json out = json::array();
    if (!dets.is_array()) return out;

    for (const auto& d : dets) {
        if (!d.is_object()) continue;

        const std::string label = d.value("label", "");
        const double score = d.value("score", 0.0);

        if (!d.contains("bbox") || !d["bbox"].is_array() || d["bbox"].size() != 4) continue;
        int x1 = int(std::round(d["bbox"][0].get<double>()));
        int y1 = int(std::round(d["bbox"][1].get<double>()));
        int x2 = int(std::round(d["bbox"][2].get<double>()));
        int y2 = int(std::round(d["bbox"][3].get<double>()));

        out.push_back({
            {"label", label},
            {"score", score},
            {"bbox", {x1,y1,x2,y2}}
        });
    }

    return out;
}

json projectUvDepthToXyz(double u, double v, double z, const sensor_msgs::msg::CameraInfo& cam)
    {
        const double fx = cam.k[0];
        const double fy = cam.k[4];
        const double cx = cam.k[2];
        const double cy = cam.k[5];

        const double X = (u - cx) * z / fx;
        const double Y = (v - cy) * z / fy;

        
        return json{{"xyz_m", {X, Y, z}}, {"depth_m", z}};
    }

void saveDepthColormap(const cv::Mat& depth32f_m, const std::string& out_path_jpg, float vmin, float vmax) {
        if (depth32f_m.empty()) return;

        // 1. Pre-process 32-bit data to kill the grid
        cv::Mat smoothed;
        // Median filter (3x3) kills single-pixel 'tile edge' noise
        cv::medianBlur(depth32f_m, smoothed, 3);

        // 2. Normalization to 8-bit
        cv::Mat clipped;
        cv::threshold(smoothed, clipped, vmax, vmax, cv::THRESH_TRUNC);
        cv::max(clipped, vmin, clipped);
        clipped = (clipped - vmin) / (vmax - vmin);

        cv::Mat depth_u8;
        clipped.convertTo(depth_u8, CV_8U, 255.0);

        // 3. Bilateral Filter: Sharpens edges (plant) while smoothing flat areas (walls)
        cv::Mat denoised;
        cv::bilateralFilter(depth_u8, denoised, 9, 75, 75);

        // 4. Soft CLAHE (Higher tiles, lower limit)
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, cv::Size(32, 32));
        cv::Mat enhanced_u8;
        clahe->apply(denoised, enhanced_u8);

        cv::Mat depth_color;
        cv::applyColorMap(enhanced_u8, depth_color, cv::COLORMAP_INFERNO);
        cv::imwrite(out_path_jpg, depth_color);
    }

    bool maxDepthInColumns(
    const cv::Mat& depth32f,
    int x1, int x2,
    double min_d, double max_d,
    double& out_max_depth)
    {
        if (depth32f.empty() || depth32f.type() != CV_32FC1) return false;

        const int W = depth32f.cols;
        const int H = depth32f.rows;

        x1 = std::max(0, std::min(x1, W - 1));
        x2 = std::max(0, std::min(x2, W - 1));
        if (x2 < x1) std::swap(x1, x2);

        double best = -1.0;
        int count = 0;

        for (int y = 0; y < H; ++y) {
            const float* row = depth32f.ptr<float>(y);
            for (int x = x1; x <= x2; ++x) {
                const float d = row[x];
                if (!std::isfinite(d)) continue;
                if (d < (float)min_d || d > (float)max_d) continue;
                if (d > best) best = d;
                count++;
            }
        }

        if (count < 30 || best <= 0.0) return false;
        out_max_depth = best;
        return true;
    }




    std::string dirnameOf(const std::string& p) {
    return std::filesystem::path(p).parent_path().string();
    }

    std::string addDepthSuffix(const std::string& dir) {
    // "/a/b/c" -> "/a/b/c_depth"
    return (std::filesystem::path(dir).string() + "_depth");
    }

    void ensureDir(const std::string& d) {
    std::filesystem::create_directories(d);
    }

    std::string stemOf(const std::string& p) {
    return std::filesystem::path(p).stem().string();
    }

    std::vector<cv::Rect> iterTiles(int full_w, int full_h, const TileCfg& cfg)
    {
    std::vector<cv::Rect> tiles;

    int y = 0;
    while (true) {
        if (y + cfg.tile_h >= full_h) y = std::max(0, full_h - cfg.tile_h);

        int x = 0;
        while (true) {
        if (x + cfg.tile_w >= full_w) x = std::max(0, full_w - cfg.tile_w);

        tiles.emplace_back(x, y, cfg.tile_w, cfg.tile_h);

        if (x + cfg.tile_w >= full_w) break;
        x += cfg.step_x();
        }

        if (y + cfg.tile_h >= full_h) break;
        y += cfg.step_y();
    }

    return tiles;
    }

    cv::Mat blendWeights(int h, int w, int border_px) {
        cv::Mat wy(h, 1, CV_32F), wx(1, w, CV_32F);
        
        auto smoothstep = [](float edge0, float edge1, float x) {
            float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
            return t * t * (3.0f - 2.0f * t);
        };

        for (int y = 0; y < h; ++y) {
            int dist = std::min(y, h - 1 - y);
            wy.at<float>(y, 0) = smoothstep(0.0f, (float)border_px, (float)dist);
        }
        for (int x = 0; x < w; ++x) {
            int dist = std::min(x, w - 1 - x);
            wx.at<float>(0, x) = smoothstep(0.0f, (float)border_px, (float)dist);
        }

        cv::Mat weights = wy * wx; 

        int kernel_size = border_px | 1; // Use the full fade width for the blur
        if (kernel_size > 1) {
            cv::GaussianBlur(weights, weights, cv::Size(kernel_size, kernel_size), 0);
        }

        return weights;
    }

    sensor_msgs::msg::CameraInfo croppedCamInfo(
        const sensor_msgs::msg::CameraInfo& full,
        int x0, int y0, int tile_w, int tile_h)
    {
        sensor_msgs::msg::CameraInfo ci = full;
        ci.width = tile_w;
        ci.height = tile_h;

        // K: [fx 0 cx; 0 fy cy; 0 0 1]
        ci.k[2] = full.k[2] - double(x0);
        ci.k[5] = full.k[5] - double(y0);

        // P: [fx 0 cx Tx; 0 fy cy Ty; 0 0 1 0]
        ci.p[2] = full.p[2] - double(x0);
        ci.p[6] = full.p[6] - double(y0);

        return ci;
    }

    // static bool fitScaleShiftToReference(
    // const cv::Mat& tile_depth,     // CV_32F
    // const cv::Mat& ref_roi,        // CV_32F, same size
    // double min_d, double max_d,
    // float& out_a, float& out_b)
    // {
    // CV_Assert(tile_depth.type() == CV_32FC1);
    // CV_Assert(ref_roi.type() == CV_32FC1);
    // CV_Assert(tile_depth.size() == ref_roi.size());

    // // Sample stats over valid pixels
    // double sum_t = 0, sum_r = 0;
    // double sum_tt = 0, sum_tr = 0;
    // int n = 0;

    // for (int y = 0; y < tile_depth.rows; ++y) {
    //     const float* trow = tile_depth.ptr<float>(y);
    //     const float* rrow = ref_roi.ptr<float>(y);
    //     for (int x = 0; x < tile_depth.cols; ++x) {
    //     const float t = trow[x];
    //     const float r = rrow[x];
    //     if (!std::isfinite(t) || !std::isfinite(r)) continue;
    //     if (t < (float)min_d || t > (float)max_d) continue;
    //     if (r < (float)min_d || r > (float)max_d) continue;

    //     sum_t  += t;
    //     sum_r  += r;
    //     sum_tt += (double)t * (double)t;
    //     sum_tr += (double)t * (double)r;
    //     n++;
    //     }
    // }

    // if (n < 500) return false;

    // const double mean_t = sum_t / n;
    // const double mean_r = sum_r / n;

    // const double var_t = (sum_tt / n) - mean_t * mean_t;
    // if (var_t < 1e-6) return false;

    // const double cov_tr = (sum_tr / n) - mean_t * mean_r;

    // const double a = cov_tr / var_t;
    // const double b = mean_r - a * mean_t;

    // // Clamp insane fits (optional but recommended)
    // if (!std::isfinite(a) || !std::isfinite(b)) return false;
    // if (a < 0.3 || a > 3.0) return false;

    // out_a = (float)a;
    // out_b = (float)b;
    // return true;
    // }


   void normalizeTileGlobally(cv::Mat& tile, float global_min, float global_max) {
        float range = global_max - global_min;
        if (range < 1e-6f) return;
        // Map tile values to 0.0 - 1.0 based on the global reference pass
        tile = (tile - global_min) / range;
    }

    cv::Mat inferDepthTiled(
    depth_anything_v3::TensorRTDepthAnything& da,
    const cv::Mat& rgb_full,
    const sensor_msgs::msg::CameraInfo& cam_full,
    const TileCfg& cfg,
    const cv::Mat& tile_weights,
    int tile_fade_px) 
    {
        const int W = rgb_full.cols;
        const int H = rgb_full.rows;

        // --- STEP 1: Global Reference Pass ---
        // We move 'ref' outside the brackets so it stays in scope
        cv::Mat global_ref; 
        {
            da.initPreprocessBuffer(W, H);
            if (!da.doInference({rgb_full}, cam_full, 1, false)) return cv::Mat();
            global_ref = da.getDepthImage().clone(); // Clone to keep memory safe
            cv::patchNaNs(global_ref, 0.0);
        }

        // --- STEP 2: Tiled Loop ---
        da.initPreprocessBuffer(cfg.tile_w, cfg.tile_h);
        cv::Mat acc(H, W, CV_32F, cv::Scalar(0));
        cv::Mat wsum(H, W, CV_32F, cv::Scalar(0));

        auto tiles = iterTiles(W, H, cfg);
        for (const auto& r : tiles) {
            cv::Mat rgb_tile = rgb_full(r);
            auto cam_tile = croppedCamInfo(cam_full, r.x, r.y, r.width, r.height);

            if (!da.doInference({rgb_tile}, cam_tile, 1, false)) continue;
            cv::Mat d = da.getDepthImage().clone();
            cv::patchNaNs(d, 0.0);

            // 1. Align tile to global reference using Scale (a) and Shift (b)
            cv::Mat ref_roi = global_ref(r); 
            double mn_ref, mx_ref, mn_tile, mx_tile;
            cv::minMaxLoc(ref_roi, &mn_ref, &mx_ref);
            cv::minMaxLoc(d, &mn_tile, &mx_tile);

            float shift = (float)mn_ref - (float)mn_tile;
            d += shift;

            cv::Mat acc_roi = acc(r);
            cv::Mat wsum_roi = wsum(r);
            acc_roi += d.mul(tile_weights);
            wsum_roi += tile_weights;
        }

        cv::Mat result;
        cv::divide(acc, wsum + 1e-6f, result); 
        cv::patchNaNs(result, 0.0);
        cv::Mat melted;
        cv::medianBlur(result, melted, 3); 
        return melted;
    }
        
} // namespace depth_anything_helpers
