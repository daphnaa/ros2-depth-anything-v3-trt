#include "depth_anything_v3/depth_helpers.hpp"
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

json computeWallProbesTopThird(
    const cv::Mat& depth32f,
    const sensor_msgs::msg::CameraInfo& cam,
    double min_depth, double max_depth,
    int hist_bins, double hist_min_frac)
{
    const int W = depth32f.cols;
    const int H = depth32f.rows;

    // Top third, small-ish boxes
    const int y_center = int(std::round(H * 0.18));             // in top third
    const int box_w = std::max(20, int(std::round(W * 0.12)));
    const int box_h = std::max(20, int(std::round(H * 0.12)));

    auto make_probe = [&](double x_frac) -> json {
        const int x_center = int(std::round(W * x_frac));
        const int x1 = x_center - box_w / 2;
        const int x2 = x_center + box_w / 2;
        const int y1 = y_center - box_h / 2;
        const int y2 = y_center + box_h / 2;

        cv::Rect roi = clampRoi(x1, y1, x2, y2, W, H);
        if (roi.width <= 0 || roi.height <= 0) return json::object();

        double Z = std::numeric_limits<double>::quiet_NaN();
        if (!depthHistModeInRoi(depth32f, roi, min_depth, max_depth, hist_bins, hist_min_frac, Z)) {
            return json::object();
        }

        const double u = roi.x + 0.5 * roi.width;
        const double v = roi.y + 0.5 * roi.height;

        json p = projectUvDepthToXyz(u, v, Z, cam);
        p["bbox"] = {roi.x, roi.y, roi.x + roi.width, roi.y + roi.height};
        return p;
    };

    return json{
        {"left",   make_probe(1.0/6.0)},
        {"middle", make_probe(3.0/6.0)},
        {"right",  make_probe(5.0/6.0)}
    };
}


void saveDepthColormap(const cv::Mat& depth32f_m,
                    const std::string& out_path_jpg,
                    float vmin,
                    float vmax)
    {
    CV_Assert(depth32f_m.type() == CV_32FC1);

    // Auto range (robust percentiles) if vmin/vmax not provided
    if (vmin < 0.0f || vmax < 0.0f) {
        std::vector<float> v;
        v.reserve(depth32f_m.total());

        for (int y = 0; y < depth32f_m.rows; ++y) {
        const float* row = depth32f_m.ptr<float>(y);
        for (int x = 0; x < depth32f_m.cols; ++x) {
            float d = row[x];
            if (std::isfinite(d) && d > 0.0f) v.push_back(d);
        }
        }
        if (v.size() < 100) return;

        std::sort(v.begin(), v.end());
        auto pct = [&](double p) -> float {
        size_t i = (size_t)std::round(p * (v.size() - 1));
        if (i >= v.size()) i = v.size() - 1;
        return v[i];
        };

        vmin = pct(0.02);
        vmax = pct(0.98);
        if (!(vmax > vmin)) { vmin = v.front(); vmax = v.back(); }
    }

    cv::Mat clipped = depth32f_m.clone();

    // Replace non-finite / <=0 with NaN-ish marker (we'll paint black later)
    for (int y = 0; y < clipped.rows; ++y) {
        float* row = clipped.ptr<float>(y);
        for (int x = 0; x < clipped.cols; ++x) {
        float d = row[x];
        if (!std::isfinite(d) || d <= 0.0f) row[x] = std::numeric_limits<float>::quiet_NaN();
        }
    }

    // Correct clamping:
    // 1) cap above vmax
    cv::threshold(clipped, clipped, vmax, vmax, cv::THRESH_TRUNC);
    // 2) clamp below vmin UP to vmin (not to zero!)
    cv::max(clipped, vmin, clipped);

    cv::Mat depth_norm, depth_u8, depth_color;
    depth_norm = (clipped - vmin) / (vmax - vmin);
    depth_norm.convertTo(depth_u8, CV_8U, 255.0);

    cv::applyColorMap(depth_u8, depth_color, cv::COLORMAP_TURBO);

    // Paint invalid as black
    for (int y = 0; y < depth32f_m.rows; ++y) {
        const float* row = depth32f_m.ptr<float>(y);
        cv::Vec3b* crow = depth_color.ptr<cv::Vec3b>(y);
        for (int x = 0; x < depth32f_m.cols; ++x) {
        float d = row[x];
        if (!std::isfinite(d) || d <= 0.0f) crow[x] = cv::Vec3b(0,0,0);
        }
    }

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



} // namespace depth_anything_helpers
