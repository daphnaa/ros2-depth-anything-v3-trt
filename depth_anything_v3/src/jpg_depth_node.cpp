// jpg_depth_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/header.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <algorithm>
#include <limits>
#include <chrono>
#include <cstring>   // memcpy


#include <nlohmann/json.hpp>
using json = nlohmann::json;
namespace fs = std::filesystem;

#include "depth_anything_v3/tensorrt_depth_anything.hpp"

// ---------------- YAML intrinsics ----------------
static bool loadIntrinsicsFromYaml(
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

static sensor_msgs::msg::CameraInfo makeCameraInfoFromIntrinsics(
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

// ---------------- ROI helpers ----------------
static cv::Rect clampRoi(int x1, int y1, int x2, int y2, int w, int h)
{
    x1 = std::max(0, std::min(x1, w - 1));
    x2 = std::max(0, std::min(x2, w - 1));
    y1 = std::max(0, std::min(y1, h - 1));
    y2 = std::max(0, std::min(y2, h - 1));
    if (x2 <= x1 || y2 <= y1) return cv::Rect();
    return cv::Rect(x1, y1, x2 - x1, y2 - y1);
}

static bool depthHistModeInRoi(
    const cv::Mat& depth32f, const cv::Rect& roi,
    double min_d, double max_d,
    int bins, double min_frac,
    double& out_z)
{
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

    std::vector<int> hist(bins, 0);
    const double inv = 1.0 / (max_d - min_d);

    for (float d : vals) {
        int b = int((d - min_d) * inv * bins);
        if (b < 0) b = 0;
        if (b >= bins) b = bins - 1;
        hist[b]++;
    }

    int best_b = 0;
    for (int i = 1; i < bins; ++i) {
        if (hist[i] > hist[best_b]) best_b = i;
    }

    const double frac = double(hist[best_b]) / double(vals.size());
    if (frac < min_frac) return false;

    const double bin_w = (max_d - min_d) / double(bins);
    out_z = min_d + (best_b + 0.5) * bin_w;
    return true;
}

// ---------------- matToImageMsg helper ----------------
static sensor_msgs::msg::Image matToImageMsg(
    const cv::Mat& mat,
    const std::string& encoding,
    const rclcpp::Time& stamp,
    const std::string& frame_id)
{
    // Ensure we publish contiguous data
    cv::Mat contig = mat.isContinuous() ? mat : mat.clone();

    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = contig.rows;
    msg.width = contig.cols;
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(contig.step);

    const size_t bytes = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
    msg.data.resize(bytes);
    std::memcpy(msg.data.data(), contig.data, bytes);
    return msg;
}


// ---------------- PointCloud (same style as video node) ----------------
static void createPointCloudRgb(
    const cv::Mat& depth_image,     // CV_32FC1 meters
    const cv::Mat& rgb_image,       // CV_8UC3 RGB
    const sensor_msgs::msg::CameraInfo& camera_info,
    const rclcpp::Time& stamp,
    const std::string& frame_id,
    sensor_msgs::msg::PointCloud2& cloud_msg)
{
    const double fx = camera_info.k[0];
    const double fy = camera_info.k[4];
    const double cx = camera_info.k[2];
    const double cy = camera_info.k[5];

    const int height = depth_image.rows;
    const int width = depth_image.cols;
    const int downsample = 2;

    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.height = 1;
    cloud_msg.is_dense = false;
    cloud_msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

    int valid_points = 0;
    for (int v = 0; v < height; v += downsample) {
        for (int u = 0; u < width; u += downsample) {
            float d = depth_image.at<float>(v, u);
            if (d > 0.0f && std::isfinite(d) && d < 100.0f) valid_points++;
        }
    }

    modifier.resize(valid_points);
    cloud_msg.width = valid_points;

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

    for (int v = 0; v < height; v += downsample) {
        for (int u = 0; u < width; u += downsample) {
            float d = depth_image.at<float>(v, u);
            if (d <= 0.0f || !std::isfinite(d) || d > 100.0f) continue;

            *iter_z = d;
            *iter_x = -(u - cx) * d / fx;
            *iter_y = -(v - cy) * d / fy;

            cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(v, u);  // RGB
            *iter_r = rgb[0];
            *iter_g = rgb[1];
            *iter_b = rgb[2];

            ++iter_x; ++iter_y; ++iter_z;
            ++iter_r; ++iter_g; ++iter_b;
        }
    }
}

class JpgDepthNode : public rclcpp::Node {
public:
    JpgDepthNode() : Node("jpg_depth_node") {
        // Params
        declare_parameter<std::string>("jpg_dir", "");
        declare_parameter<std::string>("json_dir", "");
        declare_parameter<std::string>("output_dir", "");
        declare_parameter<std::string>("camera_yaml", "");
        declare_parameter<std::string>("model_path", "onnx/DA3METRIC-LARGE.onnx");
        declare_parameter<std::string>("frame_id", "camera_link");
        declare_parameter<double>("fps", 10.0);

        declare_parameter<double>("min_depth", 0.2);
        declare_parameter<double>("max_depth", 10.0);
        declare_parameter<double>("min_score", 0.2);
        declare_parameter<int>("hist_bins", 30);
        declare_parameter<double>("hist_min_frac", 0.06);

        jpg_dir_ = get_parameter("jpg_dir").as_string();
        json_dir_ = get_parameter("json_dir").as_string();
        output_dir_ = get_parameter("output_dir").as_string();
        camera_yaml_ = get_parameter("camera_yaml").as_string();
        model_path_ = get_parameter("model_path").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        fps_ = get_parameter("fps").as_double();

        min_depth_ = get_parameter("min_depth").as_double();
        max_depth_ = get_parameter("max_depth").as_double();
        min_score_ = get_parameter("min_score").as_double();
        hist_bins_ = get_parameter("hist_bins").as_int();
        hist_min_frac_ = get_parameter("hist_min_frac").as_double();

        if (jpg_dir_.empty()) throw std::runtime_error("jpg_dir is required");
        if (output_dir_.empty()) throw std::runtime_error("output_dir is required");
        if (camera_yaml_.empty()) throw std::runtime_error("camera_yaml is required");
        if (json_dir_.empty()) json_dir_ = jpg_dir_;

        fs::create_directories(output_dir_);

        // Load YAML intrinsics (defines our inference resolution)
        if (!loadIntrinsicsFromYaml(camera_yaml_, calib_w_, calib_h_, fx0_, fy0_, cx0_, cy0_)) {
            throw std::runtime_error("Failed to load intrinsics from: " + camera_yaml_);
        }

        target_w_ = calib_w_;
        target_h_ = calib_h_;
        camera_info_ = makeCameraInfoFromIntrinsics(target_w_, target_h_, fx0_, fy0_, cx0_, cy0_, frame_id_);

        // Publishers (like video)
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("~/output/depth_image", 10);
        pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/output/point_cloud", 10);

        collectJpgs();
        if (jpg_files_.empty()) throw std::runtime_error("No jpg files found in: " + jpg_dir_);

        if (!initDepthEstimator()) throw std::runtime_error("Failed to init depth estimator");

        const int interval_ms = static_cast<int>(1000.0 / std::max(1e-3, fps_));
        timer_ = create_wall_timer(
            std::chrono::milliseconds(interval_ms),
            std::bind(&JpgDepthNode::tick, this));

        RCLCPP_INFO(get_logger(), "jpg_depth_node ready. frames=%zu  target=%dx%d",
                    jpg_files_.size(), target_w_, target_h_);
    }

private:
    void collectJpgs() {
        for (auto& p : fs::directory_iterator(jpg_dir_)) {
            if (!p.is_regular_file()) continue;
            auto ext = p.path().extension().string();
            std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
            if (ext == ".jpg" || ext == ".jpeg") jpg_files_.push_back(p.path().string());
        }
        std::sort(jpg_files_.begin(), jpg_files_.end());
    }

    bool initDepthEstimator() {
        RCLCPP_INFO(get_logger(), "Initializing depth estimator (this may take a while on first run)...");
        try {
            tensorrt_common::BuildConfig build_config;
            build_config.clip_value = 0.0f;
            build_config.calib_type_str = "MinMax";

            depth_estimator_ = std::make_unique<depth_anything_v3::TensorRTDepthAnything>(
                model_path_,
                "fp16",
                build_config,
                false,
                "",
                tensorrt_common::BatchConfig{1, 1, 1},
                (1ULL << 30)
            );

            depth_estimator_->initPreprocessBuffer(target_w_, target_h_);
            depth_estimator_->setSkyThreshold(0.3f);

            RCLCPP_INFO(get_logger(), "✓ Depth estimator initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Depth estimator init failed: %s", e.what());
            return false;
        }
    }

    void tick() {
        if (idx_ >= jpg_files_.size()) {
            RCLCPP_INFO(get_logger(), "Finished JPG sequence.");
            timer_->cancel();
            return;
        }

        const std::string jpg_path = jpg_files_[idx_++];
        cv::Mat bgr_full = cv::imread(jpg_path, cv::IMREAD_COLOR);
        if (bgr_full.empty()) {
            RCLCPP_WARN(get_logger(), "Failed to read image: %s", jpg_path.c_str());
            return;
        }

        // Resize to YAML calibration size (stable inference + stable intrinsics)
        cv::Mat bgr;
        if (bgr_full.cols != target_w_ || bgr_full.rows != target_h_) {
            cv::resize(bgr_full, bgr, cv::Size(target_w_, target_h_), 0, 0, cv::INTER_LINEAR);
        } else {
            bgr = bgr_full;
        }

        // Convert to RGB (like video node)
        cv::Mat rgb;
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);

        // Depth inference
        std::vector<cv::Mat> images = {rgb};
        const bool ok = depth_estimator_->doInference(images, camera_info_, 1, false);
        if (!ok) {
            RCLCPP_ERROR(get_logger(), "Inference failed for: %s", jpg_path.c_str());
            return;
        }
        const cv::Mat& depth32f = depth_estimator_->getDepthImage();  // CV_32FC1

        const auto stamp = now();

        // Publish depth
        auto depth_msg = matToImageMsg(depth32f, "32FC1", stamp, frame_id_);
        depth_pub_->publish(depth_msg);


        // Publish pointcloud (RGB)
        sensor_msgs::msg::PointCloud2 cloud_msg;
        createPointCloudRgb(depth32f, rgb, camera_info_, stamp, frame_id_, cloud_msg);
        pointcloud_pub_->publish(cloud_msg);

        // Load detections JSON (same basename)
        fs::path stem = fs::path(jpg_path).stem();
        fs::path json_path = fs::path(json_dir_) / (stem.string() + ".json");

        json objs = json::array();

        if (fs::exists(json_path)) {
            std::ifstream jf(json_path.string());
            json j;
            try { jf >> j; } catch (...) { j = json(); }

            // NanoOWL path: nanoowl.result.detections
            if (j.contains("nanoowl") && j["nanoowl"].contains("result") && j["nanoowl"]["result"].contains("detections")) {
                auto dets = j["nanoowl"]["result"]["detections"];
                if (dets.is_array()) {
                    for (auto& det : dets) {
                        const double score = det.value("score", 0.0);
                        if (score < min_score_) continue;

                        const std::string label = det.value("label", "");

                        auto bb = det["bbox"];
                        if (!bb.is_array() || bb.size() != 4) continue;

                        int x1 = bb[0].get<int>();
                        int y1 = bb[1].get<int>();
                        int x2 = bb[2].get<int>();
                        int y2 = bb[3].get<int>();

                        // If detections are in original image space, but we resized:
                        // scale bbox to (target_w_, target_h_) using source dims from JSON if present
                        int src_w = -1, src_h = -1;
                        if (j["nanoowl"]["result"].contains("image")) {
                            auto im = j["nanoowl"]["result"]["image"];
                            if (im.contains("width")) src_w = im["width"].get<int>();
                            if (im.contains("height")) src_h = im["height"].get<int>();
                        }

                        if (src_w > 0 && src_h > 0 && (src_w != target_w_ || src_h != target_h_)) {
                            const double sx = double(target_w_) / double(src_w);
                            const double sy = double(target_h_) / double(src_h);
                            x1 = int(std::round(x1 * sx));
                            x2 = int(std::round(x2 * sx));
                            y1 = int(std::round(y1 * sy));
                            y2 = int(std::round(y2 * sy));
                        }

                        cv::Rect roi = clampRoi(x1, y1, x2, y2, depth32f.cols, depth32f.rows);
                        if (roi.width <= 0 || roi.height <= 0) continue;

                        double Z = std::numeric_limits<double>::quiet_NaN();
                        if (!depthHistModeInRoi(depth32f, roi, min_depth_, max_depth_, hist_bins_, hist_min_frac_, Z)) {
                            continue;
                        }

                        const double u = 0.5 * (x1 + x2);
                        const double v = 0.5 * (y1 + y2);

                        const double fx = camera_info_.k[0];
                        const double fy = camera_info_.k[4];
                        const double cx = camera_info_.k[2];
                        const double cy = camera_info_.k[5];

                        const double X = (u - cx) * Z / fx;
                        const double Y = (v - cy) * Z / fy;

                        json o;
                        o["label"] = label;
                        o["score"] = score;
                        o["bbox"] = {x1, y1, x2, y2};
                        o["depth_m"] = Z;
                        o["xyz_m"] = {X, Y, Z};
                        objs.push_back(o);
                    }
                }
            }
        }

        // Write output JSON for client
        json out;
        out["image"] = fs::path(jpg_path).filename().string();
        out["stamp_unix_ms"] = (int64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        out["camera"] = {
            {"width", target_w_}, {"height", target_h_},
            {"fx", camera_info_.k[0]}, {"fy", camera_info_.k[4]},
            {"cx", camera_info_.k[2]}, {"cy", camera_info_.k[5]}
        };
        out["objects"] = objs;

        fs::path out_path = fs::path(output_dir_) / (stem.string() + ".objects.json");
        std::ofstream of(out_path.string());
        of << out.dump(2);
    }

private:
    // Paths / params
    std::string jpg_dir_, json_dir_, output_dir_;
    std::string camera_yaml_, model_path_, frame_id_;
    double fps_{10.0};

    double min_depth_{0.2}, max_depth_{10.0}, min_score_{0.2};
    int hist_bins_{30};
    double hist_min_frac_{0.06};

    // Calibration / inference size (from YAML)
    int calib_w_{-1}, calib_h_{-1};
    int target_w_{-1}, target_h_{-1};
    double fx0_{0}, fy0_{0}, cx0_{0}, cy0_{0};

    // Files
    std::vector<std::string> jpg_files_;
    size_t idx_{0};

    // ROS
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;

    // Inference
    sensor_msgs::msg::CameraInfo camera_info_;
    std::unique_ptr<depth_anything_v3::TensorRTDepthAnything> depth_estimator_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JpgDepthNode>());
    rclcpp::shutdown();
    return 0;
}
