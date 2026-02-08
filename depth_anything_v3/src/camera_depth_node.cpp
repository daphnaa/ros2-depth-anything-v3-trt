// Copyright 2025 Institute for Automotive Engineering (ika), RWTH Aachen University
//
// ROS 2 node for camera capture and depth estimation with point cloud publishing
// Usage: ros2 run depth_anything_v3 camera_depth_node

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <thread>
#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "depth_anything_v3/tensorrt_depth_anything.hpp"
#include "depth_anything_v3/camera_factory.hpp"
#include "depth_anything_v3/fisheye_undistorter.hpp"

class CameraDepthNode : public rclcpp::Node
{
public:
    CameraDepthNode() : Node("camera_depth_node")
    {
        // Declare parameters
        this->declare_parameter("camera_type", "standard");
        this->declare_parameter("camera_id", 0);
        this->declare_parameter("device_path", "/dev/video0");
        this->declare_parameter("model_path", "onnx/DA3METRIC-LARGE.onnx");
        this->declare_parameter("frame_id", "camera_link");
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("camera_width", 640);
        this->declare_parameter("camera_height", 480);
        this->declare_parameter("framerate", 30);
        this->declare_parameter("format", "UYVY");
        this->declare_parameter("sensor_mode", 0);
        this->declare_parameter("downsample_factor", 1);  // New parameter for downsampling
        
        // Fisheye undistortion parameters
        this->declare_parameter("enable_undistortion", false);  // Enable fisheye undistortion
        this->declare_parameter("undistortion_balance", 0.0);   // 0.0=crop, 1.0=full FOV
        
        // Camera calibration file path (optional)
        this->declare_parameter("camera_info_file", "");
        
        // Camera calibration parameters (optional, will use estimated values if not provided)
        // Default values are for fisheye lens calibration (1920x1536)
        this->declare_parameter("use_calibration", false);
        this->declare_parameter("fx", 824.147361);
        this->declare_parameter("fy", 823.660879);
        this->declare_parameter("cx", 958.275200);
        this->declare_parameter("cy", 767.389372);
        this->declare_parameter("k1", 1.486308);    // Fisheye D[0]
        this->declare_parameter("k2", -13.386609);  // Fisheye D[1]
        this->declare_parameter("p1", 21.409334);   // Fisheye D[2] or tangential p1
        this->declare_parameter("p2", 3.817858);    // Fisheye D[3] or tangential p2
        this->declare_parameter("k3", 0.0);
        this->declare_parameter("distortion_model", "plumb_bob");  // "fisheye" or "plumb_bob"

        // Timing debug parameters
        this->declare_parameter("debug_timing", true);
        this->declare_parameter("debug_timing_throttle_ms", 1000);
        debug_timing_ = this->get_parameter("debug_timing").as_bool();
        debug_timing_throttle_ms_ = this->get_parameter("debug_timing_throttle_ms").as_int();

        // Camera recovery parameters
        this->declare_parameter("camera_reconnect_enable", true);
        this->declare_parameter("camera_reconnect_backoff_ms", 500);
        this->declare_parameter("camera_reconnect_max_backoff_ms", 5000);
        this->declare_parameter("camera_failures_before_reconnect", 3);



        
        camera_type_ = this->get_parameter("camera_type").as_string();
        camera_id_ = this->get_parameter("camera_id").as_int();
        device_path_ = this->get_parameter("device_path").as_string();
        model_path_ = this->get_parameter("model_path").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        camera_width_ = this->get_parameter("camera_width").as_int();
        camera_height_ = this->get_parameter("camera_height").as_int();
        framerate_ = this->get_parameter("framerate").as_int();
        format_ = this->get_parameter("format").as_string();
        sensor_mode_ = this->get_parameter("sensor_mode").as_int();
        downsample_factor_ = this->get_parameter("downsample_factor").as_int();
        
        // Get undistortion parameters
        enable_undistortion_ = this->get_parameter("enable_undistortion").as_bool();
        undistortion_balance_ = this->get_parameter("undistortion_balance").as_double();
        
        // Get calibration file path
        std::string camera_info_file = this->get_parameter("camera_info_file").as_string();
        
        // Try to load calibration from file first, then fall back to parameters
        if (!camera_info_file.empty() && loadCalibrationFromFile(camera_info_file)) {
            use_calibration_ = true;
            RCLCPP_INFO(this->get_logger(), "Loaded calibration from file: %s", camera_info_file.c_str());
        } else {
            // Get calibration parameters from ROS parameters
            use_calibration_ = this->get_parameter("use_calibration").as_bool();
            calib_fx_ = this->get_parameter("fx").as_double();
            calib_fy_ = this->get_parameter("fy").as_double();
            calib_cx_ = this->get_parameter("cx").as_double();
            calib_cy_ = this->get_parameter("cy").as_double();
            calib_k1_ = this->get_parameter("k1").as_double();
            calib_k2_ = this->get_parameter("k2").as_double();
            calib_p1_ = this->get_parameter("p1").as_double();
            calib_p2_ = this->get_parameter("p2").as_double();
            calib_k3_ = this->get_parameter("k3").as_double();
            distortion_model_ = this->get_parameter("distortion_model").as_string();
        }

        // camera recovery parameters
        camera_reconnect_enable_ = this->get_parameter("camera_reconnect_enable").as_bool();
        camera_reconnect_backoff_ms_ = this->get_parameter("camera_reconnect_backoff_ms").as_int();
        camera_reconnect_max_backoff_ms_ = this->get_parameter("camera_reconnect_max_backoff_ms").as_int();
        camera_failures_before_reconnect_ = this->get_parameter("camera_failures_before_reconnect").as_int();

        
        RCLCPP_INFO(this->get_logger(), "Camera type: %s", camera_type_.c_str());
        if (camera_type_ == "standard") {
            RCLCPP_INFO(this->get_logger(), "Camera ID: %d", camera_id_);
        } else if (camera_type_ == "gmsl") {
            RCLCPP_INFO(this->get_logger(), "Device path: %s", device_path_.c_str());
            RCLCPP_INFO(this->get_logger(), "Format: %s", format_.c_str());
            RCLCPP_INFO(this->get_logger(), "Sensor mode: %d", sensor_mode_);
        }
        RCLCPP_INFO(this->get_logger(), "Model path: %s", model_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d @ %d fps", 
                    camera_width_, camera_height_, framerate_);
        if (downsample_factor_ > 1) {
            RCLCPP_INFO(this->get_logger(), "Downsampling: %dx (for faster inference)", downsample_factor_);
        }
        if (use_calibration_) {
            RCLCPP_INFO(this->get_logger(), "Using calibrated camera parameters:");
            RCLCPP_INFO(this->get_logger(), "  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                        calib_fx_, calib_fy_, calib_cx_, calib_cy_);
            RCLCPP_INFO(this->get_logger(), "  Distortion: k1=%.4f, k2=%.4f, p1=%.4f, p2=%.4f",
                        calib_k1_, calib_k2_, calib_p1_, calib_p2_);
        }
        if (enable_undistortion_) {
            RCLCPP_INFO(this->get_logger(), "Fisheye undistortion: ENABLED (balance=%.2f)", 
                        undistortion_balance_);
        }
        
        // Create publishers
        auto qos = rclcpp::SensorDataQoS().keep_last(1);
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/input/image", qos);
        depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/output/depth_image", qos);
        depth_colored_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "~/output/depth_colored", qos);
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "~/output/point_cloud", qos);
        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "~/camera_info", qos);
        
        // Initialize camera
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize fisheye undistorter (if enabled)
        if (!initUndistorter()) {
            RCLCPP_WARN(this->get_logger(), "Failed to initialize undistorter, continuing without undistortion");
            enable_undistortion_ = false;
        }
        
        // Initialize depth estimator
        if (!initDepthEstimator()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize depth estimator");
            rclcpp::shutdown();
            return;
        }
        
        // Create timer for publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&CameraDepthNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "Camera depth node started");
    }
    
    ~CameraDepthNode()
    {
        if (camera_capture_ && camera_capture_->isOpened()) {
            camera_capture_->release();
        }
    }

private:
    bool loadCalibrationFromFile(const std::string& file_path)
    {
        try {
            RCLCPP_INFO(this->get_logger(), "Loading calibration from: %s", file_path.c_str());
            
            // Check if file exists
            std::ifstream file_check(file_path);
            if (!file_check.good()) {
                RCLCPP_WARN(this->get_logger(), "Calibration file not found: %s", file_path.c_str());
                return false;
            }
            file_check.close();
            
            // Load YAML file
            YAML::Node config = YAML::LoadFile(file_path);
            
            // Read intrinsic parameters
            if (config["fx"]) calib_fx_ = config["fx"].as<double>();
            if (config["fy"]) calib_fy_ = config["fy"].as<double>();
            if (config["cx"]) calib_cx_ = config["cx"].as<double>();
            if (config["cy"]) calib_cy_ = config["cy"].as<double>();
            
            // Read distortion coefficients
            if (config["k1"]) calib_k1_ = config["k1"].as<double>();
            if (config["k2"]) calib_k2_ = config["k2"].as<double>();
            
            // Check distortion model to determine how to read coefficients
            if (config["distortion_model"]) {
                distortion_model_ = config["distortion_model"].as<std::string>();
            } else {
                distortion_model_ = "plumb_bob";  // Default for standard ROS calibration
            }
            
            if (distortion_model_ == "fisheye") {
                // Fisheye model uses k1, k2, k3, k4
                if (config["k3"]) calib_p1_ = config["k3"].as<double>();
                if (config["k4"]) calib_p2_ = config["k4"].as<double>();
                // Also try p1, p2 as fallback
                if (config["p1"] && !config["k3"]) calib_p1_ = config["p1"].as<double>();
                if (config["p2"] && !config["k4"]) calib_p2_ = config["p2"].as<double>();
                calib_k3_ = 0.0;  // Not used in fisheye
            } else {
                // Plumb_bob model uses k1, k2, p1, p2, k3
                if (config["p1"]) calib_p1_ = config["p1"].as<double>();
                if (config["p2"]) calib_p2_ = config["p2"].as<double>();
                if (config["k3"]) calib_k3_ = config["k3"].as<double>();
            }
            
            RCLCPP_INFO(this->get_logger(), "✓ Calibration loaded successfully");
            RCLCPP_INFO(this->get_logger(), "  fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", 
                        calib_fx_, calib_fy_, calib_cx_, calib_cy_);
            RCLCPP_INFO(this->get_logger(), "  Distortion model: %s", distortion_model_.c_str());
            if (distortion_model_ == "fisheye") {
                RCLCPP_INFO(this->get_logger(), "  Distortion: k1=%.6f, k2=%.6f, k3=%.6f, k4=%.6f",
                            calib_k1_, calib_k2_, calib_p1_, calib_p2_);
            } else {
                RCLCPP_INFO(this->get_logger(), "  Distortion: k1=%.4f, k2=%.4f, p1=%.4f, p2=%.4f, k3=%.4f",
                            calib_k1_, calib_k2_, calib_p1_, calib_p2_, calib_k3_);
            }
            
            return true;
        } catch (const YAML::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to load calibration file: %s", e.what());
            RCLCPP_WARN(this->get_logger(), "Will use default or parameter-based calibration");
            return false;
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Error loading calibration: %s", e.what());
            return false;
        }
    }

    bool initCamera()
    {
        RCLCPP_INFO(this->get_logger(), "Opening camera...");
        
        // Create camera configuration
        depth_anything_v3::CameraConfig config;
        config.camera_type = camera_type_;
        config.camera_id = camera_id_;
        config.device_path = device_path_;
        config.width = camera_width_;
        config.height = camera_height_;
        config.framerate = framerate_;
        config.format = format_;
        config.sensor_mode = sensor_mode_;
        
        // Create camera capture using factory
        camera_capture_ = depth_anything_v3::CameraFactory::createCamera(config);
        
        if (!camera_capture_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create camera capture instance");
            RCLCPP_ERROR(this->get_logger(), "Check camera_type parameter (must be 'standard' or 'gmsl')");
            return false;
        }
        
        // Initialize camera
        if (!camera_capture_->initialize()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera");
            return false;
        }
        
        if (!camera_capture_->isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Camera is not opened");
            return false;
        }
        
        // Get actual resolution
        frame_width_ = camera_capture_->getWidth();
        frame_height_ = camera_capture_->getHeight();
        
        RCLCPP_INFO(this->get_logger(), "✓ Camera opened successfully: %dx%d", 
                    frame_width_, frame_height_);
        
        // Create camera info
        camera_info_ = createCameraInfo(frame_width_, frame_height_);
        
        return true;
    }
    
    bool initUndistorter()
    {
        RCLCPP_INFO(this->get_logger(), "initUndistorter: enable_undistortion_=%d, use_calibration_=%d",
                    enable_undistortion_, use_calibration_);
        
        if (!enable_undistortion_) {
            RCLCPP_INFO(this->get_logger(), "Fisheye undistortion: DISABLED");
            return true;
        }
        
        if (!use_calibration_) {
            RCLCPP_WARN(this->get_logger(), "Undistortion enabled but no calibration available");
            RCLCPP_WARN(this->get_logger(), "Please provide calibration parameters via YAML file or ROS parameters");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Initializing fisheye undistorter...");
        RCLCPP_INFO(this->get_logger(), "  Image size: %dx%d", frame_width_, frame_height_);
        RCLCPP_INFO(this->get_logger(), "  Calibration: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    calib_fx_, calib_fy_, calib_cx_, calib_cy_);
        RCLCPP_INFO(this->get_logger(), "  Distortion model: %s", distortion_model_.c_str());
        RCLCPP_INFO(this->get_logger(), "  Distortion: k1=%.4f, k2=%.4f, p1=%.4f, p2=%.4f",
                    calib_k1_, calib_k2_, calib_p1_, calib_p2_);
        
        // Create undistorter configuration
        depth_anything_v3::FisheyeUndistorter::Config config;
        config.enabled = true;
        config.balance = undistortion_balance_;
        config.image_size = cv::Size(frame_width_, frame_height_);
        config.distortion_model = distortion_model_;
        
        // Create camera matrix
        config.camera_matrix = (cv::Mat_<double>(3, 3) <<
            calib_fx_, 0, calib_cx_,
            0, calib_fy_, calib_cy_,
            0, 0, 1);
        
        // Create distortion coefficients based on model
        if (distortion_model_ == "fisheye") {
            // Fisheye model uses 4 coefficients: k1, k2, k3, k4
            config.distortion_coeffs = (cv::Mat_<double>(4, 1) <<
                calib_k1_, calib_k2_, calib_p1_, calib_p2_);
        } else {
            // Standard pinhole model uses 5 coefficients: k1, k2, p1, p2, k3
            config.distortion_coeffs = (cv::Mat_<double>(5, 1) <<
                calib_k1_, calib_k2_, calib_p1_, calib_p2_, calib_k3_);
        }
        
        // Initialize undistorter
        undistorter_ = std::make_unique<depth_anything_v3::FisheyeUndistorter>();
        if (!undistorter_->initialize(config)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize fisheye undistorter");
            undistorter_.reset();
            return false;
        }
        
        // Update camera info with new camera matrix (for undistorted images)
        undistorted_camera_info_ = createUndistortedCameraInfo();
        
        RCLCPP_INFO(this->get_logger(), "✓ Fisheye undistorter initialized");
        RCLCPP_INFO(this->get_logger(), "  Undistorted size: %dx%d", 
                    undistorter_->getUndistortedSize().width,
                    undistorter_->getUndistortedSize().height);
        RCLCPP_INFO(this->get_logger(), "  New intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f",
                    undistorter_->getNewFx(), undistorter_->getNewFy(),
                    undistorter_->getNewCx(), undistorter_->getNewCy());
        
        return true;
    }
    
    sensor_msgs::msg::CameraInfo createUndistortedCameraInfo()
    {
        sensor_msgs::msg::CameraInfo camera_info;
        
        if (!undistorter_ || !undistorter_->isReady()) {
            return camera_info_;  // Return original if undistorter not ready
        }
        
        cv::Size size = undistorter_->getUndistortedSize();
        camera_info.width = size.width;
        camera_info.height = size.height;
        
        // Use new camera matrix from undistorter
        double fx = undistorter_->getNewFx();
        double fy = undistorter_->getNewFy();
        double cx = undistorter_->getNewCx();
        double cy = undistorter_->getNewCy();
        
        // Intrinsic matrix K
        camera_info.k[0] = fx;
        camera_info.k[2] = cx;
        camera_info.k[4] = fy;
        camera_info.k[5] = cy;
        camera_info.k[8] = 1.0;
        
        // Distortion coefficients are ZERO for undistorted image
        camera_info.d.resize(5, 0.0);
        
        // Rectification matrix (identity)
        camera_info.r[0] = 1.0;
        camera_info.r[4] = 1.0;
        camera_info.r[8] = 1.0;
        
        // Projection matrix
        camera_info.p[0] = fx;
        camera_info.p[2] = cx;
        camera_info.p[5] = fy;
        camera_info.p[6] = cy;
        camera_info.p[10] = 1.0;
        
        camera_info.header.frame_id = frame_id_;
        
        return camera_info;
    }
    
    bool initDepthEstimator()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing depth estimator...");
        RCLCPP_INFO(this->get_logger(), "This may take a few minutes on first run...");
        
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
            
            // Calculate actual inference resolution
            int inference_width = frame_width_ / downsample_factor_;
            int inference_height = frame_height_ / downsample_factor_;
            
            RCLCPP_INFO(this->get_logger(), "Inference resolution: %dx%d", inference_width, inference_height);
            
            depth_estimator_->initPreprocessBuffer(inference_width, inference_height);
            depth_estimator_->setSkyThreshold(0.3f);
            
            RCLCPP_INFO(this->get_logger(), "Depth estimator initialized");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error initializing depth estimator: %s", e.what());
            return false;
        }
    }
    
    sensor_msgs::msg::CameraInfo createCameraInfo(int width, int height)
    {
        sensor_msgs::msg::CameraInfo camera_info;
        
        camera_info.width = width;
        camera_info.height = height;
        
        if (use_calibration_) {
            // Use calibrated parameters, scaled for current resolution
            double scale_x = static_cast<double>(width) / 1920.0;
            double scale_y = static_cast<double>(height) / 1536.0;
            
            // Scale intrinsics
            camera_info.k[0] = calib_fx_ * scale_x;  // fx
            camera_info.k[2] = calib_cx_ * scale_x;  // cx
            camera_info.k[4] = calib_fy_ * scale_y;  // fy
            camera_info.k[5] = calib_cy_ * scale_y;  // cy
            camera_info.k[8] = 1.0;
            
            // Distortion coefficients (k1, k2, p1, p2, k3)
            camera_info.d.resize(5);
            camera_info.d[0] = calib_k1_;
            camera_info.d[1] = calib_k2_;
            camera_info.d[2] = calib_p1_;
            camera_info.d[3] = calib_p2_;
            camera_info.d[4] = calib_k3_;
            
            // Rectification matrix (identity for monocular)
            camera_info.r[0] = 1.0;
            camera_info.r[4] = 1.0;
            camera_info.r[8] = 1.0;
            
            // Projection matrix
            camera_info.p[0] = calib_fx_ * scale_x;
            camera_info.p[2] = calib_cx_ * scale_x;
            camera_info.p[5] = calib_fy_ * scale_y;
            camera_info.p[6] = calib_cy_ * scale_y;
            camera_info.p[10] = 1.0;
        } else {
            // Use estimated parameters based on 60° FOV
            const double focal_length = width / (2.0 * std::tan(60.0 * M_PI / 180.0 / 2.0));
            
            camera_info.k[0] = focal_length;
            camera_info.k[2] = width / 2.0;
            camera_info.k[4] = focal_length;
            camera_info.k[5] = height / 2.0;
            camera_info.k[8] = 1.0;
            
            camera_info.d.resize(5, 0.0);
            
            camera_info.r[0] = 1.0;
            camera_info.r[4] = 1.0;
            camera_info.r[8] = 1.0;
            
            camera_info.p[0] = focal_length;
            camera_info.p[2] = width / 2.0;
            camera_info.p[5] = focal_length;
            camera_info.p[6] = height / 2.0;
            camera_info.p[10] = 1.0;
        }
        
        camera_info.header.frame_id = frame_id_;
        
        return camera_info;
    }
    
    cv::Mat applyDepthColormap(const cv::Mat& depth_image)
    {
        cv::Mat depth_normalized, depth_8u, colored;
        
        // Find min and max depth for better contrast
        double min_depth, max_depth;
        cv::minMaxLoc(depth_image, &min_depth, &max_depth);
        
        // Use adaptive range, but limit to reasonable values
        min_depth = std::max(0.5, min_depth);  // At least 0.5m
        max_depth = std::min(20.0, max_depth);  // At most 20m
        
        // Normalize to 0-255 range
        depth_image.convertTo(depth_normalized, CV_64F);
        depth_normalized = (depth_normalized - min_depth) / (max_depth - min_depth) * 255.0;
        depth_normalized.setTo(0, depth_normalized < 0);
        depth_normalized.setTo(255, depth_normalized > 255);
        depth_normalized.convertTo(depth_8u, CV_8U);
        
        // Apply TURBO colormap (better than JET for depth)
        cv::applyColorMap(depth_8u, colored, cv::COLORMAP_TURBO);
        
        return colored;
    }
    
    void createPointCloud(
        const cv::Mat& depth_image,
        const cv::Mat& rgb_image,
        const rclcpp::Time& stamp,
        sensor_msgs::msg::PointCloud2& cloud_msg)
    {
        createPointCloudWithCameraInfo(depth_image, rgb_image, camera_info_, stamp,cloud_msg);
    }
    
    void createPointCloudWithCameraInfo(
        const cv::Mat& depth_image,
        const cv::Mat& rgb_image,
        const sensor_msgs::msg::CameraInfo& cam_info,
        const rclcpp::Time& stamp,
        sensor_msgs::msg::PointCloud2& cloud_msg)
    {
        const double fx = cam_info.k[0];
        const double fy = cam_info.k[4];
        const double cx = cam_info.k[2];
        const double cy = cam_info.k[5];
        
        const int height = depth_image.rows;
        const int width = depth_image.cols;
        const int downsample = 4;  // No downsampling for better quality
        
        // Setup PointCloud2 message
        cloud_msg.header.stamp = stamp; 
        cloud_msg.header.frame_id = frame_id_;
        cloud_msg.height = 1;
        cloud_msg.is_dense = false;
        cloud_msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        
        // Count valid points
        int valid_points = 0;
        for (int v = 0; v < height; v += downsample) {
            for (int u = 0; u < width; u += downsample) {
                float depth = depth_image.at<float>(v, u);
                if (depth > 0.0f && std::isfinite(depth) && depth < 100.0f) {
                    valid_points++;
                }
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
                float depth = depth_image.at<float>(v, u);
                
                if (depth <= 0.0f || !std::isfinite(depth) || depth > 100.0f) {
                    continue;
                }
                
                // Camera coordinate system: X=right, Y=down, Z=forward
                // Convert to ROS coordinate system for better visualization
                *iter_z = depth;                          // Z: forward (depth)
                *iter_x = -(u - cx) * depth / fx;        // X: left (negated for correct orientation)
                *iter_y = -(v - cy) * depth / fy;        // Y: up (negated for correct orientation)
                
                if (!rgb_image.empty()) {
                    cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(v, u);
                    *iter_r = rgb[0];  // RGB image, so R is at index 0
                    *iter_g = rgb[1];
                    *iter_b = rgb[2];
                } else {
                    *iter_r = 255;
                    *iter_g = 255;
                    *iter_b = 255;
                }
                
                ++iter_x;
                ++iter_y;
                ++iter_z;
                ++iter_r;
                ++iter_g;
                ++iter_b;
            }
        }
    }
    
    void timerCallback()
    {
        cv::Mat frame;

        if (!camera_capture_ || !camera_capture_->isOpened() || !camera_capture_->read(frame) || frame.empty()) {
            consecutive_read_failures_++;

            // Only start recovery after N consecutive failures (avoids reconnecting on a single glitch)
            if (consecutive_read_failures_ >= camera_failures_before_reconnect_) {
                camera_ok_ = false;
                (void)tryRecoverCamera();
            }

            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Camera read failed (%d/%d). waiting/recovering...",
                consecutive_read_failures_, camera_failures_before_reconnect_);

            return;
        }


        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        using steady_clock = std::chrono::steady_clock;
        const auto t0 = steady_clock::now();

        // Tick counter
        static uint64_t tick = 0;
        tick++;

        // Helper: ms elapsed since timepoint
        auto ms = [&](steady_clock::time_point t_start) -> double {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                    steady_clock::now() - t_start)
                .count() / 1000.0;
        };

        // One stamp for *all* published messages in this tick
        static rclcpp::Time last_stamp(0, 0, RCL_ROS_TIME);
        const rclcpp::Time stamp = this->now();
        const double dt_stamp_ms =
            last_stamp.nanoseconds() ? (stamp - last_stamp).seconds() * 1000.0 : 0.0;
        last_stamp = stamp;

        // Stage timings
        double undist_ms = 0.0, resize_ms = 0.0, cvt_ms = 0.0;
        double infer_ms = 0.0, pc_ms = 0.0, pub_ms = 0.0;

        // Apply fisheye undistortion if enabled
        cv::Mat frame_processed = frame;
        sensor_msgs::msg::CameraInfo active_camera_info = camera_info_;

        static int frame_count = 0;
        if ((frame_count++ % 100) == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "Undistortion status: enable=%d, undistorter=%s, ready=%s",
                        enable_undistortion_,
                        undistorter_ ? "yes" : "no",
                        (undistorter_ && undistorter_->isReady()) ? "yes" : "no");
        }

        if (enable_undistortion_ && undistorter_ && undistorter_->isReady()) {
            const auto t_und0 = steady_clock::now();

            cv::Mat undistorted;
            if (undistorter_->undistort(frame, undistorted)) {
                frame_processed = undistorted;
                active_camera_info = undistorted_camera_info_;
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                    "Undistortion failed, using original frame");
            }

            undist_ms = ms(t_und0);
        }

        // Downsample (for faster inference)
        cv::Mat frame_for_inference = frame_processed;
        sensor_msgs::msg::CameraInfo inference_camera_info = active_camera_info;

        if (downsample_factor_ > 1) {
            const auto t_rs0 = steady_clock::now();

            cv::Mat downsampled;
            cv::resize(frame_processed, downsampled,
                    cv::Size(frame_processed.cols / downsample_factor_,
                                frame_processed.rows / downsample_factor_),
                    0, 0, cv::INTER_LINEAR);

            frame_for_inference = downsampled;
            inference_camera_info = scaleDownCameraInfo(active_camera_info, downsample_factor_);

            resize_ms = ms(t_rs0);
        }

        // Convert to RGB
        cv::Mat frame_rgb;
        {
            const auto t_cvt0 = steady_clock::now();
            cv::cvtColor(frame_for_inference, frame_rgb, cv::COLOR_BGR2RGB);
            cvt_ms = ms(t_cvt0);
        }

        // Inference
        const cv::Mat* depth_image_ptr = nullptr;
        {
            std::vector<cv::Mat> images = {frame_rgb};
            const auto t_inf0 = steady_clock::now();
            const bool success = depth_estimator_->doInference(images, inference_camera_info, 1, false);
            infer_ms = ms(t_inf0);

            if (!success) {
                RCLCPP_ERROR(this->get_logger(), "Inference failed");
                return;
            }
            depth_image_ptr = &depth_estimator_->getDepthImage();
        }

        const cv::Mat& depth_image = *depth_image_ptr;

        // Build point cloud
        sensor_msgs::msg::PointCloud2 cloud_msg;
        {
            const auto t_pc0 = steady_clock::now();
            createPointCloudWithCameraInfo(depth_image, frame_rgb, inference_camera_info, stamp, cloud_msg);
            pc_ms = ms(t_pc0);
        }

        // Publish everything
        {
            const auto t_pub0 = steady_clock::now();

            auto input_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_for_inference).toImageMsg();
            input_msg->header.stamp = stamp;
            input_msg->header.frame_id = frame_id_;
            image_pub_->publish(*input_msg);

            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth_image).toImageMsg();
            depth_msg->header.stamp = stamp;
            depth_msg->header.frame_id = frame_id_;
            depth_pub_->publish(*depth_msg);

            cv::Mat depth_colored = applyDepthColormap(depth_image);
            auto depth_colored_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", depth_colored).toImageMsg();
            depth_colored_msg->header.stamp = stamp;
            depth_colored_msg->header.frame_id = frame_id_;
            depth_colored_pub_->publish(*depth_colored_msg);

            pointcloud_pub_->publish(cloud_msg);

            inference_camera_info.header.stamp = stamp;
            camera_info_pub_->publish(inference_camera_info);

            pub_ms = ms(t_pub0);
        }

        // Summary print (throttled)
        if (debug_timing_) {
            const double total_ms = ms(t0);
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), debug_timing_throttle_ms_,
                "[cam_depth] tick=%lu dt=%.1fms total=%.1fms infer=%.1fms pc=%.1fms pub=%.1fms "
                "(und=%.1f rs=%.1f cvt=%.1f) points=%u bytes=%zu",
                tick, dt_stamp_ms, total_ms, infer_ms, pc_ms, pub_ms,
                undist_ms, resize_ms, cvt_ms,
                cloud_msg.width, cloud_msg.data.size());
        }
    }

    bool tryRecoverCamera()
    {
        if (!camera_reconnect_enable_) {
            return false;
        }

        const auto now = std::chrono::steady_clock::now();
        if (next_reconnect_tp_.time_since_epoch().count() != 0 && now < next_reconnect_tp_) {
            return false;  // wait for backoff
        }

        // Increase backoff
        if (current_backoff_ms_ <= 0) current_backoff_ms_ = camera_reconnect_backoff_ms_;
        else current_backoff_ms_ = std::min(current_backoff_ms_ * 2, camera_reconnect_max_backoff_ms_);

        next_reconnect_tp_ = now + std::chrono::milliseconds(current_backoff_ms_);

        RCLCPP_WARN(this->get_logger(), "Camera read failing. Reconnecting in %d ms...", current_backoff_ms_);

        // Close old handle
        try {
            if (camera_capture_) {
                if (camera_capture_->isOpened()) {
                    camera_capture_->release();
                }
                camera_capture_.reset();
            }
        } catch (...) {
            // swallow
        }

        // Re-init
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Camera re-init failed (will retry).");
            camera_ok_ = false;
            return false;
        }

        // Re-init undistorter only if enabled (it depends on size + intrinsics)
        if (enable_undistortion_) {
            if (!initUndistorter()) {
                RCLCPP_WARN(this->get_logger(), "Undistorter re-init failed after camera reconnect. Disabling undistortion.");
                enable_undistortion_ = false;
            }
        }

        consecutive_read_failures_ = 0;
        camera_ok_ = true;
        current_backoff_ms_ = camera_reconnect_backoff_ms_;  // reset to base after success
        next_reconnect_tp_ = {};                             // clear

        RCLCPP_INFO(this->get_logger(), "✓ Camera reconnected successfully.");
        return true;
    }

    
    sensor_msgs::msg::CameraInfo scaleDownCameraInfo(
        const sensor_msgs::msg::CameraInfo& original, int factor)
    {
        sensor_msgs::msg::CameraInfo scaled = original;
        
        scaled.width = original.width / factor;
        scaled.height = original.height / factor;
        
        // Scale intrinsics
        double scale = 1.0 / factor;
        scaled.k[0] *= scale;  // fx
        scaled.k[2] *= scale;  // cx
        scaled.k[4] *= scale;  // fy
        scaled.k[5] *= scale;  // cy
        
        // Scale projection matrix
        scaled.p[0] *= scale;  // fx
        scaled.p[2] *= scale;  // cx
        scaled.p[5] *= scale;  // fy
        scaled.p[6] *= scale;  // cy
        
        return scaled;
    }
    
    // Parameters
    std::string camera_type_;
    int camera_id_;
    std::string device_path_;
    std::string model_path_;
    std::string frame_id_;
    int frame_width_;
    int frame_height_;
    int camera_width_;
    int camera_height_;
    int framerate_;
    std::string format_;
    int sensor_mode_;
    int downsample_factor_;  // Downsample factor for faster inference
    
    // Fisheye undistortion parameters
    bool enable_undistortion_;
    double undistortion_balance_;
    
    // Camera calibration parameters
    bool use_calibration_;
    double calib_fx_;
    double calib_fy_;
    double calib_cx_;
    double calib_cy_;
    double calib_k1_;
    double calib_k2_;
    double calib_p1_;
    double calib_p2_;
    double calib_k3_;
    std::string distortion_model_;
    
    // Camera
    std::unique_ptr<depth_anything_v3::CameraCapture> camera_capture_;
    sensor_msgs::msg::CameraInfo camera_info_;

    // Fisheye undistorter
    std::unique_ptr<depth_anything_v3::FisheyeUndistorter> undistorter_;
    sensor_msgs::msg::CameraInfo undistorted_camera_info_;
    
    // Depth estimator
    std::unique_ptr<depth_anything_v3::TensorRTDepthAnything> depth_estimator_;
    
    // ROS 2
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_colored_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

    // Debug Time measurement
    bool debug_timing_{true};
    int debug_timing_throttle_ms_{1000};

    // Camera reconnect parameters
    bool camera_reconnect_enable_{true};
    int camera_reconnect_backoff_ms_{500};
    int camera_reconnect_max_backoff_ms_{5000};
    int camera_failures_before_reconnect_{3};

    int consecutive_read_failures_{0};
    bool camera_ok_{true};

    std::chrono::steady_clock::time_point next_reconnect_tp_{};
    int current_backoff_ms_{0};


};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraDepthNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
