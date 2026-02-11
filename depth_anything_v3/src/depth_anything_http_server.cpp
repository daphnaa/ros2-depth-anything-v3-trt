// depth_anything_http_server.cpp
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>

#include <httplib.h>  

#include "depth_anything_v3/tensorrt_depth_anything.hpp"
#include "depth_anything_v3/depth_helpers.hpp"
using namespace depth_anything_helpers;


#include <unordered_map>

static std::unordered_map<std::string, std::string> parseArgs(int argc, char** argv) {
  std::unordered_map<std::string, std::string> m;
  for (int i = 1; i < argc; ++i) {
    std::string k = argv[i];
    if (k.rfind("--", 0) != 0) continue;          // must start with --
    if (i + 1 < argc && std::string(argv[i + 1]).rfind("--", 0) != 0) {
      m[k] = argv[i + 1];
      ++i;
    } else {
      m[k] = "1"; // flag
    }
  }
  return m;
}

static std::string getStr(const std::unordered_map<std::string, std::string>& a,
                          const std::string& key, const std::string& defv) {
  auto it = a.find(key);
  return (it == a.end()) ? defv : it->second;
}

static int getInt(const std::unordered_map<std::string, std::string>& a,
                  const std::string& key, int defv) {
  auto it = a.find(key);
  if (it == a.end()) return defv;
  try { return std::stoi(it->second); } catch (...) { return defv; }
}

static double getDbl(const std::unordered_map<std::string, std::string>& a,
                     const std::string& key, double defv) {
  auto it = a.find(key);
  if (it == a.end()) return defv;
  try { return std::stod(it->second); } catch (...) { return defv; }
}

static void printUsage(const char* prog) {
  std::cerr
    << "Usage:\n"
    << "  " << prog << " --model <path.onnx> --camera-yaml <camera.yaml> [options]\n"
    << "Options:\n"
    << "  --host <0.0.0.0>          (default 0.0.0.0)\n"
    << "  --port <5070>             (default 5070)\n"
    << "  --precision <fp16|fp32>   (default fp16)\n"
    << "  --min-depth <0.2>         (default 0.2)\n"
    << "  --max-depth <10.0>        (default 10.0)\n"
    << "  --hist-bins <30>          (default 30)\n"
    << "  --hist-min-frac <0.06>    (default 0.06)\n"
    << "  --save-depth <dir>        save depth colormap images under <image_dir>_depth (default: disabled)\n"
    << "  --save-every <N>          save every N-th request (default 1)\n"
    << "  --save-max-depth <m>      colormap clamp max depth (default = --max-depth)\n";
}



using json = nlohmann::json;

struct DepthService {
  int W{-1}, H{-1};
  double fx{0}, fy{0}, cx{0}, cy{0};
  sensor_msgs::msg::CameraInfo camInfo;
  std::unique_ptr<depth_anything_v3::TensorRTDepthAnything> da;

  double minDepth{0.2}, maxDepth{10.0};
  int histBins{30};
  double histMinFrac{0.06};

  std::string saveDepthDir{};
  int saveEvery{1};
  double saveMaxDepth{10.0};
  uint64_t reqCount{0};

  bool init(const std::string& model_path,
          const std::string& camera_yaml,
          const std::string& precision,
          double min_depth, double max_depth,
          int hist_bins, double hist_min_frac,
          const std::string& save_dir,
          int save_every,
          double save_max_depth)
        {
          minDepth = min_depth;
          maxDepth = max_depth;
          histBins = hist_bins;
          histMinFrac = hist_min_frac;

          saveDepthDir = save_dir;
          saveEvery = std::max(1, save_every);
          saveMaxDepth = save_max_depth;
            if (!loadIntrinsicsFromYaml(camera_yaml, W, H, fx, fy, cx, cy)) return false;
            camInfo = makeCameraInfoFromIntrinsics(W, H, fx, fy, cx, cy, "camera_link");

            tensorrt_common::BuildConfig build_config;
            build_config.clip_value = 0.0f;
            build_config.calib_type_str = "MinMax";

            da = std::make_unique<depth_anything_v3::TensorRTDepthAnything>(
              model_path,
              precision,
              build_config,
              false, "", tensorrt_common::BatchConfig{1,1,1},
              (1ULL << 30)
            );
            da->initPreprocessBuffer(W, H);
            da->setSkyThreshold(0.3f);
            return true;
          }

  json infer(const cv::Mat& bgr,
           const json* detections_or_null,
           const std::string& image_dir_param,
           const std::string& image_name)
 {
    cv::Mat bgr_resized;
    if (bgr.cols != W || bgr.rows != H) cv::resize(bgr, bgr_resized, cv::Size(W,H));
    else bgr_resized = bgr;

    cv::Mat rgb;
    cv::cvtColor(bgr_resized, rgb, cv::COLOR_BGR2RGB);

    std::vector<cv::Mat> images = { rgb };
    if (!da->doInference(images, camInfo, 1, false)) {
      return json{{"ok", false}, {"error", "doInference failed"}};
    }
    const cv::Mat& depth32f = da->getDepthImage(); // CV_32FC1 meters

    double mn, mx;
    cv::minMaxLoc(depth32f, &mn, &mx);
    cv::Scalar mean, stddev;
    cv::meanStdDev(depth32f, mean, stddev);
    std::cerr << "[DBG] depth min=" << mn << " max=" << mx
            << " mean=" << mean[0] << " std=" << stddev[0] << "\n";

    std::cerr << "[DBG] depth center=" << depth32f.at<float>(H/2, W/2)
          << " top=" << depth32f.at<float>(H/4, W/2)
          << " bottom=" << depth32f.at<float>(3*H/4, W/2) << "\n";

    std::string saved_path;

    reqCount++;
    const bool enabled = !saveDepthDir.empty();              // use --save-depth as enable flag
    const bool do_save = enabled && (reqCount % (uint64_t)saveEvery == 0);

    if (do_save && !image_dir_param.empty() && !image_name.empty()) {
      const std::string out_dir = addDepthSuffix(image_dir_param);
      ensureDir(out_dir);

      const std::string base = std::filesystem::path(image_name).stem().string();
      saved_path = out_dir + "/" + base + "_depth.jpg";

      saveDepthColormap(depth32f, saved_path, -1.0f, -1.0f);
      std::cerr << "[SAVE] " << saved_path << "\n";
    } else {
      std::cerr << "[NOT SAVED] enabled=" << enabled
                << " image_dir='" << image_dir_param << "' image_name='" << image_name << "'\n";
    }



    
    json out;
    out["ok"] = true;
    out["camera"] = {
    {"width", W}, {"height", H},
    {"fx", camInfo.k[0]}, {"fy", camInfo.k[4]},
    {"cx", camInfo.k[2]}, {"cy", camInfo.k[5]}
    };

    out["objects"] = json::array();
    if (detections_or_null) {
    json dets = parseDetectionsFromNanoowlLikeJson(*detections_or_null);

    for (auto& d : dets) {
        auto bb = d["bbox"];
        // int x1 = (int)std::round(bb[0].get<double>());
        // int y1 = (int)std::round(bb[1].get<double>());
        // int x2 = (int)std::round(bb[2].get<double>());
        // int y2 = (int)std::round(bb[3].get<double>());
        int detW = -1, detH = -1;
        if (detections_or_null) {
        if (detections_or_null->contains("nanoowl") &&
            (*detections_or_null)["nanoowl"].contains("result") &&
            (*detections_or_null)["nanoowl"]["result"].contains("image")) {
            detW = (*detections_or_null)["nanoowl"]["result"]["image"].value("width", -1);
            detH = (*detections_or_null)["nanoowl"]["result"]["image"].value("height", -1);
        }
        }
        double sx = (detW > 0) ? double(depth32f.cols) / double(detW) : 1.0;
        double sy = (detH > 0) ? double(depth32f.rows) / double(detH) : 1.0;

        // then:
        int x1 = int(std::round(bb[0].get<double>() * sx));
        int y1 = int(std::round(bb[1].get<double>() * sy));
        int x2 = int(std::round(bb[2].get<double>() * sx));
        int y2 = int(std::round(bb[3].get<double>() * sy));

        cv::Rect roi = clampRoi(x1,y1,x2,y2, depth32f.cols, depth32f.rows);
        if (roi.width <= 0 || roi.height <= 0) continue;

        double Z;
        if (!depthHistModeInRoi(depth32f, roi, minDepth, maxDepth, histBins, histMinFrac, Z)) continue;

        double u = 0.5 * (x1 + x2);
        double v = 0.5 * (y1 + y2);

        json o = d;
        json xyz = projectUvDepthToXyz(u, v, Z, camInfo);
        o["depth_m"] = xyz["depth_m"];
        o["xyz_m"]   = xyz["xyz_m"];
        out["objects"].push_back(o);
    }
    }

    // wall probes
    out["wall"] = computeWallProbesTopThird(depth32f, camInfo, minDepth, maxDepth, histBins, histMinFrac);
    // jpg path
    if (!saved_path.empty()) {
      out["depth_colorm_jpg"] = saved_path;
    }

    return out;

  }
};

int main(int argc, char** argv) {
  auto args = parseArgs(argc, argv);

  if (args.count("--help") || args.count("-h")) {
    printUsage(argv[0]);
    return 0;
  }

  const std::string host = getStr(args, "--host", "0.0.0.0");
  const int port         = getInt(args, "--port", 5070);

  const std::string model_path  = getStr(args, "--model", "");
  const std::string camera_yaml = getStr(args, "--camera-yaml", "");
  const std::string precision   = getStr(args, "--precision", "fp16");

  if (model_path.empty() || camera_yaml.empty()) {
    printUsage(argv[0]);
    std::cerr << "\nMissing required: --model and/or --camera-yaml\n";
    return 1;
  }

  // optional: pass these into infer() instead of hardcoding 0.2/10/30/0.06
  const double min_depth     = getDbl(args, "--min-depth", 0.2);
  const double max_depth     = getDbl(args, "--max-depth", 10.0);
  const int hist_bins        = getInt(args, "--hist-bins", 30);
  const double hist_min_frac = getDbl(args, "--hist-min-frac", 0.06);

  const std::string save_dir = getStr(args, "--save-depth", "");
  const int save_every       = getInt(args, "--save-every", 1);
  const double save_max_d    = getDbl(args, "--save-max-depth", max_depth);


  DepthService svc;
  if (!svc.init(model_path, camera_yaml, precision,
              min_depth, max_depth, hist_bins, hist_min_frac,
              save_dir, save_every, save_max_d))  {
    std::cerr << "Failed init. model=" << model_path << " yaml=" << camera_yaml << "\n";
    return 1;
  }

  httplib::Server app;

  app.Post("/bbox_depth", [&](const httplib::Request& req, httplib::Response& res) {
  if (!req.has_file("image")) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"missing form file 'image'"})", "application/json");
    return;
  }

  // Read the uploaded image file
  const auto file = req.get_file_value("image");
  const std::string image_name = file.filename;  

  // Optional: directory sent by client
  std::string image_dir_param;
  if (req.has_param("image_dir")) {
  image_dir_param = req.get_param_value("image_dir");
  }
  if (image_dir_param.empty() && req.has_file("image_dir")) {
    const auto idir = req.get_file_value("image_dir");
    image_dir_param = idir.content;
  }
  std::vector<uchar> buf(file.content.begin(), file.content.end());
  cv::Mat bgr = cv::imdecode(buf, cv::IMREAD_COLOR);
  if (bgr.empty()) {
    res.status = 400;
    res.set_content(R"({"ok":false,"error":"failed to decode image"})", "application/json");
    return;
  }

    json det;
    json* det_ptr = nullptr;

    bool had_detections_json = req.has_param("detections_json");
    bool had_detections_file = req.has_file("detections");

    std::string parse_error;

    if (had_detections_json) {
    try {
        det = json::parse(req.get_param_value("detections_json"));
        det_ptr = &det;
    } catch (const std::exception& e) {
        parse_error = e.what();
        det_ptr = nullptr;
    }
    } else if (had_detections_file) {
    const auto dfile = req.get_file_value("detections");
    try {
        det = json::parse(dfile.content);
        det_ptr = &det;
    } catch (const std::exception& e) {
        parse_error = e.what();
        det_ptr = nullptr;
    }
    }

    std::string image_path_param;
    if (req.has_param("image_path")) {
      image_path_param = req.get_param_value("image_path");
    }

    json out = svc.infer(bgr, det_ptr, image_dir_param, image_name);

    res.status = out.value("ok", false) ? 200 : 500;
    res.set_content(out.dump(2), "application/json");

  });

  std::cout << "Depth service listening on " << host << ":" << port << "\n"
            << "  model=" << model_path << "\n"
            << "  camera_yaml=" << camera_yaml << "\n";

  app.listen(host.c_str(), port);
  return 0;
}

