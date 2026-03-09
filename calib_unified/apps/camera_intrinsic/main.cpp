/**
 * UniCalib — 相机内参标定应用
 * 使用: unicalib_camera_intrinsic --config <config.yaml>
 *       unicalib_camera_intrinsic --images_dir <dir> --model pinhole|fisheye
 *
 * 输出:
 *   - 标定结果 YAML
 *   - 每帧检测结果图像
 *   - 重投影误差分布图
 *   - HTML 报告
 */

#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/accuracy_logger.h"
#include "unicalib/intrinsic/camera_calib.h"
#include <yaml-cpp/yaml.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>

namespace fs = std::filesystem;
using namespace ns_unicalib;

// ===================================================================
// 收集图像路径
// ===================================================================
std::vector<std::string> collect_images(const std::string& dir) {
    std::vector<std::string> paths;
    const std::vector<std::string> exts = {".png", ".jpg", ".jpeg", ".bmp", ".tiff"};
    for (const auto& entry : fs::directory_iterator(dir)) {
        std::string ext = entry.path().extension().string();
        // 转小写
        for (auto& c : ext) c = std::tolower(c);
        if (std::find(exts.begin(), exts.end(), ext) != exts.end()) {
            paths.push_back(entry.path().string());
        }
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

// ===================================================================
// 主函数
// ===================================================================
int main(int argc, char** argv) {
    std::cout << R"(
 ╔══════════════════════════════════════════════╗
 ║    UniCalib — 相机内参标定 (针孔 + 鱼眼)    ║
 ╚══════════════════════════════════════════════╝
)" << std::endl;

    UNICALIB_MAIN_TRY_BEGIN

    // --- 参数解析（先解析 output_dir 以便日志落盘）---
    std::string images_dir;
    std::string model_str = "pinhole";
    std::string output_dir = "./results";
    std::string sensor_id = "cam_0";
    CameraIntrinsicCalibrator::Config cfg;
    cfg.target.type = TargetConfig::Type::CHESSBOARD;
    cfg.target.cols = 9;
    cfg.target.rows = 6;
    cfg.target.square_size_m = 0.025;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            YAML::Node n = YAML::LoadFile(argv[++i]);
            // 统一配置: data + sensors，取第一个 camera
            if (n["data"] && n["sensors"]) {
                for (const auto& s : n["sensors"]) {
                    if (s["type"] && s["type"].as<std::string>() == "camera") {
                        sensor_id = s["id"].as<std::string>();
                        if (n["data"]["camera"] && n["data"]["camera"][sensor_id]) {
                            auto cam = n["data"]["camera"][sensor_id];
                            if (cam["images_dir"]) images_dir = cam["images_dir"].as<std::string>();
                        }
                        break;
                    }
                }
                if (n["output_dir"]) output_dir = n["output_dir"].as<std::string>();
                if (n["camera_intrinsic"]) {
                    const auto& ci = n["camera_intrinsic"];
                    if (ci["model"])        model_str = ci["model"].as<std::string>();
                    if (ci["min_images"])   cfg.min_images = ci["min_images"].as<int>();
                    if (ci["max_images"])   cfg.max_images = ci["max_images"].as<int>();
                    if (ci["target"]) {
                        auto t = ci["target"];
                        if (t["cols"])        cfg.target.cols  = t["cols"].as<int>();
                        if (t["rows"])        cfg.target.rows  = t["rows"].as<int>();
                        if (t["square_size"]) cfg.target.square_size_m = t["square_size"].as<double>();
                        if (t["type"]) {
                            std::string ts = t["type"].as<std::string>();
                            if (ts == "circles")         cfg.target.type = TargetConfig::Type::CIRCLES_GRID;
                            else if (ts == "asym_circles") cfg.target.type = TargetConfig::Type::ASYMMETRIC_CIRCLES;
                        }
                    }
                }
            } else {
                if (n["images_dir"])   images_dir  = n["images_dir"].as<std::string>();
                if (n["model"])        model_str   = n["model"].as<std::string>();
                if (n["output_dir"])   output_dir  = n["output_dir"].as<std::string>();
                if (n["sensor_id"])    sensor_id   = n["sensor_id"].as<std::string>();
                if (n["target"]) {
                    auto t = n["target"];
                    if (t["cols"])        cfg.target.cols  = t["cols"].as<int>();
                    if (t["rows"])        cfg.target.rows  = t["rows"].as<int>();
                    if (t["square_size"]) cfg.target.square_size_m = t["square_size"].as<double>();
                    if (t["type"]) {
                        std::string ts = t["type"].as<std::string>();
                        if (ts == "circles")   cfg.target.type = TargetConfig::Type::CIRCLES_GRID;
                        else if (ts == "asym_circles") cfg.target.type = TargetConfig::Type::ASYMMETRIC_CIRCLES;
                    }
                }
                if (n["min_images"]) cfg.min_images = n["min_images"].as<int>();
                if (n["max_images"]) cfg.max_images = n["max_images"].as<int>();
            }
        } else if (arg == "--images_dir" && i + 1 < argc) {
            images_dir = argv[++i];
        } else if (arg == "--model" && i + 1 < argc) {
            model_str = argv[++i];
        } else if (arg == "--output_dir" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "--sensor_id" && i + 1 < argc) {
            sensor_id = argv[++i];
        } else if (arg == "--cols" && i + 1 < argc) {
            cfg.target.cols = std::stoi(argv[++i]);
        } else if (arg == "--rows" && i + 1 < argc) {
            cfg.target.rows = std::stoi(argv[++i]);
        } else if (arg == "--sq_size" && i + 1 < argc) {
            cfg.target.square_size_m = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: unicalib_camera_intrinsic [options]\n"
                      << "  --config <yaml>        配置文件\n"
                      << "  --images_dir <dir>     棋盘格图像目录\n"
                      << "  --model pinhole|fisheye 相机模型\n"
                      << "  --sensor_id <id>       传感器ID\n"
                      << "  --output_dir <dir>     输出目录\n"
                      << "  --cols <n>             标定板列数 (default: 9)\n"
                      << "  --rows <n>             标定板行数 (default: 6)\n"
                      << "  --sq_size <m>          方格尺寸[m] (default: 0.025)\n";
            return 0;
        }
    }

    if (images_dir.empty()) {
        ns_unicalib::Logger::init("UniCalib-Camera-Intrinsic");
        UNICALIB_ERROR("No images directory specified. Use --images_dir or --config");
        return 1;
    }

    // 运行日志写入 output_dir/logs，便于追溯
    fs::create_directories(output_dir);
    std::string logs_dir = ns_unicalib::resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/camera_intrinsic_" + ns_unicalib::log_timestamp_filename() + ".log";
    ns_unicalib::Logger::init("UniCalib-Camera-Intrinsic", log_file, spdlog::level::info);
    UNICALIB_INFO("日志文件: {}", log_file);

    // 设置模型
    if (model_str == "fisheye") {
        cfg.model = CameraIntrinsics::Model::FISHEYE;
    } else {
        cfg.model = CameraIntrinsics::Model::PINHOLE;
    }

    fs::create_directories(output_dir);

    // 收集图像
    auto image_paths = collect_images(images_dir);
    UNICALIB_INFO("Found {} images in {}", image_paths.size(), images_dir);

    if (image_paths.empty()) {
        UNICALIB_ERROR("No images found in: {}", images_dir);
        return 1;
    }

    // 标定
    auto t_calib_start = std::chrono::high_resolution_clock::now();
    CameraIntrinsicCalibrator calibrator(cfg);
    calibrator.set_progress_callback([](int cur, int total) {
        if (cur % 10 == 0 || cur == total - 1) {
            std::cout << "\r  检测角点: " << cur << "/" << total << "  " << std::flush;
        }
    });

    auto result = calibrator.calibrate(image_paths);
    std::cout << std::endl;
    auto t_calib_end = std::chrono::high_resolution_clock::now();
    double elapsed_ms = std::chrono::duration<double, std::milli>(t_calib_end - t_calib_start).count();

    if (!result.has_value()) {
        append_calib_accuracy(output_dir, CalibAccuracyTask::CAM_INTRINSIC,
            false, -1.0, elapsed_ms, {});
        UNICALIB_ERROR("Camera intrinsic calibration FAILED");
        return 1;
    }

    const auto& intrin = result.value();

    // 保存 YAML
    YAML::Node out;
    out["sensor_id"] = sensor_id;
    out["model"]     = model_str;
    out["width"]     = intrin.width;
    out["height"]    = intrin.height;
    out["fx"] = intrin.fx;
    out["fy"] = intrin.fy;
    out["cx"] = intrin.cx;
    out["cy"] = intrin.cy;
    out["dist_coeffs"] = intrin.dist_coeffs;
    out["rms_reproj_error"] = intrin.rms_reproj_error;
    out["num_images"] = intrin.num_images_used;

    std::string yaml_path = output_dir + "/camera_intrinsic_" + sensor_id + ".yaml";
    {
        std::ofstream f(yaml_path);
        if (!f.is_open()) {
            UNICALIB_ERROR("无法写入结果文件: {}", yaml_path);
        } else {
            f << out;
            UNICALIB_INFO("Saved: {}", yaml_path);
        }
    }

    // 精度记录到 CSV，便于绘制曲线
    std::map<std::string, std::string> extra;
    extra["num_images"] = std::to_string(intrin.num_images_used);
    append_calib_accuracy(output_dir, CalibAccuracyTask::CAM_INTRINSIC,
        true, intrin.rms_reproj_error, elapsed_ms, extra);

    // 打印摘要
    std::cout << "\n┌────────────────────────────────────────────────┐\n";
    std::cout << "│           相机内参标定结果                      │\n";
    std::cout << "├──────────────────────┬──────────────────────────┤\n";
    std::cout << "│ 传感器 ID            │ " << std::setw(24) << sensor_id << " │\n";
    std::cout << "│ 相机模型             │ " << std::setw(24) << model_str << " │\n";
    std::cout << "│ 分辨率               │ " << std::setw(24)
              << (std::to_string(intrin.width) + "×" + std::to_string(intrin.height)) << " │\n";
    std::cout << "│ 有效图像帧           │ " << std::setw(24) << intrin.num_images_used << " │\n";
    std::cout << "│ 重投影误差 (px)      │ " << std::setw(24) << std::fixed
              << std::setprecision(4) << intrin.rms_reproj_error << " │\n";
    std::cout << "├──────────────────────┼──────────────────────────┤\n";
    std::cout << "│ fx (px)              │ " << std::setw(24) << std::fixed
              << std::setprecision(2) << intrin.fx << " │\n";
    std::cout << "│ fy (px)              │ " << std::setw(24) << intrin.fy << " │\n";
    std::cout << "│ cx (px)              │ " << std::setw(24) << intrin.cx << " │\n";
    std::cout << "│ cy (px)              │ " << std::setw(24) << intrin.cy << " │\n";
    if (!intrin.dist_coeffs.empty()) {
        std::cout << "│ k1                   │ " << std::setw(24) << std::scientific
                  << std::setprecision(4) << intrin.dist_coeffs[0] << " │\n";
        if (intrin.dist_coeffs.size() > 1)
            std::cout << "│ k2                   │ " << std::setw(24) << intrin.dist_coeffs[1] << " │\n";
    }
    std::cout << "└──────────────────────┴──────────────────────────┘\n";
    std::cout << "\n结果保存至: " << yaml_path << "\n\n";

    UNICALIB_MAIN_TRY_END(0)
}
