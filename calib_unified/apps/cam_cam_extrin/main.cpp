/**
 * UniCalib — Camera-Camera 外参标定应用
 *
 * 两阶段流程:
 *   Stage 1 粗标定: 特征匹配 + 本质矩阵初始化
 *   Stage 2 精标定: Bundle Adjustment (无目标) / 棋盘格立体标定
 *   手动校准:       --manual 触发 click-calib 风格双目对应点精化
 *
 * 用法:
 *   unicalib_cam_cam --config <config.yaml>
 *   unicalib_cam_cam --config <config.yaml> --method ba|stereo|essential
 *   unicalib_cam_cam --config <config.yaml> --manual
 */
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
using namespace ns_unicalib;

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — Camera-Camera 外参标定                   ║
 ║   两阶段: 特征匹配初始化 → BA精化(无目标优先)          ║
 ║   手动校准: --manual 触发 click-calib 对应点精化       ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_cam_cam [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --method <m>            精标定方法: ba(默认)|stereo|essential\n"
        "  --manual                精标定后启用手动校准\n"
        "  --no-targetfree         禁用无目标优先\n"
        "  --log-level <l>         trace|debug|info|warn|error\n"
        "  --output-dir <dir>      输出目录\n"
        "  --help/-h               显示帮助\n\n"
        "配置文件字段:\n"
        "  cam0_images_dir:        相机0 图像目录\n"
        "  cam1_images_dir:        相机1 图像目录\n"
        "  cam0_intrinsic_file:    相机0 内参\n"
        "  cam1_intrinsic_file:    相机1 内参\n"
        "  method:                 ba|stereo|essential\n"
        "  feature_type:           orb|sift\n"
        "  num_features:           特征点数量 (默认 2000)\n"
        "  manual_rms_threshold:   触发手动校准的重投影误差 [px] (默认 1.5)\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    std::string config_file, method_str = "ba";
    std::string output_dir = "./calib_output/cam_cam";
    std::string log_level  = "info";
    bool do_manual   = false;
    bool prefer_tf   = true;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--manual")                 do_manual = true;
        else if (a == "--no-targetfree")          prefer_tf = false;
        else if (a == "--method" && i+1 < argc)   method_str = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--log-level" && i+1 < argc) log_level = argv[++i];
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    UNICALIB_MAIN_TRY_BEGIN

    std::string logs_dir = resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/cam_cam_" + log_timestamp_filename() + ".log";
    Logger::init("Cam-Cam",
                 log_file,
                 log_level == "debug" ? spdlog::level::debug :
                 log_level == "trace" ? spdlog::level::trace :
                 log_level == "warn"  ? spdlog::level::warn  :
                                        spdlog::level::info);

    YAML::Node cfg;
    try { cfg = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
        UNICALIB_ERROR("配置文件加载失败: {}", e.what()); return 1;
    }

    if (cfg["method"]) method_str = cfg["method"].as<std::string>();
    if (cfg["output_dir"]) output_dir = cfg["output_dir"].as<std::string>();
    double manual_thresh = cfg["manual_rms_threshold"] ?
        cfg["manual_rms_threshold"].as<double>() : 1.5;

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置: {}", config_file);
    UNICALIB_INFO("精标定方法: {} | 无目标优先: {}", method_str, prefer_tf);
    UNICALIB_INFO("手动校准: {} (阈值={:.2f}px)", do_manual, manual_thresh);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─── 构建标定器配置 ───────────────────────────────────────────────
    CamCamCalibrator::Config calib_cfg;
    if      (method_str == "ba")        calib_cfg.method = CamCamCalibrator::Method::BUNDLE_ADJUSTMENT;
    else if (method_str == "stereo")    calib_cfg.method = CamCamCalibrator::Method::CHESSBOARD_STEREO;
    else if (method_str == "essential") calib_cfg.method = CamCamCalibrator::Method::ESSENTIAL_MATRIX;
    else {
        UNICALIB_WARN("未知方法 '{}', 使用 ba", method_str);
        calib_cfg.method = CamCamCalibrator::Method::BUNDLE_ADJUSTMENT;
    }

    if (cfg["num_features"])  calib_cfg.num_features = cfg["num_features"].as<int>();
    if (cfg["match_ratio"])   calib_cfg.match_ratio  = cfg["match_ratio"].as<double>();
    if (cfg["ba_max_iter"])   calib_cfg.ba_max_iter  = cfg["ba_max_iter"].as<int>();
    if (cfg["max_rms_px"])    calib_cfg.max_rms_px   = cfg["max_rms_px"].as<double>();
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");

    if (cfg["feature_type"]) {
        auto ft = cfg["feature_type"].as<std::string>();
        if (ft == "sift") calib_cfg.feature_type = CamCamCalibrator::Config::FeatureType::SIFT;
        else              calib_cfg.feature_type = CamCamCalibrator::Config::FeatureType::ORB;
    }

    CamCamCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& step, double prog) {
        if (prog >= 0)
            UNICALIB_INFO("  [进度] {}: {:.1f}%", step, prog * 100.0);
    });

    // ─── 流水线日志 ───────────────────────────────────────────────────
    PipelineConfig pipe_cfg;
    pipe_cfg.tasks             = CalibTaskType::CAM_CAM_EXTRIN;
    pipe_cfg.prefer_targetfree = prefer_tf;
    pipe_cfg.allow_manual_fallback   = do_manual;
    pipe_cfg.cam_cam_rms_threshold   = manual_thresh;
    pipe_cfg.enable_coarse_cam_cam   = false;  // 无专用 AI 模型, 用特征匹配
    pipe_cfg.output_dir        = output_dir;
    pipe_cfg.log_level         = log_level;

    CalibPipeline pipeline(pipe_cfg);

    UNICALIB_INFO("▶ Stage 1 (粗标定): 特征点匹配 + 本质矩阵");
    UNICALIB_INFO("  相机0 图像: {}",
                  cfg["cam0_images_dir"] ? cfg["cam0_images_dir"].as<std::string>() : "(未设置)");
    UNICALIB_INFO("  相机1 图像: {}",
                  cfg["cam1_images_dir"] ? cfg["cam1_images_dir"].as<std::string>() : "(未设置)");

    UNICALIB_INFO("▶ Stage 2 (精标定): {} (无目标={})", method_str, prefer_tf);

    if (do_manual) {
        UNICALIB_INFO("▶ Stage 3 (手动校准): click-calib 双目对应点精化");
        UNICALIB_INFO("  阈值: {:.2f}px 超过则提示手动校准", manual_thresh);
        UNICALIB_INFO("  使用 ManualClickRefiner::refine_cam_cam() 进行精化");
    }

    auto report = pipeline.run();
    report.print_summary();

    UNICALIB_INFO("Cam-Cam 标定完成 | 结果: {}", output_dir);
    UNICALIB_MAIN_TRY_END(0)
}
