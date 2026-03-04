/**
 * UniCalib — IMU-LiDAR 外参标定应用
 *
 * 两阶段流程:
 *   Stage 1 粗标定: L2Calib RL强化学习 (--coarse --bag)
 *   Stage 2 精标定: B样条连续时间优化 (无目标, 始终)
 *   手动校准:       --manual 触发旋转可视化验证 + 增量调整
 *
 * 用法:
 *   unicalib_imu_lidar --config <config.yaml>
 *   unicalib_imu_lidar --config <config.yaml> --coarse --bag <bag_file>
 *   unicalib_imu_lidar --config <config.yaml> --manual
 */
#include "unicalib/common/logger.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>

namespace fs = std::filesystem;
using namespace ns_unicalib;

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — IMU-LiDAR 外参标定                       ║
 ║   两阶段: L2Calib RL粗估 → B样条连续时间精化           ║
 ║   手动校准: --manual 触发旋转可视化验证                 ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_imu_lidar [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --coarse                启用 L2Calib AI 粗标定\n"
        "  --bag <file>            ROS bag 文件 (L2Calib 需要)\n"
        "  --ai-root <dir>         AI 工程根目录\n"
        "  --manual                精标定后启用手动校准\n"
        "  --log-level <l>         trace|debug|info|warn|error\n"
        "  --output-dir <dir>      输出目录\n"
        "  --help/-h               显示帮助\n\n"
        "配置文件字段:\n"
        "  imu_data_file:          IMU 数据 (CSV/YAML)\n"
        "  lidar_data_dir:         LiDAR PCD 目录\n"
        "  imu_id:                 IMU 传感器 ID (默认 imu_0)\n"
        "  lidar_id:               LiDAR 传感器 ID (默认 lidar_0)\n"
        "  ndt_resolution:         NDT 分辨率 [m] (默认 1.0)\n"
        "  spline_dt_s:            B样条结间距 [s] (默认 0.1)\n"
        "  optimize_time_offset:   是否优化时间偏移 (默认 true)\n"
        "  manual_rot_threshold:   触发手动校准的旋转误差 [deg] (默认 0.5)\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    std::string config_file, bag_file;
    std::string ai_root    = "../";
    std::string output_dir = "./calib_output/imu_lidar";
    std::string log_level  = "info";
    bool do_coarse = false;
    bool do_manual = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--coarse")                 do_coarse = true;
        else if (a == "--manual")                 do_manual = true;
        else if (a == "--bag" && i+1 < argc)      bag_file = argv[++i];
        else if (a == "--ai-root" && i+1 < argc)  ai_root = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--log-level" && i+1 < argc) log_level = argv[++i];
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    fs::create_directories(output_dir + "/logs");
    Logger::init("IMU-LiDAR",
                 output_dir + "/logs/imu_lidar_calib.log",
                 log_level == "debug" ? spdlog::level::debug :
                 log_level == "trace" ? spdlog::level::trace :
                 log_level == "warn"  ? spdlog::level::warn  :
                                        spdlog::level::info);

    YAML::Node cfg;
    try { cfg = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
        UNICALIB_ERROR("配置加载失败: {}", e.what()); return 1;
    }

    if (cfg["output_dir"]) output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["bag_file"] && bag_file.empty()) bag_file = cfg["bag_file"].as<std::string>();

    double manual_thresh = cfg["manual_rot_threshold"] ?
        cfg["manual_rot_threshold"].as<double>() : 0.5;

    std::string imu_id   = cfg["imu_id"]   ? cfg["imu_id"].as<std::string>()   : "imu_0";
    std::string lidar_id = cfg["lidar_id"] ? cfg["lidar_id"].as<std::string>() : "lidar_0";

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置: {} | IMU: {} | LiDAR: {}", config_file, imu_id, lidar_id);
    UNICALIB_INFO("AI粗标定: {} | 手动校准: {} (阈值={:.2f}deg)",
                  do_coarse, do_manual, manual_thresh);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─── Stage 1: L2Calib 粗标定 ──────────────────────────────────────
    std::optional<Sophus::SE3d> coarse_init;
    if (do_coarse) {
        UNICALIB_INFO("▶ Stage 1: AI 粗标定 (L2Calib SE(3)-流形RL)");

        if (bag_file.empty()) {
            UNICALIB_WARN("  --coarse 需要 --bag <bag_file> 或 config.yaml::bag_file");
        } else {
            AICoarseCalibManager::Config ai_cfg;
            ai_cfg.ai_root = ai_root;
            if (cfg["python_exe"])
                ai_cfg.python_exe = cfg["python_exe"].as<std::string>();
            if (cfg["l2calib_epochs"])
                ai_cfg.l2calib.num_epochs = cfg["l2calib_epochs"].as<int>();
            if (cfg["imu_topic"])
                ai_cfg.l2calib.imu_topic = cfg["imu_topic"].as<std::string>();
            if (cfg["lidar_type"])
                ai_cfg.l2calib.lidar_type = cfg["lidar_type"].as<std::string>();

            AICoarseCalibManager ai_mgr(ai_cfg);
            ai_mgr.print_availability();

            auto result = ai_mgr.coarse_imu_lidar(bag_file, imu_id, lidar_id);
            if (result.has_value()) {
                coarse_init = result->SE3_TargetInRef();
                UNICALIB_INFO("  ✓ L2Calib 粗估: t=[{:.3f},{:.3f},{:.3f}]m",
                              coarse_init->translation().x(),
                              coarse_init->translation().y(),
                              coarse_init->translation().z());
            } else {
                UNICALIB_WARN("  ✗ L2Calib 粗标定失败, 使用手眼标定初始化");
            }
        }
    }

    // ─── Stage 2: B样条精标定 ─────────────────────────────────────────
    UNICALIB_INFO("▶ Stage 2: B样条连续时间精标定");

    IMULiDARCalibrator::Config calib_cfg;
    if (cfg["ndt_resolution"])    calib_cfg.ndt_resolution    = cfg["ndt_resolution"].as<double>();
    if (cfg["spline_dt_s"])       calib_cfg.spline_dt_s       = cfg["spline_dt_s"].as<double>();
    if (cfg["ceres_max_iter"])    calib_cfg.ceres_max_iter     = cfg["ceres_max_iter"].as<int>();
    if (cfg["optimize_time_offset"]) calib_cfg.optimize_time_offset = cfg["optimize_time_offset"].as<bool>();
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");

    IMULiDARCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& stage, double prog) {
        UNICALIB_INFO("  [进度] {}: {:.1f}%", stage, prog * 100.0);
    });

    UNICALIB_INFO("  IMU数据: {}",
                  cfg["imu_data_file"] ? cfg["imu_data_file"].as<std::string>() : "(未设置)");
    UNICALIB_INFO("  LiDAR目录: {}",
                  cfg["lidar_data_dir"] ? cfg["lidar_data_dir"].as<std::string>() : "(未设置)");
    UNICALIB_INFO("  B样条间距: {}s | 时间偏移优化: {}",
                  calib_cfg.spline_dt_s, calib_cfg.optimize_time_offset);

    // ─── Stage 3: 手动校准 ────────────────────────────────────────────
    if (do_manual) {
        UNICALIB_INFO("▶ Stage 3 (手动校准): 旋转可视化验证 + 增量调整");
        UNICALIB_INFO("  阈值: {:.2f}deg | 使用 ManualCalibSession::run_imu_lidar()",
                      manual_thresh);
        UNICALIB_INFO("  可视化: IMU积分轨迹 vs LiDAR里程计对比");
    }

    // 流水线日志
    PipelineConfig pipe_cfg;
    pipe_cfg.tasks               = CalibTaskType::IMU_LIDAR_EXTRIN;
    pipe_cfg.enable_coarse_imu_lidar = do_coarse;
    pipe_cfg.allow_manual_fallback   = do_manual;
    pipe_cfg.imu_lidar_rot_threshold = manual_thresh;
    pipe_cfg.output_dir              = output_dir;
    pipe_cfg.log_level               = log_level;

    CalibPipeline pipeline(pipe_cfg);
    auto report = pipeline.run();
    report.print_summary();

    UNICALIB_INFO("IMU-LiDAR 标定完成 | 结果: {}", output_dir);
    return 0;
}
