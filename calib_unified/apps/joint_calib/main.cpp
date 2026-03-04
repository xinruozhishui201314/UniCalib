/**
 * UniCalib — 联合标定应用 (支持5种标定任务灵活组合)
 *
 * 可选标定任务:
 *   --imu-intrin          IMU 内参 (Allan方差 + Transformer-IMU粗估)
 *   --cam-intrin          相机内参 (棋盘格 + DM-Calib粗估)
 *   --imu-lidar           IMU-LiDAR 外参 (B样条 + L2Calib粗估)
 *   --lidar-cam           LiDAR-Camera 外参 (边缘对齐 + MIAS-LCEC粗估)
 *   --cam-cam             Camera-Camera 外参 (BA + 特征匹配)
 *   --all                 全部标定 (默认)
 *
 * 两阶段 + 手动校准:
 *   --coarse              启用 AI 粗标定 (所有启用的任务)
 *   --manual              精标定后允许手动校准 (外参任务)
 *   --prefer-targetfree   无目标方法优先 (默认)
 *
 * 用法:
 *   unicalib_joint --config joint_config.yaml --all --coarse
 *   unicalib_joint --config joint_config.yaml --lidar-cam --cam-cam --manual
 *   unicalib_joint --config joint_config.yaml --imu-intrin --imu-lidar --coarse
 */
#include "unicalib/common/logger.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include "unicalib/io/yaml_io.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <iomanip>

namespace fs = std::filesystem;
using namespace ns_unicalib;

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════════════╗
 ║   UniCalib v2.0 — 多传感器联合标定系统                        ║
 ║                                                               ║
 ║   支持: IMU内参 / 相机内参 / IMU-LiDAR外参 /                  ║
 ║         LiDAR-Camera外参 / Camera-Camera外参                  ║
 ║                                                               ║
 ║   两阶段: AI粗标定 → 无目标精标定 → 手动校准                  ║
 ║   AI模型: DM-Calib / MIAS-LCEC / Transformer-IMU / L2Calib   ║
 ╚═══════════════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_joint [选项]\n\n"
        "配置:\n"
        "  --config/-c <file>     YAML 配置文件 (必选)\n\n"
        "标定任务选择 (可组合, 默认 --all):\n"
        "  --all                  所有任务\n"
        "  --imu-intrin           IMU 内参标定\n"
        "  --cam-intrin           相机内参标定\n"
        "  --imu-lidar            IMU-LiDAR 外参标定\n"
        "  --lidar-cam            LiDAR-Camera 外参标定\n"
        "  --cam-cam              Camera-Camera 外参标定\n\n"
        "标定模式:\n"
        "  --coarse               启用 AI 粗标定\n"
        "  --manual               启用手动校准 (外参任务)\n"
        "  --prefer-targetfree    优先无目标方法 (默认)\n"
        "  --no-targetfree        允许使用靶标方法\n\n"
        "AI 配置:\n"
        "  --ai-root <dir>        AI 工程根目录 (默认 ../)\n"
        "  --check-ai             只检查 AI 模型可用性\n\n"
        "日志/输出:\n"
        "  --log-level <l>        trace|debug|info|warn|error\n"
        "  --output-dir <dir>     输出目录 (默认 ./calib_output)\n\n"
        "配置文件示例: config/joint_example.yaml\n\n";
}

// 打印任务选择摘要
static void print_task_summary(CalibTaskType tasks, bool do_coarse,
                                bool do_manual, bool prefer_tf) {
    auto flag = [&](CalibTaskType t) {
        return has_task(tasks, t) ? "✓" : "✗";
    };
    std::cout << "\n  标定任务:\n";
    std::cout << "    " << flag(CalibTaskType::IMU_INTRINSIC)    << " IMU 内参\n";
    std::cout << "    " << flag(CalibTaskType::CAM_INTRINSIC)    << " 相机内参\n";
    std::cout << "    " << flag(CalibTaskType::IMU_LIDAR_EXTRIN) << " IMU-LiDAR 外参\n";
    std::cout << "    " << flag(CalibTaskType::LIDAR_CAM_EXTRIN) << " LiDAR-Camera 外参\n";
    std::cout << "    " << flag(CalibTaskType::CAM_CAM_EXTRIN)   << " Camera-Camera 外参\n";
    std::cout << "\n  选项:\n";
    std::cout << "    AI粗标定:    " << (do_coarse  ? "启用" : "禁用") << "\n";
    std::cout << "    手动校准:    " << (do_manual  ? "启用" : "禁用") << "\n";
    std::cout << "    无目标优先:  " << (prefer_tf  ? "是"   : "否")   << "\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    std::string config_file;
    std::string ai_root    = "../";
    std::string output_dir = "./calib_output/joint";
    std::string log_level  = "info";

    bool do_coarse   = false;
    bool do_manual   = false;
    bool prefer_tf   = true;
    bool check_ai    = false;

    // 任务标志
    bool do_all        = false;
    bool do_imu_intrin = false;
    bool do_cam_intrin = false;
    bool do_imu_lidar  = false;
    bool do_lidar_cam  = false;
    bool do_cam_cam    = false;
    bool any_task      = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--all")             { do_all = true;        any_task = true; }
        else if (a == "--imu-intrin")      { do_imu_intrin = true; any_task = true; }
        else if (a == "--cam-intrin")      { do_cam_intrin = true; any_task = true; }
        else if (a == "--imu-lidar")       { do_imu_lidar  = true; any_task = true; }
        else if (a == "--lidar-cam")       { do_lidar_cam  = true; any_task = true; }
        else if (a == "--cam-cam")         { do_cam_cam    = true; any_task = true; }
        else if (a == "--coarse")          do_coarse = true;
        else if (a == "--manual")          do_manual = true;
        else if (a == "--prefer-targetfree") prefer_tf = true;
        else if (a == "--no-targetfree")   prefer_tf = false;
        else if (a == "--check-ai")        check_ai  = true;
        else if (a == "--ai-root" && i+1 < argc)   ai_root = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--log-level" && i+1 < argc)  log_level = argv[++i];
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    // 默认: 所有任务
    if (!any_task) do_all = true;

    // 组合任务掩码
    CalibTaskType tasks = CalibTaskType::NONE;
    if (do_all) {
        tasks = CalibTaskType::ALL;
    } else {
        if (do_imu_intrin) tasks = tasks | CalibTaskType::IMU_INTRINSIC;
        if (do_cam_intrin) tasks = tasks | CalibTaskType::CAM_INTRINSIC;
        if (do_imu_lidar)  tasks = tasks | CalibTaskType::IMU_LIDAR_EXTRIN;
        if (do_lidar_cam)  tasks = tasks | CalibTaskType::LIDAR_CAM_EXTRIN;
        if (do_cam_cam)    tasks = tasks | CalibTaskType::CAM_CAM_EXTRIN;
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    // ─── 初始化日志 ───────────────────────────────────────────────────
    fs::create_directories(output_dir + "/logs");
    Logger::init("Joint-Calib",
                 output_dir + "/logs/joint_calib.log",
                 log_level == "debug" ? spdlog::level::debug :
                 log_level == "trace" ? spdlog::level::trace :
                 log_level == "warn"  ? spdlog::level::warn  :
                                        spdlog::level::info);

    // ─── 加载配置 ─────────────────────────────────────────────────────
    YAML::Node cfg;
    try { cfg = YAML::LoadFile(config_file); }
    catch (const std::exception& e) {
        UNICALIB_ERROR("配置加载失败: {}", e.what()); return 1;
    }

    if (cfg["output_dir"])   output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["prefer_targetfree"]) prefer_tf = cfg["prefer_targetfree"].as<bool>();
    if (cfg["ai_root"])      ai_root = cfg["ai_root"].as<std::string>();

    print_task_summary(tasks, do_coarse, do_manual, prefer_tf);

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置: {}", config_file);
    UNICALIB_INFO("输出: {}", output_dir);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─── 检查 AI 可用性 (--check-ai) ─────────────────────────────────
    if (check_ai || do_coarse) {
        UNICALIB_INFO("▶ 检查 AI 模型可用性...");
        AICoarseCalibManager::Config ai_cfg;
        ai_cfg.ai_root = ai_root;
        if (cfg["python_exe"]) ai_cfg.python_exe = cfg["python_exe"].as<std::string>();
        AICoarseCalibManager ai_mgr(ai_cfg);
        ai_mgr.print_availability();
        if (check_ai) return 0;
    }

    // ─── 构建流水线 ───────────────────────────────────────────────────
    PipelineConfig pipe_cfg;
    pipe_cfg.tasks               = tasks;
    pipe_cfg.prefer_targetfree   = prefer_tf;
    pipe_cfg.allow_manual_fallback = do_manual;
    pipe_cfg.output_dir          = output_dir;
    pipe_cfg.log_level           = log_level;
    pipe_cfg.ai_models_root      = ai_root;

    // 根据命令行 --coarse 配置各任务的粗标定开关
    pipe_cfg.enable_coarse_imu_intrin = do_coarse && has_task(tasks, CalibTaskType::IMU_INTRINSIC);
    pipe_cfg.enable_coarse_cam_intrin = do_coarse && has_task(tasks, CalibTaskType::CAM_INTRINSIC);
    pipe_cfg.enable_coarse_imu_lidar  = do_coarse && has_task(tasks, CalibTaskType::IMU_LIDAR_EXTRIN);
    pipe_cfg.enable_coarse_lidar_cam  = do_coarse && has_task(tasks, CalibTaskType::LIDAR_CAM_EXTRIN);
    pipe_cfg.enable_coarse_cam_cam    = do_coarse && has_task(tasks, CalibTaskType::CAM_CAM_EXTRIN);

    // 配置文件覆盖阈值
    if (cfg["lidar_cam_rms_threshold"])
        pipe_cfg.lidar_cam_rms_threshold = cfg["lidar_cam_rms_threshold"].as<double>();
    if (cfg["cam_cam_rms_threshold"])
        pipe_cfg.cam_cam_rms_threshold   = cfg["cam_cam_rms_threshold"].as<double>();
    if (cfg["imu_lidar_rot_threshold"])
        pipe_cfg.imu_lidar_rot_threshold = cfg["imu_lidar_rot_threshold"].as<double>();

    CalibPipeline pipeline(pipe_cfg);

    // 设置进度回调
    pipeline.set_progress_callback([](CalibStage stage,
                                       const std::string& step,
                                       double progress) {
        if (progress >= 0) {
            UNICALIB_INFO("  [{}-{}] {:.1f}%",
                          stage_name(stage), step, progress * 100.0);
        }
    });

    // ─── 运行流水线 ───────────────────────────────────────────────────
    UNICALIB_INFO("▶ 开始联合标定流水线...");
    auto report = pipeline.run();

    // ─── 打印最终摘要 ─────────────────────────────────────────────────
    std::cout << "\n";
    std::cout << "  ┌─────────────────────────────────────────────────┐\n";
    std::cout << "  │            UniCalib 联合标定完成                 │\n";
    std::cout << "  ├─────────────────────────────────────────────────┤\n";
    std::cout << "  │ Pipeline ID: " << std::left << std::setw(36)
              << report.pipeline_id << "│\n";
    std::cout << "  │ 总耗时: " << std::fixed << std::setprecision(0) << std::setw(10)
              << report.total_elapsed_ms() << " ms"
              << std::setw(29) << " " << "│\n";
    std::cout << "  │ 全部收敛: " << std::setw(38)
              << (report.all_converged() ? "是 ✓" : "否 ✗ (查看日志)") << "│\n";

    for (const auto& r : report.stage_results) {
        std::string status = r.success ? "✓" : "✗";
        if (r.needs_manual_refine()) status += " (建议手动)";
        UNICALIB_INFO("  [{}] rms={:.4f} time={:.0f}ms: {} {}",
                      stage_name(r.stage), r.residual_rms, r.elapsed_ms,
                      status, r.message);
    }

    std::cout << "  ├─────────────────────────────────────────────────┤\n";
    std::cout << "  │ 结果目录: " << std::left << std::setw(38) << output_dir << "│\n";
    std::cout << "  └─────────────────────────────────────────────────┘\n\n";

    // ─── 手动校准提示 ─────────────────────────────────────────────────
    bool any_needs_manual = false;
    for (const auto& r : report.stage_results) {
        if (r.needs_manual_refine()) { any_needs_manual = true; break; }
    }

    if (any_needs_manual) {
        UNICALIB_WARN("⚠ 部分标定精度未达标, 建议手动校准:");
        UNICALIB_WARN("  重新运行并添加 --manual 标志:");
        UNICALIB_WARN("  unicalib_joint --config {} --lidar-cam --cam-cam --manual",
                      config_file);
        UNICALIB_WARN("  或直接调用 ManualCalibSession::run_*() API");
    }

    return report.all_converged() ? 0 : 1;
}
