/**
 * UniCalib — LiDAR-Camera 外参标定应用
 *
 * 两阶段流程:
 *   Stage 1 粗标定: MIAS-LCEC AI 模型 (可选, --coarse)
 *   Stage 2 精标定: 无目标边缘对齐 (默认) / 棋盘格 / B样条运动法
 *   手动校准:       --manual 标志触发 6-DOF 交互式调整
 *
 * 用法:
 *   unicalib_lidar_camera --config <config.yaml>
 *   unicalib_lidar_camera --config <config.yaml> --coarse --ai-root /path/to/ai
 *   unicalib_lidar_camera --config <config.yaml> --manual
 *   unicalib_lidar_camera --config <config.yaml> --method edge|target|motion
 */
#include "unicalib/common/logger.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/io/yaml_io.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;
using namespace ns_unicalib;

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — LiDAR-Camera 外参标定                    ║
 ║   两阶段: AI粗标定(MIAS-LCEC) → 无目标边缘精标定      ║
 ║   手动校准: --manual 触发 6-DOF 交互式调整             ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_lidar_camera [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --coarse                启用 AI 粗标定 (MIAS-LCEC)\n"
        "  --ai-root <dir>         AI 工程根目录 (默认: ../)\n"
        "  --manual                精标定后启用手动校准\n"
        "  --method <m>            精标定方法: edge(默认)|target|motion\n"
        "  --no-targetfree         禁用无目标优先 (允许使用棋盘格)\n"
        "  --log-level <l>         日志级别: trace|debug|info|warn|error\n"
        "  --output-dir <dir>      输出目录 (默认: ./calib_output)\n"
        "  --help/-h               显示此帮助\n\n"
        "配置文件字段 (calib_lidar_camera.yaml):\n"
        "  lidar_data_dir:         PCD 文件目录\n"
        "  camera_images_dir:      图像目录\n"
        "  camera_intrinsic_file:  相机内参 YAML\n"
        "  method:                 edge|target|motion\n"
        "  board_cols/rows:        棋盘格尺寸 (target 方法)\n"
        "  square_size_m:          棋盘格格子大小 [m]\n"
        "  edge_canny_low/high:    Canny 边缘检测阈值\n"
        "  coarse_rms_threshold:   粗标定通过阈值 [px]\n"
        "  fine_rms_threshold:     精标定通过阈值 [px]\n"
        "  manual_rms_threshold:   触发手动校准的阈值 [px] (默认 2.0)\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    // ─────────────────────────────────────────────────────────────────
    // 解析命令行
    // ─────────────────────────────────────────────────────────────────
    std::string config_file, method_str = "edge";
    std::string ai_root = "../";
    std::string output_dir = "./calib_output/lidar_camera";
    std::string log_level  = "info";
    bool do_coarse   = false;
    bool do_manual   = false;
    bool prefer_tf   = true;   // prefer target-free

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--coarse")                 do_coarse = true;
        else if (a == "--manual")                 do_manual = true;
        else if (a == "--no-targetfree")          prefer_tf = false;
        else if (a == "--method" && i+1 < argc)   method_str = argv[++i];
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

    // ─────────────────────────────────────────────────────────────────
    // 初始化日志
    // ─────────────────────────────────────────────────────────────────
    fs::create_directories(output_dir + "/logs");
    Logger::init("LiDAR-Camera",
                 output_dir + "/logs/lidar_camera_calib.log",
                 log_level == "debug"   ? spdlog::level::debug   :
                 log_level == "trace"   ? spdlog::level::trace   :
                 log_level == "warn"    ? spdlog::level::warn    :
                 log_level == "error"   ? spdlog::level::err     :
                                          spdlog::level::info);

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置文件: {}", config_file);
    UNICALIB_INFO("输出目录: {}", output_dir);
    UNICALIB_INFO("精标定方法: {} (无目标优先={})", method_str, prefer_tf);
    UNICALIB_INFO("AI粗标定: {} | 手动校准: {}", do_coarse, do_manual);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─────────────────────────────────────────────────────────────────
    // 加载配置
    // ─────────────────────────────────────────────────────────────────
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_file);
    } catch (const std::exception& e) {
        UNICALIB_ERROR("配置文件加载失败: {}", e.what());
        return 1;
    }

    // 从配置文件覆盖命令行默认值
    if (cfg["method"])      method_str = cfg["method"].as<std::string>();
    if (cfg["output_dir"])  output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["prefer_targetfree"]) prefer_tf = cfg["prefer_targetfree"].as<bool>();

    double manual_rms_thresh = 2.0;
    if (cfg["manual_rms_threshold"]) manual_rms_thresh = cfg["manual_rms_threshold"].as<double>();

    // ─────────────────────────────────────────────────────────────────
    // 构建流水线配置
    // ─────────────────────────────────────────────────────────────────
    PipelineConfig pipe_cfg;
    pipe_cfg.tasks               = CalibTaskType::LIDAR_CAM_EXTRIN;
    pipe_cfg.enable_coarse_lidar_cam = do_coarse;
    pipe_cfg.allow_manual_fallback   = do_manual;
    pipe_cfg.prefer_targetfree       = prefer_tf;
    pipe_cfg.lidar_cam_rms_threshold = manual_rms_thresh;
    pipe_cfg.output_dir              = output_dir;
    pipe_cfg.ai_models_root          = ai_root;
    pipe_cfg.log_level               = log_level;

    CalibPipeline pipeline(pipe_cfg);

    // ─────────────────────────────────────────────────────────────────
    // Stage 1: AI 粗标定 (可选)
    // ─────────────────────────────────────────────────────────────────
    std::optional<Sophus::SE3d> coarse_init;
    if (do_coarse) {
        UNICALIB_INFO("▶ Stage 1: AI 粗标定 (MIAS-LCEC)");

        AICoarseCalibManager::Config ai_cfg;
        ai_cfg.ai_root    = ai_root;
        ai_cfg.python_exe = cfg["python_exe"] ?
            cfg["python_exe"].as<std::string>() : "python3";

        AICoarseCalibManager ai_mgr(ai_cfg);
        ai_mgr.print_availability();

        std::string pcd_file   = cfg["coarse_pcd_file"] ?
            cfg["coarse_pcd_file"].as<std::string>() : "";
        std::string image_file = cfg["coarse_image_file"] ?
            cfg["coarse_image_file"].as<std::string>() : "";

        // 内参 (从文件加载)
        CameraIntrinsics cam_intrin;
        if (cfg["camera_intrinsic_file"]) {
            try {
                auto intrin_node = YAML::LoadFile(
                    cfg["camera_intrinsic_file"].as<std::string>());
                cam_intrin.fx = intrin_node["fx"].as<double>();
                cam_intrin.fy = intrin_node["fy"].as<double>();
                cam_intrin.cx = intrin_node["cx"].as<double>();
                cam_intrin.cy = intrin_node["cy"].as<double>();
                cam_intrin.width  = intrin_node["width"].as<int>();
                cam_intrin.height = intrin_node["height"].as<int>();
                UNICALIB_INFO("  内参: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}",
                              cam_intrin.fx, cam_intrin.fy,
                              cam_intrin.cx, cam_intrin.cy);
            } catch (...) {
                UNICALIB_WARN("  内参文件加载失败, 使用默认值");
            }
        }

        if (!pcd_file.empty() && !image_file.empty()) {
            auto coarse_result = ai_mgr.coarse_lidar_cam(
                pcd_file, image_file, cam_intrin);
            if (coarse_result.has_value()) {
                coarse_init = coarse_result->SE3_TargetInRef();
                UNICALIB_INFO("  ✓ AI粗标定成功: t=[{:.3f},{:.3f},{:.3f}]m",
                              coarse_init->translation().x(),
                              coarse_init->translation().y(),
                              coarse_init->translation().z());
            } else {
                UNICALIB_WARN("  ✗ AI粗标定失败, 使用 identity 初始化");
            }
        } else {
            UNICALIB_WARN("  未提供 coarse_pcd_file/coarse_image_file, 跳过AI粗标定");
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // Stage 2: 精标定
    // ─────────────────────────────────────────────────────────────────
    UNICALIB_INFO("▶ Stage 2: 精标定 (方法={})", method_str);

    LiDARCameraCalibrator::Config calib_cfg;
    if      (method_str == "edge")   calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
    else if (method_str == "target") calib_cfg.method = LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
    else if (method_str == "motion") calib_cfg.method = LiDARCameraCalibrator::Method::MOTION_BSPLINE;
    else {
        UNICALIB_WARN("未知方法 '{}', 使用 edge", method_str);
        calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
    }

    if (cfg["board_cols"])    calib_cfg.board_cols = cfg["board_cols"].as<int>();
    if (cfg["board_rows"])    calib_cfg.board_rows = cfg["board_rows"].as<int>();
    if (cfg["square_size_m"]) calib_cfg.square_size_m = cfg["square_size_m"].as<double>();
    if (cfg["edge_canny_low"])  calib_cfg.edge_canny_low  = cfg["edge_canny_low"].as<int>();
    if (cfg["edge_canny_high"]) calib_cfg.edge_canny_high = cfg["edge_canny_high"].as<int>();
    if (cfg["ceres_max_iter"])  calib_cfg.ceres_max_iter  = cfg["ceres_max_iter"].as<int>();
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");

    LiDARCameraCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& step, double prog) {
        if (prog >= 0)
            UNICALIB_INFO("  [进度] {}: {:.1f}%", step, prog * 100.0);
    });

    UNICALIB_INFO("  LiDAR数据目录: {}",
                  cfg["lidar_data_dir"] ? cfg["lidar_data_dir"].as<std::string>() : "(未设置)");
    UNICALIB_INFO("  图像目录: {}",
                  cfg["camera_images_dir"] ? cfg["camera_images_dir"].as<std::string>() : "(未设置)");
    UNICALIB_INFO("  注意: 实际数据加载需在此集成 PCL/OpenCV 数据读取器");

    // ─────────────────────────────────────────────────────────────────
    // Stage 3: 手动校准 (可选, 当精标定质量不足时)
    // ─────────────────────────────────────────────────────────────────
    if (do_manual) {
        UNICALIB_INFO("▶ Stage 3: 手动校准 (manual_threshold={:.2f}px)",
                      manual_rms_thresh);
        UNICALIB_INFO("  使用 ManualCalibSession 进行 6-DOF 交互式调整");
        UNICALIB_INFO("  调用示例:");
        UNICALIB_INFO("    ManualCalibSession::SessionConfig sess_cfg;");
        UNICALIB_INFO("    sess_cfg.save_dir = \"{}/manual_sessions\";", output_dir);
        UNICALIB_INFO("    ManualCalibSession session(sess_cfg);");
        UNICALIB_INFO("    auto result = session.run_lidar_cam(scan, image, intrin, auto_result, auto_rms);");
    }

    // ─────────────────────────────────────────────────────────────────
    // 运行流水线日志摘要
    // ─────────────────────────────────────────────────────────────────
    auto report = pipeline.run();
    report.print_summary();

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("LiDAR-Camera 标定流程完成");
    UNICALIB_INFO("结果目录: {}", output_dir);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    return 0;
}
