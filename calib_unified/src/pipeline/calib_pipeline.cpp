/**
 * UniCalib Unified — CalibPipeline 实现
 *
 * 核心流程:
 *   1. 为每个 stage 创建独立日志文件 (output_dir/logs/<stage>_<task>_<ts>.log)
 *   2. 记录阶段开始/结束时间、残差、收敛状态
 *   3. 汇总报告写入 output_dir/pipeline_report_<ts>.yaml
 */

#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/io/yaml_io.h"
#include "unicalib/io/ros2_data_source.h"
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ---------------------------------------------------------------------------
// 工具: 获取当前时间字符串
// ---------------------------------------------------------------------------
static std::string now_str() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
    return oss.str();
}

// ---------------------------------------------------------------------------
// 工具: CalibTaskType → 字符串
// ---------------------------------------------------------------------------
static std::string task_str(CalibTaskType t) {
    switch (t) {
        case CalibTaskType::IMU_INTRINSIC:    return "imu_intrinsic";
        case CalibTaskType::CAM_INTRINSIC:    return "cam_intrinsic";
        case CalibTaskType::IMU_LIDAR_EXTRIN: return "imu_lidar_extrin";
        case CalibTaskType::LIDAR_CAM_EXTRIN: return "lidar_cam_extrin";
        case CalibTaskType::CAM_CAM_EXTRIN:   return "cam_cam_extrin";
        case CalibTaskType::ALL:              return "all";
        default:                              return "none";
    }
}

// ===========================================================================
// PipelineReport
// ===========================================================================

void PipelineReport::print_summary() const {
    UNICALIB_INFO("====== Pipeline Report: {} ======", pipeline_id);
    UNICALIB_INFO("  Total time: {:.1f} ms", total_elapsed_ms());
    UNICALIB_INFO("  All converged: {}", all_converged() ? "YES" : "NO");
    for (const auto& r : stage_results) {
        UNICALIB_INFO("  [{}-{}] success={} rms={:.4f} time={:.1f}ms: {}",
                      stage_name(r.stage), task_str(r.task),
                      r.success, r.residual_rms, r.elapsed_ms, r.message);
        if (r.needs_manual_refine()) {
            UNICALIB_WARN("    ↳ RMS {:.4f} > threshold {:.4f} — 建议手动校准",
                          r.residual_rms, r.quality_threshold);
        }
    }
    if (final_params) {
        final_params->print_summary();
    }
    UNICALIB_INFO("================================");
}

void PipelineReport::save_report(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_WARN("Cannot write pipeline report to: {}", path);
        return;
    }
    f << "pipeline_id: " << pipeline_id << "\n";
    f << "total_elapsed_ms: " << total_elapsed_ms() << "\n";
    f << "all_converged: " << all_converged() << "\n";
    f << "stages:\n";
    for (const auto& r : stage_results) {
        f << "  - stage: " << stage_name(r.stage) << "\n";
        f << "    task: " << task_str(r.task) << "\n";
        f << "    success: " << r.success << "\n";
        f << "    residual_rms: " << r.residual_rms << "\n";
        f << "    elapsed_ms: " << r.elapsed_ms << "\n";
        f << "    message: \"" << r.message << "\"\n";
        if (!r.log_file.empty())
            f << "    log_file: " << r.log_file << "\n";
        if (r.needs_manual_refine())
            f << "    needs_manual_refine: true\n";
    }
    UNICALIB_INFO("Pipeline report saved: {}", path);
}

// ===========================================================================
// CalibPipeline
// ===========================================================================

CalibPipeline::CalibPipeline()
    : CalibPipeline(PipelineConfig{}) {}

CalibPipeline::CalibPipeline(const PipelineConfig& cfg)
    : cfg_(cfg)
    , params_(CalibParamManager::Create())
{
    // 设置全局日志级别
    spdlog::level::level_enum lv = spdlog::level::info;
    if      (cfg_.log_level == "trace")    lv = spdlog::level::trace;
    else if (cfg_.log_level == "debug")    lv = spdlog::level::debug;
    else if (cfg_.log_level == "warn")     lv = spdlog::level::warn;
    else if (cfg_.log_level == "error")    lv = spdlog::level::err;

    spdlog::set_level(lv);

    // 确保输出目录存在
    fs::create_directories(cfg_.output_dir);
    fs::create_directories(cfg_.output_dir + "/logs");

    // 生成 pipeline_id
    report_.pipeline_id = "pipeline_" + now_str();
    UNICALIB_INFO("[CalibPipeline] 初始化 ID={}", report_.pipeline_id);
    UNICALIB_INFO("[CalibPipeline] 输出目录: {}", cfg_.output_dir);
    UNICALIB_INFO("[CalibPipeline] 无目标优先: {}", cfg_.prefer_targetfree);
    UNICALIB_INFO("[CalibPipeline] 任务掩码: 0x{:02X}",
                  static_cast<uint32_t>(cfg_.tasks));
}

// ---------------------------------------------------------------------------
// 阶段日志
// ---------------------------------------------------------------------------

std::string CalibPipeline::make_stage_log_path(CalibStage stage,
                                                CalibTaskType task) const {
    return cfg_.output_dir + "/logs/" +
           std::string(stage_name(stage)) + "_" + task_str(task) + "_" +
           now_str() + ".log";
}

void CalibPipeline::setup_stage_logger(const std::string& log_path) {
    try {
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            log_path, true);
        file_sink->set_level(spdlog::level::trace);
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] %v");

        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::get_level());
        console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");

        auto logger = std::make_shared<spdlog::logger>(
            "stage", spdlog::sinks_init_list{console_sink, file_sink});
        logger->set_level(spdlog::level::trace);
        spdlog::set_default_logger(logger);
    } catch (const std::exception& e) {
        // 日志创建失败不应中断流程，使用 WARN 级别
        UNICALIB_WARN("创建阶段日志失败: {}", e.what());
        // 继续执行，将使用默认日志
    }
}

void CalibPipeline::log_stage_begin(CalibStage stage, CalibTaskType task,
                                     const std::string& detail) {
    stage_start_ = Clock::now();
    UNICALIB_INFO("");
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("▶ 开始阶段: [{}] 任务: [{}]",
                  stage_name(stage), task_str(task));
    if (!detail.empty()) UNICALIB_INFO("  描述: {}", detail);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void CalibPipeline::log_stage_end(const StageResult& result) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       Clock::now() - stage_start_).count();
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("■ 结束阶段: [{}] 任务: [{}]",
                  stage_name(result.stage), task_str(result.task));
    UNICALIB_INFO("  结果: {} | 残差RMS: {:.4f} | 耗时: {} ms",
                  result.success ? "✓ 成功" : "✗ 失败",
                  result.residual_rms, elapsed);
    UNICALIB_INFO("  消息: {}", result.message);
    if (result.needs_manual_refine()) {
        UNICALIB_WARN("  ⚠ 自动标定精度不足 (RMS={:.4f} > 阈值={:.4f})",
                      result.residual_rms, result.quality_threshold);
        UNICALIB_WARN("  建议执行手动校准 (--manual 模式)");
    }
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}

void CalibPipeline::log_step(CalibStage stage, const std::string& step,
                              const std::string& msg,
                              spdlog::level::level_enum lv) {
    spdlog::log(lv, "[{}-{}] [{}] {}",
                stage_name(stage), step, step, msg);
    if (progress_cb_) {
        progress_cb_(stage, step, -1.0);
    }
}

void CalibPipeline::log_metric(const std::string& name, double value,
                                const std::string& unit) {
    UNICALIB_INFO("  📊 {}: {:.6f} {}", name, value, unit);
}

void CalibPipeline::log_param_change(const std::string& param_name,
                                      const std::string& before,
                                      const std::string& after) {
    UNICALIB_INFO("  🔄 参数更新 [{}]: {} → {}", param_name, before, after);
}

// ---------------------------------------------------------------------------
// 运行完整流水线
// ---------------------------------------------------------------------------

PipelineReport CalibPipeline::run() {
    UNICALIB_INFO("");
    UNICALIB_INFO("╔═══════════════════════════════════════════════╗");
    UNICALIB_INFO("║   UniCalib 两阶段标定流水线 — 开始            ║");
    UNICALIB_INFO("║   Pipeline ID: {:30s} ║", report_.pipeline_id);
    UNICALIB_INFO("╚═══════════════════════════════════════════════╝");

    const uint32_t task_mask = static_cast<uint32_t>(cfg_.tasks);

    // 枚举所有可能的单任务
    const CalibTaskType single_tasks[] = {
        CalibTaskType::IMU_INTRINSIC,
        CalibTaskType::CAM_INTRINSIC,
        CalibTaskType::IMU_LIDAR_EXTRIN,
        CalibTaskType::LIDAR_CAM_EXTRIN,
        CalibTaskType::CAM_CAM_EXTRIN,
    };

    for (auto task : single_tasks) {
        if (!has_task(cfg_.tasks, task)) continue;

        // ─── Stage 1: AI 粗标定 ───────────────────────────────────────────
        bool do_coarse =
            (task == CalibTaskType::CAM_INTRINSIC    && cfg_.enable_coarse_cam_intrin)  ||
            (task == CalibTaskType::IMU_INTRINSIC    && cfg_.enable_coarse_imu_intrin)  ||
            (task == CalibTaskType::IMU_LIDAR_EXTRIN && cfg_.enable_coarse_imu_lidar)   ||
            (task == CalibTaskType::LIDAR_CAM_EXTRIN && cfg_.enable_coarse_lidar_cam)   ||
            (task == CalibTaskType::CAM_CAM_EXTRIN   && cfg_.enable_coarse_cam_cam);

        if (do_coarse) {
            auto log_path = make_stage_log_path(CalibStage::COARSE_AI, task);
            setup_stage_logger(log_path);
            log_stage_begin(CalibStage::COARSE_AI, task,
                            "AI模型提供初始估计值");
            auto r = run_coarse_stage(task);
            r.log_file = log_path;
            log_stage_end(r);
            report_.stage_results.push_back(r);
        }

        // ─── Stage 2: 精标定 ──────────────────────────────────────────────
        {
            auto log_path = make_stage_log_path(CalibStage::FINE_AUTO, task);
            setup_stage_logger(log_path);
            log_stage_begin(CalibStage::FINE_AUTO, task,
                            cfg_.prefer_targetfree ? "无目标优化" : "目标辅助优化");
            auto r = run_fine_stage(task);
            r.log_file = log_path;
            log_stage_end(r);
            report_.stage_results.push_back(r);

            // ─── Stage 3: 手动校准 (仅外参, 精度不足时) ──────────────────
            if (cfg_.allow_manual_fallback && r.needs_manual_refine()) {
                auto log_path_m = make_stage_log_path(CalibStage::MANUAL_REFINE, task);
                setup_stage_logger(log_path_m);
                UNICALIB_WARN("[Pipeline] 自动精标定 RMS={:.4f} 超过阈值, 进入手动校准",
                              r.residual_rms);
                log_stage_begin(CalibStage::MANUAL_REFINE, task,
                                "手动校准: 6-DOF 交互式调整");
                auto rm = run_manual_stage(task);
                rm.log_file = log_path_m;
                log_stage_end(rm);
                report_.stage_results.push_back(rm);
            }
        }
    }

    // 生成最终报告
    report_.final_params = params_;
    report_.print_summary();

    std::string report_path = cfg_.output_dir + "/pipeline_report_" +
                               report_.pipeline_id.substr(9) + ".yaml";
    report_.save_report(report_path);

    return report_;
}

// ---------------------------------------------------------------------------
// 粗标定阶段入口 (占位实现 — 由子类或外部注入覆盖)
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_coarse_stage(CalibTaskType task) {
    StageResult r;
    r.stage = CalibStage::COARSE_AI;
    r.task  = task;

    UNICALIB_INFO("[Coarse-AI] 任务: {}", task_str(task));
    UNICALIB_INFO("[Coarse-AI] 说明: 此阶段需通过 AICoarseCalibManager 注入");
    UNICALIB_INFO("[Coarse-AI] 模型映射:");
    switch (task) {
        case CalibTaskType::CAM_INTRINSIC:
            UNICALIB_INFO("  → DM-Calib (扩散模型单图内参估计)");
            UNICALIB_INFO("  调用: {}/DM-Calib/DMCalib/tools/infer_unicalib.py",
                          cfg_.ai_models_root);
            break;
        case CalibTaskType::IMU_INTRINSIC:
            UNICALIB_INFO("  → Transformer-IMU-Calibrator");
            UNICALIB_INFO("  调用: {}/Transformer-IMU-Calibrator/eval.py",
                          cfg_.ai_models_root);
            break;
        case CalibTaskType::IMU_LIDAR_EXTRIN:
            UNICALIB_INFO("  → L2Calib (RL强化学习 SE(3)-流形)");
            UNICALIB_INFO("  调用: {}/learn-to-calibrate/train.py",
                          cfg_.ai_models_root);
            break;
        case CalibTaskType::LIDAR_CAM_EXTRIN:
            UNICALIB_INFO("  → MIAS-LCEC (跨模态掩码匹配 C3M)");
            UNICALIB_INFO("  调用: {}/MIAS-LCEC/", cfg_.ai_models_root);
            break;
        case CalibTaskType::CAM_CAM_EXTRIN:
            UNICALIB_INFO("  → 特征匹配初始化 (ORB/SIFT + 本质矩阵)");
            break;
        default: break;
    }

    r.success  = true;  // 粗标定失败不阻断流水线
    r.message  = "AI粗标定占位 — 请通过 AICoarseCalibManager 注入实际调用";
    r.elapsed_ms = 0.0;
    return r;
}

// ---------------------------------------------------------------------------
// 精标定阶段入口
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_stage(CalibTaskType task) {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = task;

    UNICALIB_INFO("[Fine-Auto] 任务: {} | 无目标优先: {}",
                  task_str(task), cfg_.prefer_targetfree);

    // 设置质量阈值
    switch (task) {
        case CalibTaskType::LIDAR_CAM_EXTRIN:
            r.quality_threshold = cfg_.lidar_cam_rms_threshold;
            UNICALIB_INFO("[Fine-Auto] 方法: {}",
                          cfg_.prefer_targetfree ?
                          "边缘对齐互信息 (EDGE_ALIGNMENT, 无目标)" :
                          "棋盘格 PnP (TARGET_CHESSBOARD)");
            break;
        case CalibTaskType::CAM_CAM_EXTRIN:
            r.quality_threshold = cfg_.cam_cam_rms_threshold;
            UNICALIB_INFO("[Fine-Auto] 方法: {}",
                          cfg_.prefer_targetfree ?
                          "特征匹配 + Bundle Adjustment (无目标)" :
                          "OpenCV stereoCalibrate (棋盘格)");
            break;
        case CalibTaskType::IMU_LIDAR_EXTRIN:
            r.quality_threshold = cfg_.imu_lidar_rot_threshold;
            UNICALIB_INFO("[Fine-Auto] 方法: B样条连续时间优化 (无目标)");
            break;
        case CalibTaskType::IMU_INTRINSIC:
            UNICALIB_INFO("[Fine-Auto] 方法: Allan方差分析");
            break;
        case CalibTaskType::CAM_INTRINSIC:
            UNICALIB_INFO("[Fine-Auto] 方法: {}",
                          cfg_.prefer_targetfree ?
                          "DM-Calib精化 + RANSAC" :
                          "棋盘格角点检测");
            break;
        default: break;
    }

    auto t_start = std::chrono::high_resolution_clock::now();

    // ─── LiDAR-Camera 外参精标定 ─────────────────────────────────────────
    if (task == CalibTaskType::LIDAR_CAM_EXTRIN) {
        return run_fine_lidar_camera();
    }

    // ─── 其他任务（占位）─────────────────────────────────────────────────
    r.success  = true;
    r.message  = "精标定占位 — 请通过具体标定器实现";
    auto t_end = std::chrono::high_resolution_clock::now();
    r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    UNICALIB_WARN("[Fine-Auto] 任务 {} 当前为占位实现", task_str(task));
    return r;
}

// ---------------------------------------------------------------------------
// LiDAR-Camera 精标定实现
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_lidar_camera() {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = CalibTaskType::LIDAR_CAM_EXTRIN;
    r.quality_threshold = cfg_.lidar_cam_rms_threshold;

    auto t_start = std::chrono::high_resolution_clock::now();

    // ─── 数据源类型判断 ─────────────────────────────────────────────────
    enum class DataSourceType {
        FILES,
        ROS2_BAG,
        ROS2_TOPIC
    };
    
    DataSourceType data_source_type = DataSourceType::FILES;
    if (cfg_.use_ros2_bag && !cfg_.ros2_bag_file.empty()) {
        data_source_type = DataSourceType::ROS2_BAG;
    } else if (cfg_.use_ros2_topics && (!cfg_.lidar_ros2_topic.empty() || !cfg_.camera_ros2_topic.empty())) {
        data_source_type = DataSourceType::ROS2_TOPIC;
    }

    // ─── 显示数据源信息 ───────────────────────────────────────────────
    UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 开始执行精标定...");
    
    if (data_source_type == DataSourceType::ROS2_BAG) {
        UNICALIB_INFO("  数据源类型: ROS2 Bag 文件");
        UNICALIB_INFO("  ROS2 Bag 文件: {}", cfg_.ros2_bag_file);
        UNICALIB_INFO("  LiDAR 话题: {}", cfg_.lidar_ros2_topic);
        UNICALIB_INFO("  相机话题: {}", cfg_.camera_ros2_topic);
    } else if (data_source_type == DataSourceType::ROS2_TOPIC) {
        UNICALIB_INFO("  数据源类型: ROS2 实时话题订阅");
        UNICALIB_INFO("  LiDAR 话题: {}", cfg_.lidar_ros2_topic);
        UNICALIB_INFO("  相机话题: {}", cfg_.camera_ros2_topic);
        UNICALIB_INFO("  最大等待时间: {:.1f} 秒", cfg_.ros2_max_wait_time);
    } else {
        UNICALIB_INFO("  数据源类型: 文件系统");
        UNICALIB_INFO("  LiDAR 数据目录: {}",
                      cfg_.lidar_data_dir.empty() ? "(未设置)" : cfg_.lidar_data_dir);
        UNICALIB_INFO("  相机图像目录: {}",
                      cfg_.camera_images_dir.empty() ? "(未设置)" : cfg_.camera_images_dir);
    }

    // ─── 1. 数据路径/配置验证 ───────────────────────────────────────
    if (data_source_type == DataSourceType::FILES) {
        if (cfg_.lidar_data_dir.empty() || cfg_.camera_images_dir.empty()) {
            r.success = false;
            r.message = "数据路径未配置 — 请在 PipelineConfig 中设置 lidar_data_dir 和 camera_images_dir";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }

        if (!fs::exists(cfg_.lidar_data_dir)) {
            r.success = false;
            r.message = "LiDAR 数据目录不存在: " + cfg_.lidar_data_dir;
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }

        if (!fs::exists(cfg_.camera_images_dir)) {
            r.success = false;
            r.message = "相机图像目录不存在: " + cfg_.camera_images_dir;
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
    } else if (data_source_type == DataSourceType::ROS2_BAG) {
        if (!fs::exists(cfg_.ros2_bag_file)) {
            r.success = false;
            r.message = "ROS2 Bag 文件不存在: " + cfg_.ros2_bag_file;
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
    }

    // ─── 2. 加载 LiDAR 和相机数据 (文件或 ROS2) ─────────────────────────
    std::vector<LiDARScan> lidar_scans;
    std::vector<std::pair<double, cv::Mat>> camera_frames;

    if (data_source_type == DataSourceType::FILES) {
        // 从文件加载数据
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 从文件加载点云...");
        
        std::vector<fs::path> pcd_files;
        for (const auto& entry : fs::directory_iterator(cfg_.lidar_data_dir)) {
            if (entry.path().extension() == ".pcd" || entry.path().extension() == ".PCD") {
                pcd_files.push_back(entry.path());
            }
        }
        std::sort(pcd_files.begin(), pcd_files.end());

        if (pcd_files.empty()) {
            r.success = false;
            r.message = "未找到 PCD 文件: " + cfg_.lidar_data_dir;
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }

        const size_t MAX_FRAMES = 100;
        size_t step = std::max(size_t(1), pcd_files.size() / MAX_FRAMES);

        for (size_t i = 0; i < pcd_files.size() && lidar_scans.size() < MAX_FRAMES; i += step) {
            LiDARScan scan;
            scan.cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files[i].string(), *scan.cloud) == 0) {
                std::string fname = pcd_files[i].stem().string();
                try {
                    scan.timestamp = std::stod(fname);
                } catch (...) {
                    scan.timestamp = static_cast<double>(i) * 0.1;
                }
                lidar_scans.push_back(std::move(scan));
            } else {
                UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 加载失败: {}", pcd_files[i].string());
            }
        }

        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 加载 {} 帧点云 (共 {} 文件)",
                      lidar_scans.size(), pcd_files.size());

        if (lidar_scans.empty()) {
            r.success = false;
            r.message = "未能成功加载任何点云";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            return r;
        }

        // 从文件加载相机图像
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 从文件加载图像...");
    std::vector<std::pair<double, cv::Mat>> camera_frames;

    std::vector<fs::path> img_files;
    for (const auto& entry : fs::directory_iterator(cfg_.camera_images_dir)) {
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp") {
            img_files.push_back(entry.path());
        }
    }
    std::sort(img_files.begin(), img_files.end());

    if (img_files.empty()) {
        r.success = false;
        r.message = "未找到图像文件: " + cfg_.camera_images_dir;
        auto t_end = std::chrono::high_resolution_clock::now();
        r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
        return r;
    }

        size_t img_step = std::max(size_t(1), img_files.size() / MAX_FRAMES);
        for (size_t i = 0; i < img_files.size() && camera_frames.size() < MAX_FRAMES; i += img_step) {
            cv::Mat img = cv::imread(img_files[i].string());
            if (!img.empty()) {
                std::string fname = img_files[i].stem().string();
                double ts;
                try {
                    ts = std::stod(fname);
                } catch (...) {
                    ts = static_cast<double>(i) * 0.1;
                }
                camera_frames.emplace_back(ts, img);
            }
        }

        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 加载 {} 帧图像 (共 {} 文件)",
                      camera_frames.size(), img_files.size());

        if (camera_frames.empty()) {
            r.success = false;
            r.message = "未能成功加载任何图像";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            return r;
        }
    } else if (data_source_type == DataSourceType::ROS2_BAG || 
               data_source_type == DataSourceType::ROS2_TOPIC) {
        // 从 ROS2 加载数据
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 从 ROS2 加载数据...");
        
        // 配置 ROS2 数据源
        RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = cfg_.ros2_bag_file;
        ros_cfg.realtime_mode = (data_source_type == DataSourceType::ROS2_TOPIC);
        ros_cfg.realtime_timeout = cfg_.ros2_max_wait_time;
        ros_cfg.sample_interval = cfg_.ros2_sample_interval;
        ros_cfg.max_frames = cfg_.ros2_max_frames;
        
        // 设置话题映射
        if (!cfg_.lidar_ros2_topic.empty()) {
            ros_cfg.lidar_topics[cfg_.lidar_id] = cfg_.lidar_ros2_topic;
        }
        if (!cfg_.camera_ros2_topic.empty()) {
            ros_cfg.camera_topics[cfg_.camera_id] = cfg_.camera_ros2_topic;
        }
        
        // 创建 ROS2 数据源
        UnifiedDataLoader::Config unified_cfg;
        unified_cfg.source_type = (data_source_type == DataSourceType::ROS2_BAG) ?
            UnifiedDataLoader::SourceType::ROS2_BAG :
            UnifiedDataLoader::SourceType::ROS2_TOPIC;
        unified_cfg.ros_config = ros_cfg;
        unified_cfg.max_frames = cfg_.ros2_max_frames;
        
        UnifiedDataLoader loader(unified_cfg);
        
        // 加载数据
        if (!loader.load()) {
            r.success = false;
            r.message = "ROS2 数据加载失败: " + loader.get_status_message();
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
        
        // 转换数据
        lidar_scans = loader.to_lidar_scans(cfg_.lidar_id);
        camera_frames = loader.to_camera_frames(cfg_.camera_id);
        
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 从 ROS2 加载完成:");
        UNICALIB_INFO("  LiDAR: {} 帧", lidar_scans.size());
        UNICALIB_INFO("  相机: {} 帧", camera_frames.size());
        
        if (lidar_scans.empty()) {
            r.success = false;
            r.message = "未能从 ROS2 加载 LiDAR 数据";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            return r;
        }
        
        if (camera_frames.empty()) {
            r.success = false;
            r.message = "未能从 ROS2 加载相机数据";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            return r;
        }
    }

    // ─── 4. 加载/默认相机内参 ─────────────────────────────────────────────
    CameraIntrinsics cam_intrin;
    if (!cfg_.camera_intrinsic_file.empty() && fs::exists(cfg_.camera_intrinsic_file)) {
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 加载内参: {}", cfg_.camera_intrinsic_file);
        try {
            YAML::Node intrin_node = YAML::LoadFile(cfg_.camera_intrinsic_file);
            cam_intrin.fx     = intrin_node["fx"].as<double>(0.0);
            cam_intrin.fy     = intrin_node["fy"].as<double>(0.0);
            cam_intrin.cx     = intrin_node["cx"].as<double>(0.0);
            cam_intrin.cy     = intrin_node["cy"].as<double>(0.0);
            cam_intrin.width  = intrin_node["width"].as<int>(0);
            cam_intrin.height = intrin_node["height"].as<int>(0);
            if (intrin_node["dist_coeffs"]) {
                for (const auto& d : intrin_node["dist_coeffs"]) {
                    cam_intrin.dist_coeffs.push_back(d.as<double>());
                }
            }
            UNICALIB_INFO("  内参: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} {}x{}",
                          cam_intrin.fx, cam_intrin.fy, cam_intrin.cx, cam_intrin.cy,
                          cam_intrin.width, cam_intrin.height);
        } catch (const std::exception& e) {
            UNICALIB_WARN("[Fine-Auto/LiDAR-Cam] 内参加载失败: {}，使用图像尺寸推断", e.what());
            cam_intrin = infer_intrinsics_from_images(camera_frames);
        }
    } else {
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 未提供内参文件，从图像尺寸推断");
        cam_intrin = infer_intrinsics_from_images(camera_frames);
    }

    // ─── 5. 调用 LiDARCameraCalibrator ─────────────────────────────────────
    LiDARCameraCalibrator::Config calib_cfg;

    // 方法选择
    if (cfg_.lidar_cam_method == "edge") {
        calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
    } else if (cfg_.lidar_cam_method == "target") {
        calib_cfg.method = LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
    } else if (cfg_.lidar_cam_method == "motion") {
        calib_cfg.method = LiDARCameraCalibrator::Method::MOTION_BSPLINE;
    } else {
        calib_cfg.method = cfg_.prefer_targetfree ?
            LiDARCameraCalibrator::Method::EDGE_ALIGNMENT :
            LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
    }

    calib_cfg.board_cols     = cfg_.board_cols;
    calib_cfg.board_rows     = cfg_.board_rows;
    calib_cfg.square_size_m  = cfg_.square_size_m;
    calib_cfg.edge_canny_low  = cfg_.edge_canny_low;
    calib_cfg.edge_canny_high = cfg_.edge_canny_high;
    calib_cfg.ceres_max_iter  = cfg_.ceres_max_iter;
    calib_cfg.verbose = (cfg_.log_level == "debug" || cfg_.log_level == "trace");

    LiDARCameraCalibrator calibrator(calib_cfg);

    // 进度回调
    calibrator.set_progress_callback([this](const std::string& step, double prog) {
        if (prog >= 0) {
            log_step(CalibStage::FINE_AUTO, step,
                     "进度 " + std::to_string(static_cast<int>(prog * 100)) + "%");
        }
    });

    // 执行两阶段标定
    Sophus::SE3d init_guess = Sophus::SE3d();  // identity 作为初值
    auto result = calibrator.calibrate_two_stage(
        lidar_scans, camera_frames, cam_intrin,
        std::nullopt,  // 无 AI 粗估初值
        cfg_.prefer_targetfree,
        cfg_.lidar_id, cfg_.camera_id);

    // ─── 6. 结果处理 ──────────────────────────────────────────────────────
    auto t_end = std::chrono::high_resolution_clock::now();
    r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    if (result.best() != nullptr) {
        r.success = true;
        r.residual_rms = result.best_rms();
        r.needs_manual_refine();  // 更新状态

        std::ostringstream oss;
        oss << "LiDAR-Camera 精标定成功: 方法=" << result.fine_method
            << " RMS=" << std::fixed << std::setprecision(3) << result.best_rms() << "px";

        if (result.fine.has_value()) {
            const auto& T = result.fine->SE3_TargetInRef();
            Eigen::Vector3d t = T.translation();
            Eigen::Vector3d rpy = T.so3().log() * 180.0 / M_PI;
            oss << " t=[" << std::setprecision(3)
                << t.x() << "," << t.y() << "," << t.z() << "]m"
                << " rpy=[" << rpy.x() << "," << rpy.y() << "," << rpy.z() << "]deg";
        }

        r.message = oss.str();
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] {}", r.message);

        // 更新参数管理器
        if (result.fine.has_value() && params_) {
            auto extrin_ptr = params_->get_or_create_extrinsic(cfg_.lidar_id, cfg_.camera_id);
            if (extrin_ptr) {
                *extrin_ptr = *result.fine;
            }
        }

        // 保存结果到 YAML
        std::string result_yaml = cfg_.output_dir + "/lidar_cam_extrinsic.yaml";
        save_extrinsic_result(result_yaml, result);

        // 生成可视化
        if (!lidar_scans.empty() && !camera_frames.empty()) {
            std::string vis_path = cfg_.output_dir + "/lidar_cam_projection.png";
            calibrator.visualize_projection(
                lidar_scans[0], camera_frames[0].second,
                *result.best(), cam_intrin, vis_path);
        }

    } else {
        r.success = false;
        r.residual_rms = -1.0;
        r.message = "LiDAR-Camera 精标定失败 — 未能收敛或数据不匹配";
        UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
    }

    return r;
}

// ---------------------------------------------------------------------------
// 辅助函数: 从图像推断内参
// ---------------------------------------------------------------------------
CameraIntrinsics CalibPipeline::infer_intrinsics_from_images(
    const std::vector<std::pair<double, cv::Mat>>& frames) const {

    CameraIntrinsics intrin;
    if (frames.empty()) {
        intrin.width = 1280;
        intrin.height = 720;
        intrin.fx = intrin.width / 2.0;
        intrin.fy = intrin.width / 2.0;
        intrin.cx = intrin.width / 2.0;
        intrin.cy = intrin.height / 2.0;
        UNICALIB_WARN("[Fine-Auto] 无图像数据，使用默认 1280x720 内参");
        return intrin;
    }

    const auto& img = frames[0].second;
    intrin.width = img.cols;
    intrin.height = img.rows;
    // 假设约 90 度 FOV (f ≈ width)
    intrin.fx = intrin.width * 0.9;
    intrin.fy = intrin.width * 0.9;
    intrin.cx = intrin.width / 2.0;
    intrin.cy = intrin.height / 2.0;

    UNICALIB_INFO("[Fine-Auto] 推断内参: {}x{} fx={:.1f} fy={:.1f}",
                  intrin.width, intrin.height, intrin.fx, intrin.fy);
    return intrin;
}

// ---------------------------------------------------------------------------
// 辅助函数: 保存外参结果
// ---------------------------------------------------------------------------
void CalibPipeline::save_extrinsic_result(
    const std::string& path,
    const LiDARCameraCalibrator::TwoStageResult& result) const {

    YAML::Emitter out;
    out << YAML::BeginMap;

    out << YAML::Key << "calibration_type" << YAML::Value << "lidar_camera_extrinsic";
    out << YAML::Key << "reference_sensor" << YAML::Value << cfg_.lidar_id;
    out << YAML::Key << "target_sensor" << YAML::Value << cfg_.camera_id;
    out << YAML::Key << "timestamp" << YAML::Value << now_str();

    if (result.coarse.has_value()) {
        out << YAML::Key << "coarse_result";
        out << YAML::BeginMap;
        const auto& T = result.coarse->SE3_TargetInRef();
        out << YAML::Key << "method" << YAML::Value << result.coarse_method;
        out << YAML::Key << "rms" << YAML::Value << result.coarse_rms;
        out << YAML::Key << "translation" << YAML::Flow << YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << YAML::EndSeq;
        auto rpy = T.so3().log();
        out << YAML::Key << "rotation_rpy" << YAML::Flow << YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << YAML::EndSeq;
        out << YAML::EndMap;
    }

    if (result.fine.has_value()) {
        out << YAML::Key << "fine_result";
        out << YAML::BeginMap;
        const auto& T = result.fine->SE3_TargetInRef();
        out << YAML::Key << "method" << YAML::Value << result.fine_method;
        out << YAML::Key << "rms" << YAML::Value << result.fine_rms;
        out << YAML::Key << "converged" << YAML::Value << result.fine->is_converged;
        out << YAML::Key << "translation" << YAML::Flow << YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << YAML::EndSeq;
        auto rpy = T.so3().log();
        out << YAML::Key << "rotation_rpy_rad" << YAML::Flow << YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << YAML::EndSeq;
        out << YAML::Key << "rotation_rpy_deg" << YAML::Flow << YAML::BeginSeq
            << rpy.x() * 180 / M_PI << rpy.y() * 180 / M_PI << rpy.z() * 180 / M_PI << YAML::EndSeq;
        out << YAML::EndMap;
    }

    out << YAML::Key << "needs_manual_refine" << YAML::Value << result.needs_manual;
    out << YAML::Key << "manual_threshold_px" << YAML::Value << result.manual_threshold_px;

    out << YAML::EndMap;

    std::ofstream f(path);
    if (f.is_open()) {
        f << out.c_str();
        UNICALIB_INFO("[Fine-Auto] 结果已保存: {}", path);
    }
}

// ---------------------------------------------------------------------------
// 手动校准阶段入口 (占位实现)
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_manual_stage(CalibTaskType task) {
    StageResult r;
    r.stage = CalibStage::MANUAL_REFINE;
    r.task  = task;

    UNICALIB_INFO("[Manual-Refine] 任务: {}", task_str(task));
    UNICALIB_INFO("[Manual-Refine] 进入交互式手动校准模式");
    UNICALIB_INFO("[Manual-Refine] 操作说明:");
    UNICALIB_INFO("  方向键/WASD: 调整平移 (±{:.1f}cm/步)", 0.5);
    UNICALIB_INFO("  QE/RF/TG:   调整 RPY (±{:.1f}deg/步)", 0.1);
    UNICALIB_INFO("  Shift+键:   10倍步长 (快速模式)");
    UNICALIB_INFO("  P:          点击对应点 (click-calib 精化)");
    UNICALIB_INFO("  S:          保存当前结果");
    UNICALIB_INFO("  ESC:        退出手动校准");
    UNICALIB_INFO("[Manual-Refine] 请通过 ManualCalibSession 进行实际交互");

    r.success  = true;
    r.message  = "手动校准占位 — 请通过 ManualCalibSession 运行交互界面";
    r.elapsed_ms = 0.0;
    return r;
}

}  // namespace ns_unicalib
