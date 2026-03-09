/**
 * UniCalib Unified — CalibPipeline 实现
 *
 * 核心流程:
 *   1. 为每个 stage 创建独立日志文件 (output_dir/logs/<stage>_<task>_<ts>.log)
 *   2. 记录阶段开始/结束时间、残差、收敛状态
 *   3. 汇总报告写入 output_dir/pipeline_report_<ts>.yaml
 */

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cmath>

// ROS2 头必须在 namespace 外包含，避免 sensor_msgs 等被注入 ns_unicalib 导致编译错误
#include "unicalib/io/ros2_data_source.h"

#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/accuracy_logger.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/extrinsic/cam_cam_calib.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/io/yaml_io.h"
#include <pcl/io/pcd_io.h>
#include <opencv2/imgcodecs.hpp>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

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
    std::string logs_dir = resolve_logs_dir(cfg_.output_dir);
    return logs_dir + "/" + std::string(stage_name(stage)) + "_" +
           task_str(task) + "_" + now_str() + ".log";
}

void CalibPipeline::setup_stage_logger(const std::string& log_path) {
    try {
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            log_path, true);
        file_sink->set_level(spdlog::level::trace);
        // 与统一日志一致：每条记录带完整日期时间
        file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v");

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
            StageResult r;
            try {
                r = run_coarse_stage(task);
            } catch (const UniCalibException& e) {
                r = StageResult{};
                r.stage = CalibStage::COARSE_AI;
                r.task = task;
                r.success = false;
                r.message = std::string("[") + errorCodeName(e.code()) + "] " + e.message() +
                            " (at " + e.file() + ":" + std::to_string(e.line()) + ")";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 粗标定阶段异常: {}", r.message);
            } catch (const std::exception& e) {
                r = StageResult{};
                r.stage = CalibStage::COARSE_AI;
                r.task = task;
                r.success = false;
                r.message = std::string("std::exception: ") + e.what();
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 粗标定阶段异常: {}", r.message);
            } catch (...) {
                r = StageResult{};
                r.stage = CalibStage::COARSE_AI;
                r.task = task;
                r.success = false;
                r.message = "未知异常 (非 std::exception)";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 粗标定阶段未知异常");
            }
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
            StageResult r;
            try {
                r = run_fine_stage(task);
            } catch (const UniCalibException& e) {
                r = StageResult{};
                r.stage = CalibStage::FINE_AUTO;
                r.task = task;
                r.success = false;
                r.message = std::string("[") + errorCodeName(e.code()) + "] " + e.message() +
                            " (at " + e.file() + ":" + std::to_string(e.line()) + ")";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 精标定阶段异常: {}", r.message);
            } catch (const std::exception& e) {
                r = StageResult{};
                r.stage = CalibStage::FINE_AUTO;
                r.task = task;
                r.success = false;
                r.message = std::string("std::exception: ") + e.what();
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 精标定阶段异常: {}", r.message);
            } catch (...) {
                r = StageResult{};
                r.stage = CalibStage::FINE_AUTO;
                r.task = task;
                r.success = false;
                r.message = "未知异常 (非 std::exception)";
                r.elapsed_ms = 0;
                UNICALIB_ERROR("[Pipeline] 精标定阶段未知异常");
            }
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
                StageResult rm;
                try {
                    rm = run_manual_stage(task);
                } catch (const UniCalibException& e) {
                    rm = StageResult{};
                    rm.stage = CalibStage::MANUAL_REFINE;
                    rm.task = task;
                    rm.success = false;
                    rm.message = std::string("[") + errorCodeName(e.code()) + "] " + e.message();
                    rm.elapsed_ms = 0;
                    UNICALIB_ERROR("[Pipeline] 手动校准阶段异常: {}", rm.message);
                } catch (const std::exception& e) {
                    rm = StageResult{};
                    rm.stage = CalibStage::MANUAL_REFINE;
                    rm.task = task;
                    rm.success = false;
                    rm.message = std::string("std::exception: ") + e.what();
                    rm.elapsed_ms = 0;
                    UNICALIB_ERROR("[Pipeline] 手动校准阶段异常: {}", rm.message);
                } catch (...) {
                    rm = StageResult{};
                    rm.stage = CalibStage::MANUAL_REFINE;
                    rm.task = task;
                    rm.success = false;
                    rm.message = "未知异常";
                    rm.elapsed_ms = 0;
                    UNICALIB_ERROR("[Pipeline] 手动校准阶段未知异常");
                }
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

    // 将各阶段精度追加到对应 CSV，便于绘制曲线
    static const auto task_to_accuracy = [](CalibTaskType t) -> std::optional<CalibAccuracyTask> {
        switch (t) {
            case CalibTaskType::CAM_INTRINSIC:     return CalibAccuracyTask::CAM_INTRINSIC;
            case CalibTaskType::IMU_INTRINSIC:     return CalibAccuracyTask::IMU_INTRINSIC;
            case CalibTaskType::LIDAR_CAM_EXTRIN:  return CalibAccuracyTask::LIDAR_CAM_EXTRIN;
            case CalibTaskType::CAM_CAM_EXTRIN:    return CalibAccuracyTask::CAM_CAM_EXTRIN;
            case CalibTaskType::IMU_LIDAR_EXTRIN:   return CalibAccuracyTask::IMU_LIDAR_EXTRIN;
            default: return std::nullopt;
        }
    };
    for (const auto& r : report_.stage_results) {
        if (r.stage != CalibStage::FINE_AUTO && r.stage != CalibStage::MANUAL_REFINE)
            continue;
        auto at = task_to_accuracy(r.task);
        if (!at.has_value()) continue;
        append_stage_result_accuracy(cfg_.output_dir, *at,
            r.success, r.residual_rms, r.elapsed_ms, r.message);
    }

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

    // ─── IMU 内参精标定 ─────────────────────────────────────────────────
    if (task == CalibTaskType::IMU_INTRINSIC) {
        return run_fine_imu_intrinsic();
    }

    // ─── Camera-Camera 外参精标定（多相机按数目自动两两标定）──────────────
    if (task == CalibTaskType::CAM_CAM_EXTRIN) {
        return run_fine_cam_cam();
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
            r.message = "ROS2 Bag 路径不存在: " + cfg_.ros2_bag_file + "（可为目录，rosbag2 格式）";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            UNICALIB_ERROR("[Fine-Auto/LiDAR-Cam] {}", r.message);
            return r;
        }
    } else if (data_source_type == DataSourceType::ROS2_TOPIC) {
        if (cfg_.lidar_ros2_topic.empty() || cfg_.camera_ros2_topic.empty()) {
            r.success = false;
            r.message = "ROS2 实时模式需同时配置 LiDAR 与相机话题（config 中 ros2.lidar_topic / ros2.camera_topic 或 sensors[].topic）";
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
        UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 开始从文件加载点云");
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

        UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 点云加载完成 — {} 帧 (共 {} 文件)",
                          lidar_scans.size(), pcd_files.size());

        if (lidar_scans.empty()) {
            r.success = false;
            r.message = "未能成功加载任何点云";
            auto t_end = std::chrono::high_resolution_clock::now();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            return r;
        }

        // 从文件加载相机图像
        UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 开始从文件加载图像");
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

        UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 图像加载完成 — {} 帧 (共 {} 文件)",
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
        ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
        
        // 传感器话题全可配置：优先使用 *_topics 映射，否则回退到单话题
        if (!cfg_.lidar_topics.empty()) {
            ros_cfg.lidar_topics = cfg_.lidar_topics;
            ros_cfg.lidar_ros2_topic = cfg_.lidar_topics.begin()->second;
        } else if (!cfg_.lidar_ros2_topic.empty()) {
            ros_cfg.lidar_topics[cfg_.lidar_id] = cfg_.lidar_ros2_topic;
            ros_cfg.lidar_ros2_topic = cfg_.lidar_ros2_topic;
        }
        if (!cfg_.camera_topics.empty()) {
            ros_cfg.camera_topics = cfg_.camera_topics;
            ros_cfg.camera_ros2_topic = cfg_.camera_topics.begin()->second;
        } else if (!cfg_.camera_ros2_topic.empty()) {
            ros_cfg.camera_topics[cfg_.camera_id] = cfg_.camera_ros2_topic;
            ros_cfg.camera_ros2_topic = cfg_.camera_ros2_topic;
        }
        if (!cfg_.imu_topics.empty()) {
            ros_cfg.imu_topics = cfg_.imu_topics;
            ros_cfg.imu_ros2_topic = cfg_.imu_topics.begin()->second;
        } else if (!cfg_.imu_ros2_topic.empty()) {
            ros_cfg.imu_topics["imu_0"] = cfg_.imu_ros2_topic;
            ros_cfg.imu_ros2_topic = cfg_.imu_ros2_topic;
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
    UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 加载相机内参");
    CameraIntrinsics cam_intrin;
    if (!cfg_.camera_intrinsic_file.empty() && fs::exists(cfg_.camera_intrinsic_file)) {
        UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 加载内参: {}", cfg_.camera_intrinsic_file);
        try {
            cam_intrin = YamlIO::load_camera_intrinsics(cfg_.camera_intrinsic_file);
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
    UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 开始 LiDAR-Camera 标定 (方法: {})",
                      cfg_.prefer_targetfree ? "边缘对齐" : "棋盘格目标");
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
    UNICALIB_INFO("[Fine-Auto/LiDAR-Cam] 执行标定: 点云 {} 帧, 图像 {} 帧",
                  lidar_scans.size(), camera_frames.size());
    Sophus::SE3d init_guess = Sophus::SE3d();  // identity 作为初值
    auto result = calibrator.calibrate_two_stage(
        lidar_scans, camera_frames, cam_intrin,
        std::nullopt,  // 无 AI 粗估初值
        cfg_.prefer_targetfree,
        cfg_.lidar_id, cfg_.camera_id);

    // ─── 6. 结果处理 ──────────────────────────────────────────────────────
    auto t_end = std::chrono::high_resolution_clock::now();
    r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    UNICALIB_LOG_STEP("Fine-Auto/LiDAR-Cam", "步骤: 标定计算完成 — 耗时 {:.1f} ms", r.elapsed_ms);

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
// Camera-Camera 精标定：多相机按数目自动两两标定（环视+前视兼容）
// ROS2 bag 离线时若未配置 camera_topics，则自动读取 bag 内全部相机并标定
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_cam_cam() {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = CalibTaskType::CAM_CAM_EXTRIN;
    r.quality_threshold = cfg_.cam_cam_rms_threshold;

    auto t_start = std::chrono::high_resolution_clock::now();

    if (!cfg_.use_ros2_bag || cfg_.ros2_bag_file.empty()) {
        r.success = false;
        r.message = "Camera-Camera 多目标定当前需使用 ROS2 bag（use_ros2_bag: true 且 ros2_bag_file 已配置）";
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        UNICALIB_WARN("[Fine-Auto/Cam-Cam] {}", r.message);
        return r;
    }

    // 参与标定的相机 ID：配置了则用配置；否则留空，加载 bag 后从 loader 取「全部相机」
    std::vector<std::string> camera_ids;
    if (!cfg_.camera_topics.empty()) {
        for (const auto& [id, _] : cfg_.camera_topics)
            camera_ids.push_back(id);
        std::sort(camera_ids.begin(), camera_ids.end());
    } else if (!cfg_.camera_id.empty()) {
        camera_ids.push_back(cfg_.camera_id);
    }

    std::map<std::string, std::vector<std::pair<double, cv::Mat>>> frames_per_cam;
    {
        RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = cfg_.ros2_bag_file;
        ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
        if (!cfg_.camera_topics.empty())
            ros_cfg.camera_topics = cfg_.camera_topics;
        else if (!cfg_.camera_id.empty())
            ros_cfg.camera_topics[cfg_.camera_id] = cfg_.camera_ros2_topic;
        // camera_topics 为空时：不设置 ros_cfg.camera_topics，数据源会加载 bag 内全部 Image 话题
        ros_cfg.max_frames = cfg_.ros2_max_frames;
        UnifiedDataLoader::Config load_cfg;
        load_cfg.source_type = UnifiedDataLoader::SourceType::ROS2_BAG;
        load_cfg.ros_config = ros_cfg;
        UnifiedDataLoader loader(load_cfg);
        if (!loader.load()) {
            r.success = false;
            r.message = "ROS2 数据加载失败: " + loader.get_status_message();
            r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
            return r;
        }
        // 离线 bag 且未配置相机列表时：直接使用 loader 中已加载的全部相机
        if (camera_ids.empty()) {
            camera_ids = loader.get_camera_ids();
            std::sort(camera_ids.begin(), camera_ids.end());
            UNICALIB_INFO("[Fine-Auto/Cam-Cam] ROS2 bag 离线模式，使用 bag 内全部相机: {} 个", camera_ids.size());
        }
        for (const auto& cid : camera_ids) {
            auto fr = loader.to_camera_frames(cid);
            if (!fr.empty())
                frames_per_cam[cid] = std::move(fr);
        }
    }

    if (camera_ids.size() < 2u) {
        r.success = true;
        r.message = "Camera-Camera 标定需要至少 2 个相机，当前: " + std::to_string(camera_ids.size());
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        UNICALIB_WARN("[Fine-Auto/Cam-Cam] {}", r.message);
        return r;
    }

    UNICALIB_INFO("[Fine-Auto/Cam-Cam] 相机数: {}，将依次标定相邻对: (0-1), (1-2), ...", camera_ids.size());

    if (frames_per_cam.size() < 2u) {
        r.success = false;
        r.message = "至少需要 2 个相机有有效帧数据，当前: " + std::to_string(frames_per_cam.size());
        r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
        UNICALIB_ERROR("[Fine-Auto/Cam-Cam] {}", r.message);
        return r;
    }

    // 按 camera_ids 顺序取有数据的相机，保证相邻对
    std::vector<std::string> ordered;
    for (const auto& cid : camera_ids) {
        if (frames_per_cam.count(cid))
            ordered.push_back(cid);
    }

    CamCamCalibrator::Config calib_cfg;
    calib_cfg.target.cols = cfg_.board_cols;
    calib_cfg.target.rows = cfg_.board_rows;
    calib_cfg.target.square_size_m = cfg_.square_size_m;
    calib_cfg.max_rms_px = cfg_.cam_cam_rms_threshold;
    calib_cfg.method = cfg_.prefer_targetfree ? CamCamCalibrator::Method::ESSENTIAL_MATRIX : CamCamCalibrator::Method::CHESSBOARD_STEREO;

    auto load_intrin = [this](const std::string& id) -> std::optional<CameraIntrinsics> {
        if (params_->camera_intrinsics.count(id) && params_->camera_intrinsics.at(id))
            return *params_->camera_intrinsics.at(id);
        auto it = cfg_.camera_intrinsic_files.find(id);
        if (it != cfg_.camera_intrinsic_files.end() && fs::exists(it->second)) {
            try {
                return YamlIO::load_camera_intrinsics(it->second);
            } catch (...) {}
        }
        return std::nullopt;
    };

    int pairs_done = 0;
    double max_rms = 0.0;
    for (size_t i = 0; i + 1 < ordered.size(); ++i) {
        const std::string& id0 = ordered[i];
        const std::string& id1 = ordered[i + 1];
        auto in0 = load_intrin(id0);
        auto in1 = load_intrin(id1);
        if (!in0.has_value() || !in1.has_value()) {
            UNICALIB_WARN("[Fine-Auto/Cam-Cam] 跳过 {}->{}: 缺少内参 (请先做相机内参标定或配置 camera_intrinsic_files)", id0, id1);
            continue;
        }

        CamCamCalibrator calib(calib_cfg);
        auto result = calib.calibrate_two_stage(
            frames_per_cam.at(id0), frames_per_cam.at(id1),
            *in0, *in1, cfg_.prefer_targetfree, id0, id1);

        if (result.best()) {
            auto ext = params_->get_or_create_extrinsic(id0, id1);
            ext->ref_sensor_id = id0;
            ext->target_sensor_id = id1;
            ext->set_SE3(result.best()->SE3_TargetInRef());
            ext->residual_rms = result.best_rms();
            ext->is_converged = (result.best_rms() <= cfg_.cam_cam_rms_threshold);
            max_rms = std::max(max_rms, result.best_rms());
            pairs_done++;
            UNICALIB_INFO("[Fine-Auto/Cam-Cam] {} -> {} 完成, rms={:.4f} px", id0, id1, result.best_rms());
        } else {
            UNICALIB_WARN("[Fine-Auto/Cam-Cam] {} -> {} 未收敛", id0, id1);
        }
    }

    r.success = (pairs_done > 0);
    r.residual_rms = max_rms;
    r.message = "Camera-Camera 标定完成: " + std::to_string(pairs_done) + " 对, 最大 RMS=" + (r.success ? std::to_string(max_rms) + " px" : "N/A");
    r.elapsed_ms = std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t_start).count();
    UNICALIB_INFO("[Fine-Auto/Cam-Cam] {}", r.message);
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

    ::YAML::Emitter out;
    out << ::YAML::BeginMap;

    out << ::YAML::Key << "calibration_type" << ::YAML::Value << "lidar_camera_extrinsic";
    out << ::YAML::Key << "reference_sensor" << ::YAML::Value << cfg_.lidar_id;
    out << ::YAML::Key << "target_sensor" << ::YAML::Value << cfg_.camera_id;
    out << ::YAML::Key << "timestamp" << ::YAML::Value << now_str();

    if (result.coarse.has_value()) {
        out << ::YAML::Key << "coarse_result";
        out << ::YAML::BeginMap;
        const auto& T = result.coarse->SE3_TargetInRef();
        out << ::YAML::Key << "method" << ::YAML::Value << result.coarse_method;
        out << ::YAML::Key << "rms" << ::YAML::Value << result.coarse_rms;
        out << ::YAML::Key << "translation" << ::YAML::Flow << ::YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << ::YAML::EndSeq;
        auto rpy = T.so3().log();
        out << ::YAML::Key << "rotation_rpy" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << ::YAML::EndSeq;
        out << ::YAML::EndMap;
    }

    if (result.fine.has_value()) {
        out << ::YAML::Key << "fine_result";
        out << ::YAML::BeginMap;
        const auto& T = result.fine->SE3_TargetInRef();
        out << ::YAML::Key << "method" << ::YAML::Value << result.fine_method;
        out << ::YAML::Key << "rms" << ::YAML::Value << result.fine_rms;
        out << ::YAML::Key << "converged" << ::YAML::Value << result.fine->is_converged;
        out << ::YAML::Key << "translation" << ::YAML::Flow << ::YAML::BeginSeq
            << T.translation().x() << T.translation().y() << T.translation().z() << ::YAML::EndSeq;
        auto rpy = T.so3().log();
        out << ::YAML::Key << "rotation_rpy_rad" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() << rpy.y() << rpy.z() << ::YAML::EndSeq;
        out << ::YAML::Key << "rotation_rpy_deg" << ::YAML::Flow << ::YAML::BeginSeq
            << rpy.x() * 180 / M_PI << rpy.y() * 180 / M_PI << rpy.z() * 180 / M_PI << ::YAML::EndSeq;
        out << ::YAML::EndMap;
    }

    out << ::YAML::Key << "needs_manual_refine" << ::YAML::Value << result.needs_manual;
    out << ::YAML::Key << "manual_threshold_px" << ::YAML::Value << result.manual_threshold_px;

    out << ::YAML::EndMap;

    std::ofstream f(path);
    if (f.is_open()) {
        f << out.c_str();
        UNICALIB_INFO("[Fine-Auto] 结果已保存: {}", path);
    }
}

// ---------------------------------------------------------------------------
// IMU 内参精标定实现
// ---------------------------------------------------------------------------
StageResult CalibPipeline::run_fine_imu_intrinsic() {
    StageResult r;
    r.stage = CalibStage::FINE_AUTO;
    r.task  = CalibTaskType::IMU_INTRINSIC;
    r.quality_threshold = cfg_.imu_intrin_rms_threshold;

    auto t_start = std::chrono::high_resolution_clock::now();

    UNICALIB_INFO("[Fine-Auto/IMU-Intrin] 开始执行IMU内参标定...");

    // ─── 1. 数据加载 ─────────────────────────────────────────────────
    ns_unicalib::IMURawData imu_data;

    if (cfg_.use_ros2_bag && !cfg_.ros2_bag_file.empty()) {
        UNICALIB_INFO("  数据源类型: ROS2 Bag 文件");
        UNICALIB_INFO("  ROS2 Bag 文件: {}", cfg_.ros2_bag_file);

        if (!fs::exists(cfg_.ros2_bag_file)) {
            r.success = false;
            r.message = "ROS2 Bag 路径不存在: " + cfg_.ros2_bag_file;
            return r;
        }

        RosDataSourceConfig ros_cfg;
        ros_cfg.bag_file = cfg_.ros2_bag_file;
        ros_cfg.realtime_mode = false;
        ros_cfg.max_frames = 0;  // 不限制，IMU 内参需要尽量多静置数据
        ros_cfg.strict_topic_match = cfg_.ros2_strict_topic_match;
        if (!cfg_.imu_topics.empty()) {
            ros_cfg.imu_topics = cfg_.imu_topics;
            ros_cfg.imu_ros2_topic = cfg_.imu_topics.count(cfg_.imu_sensor_id) ?
                cfg_.imu_topics.at(cfg_.imu_sensor_id) : cfg_.imu_topics.begin()->second;
        } else if (!cfg_.imu_ros2_topic.empty()) {
            ros_cfg.imu_topics[cfg_.imu_sensor_id] = cfg_.imu_ros2_topic;
            ros_cfg.imu_ros2_topic = cfg_.imu_ros2_topic;
        }

        UnifiedDataLoader::Config unified_cfg;
        unified_cfg.source_type = UnifiedDataLoader::SourceType::ROS2_BAG;
        unified_cfg.ros_config = ros_cfg;
        unified_cfg.max_frames = 0;

        UnifiedDataLoader loader(unified_cfg);
        if (!loader.load()) {
            r.success = false;
            r.message = "ROS2 数据加载失败: " + loader.get_status_message();
            return r;
        }

        imu_data = loader.to_imu_raw_data(cfg_.imu_sensor_id);
        UNICALIB_INFO("  加载IMU数据: {} 帧 (sensor_id={})", imu_data.size(), cfg_.imu_sensor_id);

    } else if (!cfg_.imu_data_file.empty()) {
        UNICALIB_INFO("  数据源类型: CSV文件");
        UNICALIB_INFO("  IMU数据文件: {}", cfg_.imu_data_file);
        
        // TODO: 实现CSV加载功能
        UNICALIB_ERROR("CSV数据加载功能暂未实现");
        r.success = false;
        r.message = "CSV数据加载未实现";
        return r;
    } else {
        UNICALIB_ERROR("未指定数据源 (ros2_bag_file 或 imu_data_file)");
        r.success = false;
        r.message = "数据源未配置";
        return r;
    }

    if (imu_data.empty()) {
        UNICALIB_ERROR("IMU数据为空，请检查ROS2 bag文件是否包含IMU数据");
        r.success = false;
        r.message = "IMU数据加载失败或bag文件中无IMU数据";
        return r;
    }

    // ─── 2. Allan方差分析 ─────────────────────────────────────────────
    UNICALIB_LOG_STEP("IMU-Intrin", "Step 1: Allan 方差分析...");
    
    ns_unicalib::IMUIntrinsicCalibrator calibrator(cfg_.imu_intrinsic_calib_cfg);
    
    if (progress_cb_) {
        calibrator.set_progress_callback([&](const std::string& stage, double progress) {
            progress_cb_(CalibStage::FINE_AUTO, "Allan方差分析: " + stage, 0.1 + 0.6 * progress);
        });
    }
    
    auto intrinsics = calibrator.calibrate(imu_data);

    // ─── 3. 保存结果 ─────────────────────────────────────────────────
    UNICALIB_LOG_STEP("IMU-Intrin", "Step 2: 保存标定结果...");

    std::string output_yaml = cfg_.output_dir + "/imu_intrinsic.yaml";
    save_imu_intrinsic_yaml(intrinsics, output_yaml);

    // 保存 Allan 偏差图（analyze_allan 与 calibrate 内部逻辑一致，仅用于绘图）
    ns_unicalib::AllanResult allan_result = calibrator.analyze_allan(imu_data);
    if (!allan_result.gyro[0].taus.empty()) {
        std::string plot_path = cfg_.output_dir + "/imu_allan_variance.png";
        calibrator.save_allan_plot(allan_result, plot_path);
    }

    auto t_end = std::chrono::high_resolution_clock::now();
    r.elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    r.success = true;
    r.residual_rms = intrinsics.allan_fit_rms;
    std::ostringstream msg_oss;
    msg_oss << "IMU内参标定完成: gyro_noise=" << intrinsics.noise_gyro
            << " rad/s/√Hz, gyro_bias=" << intrinsics.bias_instab_gyro
            << " rad/s, accel_noise=" << intrinsics.noise_acce
            << " m/s²/√Hz, accel_bias=" << intrinsics.bias_instab_acce << " m/s²";
    r.message = msg_oss.str();
    
    UNICALIB_INFO("[Fine-Auto/IMU-Intrin] 标定完成: RMS={:.4f} | 耗时: {:.1f} ms", 
                  r.residual_rms, r.elapsed_ms);
    return r;
}

// ---------------------------------------------------------------------------
// 保存 IMU 内参结果到 YAML
// ---------------------------------------------------------------------------
void CalibPipeline::save_imu_intrinsic_yaml(const ns_unicalib::IMUIntrinsics& intrinsics,
                                           const std::string& path) const {
    ::YAML::Emitter out;
    out << ::YAML::BeginMap;
    out << ::YAML::Key << "imu_id" << ::YAML::Value << cfg_.imu_sensor_id;

    // 陀螺仪参数 (noise_density: rad/s/√Hz, bias_instability: rad/s)
    out << ::YAML::Key << "gyroscope" << ::YAML::BeginMap;
    out << ::YAML::Key << "noise_density" << ::YAML::Value << intrinsics.noise_gyro;
    out << ::YAML::Key << "bias_instability" << ::YAML::Value << intrinsics.bias_instab_gyro;
    out << ::YAML::Key << "random_walk" << ::YAML::Value << 0.0;
    out << ::YAML::EndMap;

    // 加速度计参数 (noise_density: m/s²/√Hz, bias_instability: m/s²)
    out << ::YAML::Key << "accelerometer" << ::YAML::BeginMap;
    out << ::YAML::Key << "noise_density" << ::YAML::Value << intrinsics.noise_acce;
    out << ::YAML::Key << "bias_instability" << ::YAML::Value << intrinsics.bias_instab_acce;
    out << ::YAML::Key << "random_walk" << ::YAML::Value << 0.0;
    out << ::YAML::EndMap;

    // 元数据
    out << ::YAML::Key << "metadata" << ::YAML::BeginMap;
    out << ::YAML::Key << "method" << ::YAML::Value << "allan_variance";
    out << ::YAML::Key << "num_samples" << ::YAML::Value << intrinsics.num_samples_used;
    out << ::YAML::Key << "allan_fit_rms" << ::YAML::Value << intrinsics.allan_fit_rms;
    out << ::YAML::EndMap;

    out << ::YAML::EndMap;

    std::ofstream f(path);
    if (f.is_open()) {
        f << out.c_str();
        UNICALIB_INFO("[Fine-Auto] IMU内参已保存: {}", path);
    } else {
        UNICALIB_ERROR("[Fine-Auto] 无法保存IMU内参到: {}", path);
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
