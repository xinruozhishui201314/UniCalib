/**
 * UniCalib Unified — CalibPipeline 实现
 *
 * 核心流程:
 *   1. 为每个 stage 创建独立日志文件 (output_dir/logs/<stage>_<task>_<ts>.log)
 *   2. 记录阶段开始/结束时间、残差、收敛状态
 *   3. 汇总报告写入 output_dir/pipeline_report_<ts>.yaml
 */

#include "unicalib/pipeline/calib_pipeline.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

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
// 精标定阶段入口 (占位实现)
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

    r.success  = true;
    r.message  = "精标定占位 — 请通过具体标定器实现";
    r.elapsed_ms = 0.0;
    return r;
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
