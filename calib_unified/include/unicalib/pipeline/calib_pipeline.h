#pragma once
/**
 * UniCalib Unified — 两阶段标定流水线
 *
 * 架构:
 *   Stage 1 (粗标定, Coarse): AI模型提供初始值
 *     - DM-Calib        → 相机内参粗估
 *     - MIAS-LCEC       → LiDAR-Camera 外参粗估 (跨模态掩码匹配)
 *     - Transformer-IMU → IMU 内参粗估
 *     - L2Calib         → IMU-LiDAR 外参粗估 (强化学习)
 *
 *   Stage 2 (精标定, Fine): 无目标优化为首选
 *     - IMU内参    → Allan方差分析
 *     - 相机内参   → 无目标 (DM-Calib 精化) / 棋盘格
 *     - IMU-LiDAR → B样条连续时间
 *     - LiDAR-Cam → 边缘对齐互信息 (无目标优先)
 *     - Cam-Cam   → Bundle Adjustment (无目标优先)
 *
 *   Stage 3 (手动校准, Manual, 可选):
 *     当自动标定精度不足时, 提供6-DOF交互式调整
 *
 * 日志系统:
 *   每个环节均有 TRACE/DEBUG/INFO/WARN/ERROR 层级日志
 *   支持阶段耗时统计 / 收敛状态 / 残差指标
 */

#include "unicalib/common/logger.h"
#include "unicalib/common/calib_param.h"
#include <chrono>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace ns_unicalib {

// ===========================================================================
// 标定任务类型 (可灵活组合)
// ===========================================================================
enum class CalibTaskType : uint32_t {
    NONE            = 0x00,
    IMU_INTRINSIC   = 0x01,  // IMU 内参
    CAM_INTRINSIC   = 0x02,  // 相机内参
    IMU_LIDAR_EXTRIN = 0x04, // IMU-LiDAR 外参
    LIDAR_CAM_EXTRIN = 0x08, // LiDAR-Camera 外参
    CAM_CAM_EXTRIN  = 0x10,  // Camera-Camera 外参
    ALL             = 0x1F,  // 所有
};

inline CalibTaskType operator|(CalibTaskType a, CalibTaskType b) {
    return static_cast<CalibTaskType>(
        static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
inline bool has_task(CalibTaskType tasks, CalibTaskType check) {
    return (static_cast<uint32_t>(tasks) & static_cast<uint32_t>(check)) != 0;
}

// ===========================================================================
// 标定阶段枚举
// ===========================================================================
enum class CalibStage {
    COARSE_AI,      // AI 粗标定
    FINE_AUTO,      // 自动精标定
    MANUAL_REFINE,  // 手动精校准
};

inline const char* stage_name(CalibStage s) {
    switch (s) {
        case CalibStage::COARSE_AI:     return "Coarse-AI";
        case CalibStage::FINE_AUTO:     return "Fine-Auto";
        case CalibStage::MANUAL_REFINE: return "Manual-Refine";
    }
    return "Unknown";
}

// ===========================================================================
// 阶段执行结果
// ===========================================================================
struct StageResult {
    CalibStage stage;
    CalibTaskType task;
    bool success = false;
    double residual_rms = 0.0;       // 残差均方根
    double elapsed_ms   = 0.0;       // 耗时 [ms]
    std::string message;             // 人类可读状态描述
    std::string log_file;            // 本阶段详细日志路径

    // 质量阈值 (供下游判断是否需要手动校准)
    double quality_threshold = -1.0; // < 0 表示不评估
    bool needs_manual_refine() const {
        return quality_threshold > 0.0 && residual_rms > quality_threshold;
    }
};

// ===========================================================================
// 流水线总结报告
// ===========================================================================
struct PipelineReport {
    std::string pipeline_id;                    // e.g. "lidar_cam_20260303_142305"
    std::vector<StageResult> stage_results;
    CalibParamManager::Ptr final_params;

    // 指标摘要
    bool all_converged() const {
        for (const auto& r : stage_results) {
            if (!r.success) return false;
        }
        return true;
    }
    double total_elapsed_ms() const {
        double total = 0;
        for (const auto& r : stage_results) total += r.elapsed_ms;
        return total;
    }
    void print_summary() const;
    void save_report(const std::string& path) const;
};

// ===========================================================================
// 进度回调
// ===========================================================================
using StageProgressCb = std::function<void(
    CalibStage stage, const std::string& step, double progress_0_1)>;

// ===========================================================================
// 流水线配置
// ===========================================================================
struct PipelineConfig {
    // 要执行的任务组合
    CalibTaskType tasks = CalibTaskType::ALL;

    // 是否启用 AI 粗标定 (各子项)
    bool enable_coarse_imu_intrin   = true;  // Transformer-IMU-Calibrator
    bool enable_coarse_cam_intrin   = true;  // DM-Calib
    bool enable_coarse_imu_lidar    = true;  // learn-to-calibrate (L2Calib)
    bool enable_coarse_lidar_cam    = true;  // MIAS-LCEC
    bool enable_coarse_cam_cam      = false; // 暂无专用模型, 用特征匹配代替

    // Python 解释器路径 (用于调用 AI 模型)
    std::string python_executable   = "python3";
    // AI 模型根目录 (相对于 workspace)
    std::string ai_models_root      = "../";

    // 是否允许手动校准降级 (当精标定残差超过阈值时提示用户)
    bool allow_manual_fallback      = true;

    // 残差质量阈值 (超过则建议手动校准)
    double lidar_cam_rms_threshold  = 2.0;   // px
    double cam_cam_rms_threshold    = 1.5;   // px
    double imu_lidar_rot_threshold  = 0.5;   // deg

    // 输出目录
    std::string output_dir          = "./calib_output";

    // 日志级别: trace/debug/info/warn/error
    std::string log_level           = "info";

    // 无目标优先 (false 则允许使用靶标方法)
    bool prefer_targetfree          = true;
};

// ===========================================================================
// 两阶段标定流水线主类
// ===========================================================================
class CalibPipeline {
public:
    using Ptr = std::shared_ptr<CalibPipeline>;

    explicit CalibPipeline(const PipelineConfig& cfg);
    CalibPipeline();

    // -----------------------------------------------------------------------
    // 初始化参数管理器 (从外部传入或内部创建)
    // -----------------------------------------------------------------------
    void set_param_manager(CalibParamManager::Ptr pm) { params_ = pm; }
    CalibParamManager::Ptr get_param_manager() const { return params_; }

    // -----------------------------------------------------------------------
    // 进度回调 (UI/日志集成)
    // -----------------------------------------------------------------------
    void set_progress_callback(StageProgressCb cb) { progress_cb_ = cb; }

    // -----------------------------------------------------------------------
    // 阶段日志记录 (每个环节细粒度记录)
    // -----------------------------------------------------------------------
    void log_stage_begin(CalibStage stage, CalibTaskType task,
                         const std::string& detail = "");
    void log_stage_end(const StageResult& result);
    void log_step(CalibStage stage, const std::string& step,
                  const std::string& msg, spdlog::level::level_enum lv
                      = spdlog::level::info);
    void log_metric(const std::string& name, double value,
                    const std::string& unit = "");
    void log_param_change(const std::string& param_name,
                          const std::string& before,
                          const std::string& after);

    // -----------------------------------------------------------------------
    // 运行完整流水线
    // -----------------------------------------------------------------------
    PipelineReport run();

    // -----------------------------------------------------------------------
    // 单阶段入口 (细粒度控制)
    // -----------------------------------------------------------------------
    StageResult run_coarse_stage(CalibTaskType task);
    StageResult run_fine_stage(CalibTaskType task);
    StageResult run_manual_stage(CalibTaskType task);

    // -----------------------------------------------------------------------
    // 获取最终报告
    // -----------------------------------------------------------------------
    const PipelineReport& get_report() const { return report_; }

protected:
    // -----------------------------------------------------------------------
    // 内部工具
    // -----------------------------------------------------------------------
    std::string make_stage_log_path(CalibStage stage, CalibTaskType task) const;
    void setup_stage_logger(const std::string& log_path);

    using Clock = std::chrono::steady_clock;
    Clock::time_point stage_start_;

    PipelineConfig cfg_;
    CalibParamManager::Ptr params_;
    StageProgressCb progress_cb_;
    PipelineReport report_;

    // 阶段子日志 (每个 stage 独立文件)
    std::map<std::string, std::shared_ptr<spdlog::logger>> stage_loggers_;
};

}  // namespace ns_unicalib
