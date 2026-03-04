#pragma once
/**
 * UniCalib Unified — 手动校准接口
 *
 * 功能:
 *   1. ManualExtrinsicAdjuster: 6-DOF 外参交互式微调
 *      - 基于 click_calib 思路: 用户在图像/投影上点击对应点
 *      - 支持键盘增量调整 (roll/pitch/yaw/tx/ty/tz)
 *      - 实时显示投影结果 (LiDAR点云→相机图像, 相机→相机)
 *      - 基于用户点击对应点进行非线性最小二乘精化
 *
 *   2. ManualClickRefiner: click-calib 风格人工对应点采集 + 优化
 *      - 针对 LiDAR-Camera: 点云投影图 + RGB图对应点
 *      - 针对 Camera-Camera: 双目图像对应点
 *      - 针对 IMU-LiDAR: 通过可视化旋转验证
 *
 *   3. ManualCalibSession: 完整手动标定会话管理
 *      - 记录每次调整历史 (可撤销/重做)
 *      - 保存/恢复会话状态
 *      - 评估调整后的误差
 *
 * 适用场景:
 *   - 自动标定收敛到局部最优
 *   - 特殊安装结构导致自动方法退化
 *   - 实车标定时快速验证与微调
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include <opencv2/core.hpp>
#include <functional>
#include <optional>
#include <deque>
#include <string>
#include <vector>

namespace ns_unicalib {

// ===========================================================================
// 手动调整增量步长配置
// ===========================================================================
struct ManualAdjustStep {
    double rot_step_deg  = 0.1;   // 旋转步长 [deg]
    double trans_step_m  = 0.005; // 平移步长 [m]
    // 快速模式 (Shift键)
    double rot_fast_deg  = 1.0;
    double trans_fast_m  = 0.05;
};

// ===========================================================================
// 手动调整命令
// ===========================================================================
enum class AdjustCmd {
    ROLL_PLUS, ROLL_MINUS,
    PITCH_PLUS, PITCH_MINUS,
    YAW_PLUS, YAW_MINUS,
    TX_PLUS, TX_MINUS,
    TY_PLUS, TY_MINUS,
    TZ_PLUS, TZ_MINUS,
    UNDO,
    ACCEPT,
    CANCEL,
};

// ===========================================================================
// 外参手动微调器
// ===========================================================================
class ManualExtrinsicAdjuster {
public:
    struct Config {
        ManualAdjustStep step;
        bool  show_window    = true;   // 显示 OpenCV 窗口
        bool  enable_undo    = true;   // 支持撤销
        int   max_undo_depth = 50;
        double quality_warn_threshold = 3.0;  // 残差超过此值时警告
        std::string window_title = "UniCalib Manual Adjust";
    };

    explicit ManualExtrinsicAdjuster(const Config& cfg) : cfg_(cfg) {}
    ManualExtrinsicAdjuster() : cfg_(Config{}) {}

    // -----------------------------------------------------------------------
    // 设置初始外参 (从自动标定结果载入)
    // -----------------------------------------------------------------------
    void set_initial_extrinsic(const ExtrinsicSE3& extrin);

    // -----------------------------------------------------------------------
    // 应用一次增量调整
    // -----------------------------------------------------------------------
    void apply(AdjustCmd cmd, bool fast_mode = false);
    void apply_delta_rotation(double roll_deg, double pitch_deg, double yaw_deg);
    void apply_delta_translation(double dx_m, double dy_m, double dz_m);

    // -----------------------------------------------------------------------
    // 撤销/重做
    // -----------------------------------------------------------------------
    bool undo();
    bool redo();

    // -----------------------------------------------------------------------
    // 获取当前外参
    // -----------------------------------------------------------------------
    const ExtrinsicSE3& current() const { return current_; }
    ExtrinsicSE3& current() { return current_; }

    // -----------------------------------------------------------------------
    // 打印当前外参状态
    // -----------------------------------------------------------------------
    void print_current() const;

    // -----------------------------------------------------------------------
    // 序列化当前外参到 YAML
    // -----------------------------------------------------------------------
    void save(const std::string& path) const;
    void load(const std::string& path);

private:
    Config cfg_;
    ExtrinsicSE3 current_;
    std::deque<ExtrinsicSE3> undo_stack_;
    std::deque<ExtrinsicSE3> redo_stack_;
};

// ===========================================================================
// Click-Calib 风格人工对应点采集
// ===========================================================================
struct ClickPoint2D3D {
    Eigen::Vector2d img_pt;   // 图像像素坐标
    Eigen::Vector3d world_pt; // LiDAR/世界坐标 (可选, 仅 LiDAR-Cam 场景)
    bool has_3d = false;
};

struct ClickCorrespondence {
    // 对于 Cam-Cam: 两张图像中对应的像素点
    Eigen::Vector2d pt_cam0;
    Eigen::Vector2d pt_cam1;
    double confidence = 1.0;
};

class ManualClickRefiner {
public:
    struct Config {
        int    min_points_lidar_cam = 8;  // LiDAR-Cam 最少对应点数
        int    min_points_cam_cam   = 10; // Cam-Cam 最少对应点数
        double ransac_thresh_px     = 2.0;
        int    ceres_max_iter       = 100;
        bool   show_residual_map    = true; // 显示残差热力图
        std::string output_dir      = "./calib_output/manual";
    };

    explicit ManualClickRefiner(const Config& cfg) : cfg_(cfg) {}
    ManualClickRefiner() : cfg_(Config{}) {}

    // -----------------------------------------------------------------------
    // LiDAR-Camera: 在点云投影图上采集对应点
    // 交互方式: 左键点击图像中的角点, 右键选择点云中对应3D点
    // -----------------------------------------------------------------------
    std::optional<ExtrinsicSE3> refine_lidar_cam(
        const LiDARScan& scan,
        const cv::Mat& image,
        const CameraIntrinsics& cam_intrin,
        const ExtrinsicSE3& init_extrin,
        const std::vector<ClickPoint2D3D>& user_clicks = {});

    // -----------------------------------------------------------------------
    // Camera-Camera: 在双目图像上采集对应点
    // 交互方式: 左键在 cam0 上点击, 右键在 cam1 上点击对应点
    // -----------------------------------------------------------------------
    std::optional<ExtrinsicSE3> refine_cam_cam(
        const cv::Mat& img_cam0,
        const cv::Mat& img_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& init_extrin,
        const std::vector<ClickCorrespondence>& user_clicks = {});

    // -----------------------------------------------------------------------
    // IMU-LiDAR: 通过旋转可视化验证 (无对应点, 纯视觉验证)
    // -----------------------------------------------------------------------
    bool visualize_imu_lidar(
        const std::vector<LiDARScan>& scans,
        const std::vector<IMUFrame>& imu_data,
        const ExtrinsicSE3& extrin);

    // -----------------------------------------------------------------------
    // 从文件加载/保存用户点击结果
    // -----------------------------------------------------------------------
    void save_clicks(const std::string& path,
                     const std::vector<ClickCorrespondence>& clicks) const;
    std::vector<ClickCorrespondence> load_clicks(const std::string& path) const;

    // 评估当前标定的重投影误差
    double evaluate_lidar_cam(
        const LiDARScan& scan,
        const cv::Mat& image,
        const CameraIntrinsics& cam_intrin,
        const ExtrinsicSE3& extrin) const;

    double evaluate_cam_cam(
        const std::vector<ClickCorrespondence>& clicks,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& extrin) const;

private:
    Config cfg_;

    // Ceres 优化: 最小化点对点重投影误差
    std::optional<ExtrinsicSE3> optimize_from_clicks(
        const std::vector<ClickPoint2D3D>& clicks,
        const CameraIntrinsics& cam_intrin,
        const ExtrinsicSE3& init_extrin,
        const std::string& log_prefix);

    std::optional<ExtrinsicSE3> optimize_cam_cam_from_clicks(
        const std::vector<ClickCorrespondence>& clicks,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& init_extrin,
        const std::string& log_prefix);

    // 可视化投影结果
    cv::Mat render_lidar_projection(
        const LiDARScan& scan,
        const cv::Mat& image,
        const CameraIntrinsics& cam_intrin,
        const ExtrinsicSE3& extrin) const;

    cv::Mat render_stereo_epipolar(
        const cv::Mat& img0,
        const cv::Mat& img1,
        const std::vector<ClickCorrespondence>& clicks,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& extrin) const;
};

// ===========================================================================
// 完整手动标定会话 (含历史管理 + 评估 + 保存)
// ===========================================================================
class ManualCalibSession {
public:
    struct SessionConfig {
        std::string session_id;             // 唯一标识, 空则自动生成时间戳
        std::string save_dir = "./calib_output/manual_sessions";
        bool auto_save = true;              // 每次调整后自动保存
        bool log_all_steps = true;          // 详细记录每步
    };

    explicit ManualCalibSession(const SessionConfig& cfg);
    ManualCalibSession();

    // -----------------------------------------------------------------------
    // 启动手动校准会话 (对应 LiDAR-Cam / Cam-Cam / IMU-LiDAR)
    // -----------------------------------------------------------------------
    ExtrinsicSE3 run_lidar_cam(
        const LiDARScan& scan,
        const cv::Mat& image,
        const CameraIntrinsics& cam_intrin,
        const ExtrinsicSE3& auto_result,
        double auto_rms);

    ExtrinsicSE3 run_cam_cam(
        const cv::Mat& img_cam0,
        const cv::Mat& img_cam1,
        const CameraIntrinsics& intrin0,
        const CameraIntrinsics& intrin1,
        const ExtrinsicSE3& auto_result,
        double auto_rms);

    ExtrinsicSE3 run_imu_lidar(
        const std::vector<LiDARScan>& scans,
        const std::vector<IMUFrame>& imu_data,
        const ExtrinsicSE3& auto_result,
        double auto_rms);

    // -----------------------------------------------------------------------
    // 会话持久化
    // -----------------------------------------------------------------------
    void save_session(const std::string& path = "") const;
    bool load_session(const std::string& path);

    // -----------------------------------------------------------------------
    // 导出最终结果
    // -----------------------------------------------------------------------
    void export_results(CalibParamManager& pm) const;

    const std::string& session_id() const { return session_id_; }

private:
    void log_session_event(const std::string& event,
                           const std::string& detail = "");

    SessionConfig cfg_;
    std::string session_id_;
    ManualExtrinsicAdjuster adjuster_;
    ManualClickRefiner click_refiner_;

    struct SessionEvent {
        std::string timestamp;
        std::string event;
        std::string detail;
        ExtrinsicSE3 extrin_snapshot;
        double rms_snapshot = 0.0;
    };
    std::vector<SessionEvent> history_;
    std::map<std::string, ExtrinsicSE3> final_extrinsics_;
};

}  // namespace ns_unicalib
