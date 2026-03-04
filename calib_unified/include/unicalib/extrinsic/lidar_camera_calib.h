#pragma once
/**
 * UniCalib Unified — LiDAR-Camera 外参标定
 *
 * 两阶段标定流程:
 *   粗标定 (Coarse): MIAS-LCEC AI 模型 (跨模态掩码匹配 C3M)
 *                    → 提供初始外参估计
 *   精标定 (Fine): 无目标优先
 *     Method A (运动法, B样条): 需要 IMU-LiDAR 外参已知
 *     Method B (目标法, 棋盘格): 3D-2D 对应点 PnP
 *     Method C (无目标边缘法, 优先): LiDAR强度图边缘 vs 相机边缘 — 最大化互信息
 *   手动校准 (Manual): 6-DOF 交互式调整 + click-calib 点击精化
 *
 * 日志: 每个阶段有独立 spdlog 日志, 记录迭代残差/收敛状态
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <optional>
#include <functional>

namespace ns_unicalib {

// ===================================================================
// 3D-2D 对应点
// ===================================================================
struct Point3D2DCorr {
    Eigen::Vector3d point_3d;   // LiDAR 坐标系下的 3D 点
    Eigen::Vector2d point_2d;   // 图像坐标系下的 2D 点 [px]
    double weight = 1.0;
};

// ===================================================================
// 标定板检测结果
// ===================================================================
struct CalibBoardDetection {
    double timestamp;
    std::vector<Eigen::Vector3d> corners_3d;    // LiDAR 坐标系
    std::vector<Eigen::Vector2d> corners_2d;    // 图像坐标系
    cv::Mat image;
    bool valid = false;
};

// ===================================================================
// 边缘对齐评估
// ===================================================================
struct EdgeAlignmentScore {
    double mutual_information = 0.0;
    double edge_overlap_ratio = 0.0;
    double chamfer_distance   = 0.0;
    double ncc_score          = 0.0;  // 归一化互相关
    int num_lidar_edge_pts    = 0;
    int num_image_edge_pts    = 0;
};

// ===================================================================
// LiDAR-Camera 外参标定器
// ===================================================================
class LiDARCameraCalibrator {
public:
    enum class Method {
        MOTION_BSPLINE,     // 运动法 (需要 IMU 数据和 IMU-LiDAR 外参)
        TARGET_CHESSBOARD,  // 棋盘格目标法
        EDGE_ALIGNMENT,     // 无目标边缘对齐
    };

    struct Config {
        Method method = Method::TARGET_CHESSBOARD;

        // 棋盘格配置
        int board_cols = 9, board_rows = 6;
        double square_size_m = 0.025;

        // B样条运动法
        double spline_dt_s = 0.1;
        int    spline_order = 4;
        double time_offset_init_s = 0.0;
        bool   optimize_time_offset = true;
        bool   optimize_intrinsic = false;  // 是否同时优化相机内参

        // 边缘对齐法
        int    edge_canny_low = 50;
        int    edge_canny_high = 150;
        double lidar_intensity_threshold = 10.0;
        int    optimizer_max_iter = 50;

        // 通用
        double max_reproj_error_px = 3.0;
        int    ceres_max_iter = 50;
        bool   verbose = true;
    };

    explicit LiDARCameraCalibrator(const Config& cfg) : cfg_(cfg) {}
    LiDARCameraCalibrator() : cfg_(Config{}) {}

    // ===================================================================
    // 两阶段标定接口 (推荐使用)
    // ===================================================================

    // 两阶段结果 (粗标定 + 精标定)
    struct TwoStageResult {
        // 粗标定 (AI初始值)
        std::optional<ExtrinsicSE3> coarse;
        double coarse_rms = -1.0;
        std::string coarse_method;     // "MIAS-LCEC" / "edge_init" / "identity"

        // 精标定
        std::optional<ExtrinsicSE3> fine;
        double fine_rms = -1.0;
        std::string fine_method;       // "EDGE_ALIGNMENT" / "TARGET_CHESSBOARD" / "MOTION_BSPLINE"

        // 是否需要手动校准
        bool needs_manual = false;
        double manual_threshold_px = 2.0;

        // 最终推荐结果
        const ExtrinsicSE3* best() const {
            if (fine.has_value())   return &fine.value();
            if (coarse.has_value()) return &coarse.value();
            return nullptr;
        }
        double best_rms() const {
            if (fine_rms > 0)   return fine_rms;
            if (coarse_rms > 0) return coarse_rms;
            return -1.0;
        }
    };

    // 两阶段标定 (coarse_init 为 AI 粗估结果, 为空则用 identity 初始化)
    // prefer_targetfree=true 时优先使用边缘对齐 (无目标)
    TwoStageResult calibrate_two_stage(
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const CameraIntrinsics& cam_intrin,
        const std::optional<Sophus::SE3d>& coarse_init = std::nullopt,
        bool prefer_targetfree = true,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0");

    // 手动校准入口: 使用 click-calib 方式采集对应点后精化
    // 返回精化后的外参 (需要调用者提供对应点)
    std::optional<ExtrinsicSE3> calibrate_manual(
        const LiDARScan& scan,
        const cv::Mat& image,
        const CameraIntrinsics& cam_intrin,
        const ExtrinsicSE3& init_extrin,
        const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector2d>>& clicks_3d2d);

    // 日志辅助: 记录每次迭代的残差 (供 Ceres 回调调用)
    void log_iter_residual(int iter, double cost, double delta_cost = 0.0) const {
        UNICALIB_TRACE("[LiDAR-Cam] 迭代 {:3d}: cost={:.6f} delta={:.6f}",
                       iter, cost, delta_cost);
    }

    // ===================================================================
    // 单方法标定接口 (原有接口保留)
    // ===================================================================

    // 方法 A: 运动法 (B样条)
    // 需要: IMU 数据 + IMU-LiDAR 外参 + 相机图像序列
    std::optional<ExtrinsicSE3> calibrate_motion(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const ExtrinsicSE3& T_lidar_in_imu,
        const CameraIntrinsics& cam_intrin,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0");

    // 方法 B: 棋盘格目标法
    std::optional<ExtrinsicSE3> calibrate_target(
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const CameraIntrinsics& cam_intrin,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0");

    // 方法 C: 边缘对齐 (无目标)
    std::optional<ExtrinsicSE3> calibrate_edge_align(
        const std::vector<LiDARScan>& lidar_scans,
        const std::vector<std::pair<double, cv::Mat>>& camera_frames,
        const CameraIntrinsics& cam_intrin,
        const Sophus::SE3d& init_guess,
        const std::string& lidar_id = "lidar_0",
        const std::string& cam_id   = "cam_0");

    // 评估标定质量 — 生成点云投影到图像上的可视化
    void visualize_projection(
        const LiDARScan& scan,
        const cv::Mat& image,
        const ExtrinsicSE3& extrin,
        const CameraIntrinsics& cam_intrin,
        const std::string& output_path);

    // 计算边缘对齐评分
    EdgeAlignmentScore evaluate_edge_alignment(
        const LiDARScan& scan,
        const cv::Mat& image,
        const ExtrinsicSE3& extrin,
        const CameraIntrinsics& cam_intrin);

    using ProgressCallback = std::function<void(const std::string&, double)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = cb; }

private:
    Config cfg_;
    ProgressCallback progress_cb_;

    // 从 LiDAR 点云检测棋盘格角点
    bool detect_board_in_lidar(const LiDARScan& scan,
                               std::vector<Eigen::Vector3d>& corners_3d);

    // 生成 LiDAR 强度图 (用于边缘检测)
    cv::Mat lidar_to_intensity_image(
        const LiDARScan& scan,
        const Sophus::SE3d& T_cam_in_lidar,
        const CameraIntrinsics& cam_intrin);
};

}  // namespace ns_unicalib
