#pragma once
/**
 * UniCalib Unified — IMU-LiDAR 外参标定
 *
 * 两阶段标定流程 (无目标方法全程):
 *   粗标定 (Coarse): L2Calib RL强化学习初始估计
 *     → SE(3)-流形 PPO + Bingham 策略参数化
 *   精标定 (Fine, 无目标):
 *     Step 1: LiDAR 里程计 (NDT/ICP) 获取旋转序列
 *     Step 2: IMU 积分获取旋转序列
 *     Step 3: 手眼标定 (Daniilidis/QEF) 获取初始旋转外参 (或使用粗标定结果)
 *     Step 4: B样条连续时间精细优化 (旋转+平移+时间偏移)
 *   手动校准 (Manual): 通过可视化旋转验证 + 增量调整
 *
 * 算法来源: iKalibr calib_solver (init_prep_li_align + B样条优化)
 * 日志: Step 级别日志, 每次 B样条迭代记录残差
 */

#include "unicalib/common/calib_param.h"
#include "unicalib/common/logger.h"
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
#include <string>
#include <functional>
#include <optional>
#include <memory>

// 前向声明 (避免 PCL 头文件污染)
namespace pcl { template<typename T> class PointCloud; struct PointXYZI; }

namespace ns_unicalib {

// ===================================================================
// 传感器帧数据 (共用)
// ===================================================================
struct IMUFrame {
    double timestamp;
    Eigen::Vector3d gyro;   // rad/s
    Eigen::Vector3d accel;  // m/s^2
};

struct LiDARScan {
    double timestamp;
    using PointT = pcl::PointXYZI;
    std::shared_ptr<pcl::PointCloud<PointT>> cloud;
    // 对于旋转式 LiDAR, 每点有时间戳 (运动补偿用)
    std::vector<double> point_timestamps;
};

// ===================================================================
// LiDAR 里程计配对 (相邻帧间旋转)
// ===================================================================
struct LiDARRotPair {
    double t_begin;
    double t_end;
    Sophus::SO3d rot_LiDAR;     // LiDAR 坐标系下的旋转
    Sophus::SO3d rot_IMU;       // IMU 积分对应的旋转 (初始外参估计前)
    double quality = 1.0;       // 配准质量 (越高越好)
};

// ===================================================================
// IMU-LiDAR 外参标定器
// ===================================================================
class IMULiDARCalibrator {
public:
    struct Config {
        // LiDAR 里程计
        double ndt_resolution   = 1.0;   // NDT 体素分辨率 [m]
        double ndt_epsilon      = 0.01;  // NDT 收敛阈值
        int    ndt_max_iter     = 30;    // NDT 最大迭代次数
        double icp_max_corr_dist = 2.0;  // ICP 最大对应距离 [m]
        bool   use_ndt = true;           // true=NDT, false=ICP

        // 静态片段检测
        double static_gyro_thresh = 0.05;    // rad/s — 静态判断
        double min_motion_rot_deg = 3.0;     // 最小运动量 [deg]

        // 可观测性检测参数 (处理平面运动)
        double min_axis_diversity = 0.3;     // 旋转轴多样性阈值 [0.0=差, 1.0=好]
        double min_pitch_motion_deg = 5.0;   // pitch 方向最小运动 [deg]
        double min_roll_motion_deg = 5.0;    // roll 方向最小运动 [deg]
        double min_yaw_motion_deg = 5.0;     // yaw 方向最小运动 [deg]
        double max_z_trans_ratio = 0.2;     // z 平移占比上限 (超过此值认为非平面运动)

        bool   enable_planar_warning = true; // 是否启用平面运动警告

        bool   use_planar_prior = true;     // 平面运动先验 (约束 z 方向)

        // B样条优化
        double spline_dt_s  = 0.1;          // 样条结时间间隔 [s]
        int    spline_order = 4;            // B样条阶数 (4=cubic)
        double time_offset_init_s = 0.0;   // 初始时间偏移估计
        double time_offset_max_s  = 0.2;   // 最大时间偏移范围
        bool   optimize_time_offset = true;
        bool   optimize_gravity = true;

        // 优化参数
        int    ceres_max_iter   = 50;
        double ceres_loss_scale = 1.0;
        bool   verbose = true;
    };

    // 可观测性诊断结果
    struct ObservabilityDiagnosis {
        double axis_diversity = 0.0;      // 旋转轴多样性 [0,1]
        double pitch_motion_deg = 0.0;    // pitch 方向总运动 [deg]
        double roll_motion_deg = 0.0;     // roll 方向总运动 [deg]
        double yaw_motion_deg = 0.0;      // yaw 方向总运动 [deg]
        double z_trans_ratio = 0.0;       // z 平移占比
        bool   is_planar_motion = false;  // 是否为平面运动
        std::string recommendation;       // 建议
        std::vector<std::string> warnings; // 警告列表
    };

    explicit IMULiDARCalibrator(const Config& cfg) : cfg_(cfg) {}
    IMULiDARCalibrator() : cfg_(Config{}) {}
    ~IMULiDARCalibrator() = default;  // public 析构，供 JointCalibSolver 等栈上析构

    // ===================================================================
    // 两阶段标定接口 (推荐使用)
    // ===================================================================

    struct TwoStageResult {
        std::optional<ExtrinsicSE3> coarse;   // L2Calib AI 粗估
        double coarse_rot_err_deg = -1.0;     // 旋转误差 [deg]
        std::string coarse_method;            // "L2Calib" / "handeye_only"

        std::optional<ExtrinsicSE3> fine;     // B样条精标定
        double fine_rot_err_deg  = -1.0;
        double fine_trans_err_m  = -1.0;
        double fine_time_offset_s = 0.0;
        std::string fine_method;              // "bspline_full" / "handeye_bspline"

        bool needs_manual = false;
        double manual_threshold_deg = 0.5;

        const ExtrinsicSE3* best() const {
            if (fine.has_value())   return &fine.value();
            if (coarse.has_value()) return &coarse.value();
            return nullptr;
        }
    };

    // 两阶段标定
    // coarse_init: AI粗估初始值 (来自 L2Calib), 为空则用手眼标定初始化
    TwoStageResult calibrate_two_stage(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const std::string& imu_id = "imu_0",
        const std::string& lidar_id = "lidar_0",
        const IMUIntrinsics* imu_intrin = nullptr,
        const std::optional<Sophus::SE3d>& coarse_init = std::nullopt);

    // 手动校准: 增量调整外参后重新计算旋转一致性
    ExtrinsicSE3 calibrate_manual_verify(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const ExtrinsicSE3& init_extrin);

    // 日志辅助: 记录手眼标定每对旋转的残差
    void log_handeye_pair(int idx, double rot_err_deg, double quality) const {
        UNICALIB_TRACE("[IMU-LiDAR-HE] 对 {:3d}: rot_err={:.4f}deg quality={:.3f}",
                       idx, rot_err_deg, quality);
    }

    // 日志辅助: B样条迭代残差
    void log_spline_iter(int iter, double cost, double time_offset) const {
        UNICALIB_TRACE("[IMU-LiDAR-BS] 迭代 {:3d}: cost={:.6f} dt={:.4f}s",
                       iter, cost, time_offset);
    }

    // ===================================================================
    // 单方法标定接口 (原有接口保留)
    // ===================================================================

    // 主标定函数
    // imu_id: IMU 传感器 ID (作为参考系 Br)
    // lidar_id: LiDAR 传感器 ID
    std::optional<ExtrinsicSE3> calibrate(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const std::string& imu_id = "imu_0",
        const std::string& lidar_id = "lidar_0",
        const IMUIntrinsics* imu_intrin = nullptr);

    // 仅执行手眼旋转标定 (不含B样条优化)
    std::optional<Sophus::SO3d> calibrate_rotation_handeye(
        const std::vector<LiDARRotPair>& rot_pairs);

    // 获取 LiDAR 里程计结果 (SE3 序列)
    const std::vector<std::pair<double, Sophus::SE3d>>& get_lidar_odom() const {
        return lidar_odom_;
    }

    using ProgressCallback = std::function<void(const std::string& stage, double progress)>;
    void set_progress_callback(ProgressCallback cb) { progress_cb_ = cb; }

private:
    Config cfg_;
    ProgressCallback progress_cb_;

    std::vector<std::pair<double, Sophus::SE3d>> lidar_odom_;  // LiDAR 里程计结果

    // Step 1: 运行 LiDAR 里程计
    bool run_lidar_odometry(const std::vector<LiDARScan>& scans);

    // Step 2: 构建旋转对 (imu_intrin 可选，用于积分时扣除陀螺零偏)
    std::vector<LiDARRotPair> build_rotation_pairs(
        const std::vector<IMUFrame>& imu_data,
        const IMUIntrinsics* imu_intrin = nullptr);

    // Step 3: 手眼旋转标定 (QEF / Daniilidis)
    std::optional<Sophus::SO3d> solve_handeye_rotation(
        const std::vector<LiDARRotPair>& pairs);

    // Step 4: 估计平移 (加速度约束)
    std::optional<Eigen::Vector3d> estimate_translation(
        const std::vector<IMUFrame>& imu_data,
        const Sophus::SO3d& rot_LiDAR_in_IMU,
        double time_offset);

    // Step 5: B样条精细优化
    ExtrinsicSE3 refine_with_spline(
        const std::vector<IMUFrame>& imu_data,
        const std::vector<LiDARScan>& lidar_scans,
        const Sophus::SE3d& init_extrinsic,
        const std::string& imu_id,
        const std::string& lidar_id,
        const IMUIntrinsics* imu_intrin);

    // Step 辅助: IMU 旋转积分 (imu_intrin 非空时扣除 bias_gyro)
    Sophus::SO3d integrate_imu_rotation(
        const std::vector<IMUFrame>& imu_data,
        double t_begin,
        double t_end,
        const IMUIntrinsics* imu_intrin = nullptr);
};

}  // namespace ns_unicalib
