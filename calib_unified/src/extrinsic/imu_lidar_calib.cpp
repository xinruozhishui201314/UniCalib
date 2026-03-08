/**
 * UniCalib Unified — IMU-LiDAR 外参标定实现
 *
 * 两阶段标定流程:
 *   Stage 1 粗标定: LiDAR 里程计 + 手眼标定
 *   Stage 2 精标定: B样条连续时间优化 (SE3 轨迹 + 外参 + 时间偏移)
 */

#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/math_safety.h"
#include "ikalibr/core/lidar_odometer.h"
#include "ikalibr/sensor/lidar.h"
#include "ikalibr/util/cloud_define.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <numeric>
#include <ceres/ceres.h>
#include <basalt/spline/se3_spline.h>
#include <basalt/utils/sophus_utils.hpp>
#include <cmath>

namespace ns_unicalib {

using namespace ns_ikalibr;
using namespace ns_ctraj;

// 构造函数/析构函数已在头文件中 inline 定义，此处不再重复定义

// ===================================================================
// 主标定函数
// ===================================================================

std::optional<ExtrinsicSE3> IMULiDARCalibrator::calibrate(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin) {

    if (imu_data.empty()) {
        UNICALIB_ERROR("[IMU-LiDAR] IMU 数据为空");
        return std::nullopt;
    }

    if (lidar_scans.empty()) {
        UNICALIB_ERROR("[IMU-LiDAR] LiDAR 数据为空");
        return std::nullopt;
    }

    UNICALIB_INFO("[IMU-LiDAR] 开始两阶段标定");
    UNICALIB_INFO("  IMU 数据: {} 帧", imu_data.size());
    UNICALIB_INFO("  LiDAR 数据: {} 帧", lidar_scans.size());

    try {
    // Step 1: 运行 LiDAR 里程计
    if (!run_lidar_odometry(lidar_scans)) {
        UNICALIB_ERROR("[IMU-LiDAR] LiDAR 里程计失败");
        return std::nullopt;
    }

    // Step 2: 构建旋转对 (传入 imu_intrin 以便积分时扣除陀螺零偏)
    auto rot_pairs = build_rotation_pairs(imu_data, imu_intrin);
    if (rot_pairs.empty()) {
        UNICALIB_ERROR("[IMU-LiDAR] 无有效旋转对");
        return std::nullopt;
    }

    UNICALIB_INFO("  构建旋转对: {} 对", rot_pairs.size());

    // Step 3: 手眼旋转标定
    auto rot_result = solve_handeye_rotation(rot_pairs);
    if (!rot_result.has_value()) {
        UNICALIB_WARN("[IMU-LiDAR] 手眼旋转标定失败");
        return std::nullopt;
    }

    UNICALIB_INFO("  手眼旋转标定成功");

    // Step 4: 估计平移（简化实现)
    auto trans_result = estimate_translation(
        imu_data, rot_result.value(), 0.0);

    ExtrinsicSE3 extrinsic;
    extrinsic.ref_sensor_id = imu_id;
    extrinsic.target_sensor_id = lidar_id;
    extrinsic.SO3_TargetInRef = rot_result.value();
    extrinsic.POS_TargetInRef = trans_result.value_or(Eigen::Vector3d::Zero());
    extrinsic.time_offset_s = 0.0;

    UNICALIB_INFO("[IMU-LiDAR] 标定完成");
    UNICALIB_INFO("  旋转: {}", extrinsic.SO3_TargetInRef.unit_quaternion().coeffs().transpose());
    UNICALIB_INFO("  平移: [{:.3f}, {:.3f}, {:.3f}] m",
                  extrinsic.POS_TargetInRef.x(),
                  extrinsic.POS_TargetInRef.y(),
                  extrinsic.POS_TargetInRef.z());

    return extrinsic;

    } catch (const UniCalibException& e) {
        UNICALIB_ERROR("[IMU-LiDAR] 标定异常 [{}]: {} ({}:{})",
                       errorCodeName(e.code()), e.message(), e.file(), e.line());
        return std::nullopt;
    } catch (const std::exception& e) {
        UNICALIB_ERROR("[IMU-LiDAR] 标定异常: {}", e.what());
        return std::nullopt;
    } catch (...) {
        UNICALIB_ERROR("[IMU-LiDAR] 标定未知异常");
        return std::nullopt;
    }
}

// ===================================================================
// 两阶段标定
// ===================================================================
IMULiDARCalibrator::TwoStageResult IMULiDARCalibrator::calibrate_two_stage(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin,
    const std::optional<Sophus::SE3d>& coarse_init) {

    TwoStageResult result;
    result.coarse_method = coarse_init.has_value() ? "L2Calib_or_user" : "handeye_only";
    result.fine_method  = "bspline_full";

    Sophus::SE3d init_se3;
    if (coarse_init.has_value()) {
        init_se3 = coarse_init.value();
        result.coarse = ExtrinsicSE3{};
        result.coarse->ref_sensor_id = imu_id;
        result.coarse->target_sensor_id = lidar_id;
        result.coarse->SO3_TargetInRef = init_se3.so3();
        result.coarse->POS_TargetInRef = init_se3.translation();
        result.coarse->time_offset_s = 0.0;
        result.coarse_rot_err_deg = 0.0;
    } else {
        auto extrinsic = calibrate(imu_data, lidar_scans, imu_id, lidar_id, imu_intrin);
        if (!extrinsic.has_value()) {
            result.needs_manual = true;
            return result;
        }
        result.coarse = extrinsic;
        result.coarse_rot_err_deg = 0.0;
        init_se3 = extrinsic->SE3_TargetInRef();
    }

    // B样条精细优化
    ExtrinsicSE3 refined = refine_with_spline(
        imu_data, lidar_scans, init_se3, imu_id, lidar_id, imu_intrin);
    result.fine = refined;
    result.fine_time_offset_s = refined.time_offset_s;
    result.fine_rot_err_deg = 0.0;
    result.fine_trans_err_m  = 0.0;

    return result;
}

// ===================================================================
// LiDAR 里程计 (使用 NDT 配准)
// ===================================================================
bool IMULiDARCalibrator::run_lidar_odometry(const std::vector<LiDARScan>& scans) {
    if (scans.empty()) return false;

    UNICALIB_INFO("[LiDAR-Odom] 开始 LiDAR 里程计 (NDT 配准)");

    // 创建 LiDAROdometer
    auto lidar_odometer = LiDAROdometer::Create(
        static_cast<float>(cfg_.ndt_resolution),
        4  // 使用 4 线程进行 NDT 配准
    );

    lidar_odom_.clear();
    lidar_odom_.reserve(scans.size());

    size_t valid_frames = 0;
    for (size_t i = 0; i < scans.size(); ++i) {
        if (!scans[i].cloud || scans[i].cloud->empty()) {
            UNICALIB_WARN("[LiDAR-Odom] 帧 {} 点云为空", i);
            continue;
        }

        // 转换点云类型: pcl::PointXYZI -> IKalibrPoint
        IKalibrPointCloud::Ptr ikalibr_cloud = std::make_shared<IKalibrPointCloud>();
        ikalibr_cloud->reserve(scans[i].cloud->size());
        
        for (const auto& pt : scans[i].cloud->points) {
            // 跳过 NaN 点
            if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
                continue;
            }
            IKalibrPoint ikalibr_pt;
            ikalibr_pt.x = pt.x;
            ikalibr_pt.y = pt.y;
            ikalibr_pt.z = pt.z;
            // 使用扫描帧的时间戳作为点云时间戳基准
            // 对于旋转式 LiDAR，如果有点级时间戳则使用，否则使用帧时间戳
            ikalibr_pt.timestamp = scans[i].timestamp;
            ikalibr_cloud->push_back(ikalibr_pt);
        }

        // 如果转换后点云为空，跳过
        if (ikalibr_cloud->empty()) {
            UNICALIB_WARN("[LiDAR-Odom] 帧 {} 转换后点云为空", i);
            continue;
        }

        // 创建 LiDARFrame
        auto frame = LiDARFrame::Create(scans[i].timestamp, ikalibr_cloud);

        // 使用 NDT 配准
        ns_ctraj::Posed pose = lidar_odometer->FeedFrame(frame);

        // 保存结果 (将 ns_ctraj::Posed 转换为 Sophus::SE3d)
        Sophus::SO3d so3(pose.so3.matrix());
        Sophus::SE3d se3_pose(so3, pose.t);
        lidar_odom_.emplace_back(scans[i].timestamp, se3_pose);

        valid_frames++;

        if (progress_cb_) {
            progress_cb_("LiDAR-Odom", static_cast<double>(i + 1) / scans.size());
        }
    }

    UNICALIB_INFO("[LiDAR-Odom] 完成, 处理 {} 帧 (有效 {} 帧)", scans.size(), valid_frames);
    return valid_frames > 0;
}

// ===================================================================
// 构建旋转对 (简化实现)
// ===================================================================
std::vector<LiDARRotPair> IMULiDARCalibrator::build_rotation_pairs(
    const std::vector<IMUFrame>& imu_data,
    const IMUIntrinsics* imu_intrin) {

    std::vector<LiDARRotPair> pairs;
    if (imu_data.empty() || lidar_odom_.size() < 2) return pairs;

    UNICALIB_INFO("[Build-RotPairs] 构建旋转对");

    // 为每对相邻的 LiDAR 里程计位姿， 创建旋转对
    for (size_t i = 1; i < lidar_odom_.size(); ++i) {
        const auto& pose0 = lidar_odom_[i - 1];
        const auto& pose1 = lidar_odom_[i];

        double t_begin = pose0.first;
        double t_end = pose1.first;

        // 计算两帧之间的相对旋转
        Sophus::SO3d rot_lidar = (pose0.second.inverse() * pose1.second).so3();

        // 从 IMU 数据中积分得到对应的旋转 (传入 imu_intrin 时扣除陀螺零偏)
        Sophus::SO3d rot_imu = integrate_imu_rotation(imu_data, t_begin, t_end, imu_intrin);
        // 旋转角度(弧度) = log(R).norm()，与配置中的 min_motion_rot_deg 比较
        if (rot_imu.log().norm() > cfg_.min_motion_rot_deg * MathSafety::DEG_TO_RAD) {
            LiDARRotPair pair;
            pair.t_begin = t_begin;
            pair.t_end = t_end;
            pair.rot_LiDAR = rot_lidar;
            pair.rot_IMU = rot_imu;
            pair.quality = 1.0; // TODO: 巻加配准质量评估
            pairs.push_back(pair);
        }
    }

    UNICALIB_INFO("[Build-RotPairs] 构建了 {} 个有效旋转对", pairs.size());
    return pairs;
}

// ===================================================================
// IMU 旋转积分 (改进实现)
// ===================================================================
Sophus::SO3d IMULiDARCalibrator::integrate_imu_rotation(
    const std::vector<IMUFrame>& imu_data,
    double t_begin,
    double t_end,
    const IMUIntrinsics* imu_intrin) {
    // 找到时间范围内的 IMU 数据
    std::vector<IMUFrame> segment;
    for (const auto& frame : imu_data) {
        if (frame.timestamp >= t_begin && frame.timestamp <= t_end) {
            segment.push_back(frame);
        }
    }

    if (segment.empty()) {
        UNICALIB_WARN("[IMU-Integrate] 时间范围内无 IMU 数据");
        return Sophus::SO3d();
    }
    
    if (segment.size() < 2) {
        UNICALIB_WARN("[IMU-Integrate] IMU 数据点太少");
        return Sophus::SO3d();
    }

    // 按时间戳排序
    std::sort(segment.begin(), segment.end(), 
              [](const IMUFrame& a, const IMUFrame& b) {
                  return a.timestamp < b.timestamp;
              });

    // 使用中值积分 (比欧拉积分更准确)
    // 对于旋转积分: R(t_end) = R(t_begin) * Prod_i[Exp(omega_i * dt_i)]
    Sophus::SO3d rot = Sophus::SO3d();
    
    for (size_t i = 1; i < segment.size(); ++i) {
        double dt = segment[i].timestamp - segment[i-1].timestamp;
        
        // 时间间隔有效性检查
        if (dt <= 0 || dt > 1.0) {  // 排除异常时间间隔 (>1秒)
            UNICALIB_WARN("[IMU-Integrate] 异常时间间隔: dt={:.6f}s", dt);
            continue;
        }
        
        // 使用两个时间点的平均角速度，若提供内参则扣除陀螺零偏
        Eigen::Vector3d avg_gyro = 0.5 * (segment[i-1].gyro + segment[i].gyro);
        if (imu_intrin) {
            avg_gyro -= imu_intrin->bias_gyro;
        }
        // 数值有效性检查
        if (!avg_gyro.allFinite()) {
            UNICALIB_WARN("[IMU-Integrate] 角速度包含无效值");
            continue;
        }
        
        // 角速度模长检查 (过大的角速度可能是噪声)
        double gyro_norm = avg_gyro.norm();
        if (gyro_norm > 10.0) {  // > 10 rad/s ≈ 573 deg/s
            UNICALIB_WARN("[IMU-Integrate] 角速度过大: {:.4f} rad/s, 可能是噪声", gyro_norm);
            continue;
        }
        
        // 计算旋转增量
        Eigen::Vector3d delta_rot = avg_gyro * dt;
        Sophus::SO3d delta_rot_so3 = Sophus::SO3d::exp(delta_rot);
        
        // 累积旋转
        rot = rot * delta_rot_so3;
    }

    return rot;
}

// ===================================================================
// 手眼旋转标定 (使用 SVD 分解方法)
// ===================================================================
std::optional<Sophus::SO3d> IMULiDARCalibrator::solve_handeye_rotation(
    const std::vector<LiDARRotPair>& pairs) {
    if (pairs.empty()) {
        UNICALIB_ERROR("[Handeye] 旋转对为空");
        return std::nullopt;
    }

    UNICALIB_INFO("[IMU-LiDAR] 步骤: 手眼旋转标定开始 — 旋转对 {} 个", pairs.size());

    // 使用 SVD 求解手眼方程 AX = XB
    // 对于 IMU-LiDAR 手眼标定，方程形式为: R_imu * X = X * R_lidar
    // 等价于: R_imu * X - X * R_lidar = 0
    // 对于每个旋转对 (R_A, R_B)，构建约束方程
    
    const size_t N = pairs.size();
    
    // 构建矩阵 M，每个旋转对贡献 4 行
    // 使用四元数表示: q_A * q_X = q_X * q_B
    // 展开: (q_A.w * q_X.w - q_A.vec · q_X.vec - q_X.w * q_B.w + q_X.vec · q_B.vec) = 0
    //       (q_A.w * q_X.vec + q_A.vec * q_X.w - q_X.w * q_B.vec - q_X.vec * q_B.w) = 0
    //       ...
    
    Eigen::MatrixXd M(4 * N, 4);
    M.setZero();
    
    for (size_t i = 0; i < N; ++i) {
        const auto& R_A = pairs[i].rot_IMU;   // IMU 旋转
        const auto& R_B = pairs[i].rot_LiDAR;  // LiDAR 旋转
        
        Eigen::Quaterniond q_A = R_A.unit_quaternion();
        Eigen::Quaterniond q_B = R_B.unit_quaternion();
        
        // 确保四元数符号一致性
        if (q_A.w() < 0) q_A.coeffs() = -q_A.coeffs();
        if (q_B.w() < 0) q_B.coeffs() = -q_B.coeffs();
        
        // 构建约束矩阵
        // 来自: Robust Hand-Eye Calibration Using Dual Quaternions (Daniilidis 1999)
        double row[4][4];
        
        // (q_A.w - q_B.w) * q_X.w - (q_A.vec + q_B.vec) · q_X.vec = 0
        row[0][0] = q_A.w() - q_B.w();
        row[0][1] = -q_A.x() - q_B.x();
        row[0][2] = -q_A.y() - q_B.y();
        row[0][3] = -q_A.z() - q_B.z();
        
        // (q_A.x + q_B.x) * q_X.w + (q_A.w + q_B.w) * q_X.x + (q_A.z - q_B.z) * q_X.y - (q_A.y + q_B.y) * q_X.z = 0
        row[1][0] = q_A.x() + q_B.x();
        row[1][1] = q_A.w() + q_B.w();
        row[1][2] = q_A.z() - q_B.z();
        row[1][3] = -q_A.y() - q_B.y();
        
        // (q_A.y + q_B.y) * q_X.w + (q_A.y + q_B.z) * q_X.x + (q_A.w + q_B.w) * q_X.y - (q_A.z + q_B.x) * q_X.z = 0
        row[2][0] = q_A.y() + q_B.y();
        row[2][1] = q_A.y() - q_B.z();
        row[2][2] = q_A.w() + q_B.w();
        row[2][3] = -q_A.z() - q_B.x();
        
        // (q_A.z + q_B.z) * q_X.w + (q_A.z + q_B.y) * q_X.x + (q_A.y + q_B.x) * q_X.y + (q_A.w + q_B.w) * q_X.z = 0
        row[3][0] = q_A.z() + q_B.z();
        row[3][1] = q_A.z() + q_B.y();
        row[3][2] = q_A.y() + q_B.x();
        row[3][3] = q_A.w() + q_B.w();
        
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                M(4 * i + r, c) = row[r][c];
            }
        }
    }
    
    // 使用 SVD 求解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
    Eigen::Vector4d q_vec = svd.matrixV().col(3);  // 最小奇异值对应的向量
    
    // 归一化四元数
    double q_norm = q_vec.norm();
    if (q_norm < 1e-10) {
        UNICALIB_ERROR("[Handeye] SVD 结果接近零，无法求解有效四元数");
        return std::nullopt;
    }
    
    q_vec /= q_norm;
    
    // 确保四元数实部为正（符号一致性）
    if (q_vec(3) < 0) {
        q_vec = -q_vec;
    }
    
    // 构建四元数 (Eigen::Quaterniond 存储顺序是 [x, y, z, w])
    Eigen::Quaterniond q(q_vec(3), q_vec(0), q_vec(1), q_vec(2));
    
    // 计算残差用于诊断
    double total_error = 0.0;
    for (size_t i = 0; i < N; ++i) {
        const auto& R_A = pairs[i].rot_IMU;
        const auto& R_B = pairs[i].rot_LiDAR;
        Sophus::SO3d X(q);
        // 验证: R_A * X ≈ X * R_B
        Sophus::SO3d lhs = R_A * X;
        Sophus::SO3d rhs = X * R_B;
        Sophus::SO3d err = lhs * rhs.inverse();
        total_error += err.log().norm();
    }
    double avg_error_deg = MathSafety::radToDeg(total_error / N);
    
    UNICALIB_INFO("[IMU-LiDAR] 步骤: 手眼旋转标定完成 — 平均误差: {:.4f} deg", avg_error_deg);
    
    if (avg_error_deg > 10.0) {
        UNICALIB_WARN("[Handeye] 平均误差较大，建议检查数据质量");
    }
    
    return Sophus::SO3d(q);
}

// ===================================================================
// 平移估计 (基于 LiDAR 里程计速度 + IMU 角速度约束)
// 原理: omega x t = v_imu - R * v_lidar
// 使用 MAD 滤除离群点， 保持简洁
// ===================================================================
std::optional<Eigen::Vector3d> IMULiDARCalibrator::estimate_translation(
    const std::vector<IMUFrame>& imu_data,
    const Sophus::SO3d& rot_LiDAR_in_IMU,
    double time_offset) {

    if (lidar_odom_.size() < 2) {
        UNICALIB_ERROR("[Trans-Est] LiDAR 里程计数据不足");
        return std::nullopt;
    }

    UNICALIB_INFO("[IMU-LiDAR] 步骤: 平移估计开始 (里程计 {} 帧)", lidar_odom_.size());
    // 阈值参数
    constexpr double MIN_VELOCITY = 0.1;       // 最小速度 [m/s]
    constexpr double MIN_GYRO = 0.05;          // 最小角速度 [rad/s]
    constexpr double MAX_VELOCITY = 10.0;      // 最大速度 [m/s]
    constexpr double MAX_GYRO = 5.0;           // 最大角速度 [rad/s]
    constexpr size_t MIN_CONSTRAINTS = 8;      // 最小约束数
    constexpr double MAD_THRESHOLD = 2.5;      // MAD 离群点阈值

    Eigen::Matrix3d R = rot_LiDAR_in_IMU.matrix();

    // 收集约束数据
    struct Constraint {
        Eigen::Vector3d omega;     // IMU 角速度
        Eigen::Vector3d vel_diff;  // v_imu - R * v_lidar
        double residual_norm;      // 用于离群点检测
    };
    std::vector<Constraint> constraints;

    for (size_t i = 1; i < lidar_odom_.size(); ++i) {
        const auto& [t0, pose0] = lidar_odom_[i - 1];
        const auto& [t1, pose1] = lidar_odom_[i];
        double dt = t1 - t0;
        if (dt < 0.05 || dt > 2.0) continue;  // 时间间隔有效性

        // LiDAR 速度 (LiDAR 坐标系)
        Eigen::Vector3d v_lidar = (pose1.translation() - pose0.translation()) / dt;
        double v_norm = v_lidar.norm();
        if (v_norm < MIN_VELOCITY || v_norm > MAX_VELOCITY) continue;  // 运动量检查

        // IMU 平均角速度
        Eigen::Vector3d avg_gyro = Eigen::Vector3d::Zero();
        int count = 0;
        for (const auto& frame : imu_data) {
            if (frame.timestamp >= t0 && frame.timestamp <= t1) {
                avg_gyro += frame.gyro;
                count++;
            }
        }
        if (count == 0) continue;
        avg_gyro /= count;

        double gyro_norm = avg_gyro.norm();
        if (gyro_norm < MIN_GYRO || gyro_norm > MAX_GYRO) continue;  // 角速度检查

        // 约束: omega x t = v_imu - R * v_lidar
        // 计算 v_imu: 通过 LiDAR 里程计差分 + IMU 加速度积分
        // v_imu 可以从以下方式估计:
        // 方法1: LiDAR里程计速度差分 (推荐，精度高)
        // 方法2: 如果数据充足， IMU预积分 (备选, 更鲁棒但需要IMU内参)
        // 方法3: 如果无里程计信息, 使用零速度假设 (向后兼容)
        
        Eigen::Vector3d v_imu = Eigen::Vector3d::Zero();
        
        // 方法1: 从 LiDAR 里程计估计 v_imu (使用相邻帧差分)
        if (i >= 2) {
            // 计算里程计帧间的速度 (在 LiDAR 坐标系下)
            const Sophus::SE3d& T_w_lidar_curr = pose1;
            const Sophus::SE3d& T_w_lidar_prev = pose0;
            
            // LiDAR 在世界坐标系的速度
            Eigen::Vector3d v_lidar_w = (T_w_lidar_curr.translation() - T_w_lidar_prev.translation()) / dt;
            
            // 变换到 IMU 坐标系
            Eigen::Vector3d v_lidar_imu = rot_LiDAR_in_IMU * v_lidar_w;
            v_imu = v_lidar_imu;
        } else {
            // 方法2: IMU 预积分 (如果提供了 IMU 内参和有足够的数据)
            // 注意: 这需要准确的 IMU 内参, 且数据充足
            // 当前简化实现使用里程计差分
            
            // 方法3: 鰧零速度假设 (向后兼容)
            // 当无法从里程计或 IMU 获取有效速度估计时使用零速度
            v_imu.setZero();
            UNICALIB_DEBUG("[Trans-Est] 无法估计 v_imu, 使用零速度假设 (可能降低精度)");
        }
        
        Constraint c;
        c.omega = avg_gyro;
        c.vel_diff = v_imu - R * v_lidar;
        c.residual_norm = 0.0;      // 后续计算
        constraints.push_back(c);
    }

    if (constraints.size() < MIN_CONSTRAINTS) {
        UNICALIB_WARN("[Trans-Est] 有效约束不足: {} < {}", constraints.size(), MIN_CONSTRAINTS);
        return Eigen::Vector3d::Zero();
    }

    UNICALIB_INFO("[Trans-Est] 收集 {} 个约束", constraints.size());

    // === 使用 MAD 滤除离群点 ===
    // 先用所有数据做初步估计
    auto solve_constraints = [](const std::vector<Constraint>& cons) -> Eigen::Vector3d {
        const size_t N = cons.size();
        Eigen::MatrixXd A(3 * N, 3);
        Eigen::VectorXd b(3 * N);
        for (size_t i = 0; i < N; ++i) {
            const auto& w = cons[i].omega;
            A.row(3*i+0) << 0, -w.z(), w.y();
            A.row(3*i+1) << w.z(), 0, -w.x();
            A.row(3*i+2) << -w.y(), w.x(), 0;
            b.segment<3>(3*i) = cons[i].vel_diff;
        }
        return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    };

    Eigen::Vector3d t_init = solve_constraints(constraints);

    // 计算残差
    std::vector<double> residuals;
    for (auto& c : constraints) {
        Eigen::Matrix3d skew;
        skew << 0, -c.omega.z(), c.omega.y(),
                c.omega.z(), 0, -c.omega.x(),
                -c.omega.y(), c.omega.x(), 0;
        Eigen::Vector3d pred = skew * t_init;
        c.residual_norm = (pred - c.vel_diff).norm();
        residuals.push_back(c.residual_norm);
    }

    // 计算 MAD (Median Absolute Deviation)
    std::vector<double> sorted_residuals = residuals;
    std::sort(sorted_residuals.begin(), sorted_residuals.end());
    double median = sorted_residuals[sorted_residuals.size() / 2];
    std::vector<double> abs_dev;
    for (double r : residuals) abs_dev.push_back(std::abs(r - median));
    std::sort(abs_dev.begin(), abs_dev.end());
    double mad = abs_dev[abs_dev.size() / 2] * 1.4826;  // 缩放因子

    // 滤除离群点
    std::vector<Constraint> filtered;
    for (const auto& c : constraints) {
        double z_score = (mad > 1e-6) ? std::abs(c.residual_norm - median) / mad : 0.0;
        if (z_score < MAD_THRESHOLD) {
            filtered.push_back(c);
        }
    }

    UNICALIB_INFO("[Trans-Est] MAD 滤波: {} -> {} 约束", constraints.size(), filtered.size());

    if (filtered.size() < MIN_CONSTRAINTS) {
        UNICALIB_WARN("[Trans-Est] 滤波后约束不足， 返回初步估计");
        UNICALIB_INFO("  t = [{:.3f}, {:.3f}, {:.3f}] m", t_init.x(), t_init.y(), t_init.z());
        return t_init;
    }

    // 最终求解
    Eigen::Vector3d t_final = solve_constraints(filtered);

    // 计算最终残差
    double rms_error = 0.0;
    for (const auto& c : filtered) {
        Eigen::Matrix3d skew;
        skew << 0, -c.omega.z(), c.omega.y(),
                c.omega.z(), 0, -c.omega.x(),
                -c.omega.y(), c.omega.x(), 0;
        Eigen::Vector3d pred = skew * t_final;
        double err = (pred - c.vel_diff).norm();
        rms_error += err * err;
    }
    rms_error = std::sqrt(rms_error / filtered.size());

    UNICALIB_INFO("[IMU-LiDAR] 步骤: 平移估计完成 — t=[{:.3f}, {:.3f}, {:.3f}] m, RMS={:.4f} m/s",
                  t_final.x(), t_final.y(), t_final.z(), rms_error);
    UNICALIB_INFO("  t = [{:.3f}, {:.3f}, {:.3f}] m", t_final.x(), t_final.y(), t_final.z());
    UNICALIB_INFO("  RMS 误差 = {:.4f} m/s (基于 {} 个约束)", rms_error, filtered.size());

    if (rms_error > 0.3) {
        UNICALIB_WARN("[Trans-Est] 残差较大 ({:.3f} m/s)， 结果可能不准确", rms_error);
    }

    return t_final;
}

// ===================================================================
// 辅助: 在 (timestamp, SE3) 序列上线性/Slerp 插值
// ===================================================================
namespace {
Sophus::SE3d interpolate_pose(
    const std::vector<std::pair<double, Sophus::SE3d>>& poses,
    double t) {
    if (poses.empty()) return Sophus::SE3d();
    if (poses.size() == 1) return poses[0].second;
    if (t <= poses.front().first) return poses.front().second;
    if (t >= poses.back().first) return poses.back().second;

    size_t i = 0;
    while (i + 1 < poses.size() && poses[i + 1].first < t) ++i;
    double t0 = poses[i].first, t1 = poses[i + 1].first;
    double alpha = (t1 > t0) ? ((t - t0) / (t1 - t0)) : 0.0;
    alpha = std::max(0.0, std::min(1.0, alpha));

    const Sophus::SE3d& T0 = poses[i].second;
    const Sophus::SE3d& T1 = poses[i + 1].second;
    Sophus::SO3d R = Sophus::SO3d::exp(alpha * (T1.so3() * T0.so3().inverse()).log()) * T0.so3();
    Eigen::Vector3d p = (1.0 - alpha) * T0.translation() + alpha * T1.translation();
    return Sophus::SE3d(R, p);
}

// Ceres 代价: 外参(6) + 时间偏移(1) vs 固定 B 样条与 LiDAR 位姿
// 使用 basalt-headers 的 pose() 与 d_pose_d_t() 提供精确（解析）雅可比，替代数值微分
struct SplineExtrinsicCost {
    const basalt::Se3Spline<4>* spline;
    int64_t t_lidar_ns;
    Sophus::SE3d T_w_lidar;

    bool operator()(const double* const extr_rot_trans, const double* const time_offset,
                    double* residual) const {
        Eigen::Map<const Eigen::Vector3d> rot_vec(extr_rot_trans);
        Eigen::Map<const Eigen::Vector3d> trans(extr_rot_trans + 3);
        double dt = time_offset[0];

        int64_t t_query_ns = t_lidar_ns + static_cast<int64_t>(1e9 * dt);
        Sophus::SE3d T_w_imu = spline->pose(t_query_ns);
        Sophus::SE3d T_imu_lidar(Sophus::SO3d::exp(rot_vec), trans);
        Sophus::SE3d T_w_lidar_pred = T_w_imu * T_imu_lidar;
        Eigen::Matrix<double, 6, 1> log_err =
            (T_w_lidar_pred.inverse() * T_w_lidar).log();
        for (int i = 0; i < 6; ++i) residual[i] = log_err(i);
        return true;
    }
};

// 解析雅可比代价函数：基于 basalt-headers 的 d_pose_d_t 实现精确估计
// 残差对 外参(6) 与 时间偏移(1) 的导数由解析公式给出，避免数值微分误差
class SplineExtrinsicAnalyticCost : public ceres::CostFunction {
 public:
    SplineExtrinsicAnalyticCost(const basalt::Se3Spline<4>* spline,
                                int64_t t_lidar_ns,
                                const Sophus::SE3d& T_w_lidar)
        : spline_(spline), t_lidar_ns_(t_lidar_ns), T_w_lidar_(T_w_lidar) {
        set_num_residuals(6);
        mutable_parameter_block_sizes()->push_back(6);   // extr
        mutable_parameter_block_sizes()->push_back(1);   // time_offset
    }

    bool Evaluate(double const* const* parameters,
                  double* residuals,
                  double** jacobians) const override {
        const double* extr = parameters[0];
        const double dt = parameters[1][0];
        Eigen::Map<const Eigen::Vector3d> rot_vec(extr);
        Eigen::Map<const Eigen::Vector3d> trans(extr + 3);

        const int64_t t_query_ns = t_lidar_ns_ + static_cast<int64_t>(1e9 * dt);
        Sophus::SE3d T_w_imu = spline_->pose(t_query_ns);
        Sophus::SE3d T_imu_lidar(Sophus::SO3d::exp(rot_vec), trans);
        Sophus::SE3d T_w_lidar_pred = T_w_imu * T_imu_lidar;
        Sophus::SE3d T_err = T_w_lidar_pred.inverse() * T_w_lidar_;
        Eigen::Map<Eigen::Matrix<double, 6, 1>> r(residuals);
        r = T_err.log();

        if (!jacobians) return true;

        // 使用 basalt 的 decoupled SE(3) 右雅可比逆: r = log(T_err), dr/d(eps_pred) = -J_r^{-1}(r)
        Eigen::Matrix<double, 6, 6> J_r_inv;
        Sophus::rightJacobianInvSE3Decoupled(r, J_r_inv);
        // 外参 extr = [rot_vec(3), trans(3)]，T_imu_lidar = (exp(rot_vec), trans)，decoupled tangent phi = (trans, rot_vec)
        // d(phi)/d(extr) = [0 I; I 0]，且 tangent_pred (body) = Ad(T_imu_lidar^{-1}) * phi_imu_lidar
        Eigen::Matrix<double, 6, 6> d_phi_imu_lidar_d_extr;
        d_phi_imu_lidar_d_extr.setZero();
        d_phi_imu_lidar_d_extr.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        d_phi_imu_lidar_d_extr.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> Ad_inv = T_imu_lidar.inverse().Adj();
        Eigen::Matrix<double, 6, 6> d_res_d_extr =
            -J_r_inv * Ad_inv * d_phi_imu_lidar_d_extr;

        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> J_extr(jacobians[0]);
            J_extr = d_res_d_extr;
        }

        // d(residual)/d(time_offset): d(residual)/d(dt) = d(residual)/d(T_pred) * d(T_pred)/d(t) * 1e9
        // basalt: d_pose_d_t 给出 body 系角速度与线速度 (omega, v_body)
        if (jacobians[1]) {
            Eigen::Matrix<double, 6, 1> d_pose_d_t;
            spline_->d_pose_d_t(t_query_ns, d_pose_d_t);
            // T_pred 对 t 的导数 (在 T_w_imu 处): d(T_w_imu)/dt = T_w_imu * [omega_body; v_body] (右扰动)
            // 在 T_pred 的 body 系: d(T_pred)/d(t) = T_imu_lidar^{-1}.Adj() * d_pose_d_t
            Eigen::Matrix<double, 6, 1> tangent_pred = T_imu_lidar.inverse().Adj() * d_pose_d_t;
            Eigen::Matrix<double, 6, 1> d_res_d_t = -J_r_inv * tangent_pred * 1e9;
            jacobians[1][0] = d_res_d_t(0);
            jacobians[1][1] = d_res_d_t(1);
            jacobians[1][2] = d_res_d_t(2);
            jacobians[1][3] = d_res_d_t(3);
            jacobians[1][4] = d_res_d_t(4);
            jacobians[1][5] = d_res_d_t(5);
        }
        return true;
    }

 private:
    const basalt::Se3Spline<4>* spline_;
    int64_t t_lidar_ns_;
    Sophus::SE3d T_w_lidar_;
};
}  // namespace

// ===================================================================
// B样条精细优化 (完整实现)
// 流程: LiDAR 里程计 → T_w_imu 序列 → SE3 B样条 → 优化外参 + 时间偏移
// ===================================================================
ExtrinsicSE3 IMULiDARCalibrator::refine_with_spline(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const Sophus::SE3d& init_extrinsic,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin) {

    UNICALIB_INFO("[IMU-LiDAR] 步骤: B样条精细优化开始 (LiDAR {} 帧)", lidar_scans.size());
    ExtrinsicSE3 result;
    result.ref_sensor_id = imu_id;
    result.target_sensor_id = lidar_id;
    result.SO3_TargetInRef = init_extrinsic.so3();
    result.POS_TargetInRef = init_extrinsic.translation();
    result.time_offset_s = 0.0;

    if (lidar_scans.empty()) {
        UNICALIB_WARN("[B-spline] LiDAR 数据为空，返回初值");
        return result;
    }

    // Step 1: 运行 LiDAR 里程计
    if (!run_lidar_odometry(lidar_scans)) {
        UNICALIB_WARN("[B-spline] LiDAR 里程计失败，返回初值");
        return result;
    }
    if (lidar_odom_.size() < 4) {
        UNICALIB_WARN("[B-spline] 里程计帧数不足 (need >= 4)，返回初值");
        return result;
    }

    // T_lidar_imu = T_imu_lidar^{-1}，即 LiDAR 在 IMU 系下位姿的逆
    Sophus::SE3d T_lidar_imu = init_extrinsic.inverse();
    std::vector<std::pair<double, Sophus::SE3d>> T_w_imu_poses;
    T_w_imu_poses.reserve(lidar_odom_.size());
    for (const auto& [t, T_w_lidar] : lidar_odom_)
        T_w_imu_poses.emplace_back(t, T_w_lidar * T_lidar_imu);

    double t_min = T_w_imu_poses.front().first;
    double t_max = T_w_imu_poses.back().first;
    double dt_s = std::max(0.05, cfg_.spline_dt_s);
    const int64_t dt_ns = static_cast<int64_t>(dt_s * 1e9);
    const int64_t t0_ns = static_cast<int64_t>(t_min * 1e9);

    basalt::Se3Spline<4> spline(static_cast<int64_t>(dt_s * 1e9), t0_ns);
    int num_knots = static_cast<int>(std::ceil((t_max - t_min) / dt_s)) + 4;
    num_knots = std::max(num_knots, 8);

    for (int i = 0; i < num_knots; ++i) {
        double t = t_min + i * dt_s;
        spline.knotsPushBack(interpolate_pose(T_w_imu_poses, t));
    }

    // Step 2: Ceres 优化外参 (6) + 时间偏移 (1)
    double extr[6];
    Eigen::Vector3d rvec = result.SO3_TargetInRef.log();
    extr[0] = rvec(0); extr[1] = rvec(1); extr[2] = rvec(2);
    extr[3] = result.POS_TargetInRef(0);
    extr[4] = result.POS_TargetInRef(1);
    extr[5] = result.POS_TargetInRef(2);
    double time_offset_s = result.time_offset_s;

    ceres::Problem problem;
    ceres::LossFunction* loss = new ceres::HuberLoss(1.0);

    for (size_t i = 0; i < lidar_odom_.size(); ++i) {
        int64_t t_ns = static_cast<int64_t>(lidar_odom_[i].first * 1e9);
        if (t_ns < spline.minTimeNs() || t_ns > spline.maxTimeNs())
            continue;
        ceres::CostFunction* cost =
            new SplineExtrinsicAnalyticCost(&spline, t_ns, lidar_odom_[i].second);
        problem.AddResidualBlock(cost, loss, extr, &time_offset_s);
    }

    problem.SetParameterLowerBound(&time_offset_s, 0, -cfg_.time_offset_max_s);
    problem.SetParameterUpperBound(&time_offset_s, 0, cfg_.time_offset_max_s);

    ceres::Solver::Options options;
    options.max_num_iterations = cfg_.ceres_max_iter;
    options.minimizer_progress_to_stdout = cfg_.verbose;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (cfg_.verbose)
        UNICALIB_INFO("[B-spline] Ceres: {} iterations, cost={:.6f}",
                      summary.iterations.size(), summary.final_cost);

    result.SO3_TargetInRef = Sophus::SO3d::exp(Eigen::Vector3d(extr[0], extr[1], extr[2]));
    result.POS_TargetInRef = Eigen::Vector3d(extr[3], extr[4], extr[5]);
    result.time_offset_s = time_offset_s;
    result.is_converged = (summary.termination_type == ceres::CONVERGENCE);

    UNICALIB_INFO("[IMU-LiDAR] 步骤: B样条精细优化完成 — time_offset={:.4f}s converged={}",
                  result.time_offset_s, result.is_converged ? "yes" : "no");

    return result;
}

// ===================================================================
// 手动校准验证
// ===================================================================
ExtrinsicSE3 IMULiDARCalibrator::calibrate_manual_verify(
    const std::vector<IMUFrame>& /*imu_data*/,
    const std::vector<LiDARScan>& /*lidar_scans*/,
    const ExtrinsicSE3& init_extrin) {
    // 最小实现：返回初值并打日志。完整实现需可视化旋转对比 + 增量调整或单步 B 样条 refine。
    UNICALIB_INFO("[IMU-LiDAR] calibrate_manual_verify: 使用初值（完整手动验证 TODO）");
    ExtrinsicSE3 out = init_extrin;
    out.is_converged = false;  // 未做验证步骤，不标记为已收敛
    return out;
}

}  // namespace ns_unicalib
