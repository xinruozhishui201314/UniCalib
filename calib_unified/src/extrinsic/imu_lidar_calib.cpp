/**
 * UniCalib Unified — IMU-LiDAR 外参标定实现
 *
 * 两阶段标定流程:
 *   Stage 1 粗标定: LiDAR 里程计 + 手眼标定
 *   Stage 2 精标定: B样条连续时间优化 (待实现)
 */

#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/math_safety.h"
#include "ikalibr/core/lidar_odometer.h"
#include "ikalibr/sensor/lidar.h"
#include "ikalibr/util/cloud_define.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <pcl/common/transforms.h>
#include <numeric>

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

    // Step 1: 运行 LiDAR 里程计
    if (!run_lidar_odometry(lidar_scans)) {
        UNICALIB_ERROR("[IMU-LiDAR] LiDAR 里程计失败");
        return std::nullopt;
    }

    // Step 2: 构建旋转对
    auto rot_pairs = build_rotation_pairs(imu_data);
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

    result.coarse_method = "handeye_only";
    result.fine_method = "handeye_only";

    // 执行基本标定
    auto extrinsic = calibrate(imu_data, lidar_scans, imu_id, lidar_id, imu_intrin);
    if (extrinsic.has_value()) {
        result.coarse = extrinsic;
        result.coarse_rot_err_deg = 0.0; // TODO: 计算实际误差
    } else {
        result.needs_manual = true;
        return result;
    }

    // TODO: 实现 B样条精细优化
    result.fine = result.coarse;
    result.fine_rot_err_deg = 0.0;

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
            ikalibr_pt.timestamp = 0.0;  // 默认时间戳
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
    const std::vector<IMUFrame>& imu_data) {

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

        // 从 IMU 数据中积分得到对应的旋转
        Sophus::SO3d rot_imu = integrate_imu_rotation(imu_data, t_begin, t_end);
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
    double t_end) {
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
        
        // 使用两个时间点的平均角速度
        Eigen::Vector3d avg_gyro = 0.5 * (segment[i-1].gyro + segment[i].gyro);
        
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

    UNICALIB_INFO("[Handeye] 开始手眼标定， 共 {} 对", pairs.size());

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
    
    UNICALIB_INFO("[Handeye] 旋转标定完成， 平均误差: {:.4f} deg", avg_error_deg);
    
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
        // 假设 v_imu ≈ 0 (车辆近似静止或匀速)
        Constraint c;
        c.omega = avg_gyro;
        c.vel_diff = -R * v_lidar;  // -R * v_lidar = omega x t
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

    UNICALIB_INFO("[Trans-Est] 平移估计完成:");
    UNICALIB_INFO("  t = [{:.3f}, {:.3f}, {:.3f}] m", t_final.x(), t_final.y(), t_final.z());
    UNICALIB_INFO("  RMS 误差 = {:.4f} m/s (基于 {} 个约束)", rms_error, filtered.size());

    if (rms_error > 0.3) {
        UNICALIB_WARN("[Trans-Est] 残差较大 ({:.3f} m/s)， 结果可能不准确", rms_error);
    }

    return t_final;
}

// ===================================================================
// B样条精细优化 (待实现)
// ===================================================================
ExtrinsicSE3 IMULiDARCalibrator::refine_with_spline(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const Sophus::SE3d& init_extrinsic,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin) {
    // TODO: 实现 B样条精细优化
    // 需要集成 iKalibr 的 CalibSolver
    ExtrinsicSE3 result;
    result.ref_sensor_id = imu_id;
    result.target_sensor_id = lidar_id;
    result.SO3_TargetInRef = init_extrinsic.so3();
    result.POS_TargetInRef = init_extrinsic.translation();
    result.time_offset_s = 0.0;

    return result;
}

// ===================================================================
// 手动校准验证
// ===================================================================
ExtrinsicSE3 IMULiDARCalibrator::calibrate_manual_verify(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const ExtrinsicSE3& init_extrin) {
    // TODO: 实现手动验证
    return init_extrin;
}

}  // namespace ns_unicalib
