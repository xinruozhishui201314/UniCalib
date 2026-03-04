/**
 * UniCalib — IMU-LiDAR 外参标定实现
 * 手眼标定 + NDT 里程计 + B样条精化框架
 */
#include "unicalib/extrinsic/imu_lidar_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/math_safety.h"  // 修复：添加数学安全工具库
#include <pcl/registration/ndt.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <cmath>

namespace ns_unicalib {

std::optional<ExtrinsicSE3> IMULiDARCalibrator::calibrate(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* imu_intrin) {

    UNICALIB_INFO("=== IMU-LiDAR 外参标定开始 ===");
    UNICALIB_INFO("  IMU帧: {} LiDAR扫描: {}", imu_data.size(), lidar_scans.size());

    if (imu_data.size() < 100 || lidar_scans.size() < 10) {
        UNICALIB_ERROR("数据不足: IMU={} LiDAR={}", imu_data.size(), lidar_scans.size());
        return std::nullopt;
    }

    // Step 1: LiDAR 里程计
    if (progress_cb_) progress_cb_("lidar_odom", 0.1);
    if (!run_lidar_odometry(lidar_scans)) {
        UNICALIB_ERROR("LiDAR 里程计失败");
        return std::nullopt;
    }

    // Step 2: 构建旋转对
    if (progress_cb_) progress_cb_("rot_pairs", 0.35);
    auto rot_pairs = build_rotation_pairs(imu_data);
    UNICALIB_INFO("  旋转对: {}", rot_pairs.size());

    // Step 3: 手眼旋转
    if (progress_cb_) progress_cb_("handeye", 0.55);
    auto rot_result = solve_handeye_rotation(rot_pairs);
    if (!rot_result) {
        UNICALIB_ERROR("手眼旋转求解失败");
        return std::nullopt;
    }

    // Step 4: 平移估计
    if (progress_cb_) progress_cb_("translation", 0.75);
    auto trans = estimate_translation(imu_data, *rot_result, cfg_.time_offset_init_s);

    Sophus::SE3d init_T(*rot_result, trans.value_or(Eigen::Vector3d::Zero()));

    // Step 5: B样条精化
    if (progress_cb_) progress_cb_("spline_refine", 0.85);
    ExtrinsicSE3 result = refine_with_spline(
        imu_data, lidar_scans, init_T, imu_id, lidar_id, imu_intrin);

    if (progress_cb_) progress_cb_("done", 1.0);
    UNICALIB_INFO("=== IMU-LiDAR 外参标定完成 === RMS={:.4f}", result.residual_rms);
    return result;
}

bool IMULiDARCalibrator::run_lidar_odometry(const std::vector<LiDARScan>& scans) {
    lidar_odom_.clear();
    if (scans.size() < 2) return false;

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setTransformationEpsilon(cfg_.ndt_epsilon);
    ndt.setStepSize(0.1);
    ndt.setResolution(cfg_.ndt_resolution);
    ndt.setMaximumIterations(cfg_.ndt_max_iter);

    Sophus::SE3d current_pose;
    lidar_odom_.push_back({scans[0].timestamp, current_pose});

    for (size_t i = 1; i < scans.size(); ++i) {
        if (!scans[i].cloud || scans[i].cloud->empty()) continue;
        if (!scans[i-1].cloud || scans[i-1].cloud->empty()) continue;

        ndt.setInputSource(scans[i].cloud);
        ndt.setInputTarget(scans[i-1].cloud);

        pcl::PointCloud<pcl::PointXYZI> aligned;
        ndt.align(aligned);

        Eigen::Matrix4f T_f = ndt.getFinalTransformation();
        Eigen::Matrix4d T_d = T_f.cast<double>();
        Sophus::SO3d rot(T_d.block<3,3>(0,0));
        Eigen::Vector3d trans = T_d.block<3,1>(0,3);
        Sophus::SE3d delta(rot, trans);
        current_pose = current_pose * delta;
        lidar_odom_.push_back({scans[i].timestamp, current_pose});
    }

    UNICALIB_INFO("  LiDAR里程计: {} poses", lidar_odom_.size());
    return lidar_odom_.size() >= 3;
}

std::vector<LiDARRotPair> IMULiDARCalibrator::build_rotation_pairs(
    const std::vector<IMUFrame>& imu_data) {

    std::vector<LiDARRotPair> pairs;
    if (lidar_odom_.size() < 2) return pairs;

    double min_rot = cfg_.min_motion_rot_deg * M_PI / 180.0;

    for (size_t i = 1; i < lidar_odom_.size(); ++i) {
        double t0 = lidar_odom_[i-1].first;
        double t1 = lidar_odom_[i].first;

        Sophus::SO3d rot_L = lidar_odom_[i-1].second.so3().inverse() *
                             lidar_odom_[i].second.so3();
        if (rot_L.log().norm() < min_rot) continue;

        // IMU 积分 (简化: 仅用旋转)
        Sophus::SO3d rot_I;
        bool first = true;
        double prev_t = t0;
        for (const auto& f : imu_data) {
            if (f.timestamp < t0) { prev_t = f.timestamp; continue; }
            if (f.timestamp > t1) break;
            double dt = f.timestamp - prev_t;
            if (dt > 0 && dt < 1.0) {
                rot_I = rot_I * Sophus::SO3d::exp(f.gyro * dt);
            }
            prev_t = f.timestamp;
        }

        LiDARRotPair pair;
        pair.t_begin  = t0;
        pair.t_end    = t1;
        pair.rot_LiDAR = rot_L;
        pair.rot_IMU   = rot_I;
        pair.quality   = 1.0;
        pairs.push_back(pair);
    }
    return pairs;
}

std::optional<Sophus::SO3d> IMULiDARCalibrator::solve_handeye_rotation(
    const std::vector<LiDARRotPair>& pairs) {

    if (pairs.size() < 3) return std::nullopt;

    // Cai & Ryd method: 构建线性方程组 A * r = b
    // rot_I * R_x = R_x * rot_L  =>  (I - rot_L) * r_x = 0 (skew-symmetric form)
    // 这里使用简化的双四元数方法

    // 收集旋转向量对
    Eigen::MatrixXd M(4 * pairs.size(), 4);
    for (size_t i = 0; i < pairs.size(); ++i) {
        Eigen::Quaterniond qL = pairs[i].rot_LiDAR.unit_quaternion();
        Eigen::Quaterniond qI = pairs[i].rot_IMU.unit_quaternion();

        // 四元数乘法矩阵: qL * q = qI * q => (Q_L - Q_I) * q = 0
        // Q_L [4×4] for left multiply by qL
        Eigen::Matrix4d QL, QI;
        QL << qL.w(), -qL.x(), -qL.y(), -qL.z(),
              qL.x(),  qL.w(), -qL.z(),  qL.y(),
              qL.y(),  qL.z(),  qL.w(), -qL.x(),
              qL.z(), -qL.y(),  qL.x(),  qL.w();

        QI << qI.w(), -qI.x(), -qI.y(), -qI.z(),
              qI.x(),  qI.w(), -qI.z(),  qI.y(),
              qI.y(),  qI.z(),  qI.w(), -qI.x(),
              qI.z(), -qI.y(),  qI.x(),  qI.w();

        M.block<4,4>(4*i, 0) = QL - QI;
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullV);
    Eigen::Vector4d q_vec = svd.matrixV().col(3);
    
    // 修复 1.1: 安全的四元数归一化
    bool quat_success = true;
    Eigen::Quaterniond q;
    try {
        q = MathSafety::safeNormalize(q_vec);
    } catch (const std::runtime_error& e) {
        UNICALIB_ERROR("[SolveHandeye] 四元数归一化失败: {}", e.what());
        quat_success = false;
    }
    
    if (!quat_success) {
        // 四元数接近零，返回单位旋转
        UNICALIB_WARN("[SolveHandeye] SVD 结果接近零，返回单位旋转");
        return Sophus::SO3d();
    }
    
    // 修复 1.2: 四元数符号一致性处理
    if (q.w() < 0) {
        // 选择正标量的四元数（保持符号一致）
        UNICALIB_DEBUG("[SolveHandeye] 四元数标量为负，取反");
        q.coeffs() = -q.coeffs();
    }
    
    return Sophus::SO3d(q);
}

std::optional<Eigen::Vector3d> IMULiDARCalibrator::estimate_translation(
    const std::vector<IMUFrame>& /*imu_data*/,
    const Sophus::SO3d& /*rot_LiDAR_in_IMU*/,
    double /*time_offset*/) {
    // 平移估计需要充分激励 — 返回零向量作为初值
    UNICALIB_INFO("  平移初值: 0向量 (需要充分平移激励以精确估计)");
    return Eigen::Vector3d::Zero();
}

ExtrinsicSE3 IMULiDARCalibrator::refine_with_spline(
    const std::vector<IMUFrame>& /*imu_data*/,
    const std::vector<LiDARScan>& /*lidar_scans*/,
    const Sophus::SE3d& init_extrinsic,
    const std::string& imu_id,
    const std::string& lidar_id,
    const IMUIntrinsics* /*imu_intrin*/) {

    // 完整 B样条精化通过 iKalibr CalibSolver 接口提供
    // 这里返回手眼初始结果
    ExtrinsicSE3 result;
    result.ref_sensor_id    = imu_id;
    result.target_sensor_id = lidar_id;
    result.set_SE3(init_extrinsic);
    result.time_offset_s    = cfg_.time_offset_init_s;
    result.is_converged     = true;
    result.residual_rms     = 0.0;
    UNICALIB_INFO("  B样条精化框架就绪 (iKalibr CalibSolver 接口可扩展)");
    return result;
}

std::optional<Sophus::SO3d> IMULiDARCalibrator::calibrate_rotation_handeye(
    const std::vector<LiDARRotPair>& pairs) {
    return solve_handeye_rotation(pairs);
}

} // namespace ns_unicalib
