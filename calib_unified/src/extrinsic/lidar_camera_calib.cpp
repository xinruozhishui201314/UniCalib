/**
 * UniCalib Unified — LiDAR-Camera 外参标定实现
 * 方法:
 *   B: 棋盘格目标法 — LiDAR角点+图像角点 PnP
 *   C: 边缘对齐法   — NCC 最大化 (MIAS-LCEC 思路)
 *   A: 运动法       — 手眼 + B样条连续时间
 */

#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/common/logger.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace ns_unicalib {

// ===================================================================
// 生成 LiDAR 强度投影图
// ===================================================================
cv::Mat LiDARCameraCalibrator::lidar_to_intensity_image(
    const LiDARScan& scan,
    const Sophus::SE3d& T_cam_in_lidar,
    const CameraIntrinsics& cam_intrin) {

    cv::Mat img = cv::Mat::zeros(cam_intrin.height, cam_intrin.width, CV_32F);
    if (!scan.cloud) return img;

    // T_cam_in_lidar: 将 LiDAR 点变换到 cam 坐标系
    for (const auto& pt : scan.cloud->points) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T_cam_in_lidar * p_l;
        if (p_c.z() < 0.1) continue;

        double u = cam_intrin.fx * p_c.x() / p_c.z() + cam_intrin.cx;
        double v = cam_intrin.fy * p_c.y() / p_c.z() + cam_intrin.cy;
        int iu = static_cast<int>(std::round(u));
        int iv = static_cast<int>(std::round(v));
        if (iu < 0 || iu >= cam_intrin.width || iv < 0 || iv >= cam_intrin.height) continue;
        if (img.at<float>(iv, iu) < pt.intensity)
            img.at<float>(iv, iu) = pt.intensity;
    }
    return img;
}

// ===================================================================
// 从 LiDAR 点云检测棋盘格角点 (简化实现)
// ===================================================================
bool LiDARCameraCalibrator::detect_board_in_lidar(
    const LiDARScan& scan,
    std::vector<Eigen::Vector3d>& corners_3d) {

    corners_3d.clear();
    if (!scan.cloud || scan.cloud->empty()) return false;

    // 完整实现需要:
    //   1. RANSAC 平面拟合 (识别棋盘格平面)
    //   2. 投影到平面, 检测强度边缘
    //   3. 亚像素级角点定位
    // 这里返回简化结果以保证接口完整性
    UNICALIB_WARN("detect_board_in_lidar: 简化实现, 需要完整 RANSAC 平面 + 角点检测");
    return false;
}

// ===================================================================
// 方法 B: 棋盘格目标法
// ===================================================================
std::optional<ExtrinsicSE3> LiDARCameraCalibrator::calibrate_target(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const CameraIntrinsics& cam_intrin,
    const std::string& lidar_id,
    const std::string& cam_id) {

    UNICALIB_INFO("=== LiDAR-Camera 棋盘格目标法 ===");
    if (lidar_scans.empty() || camera_frames.empty()) {
        UNICALIB_ERROR("数据为空");
        return std::nullopt;
    }

    // 同步帧 (时间戳最近匹配)
    std::vector<cv::Point3f> pts3d_all;
    std::vector<cv::Point2f> pts2d_all;

    // 图像棋盘格检测
    cv::Size pattern_size(cfg_.board_cols, cfg_.board_rows);

    for (const auto& [ts_cam, img_cam] : camera_frames) {
        if (img_cam.empty()) continue;

        // 找最近的 LiDAR 帧
        const LiDARScan* best_scan = nullptr;
        double best_dt = 1e9;
        for (const auto& scan : lidar_scans) {
            double dt = std::abs(scan.timestamp - ts_cam);
            if (dt < best_dt) { best_dt = dt; best_scan = &scan; }
        }
        if (!best_scan || best_dt > 0.1) continue;

        // 图像角点检测
        cv::Mat gray;
        if (img_cam.channels() == 3) cv::cvtColor(img_cam, gray, cv::COLOR_BGR2GRAY);
        else gray = img_cam.clone();
        std::vector<cv::Point2f> img_corners;
        bool found = cv::findChessboardCorners(gray, pattern_size, img_corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
        if (!found) continue;
        cv::cornerSubPix(gray, img_corners, cv::Size(11,11), cv::Size(-1,-1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

        // LiDAR 角点检测
        std::vector<Eigen::Vector3d> lidar_corners;
        if (!detect_board_in_lidar(*best_scan, lidar_corners)) continue;
        if (lidar_corners.size() != img_corners.size()) continue;

        for (size_t i = 0; i < lidar_corners.size(); ++i) {
            pts3d_all.emplace_back(
                static_cast<float>(lidar_corners[i].x()),
                static_cast<float>(lidar_corners[i].y()),
                static_cast<float>(lidar_corners[i].z()));
            pts2d_all.push_back(img_corners[i]);
        }
    }

    if (pts3d_all.size() < 20) {
        UNICALIB_ERROR("有效点对不足 (需≥20, 实有{})", pts3d_all.size());
        UNICALIB_INFO("  提示: detect_board_in_lidar 需要完整实现才能进行棋盘格法标定");
        return std::nullopt;
    }

    cv::Mat K = (cv::Mat_<double>(3,3) <<
        cam_intrin.fx, 0, cam_intrin.cx,
        0, cam_intrin.fy, cam_intrin.cy, 0, 0, 1);
    cv::Mat dist = cv::Mat(cam_intrin.dist_coeffs).reshape(1, 1);
    cv::Mat rvec, tvec;
    cv::Mat inliers;
    cv::solvePnPRansac(pts3d_all, pts2d_all, K, dist, rvec, tvec,
                       false, 200, 8.0f, 0.99, inliers, cv::SOLVEPNP_ITERATIVE);
    cv::solvePnPRefineLM(pts3d_all, pts2d_all, K, dist, rvec, tvec);

    std::vector<cv::Point2f> proj;
    cv::projectPoints(pts3d_all, rvec, tvec, K, dist, proj);
    double rms = 0.0;
    for (size_t i = 0; i < pts3d_all.size(); ++i) {
        double dx = pts2d_all[i].x - proj[i].x;
        double dy = pts2d_all[i].y - proj[i].y;
        rms += dx*dx + dy*dy;
    }
    rms = std::sqrt(rms / pts3d_all.size());
    UNICALIB_INFO("  PnP: RMS={:.3f}px 内点={}/{}", rms, inliers.rows, pts3d_all.size());

    Eigen::Vector3d rv(rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2));
    Eigen::Vector3d tv(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    Eigen::AngleAxisd aa(rv.norm(), rv.normalized());

    ExtrinsicSE3 result;
    result.ref_sensor_id    = lidar_id;
    result.target_sensor_id = cam_id;
    result.SO3_TargetInRef  = Sophus::SO3d(aa.toRotationMatrix());
    result.POS_TargetInRef  = tv;
    result.residual_rms     = rms;
    result.is_converged     = (rms < cfg_.max_reproj_error_px);
    return result;
}

// ===================================================================
// 方法 C: 边缘对齐法
// ===================================================================
std::optional<ExtrinsicSE3> LiDARCameraCalibrator::calibrate_edge_align(
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const CameraIntrinsics& cam_intrin,
    const Sophus::SE3d& init_guess,
    const std::string& lidar_id,
    const std::string& cam_id) {

    UNICALIB_INFO("=== LiDAR-Camera 边缘对齐法 ===");
    size_t N = std::min(lidar_scans.size(), camera_frames.size());
    if (N == 0) { UNICALIB_ERROR("数据为空"); return std::nullopt; }

    Sophus::SE3d T_cam_lidar = init_guess;
    double best_ncc = -1e9;

    // 简化: 在初值附近直接评估 (完整版需要 Ceres 梯度优化)
    for (size_t fi = 0; fi < std::min(N, size_t(30)); fi += std::max(size_t(1), N/30)) {
        const auto& [ts_cam, img_cam] = camera_frames[fi];
        if (img_cam.empty()) continue;

        // 找同步 LiDAR 帧
        const LiDARScan* best_scan = nullptr;
        double best_dt = 1e9;
        for (const auto& scan : lidar_scans) {
            double dt = std::abs(scan.timestamp - ts_cam);
            if (dt < best_dt) { best_dt = dt; best_scan = &scan; }
        }
        if (!best_scan) continue;

        cv::Mat gray;
        if (img_cam.channels() == 3) cv::cvtColor(img_cam, gray, cv::COLOR_BGR2GRAY);
        else gray = img_cam.clone();
        cv::Mat img_edges;
        cv::Canny(gray, img_edges, cfg_.edge_canny_low, cfg_.edge_canny_high);

        cv::Mat lidar_img = lidar_to_intensity_image(*best_scan, T_cam_lidar, cam_intrin);
        if (lidar_img.empty()) continue;
        double max_val;
        cv::minMaxLoc(lidar_img, nullptr, &max_val);
        if (max_val < 1e-3) continue;
        cv::Mat lidar_8u;
        lidar_img.convertTo(lidar_8u, CV_8U, 255.0 / max_val);
        cv::Mat lidar_edges;
        cv::Canny(lidar_8u, lidar_edges, 30, 100);

        // NCC
        cv::Mat e1, e2;
        img_edges.convertTo(e1, CV_32F, 1.0/255);
        lidar_edges.convertTo(e2, CV_32F, 1.0/255);
        cv::Scalar m1, s1, m2, s2;
        cv::meanStdDev(e1, m1, s1);
        cv::meanStdDev(e2, m2, s2);
        if (s1[0] < 1e-5 || s2[0] < 1e-5) continue;
        
        // 修复除零风险：检查分母
        double denom = s1[0] * s2[0] * e1.total();
        if (std::abs(denom) < 1e-10) {
            // 方差接近零，NCC 无意义，跳过此帧
            UNICALIB_DEBUG("[EdgeAlign] 方差过小，跳过帧: denom={:.2e}", denom);
            continue;
        }
        double ncc = (e1 - m1[0]).dot(e2 - m2[0]) / denom;
        best_ncc = std::max(best_ncc, ncc);
    }

    UNICALIB_INFO("  边缘对齐 NCC={:.4f}", best_ncc);
    UNICALIB_WARN("  边缘对齐使用初值评估, 完整梯度优化需 Ceres 集成");

    ExtrinsicSE3 result;
    result.ref_sensor_id    = lidar_id;
    result.target_sensor_id = cam_id;
    result.set_SE3(T_cam_lidar);
    result.residual_rms  = 1.0 - std::max(0.0, best_ncc);
    result.is_converged  = (best_ncc > 0.3);
    return result;
}

// ===================================================================
// 方法 A: 运动法 (手眼)
// ===================================================================
std::optional<ExtrinsicSE3> LiDARCameraCalibrator::calibrate_motion(
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& camera_frames,
    const ExtrinsicSE3& T_lidar_in_imu,
    const CameraIntrinsics& cam_intrin,
    const std::string& lidar_id,
    const std::string& cam_id) {

    UNICALIB_INFO("=== LiDAR-Camera 运动法 ===");
    (void)imu_data; (void)cam_intrin;

    // 简化: 使用 LiDAR 和相机的相对运动进行手眼标定
    // 完整版需要 B样条插值 + Ceres 联合优化
    UNICALIB_WARN("运动法当前使用简化实现, 完整版需要 iKalibr CalibSolver 接口");

    if (lidar_scans.size() < 4 || camera_frames.size() < 4) {
        UNICALIB_ERROR("数据不足");
        return std::nullopt;
    }

    ExtrinsicSE3 result;
    result.ref_sensor_id    = lidar_id;
    result.target_sensor_id = cam_id;
    result.set_SE3(T_lidar_in_imu.SE3_TargetInRef());
    result.is_converged = false;
    result.residual_rms = 999.0;
    UNICALIB_INFO("  运动法: 返回 IMU-LiDAR 外参作为 LiDAR-Camera 初值 (需要完整实现)");
    return result;
}

// ===================================================================
// 可视化投影
// ===================================================================
void LiDARCameraCalibrator::visualize_projection(
    const LiDARScan& scan,
    const cv::Mat& image,
    const ExtrinsicSE3& extrin,
    const CameraIntrinsics& cam_intrin,
    const std::string& output_path) {

    if (!scan.cloud || image.empty()) return;

    cv::Mat vis = image.clone();
    if (vis.channels() == 1)
        cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);

    Sophus::SE3d T = extrin.SE3_TargetInRef();
    // T 是 cam_in_lidar 还是 lidar_in_cam 取决于约定
    // 这里假设 extrin 是 lidar_in_cam (cam 为 ref)

    double d_min = 1e9, d_max = -1e9;
    for (const auto& pt : scan.cloud->points) {
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T * p_l;
        if (p_c.z() > 0) {
            d_min = std::min(d_min, p_c.z());
            d_max = std::max(d_max, p_c.z());
        }
    }
    if (d_max <= d_min) return;

    for (const auto& pt : scan.cloud->points) {
        if (std::isnan(pt.x)) continue;
        Eigen::Vector3d p_l(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_c = T * p_l;
        if (p_c.z() < 0.1) continue;

        double u = cam_intrin.fx * p_c.x() / p_c.z() + cam_intrin.cx;
        double v = cam_intrin.fy * p_c.y() / p_c.z() + cam_intrin.cy;
        int iu = static_cast<int>(std::round(u));
        int iv = static_cast<int>(std::round(v));
        if (iu < 2 || iu >= cam_intrin.width-2 ||
            iv < 2 || iv >= cam_intrin.height-2) continue;

        // 深度着色 (近=红, 远=蓝)
        double ratio = (p_c.z() - d_min) / (d_max - d_min);
        int r = static_cast<int>((1-ratio) * 255);
        int b = static_cast<int>(ratio * 255);
        cv::circle(vis, {iu, iv}, 2, cv::Scalar(b, 80, r), -1);
    }

    cv::imwrite(output_path, vis);
    UNICALIB_INFO("投影可视化保存: {}", output_path);
}

// ===================================================================
// 边缘对齐评分
// ===================================================================
EdgeAlignmentScore LiDARCameraCalibrator::evaluate_edge_alignment(
    const LiDARScan& scan,
    const cv::Mat& image,
    const ExtrinsicSE3& extrin,
    const CameraIntrinsics& cam_intrin) {

    EdgeAlignmentScore score;
    if (!scan.cloud || image.empty()) return score;

    Sophus::SE3d T = extrin.SE3_TargetInRef();
    cv::Mat lidar_img = lidar_to_intensity_image(scan, T, cam_intrin);

    cv::Mat gray;
    if (image.channels() == 3) cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    else gray = image.clone();
    cv::Mat img_edges, lidar_edges;
    cv::Canny(gray, img_edges, cfg_.edge_canny_low, cfg_.edge_canny_high);

    double max_val;
    cv::minMaxLoc(lidar_img, nullptr, &max_val);
    if (max_val < 1e-3) return score;
    cv::Mat lidar_8u;
    lidar_img.convertTo(lidar_8u, CV_8U, 255.0/max_val);
    cv::Canny(lidar_8u, lidar_edges, 30, 100);

    score.num_lidar_edge_pts = cv::countNonZero(lidar_edges);
    score.num_image_edge_pts = cv::countNonZero(img_edges);

    cv::Mat e1, e2;
    img_edges.convertTo(e1, CV_32F, 1.0/255);
    lidar_edges.convertTo(e2, CV_32F, 1.0/255);
    cv::Scalar m1, s1, m2, s2;
    cv::meanStdDev(e1, m1, s1);
    cv::meanStdDev(e2, m2, s2);
    if (s1[0] > 1e-5 && s2[0] > 1e-5)
        score.ncc_score = (e1 - m1[0]).dot(e2 - m2[0]) /
                          (s1[0] * s2[0] * e1.total() + 1e-10);

    return score;
}

}  // namespace ns_unicalib
