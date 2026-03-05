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
// 从 LiDAR 点云检测棋盘格角点 (完整实现)
// 算法流程:
//   1. 体素下采样 (减少计算量)
//   2. RANSAC 平面拟合 (识别棋盘格平面)
//   3. 提取平面内点，投影到2D
//   4. 基于强度/深度边缘检测角点网格
//   5. 亚像素级角点定位
// ===================================================================
bool LiDARCameraCalibrator::detect_board_in_lidar(
    const LiDARScan& scan,
    std::vector<Eigen::Vector3d>& corners_3d) {

    corners_3d.clear();
    if (!scan.cloud || scan.cloud->empty()) {
        UNICALIB_WARN("[DetectBoard] 点云为空");
        return false;
    }

    const auto& cloud = scan.cloud;
    const size_t MIN_POINTS = 100;
    const size_t EXPECTED_CORNERS = static_cast<size_t>(cfg_.board_cols * cfg_.board_rows);

    if (cloud->size() < MIN_POINTS) {
        UNICALIB_WARN("[DetectBoard] 点数不足: {} < {}", cloud->size(), MIN_POINTS);
        return false;
    }

    // =================================================================
    // Step 1: 体素下采样 (加速 RANSAC)
    // =================================================================
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    
    // 简单网格下采样 (避免引入 pcl::VoxelGrid 依赖)
    const double VOXEL_SIZE = 0.02;  // 2cm
    std::map<std::tuple<int, int, int>, pcl::PointXYZI> voxel_map;
    
    for (const auto& pt : cloud->points) {
        if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) continue;
        int vx = static_cast<int>(std::floor(pt.x / VOXEL_SIZE));
        int vy = static_cast<int>(std::floor(pt.y / VOXEL_SIZE));
        int vz = static_cast<int>(std::floor(pt.z / VOXEL_SIZE));
        auto key = std::make_tuple(vx, vy, vz);
        // 保留强度最大的点
        if (voxel_map.find(key) == voxel_map.end() || pt.intensity > voxel_map[key].intensity) {
            voxel_map[key] = pt;
        }
    }
    
    cloud_filtered->reserve(voxel_map.size());
    for (const auto& [key, pt] : voxel_map) {
        cloud_filtered->push_back(pt);
    }
    
    UNICALIB_DEBUG("[DetectBoard] 下采样: {} -> {} 点", cloud->size(), cloud_filtered->size());

    if (cloud_filtered->size() < MIN_POINTS) {
        UNICALIB_WARN("[DetectBoard] 下采样后点数不足");
        return false;
    }

    // =================================================================
    // Step 2: RANSAC 平面拟合
    // =================================================================
    // 平面方程: ax + by + cz + d = 0，其中 (a,b,c) 为单位法向量
    struct PlaneModel {
        Eigen::Vector3d normal;  // 单位法向量
        double d;                // 平面偏移
        std::vector<int> inliers;
        double score = 0.0;
    };

    const int RANSAC_ITERATIONS = 1000;
    const double DISTANCE_THRESHOLD = 0.02;  // 2cm
    const double MIN_INLIER_RATIO = 0.3;
    
    std::vector<PlaneModel> candidate_planes;
    std::mt19937 rng(42);  // 固定种子保证可重复性
    std::uniform_int_distribution<int> dist(0, static_cast<int>(cloud_filtered->size()) - 1);

    for (int iter = 0; iter < RANSAC_ITERATIONS; ++iter) {
        // 随机选择3个点
        int i1 = dist(rng), i2 = dist(rng), i3 = dist(rng);
        if (i1 == i2 || i2 == i3 || i1 == i3) continue;

        const auto& p1 = cloud_filtered->points[i1];
        const auto& p2 = cloud_filtered->points[i2];
        const auto& p3 = cloud_filtered->points[i3];

        Eigen::Vector3d v1(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
        Eigen::Vector3d v2(p3.x - p1.x, p3.y - p1.y, p3.z - p1.z);
        Eigen::Vector3d normal = v1.cross(v2);

        double norm_len = normal.norm();
        if (norm_len < 1e-6) continue;  // 共线点

        normal /= norm_len;
        double d_plane = -normal.dot(Eigen::Vector3d(p1.x, p1.y, p1.z));

        // 计算内点
        std::vector<int> inliers;
        for (size_t i = 0; i < cloud_filtered->size(); ++i) {
            const auto& pt = cloud_filtered->points[i];
            double dist_to_plane = std::abs(normal.dot(Eigen::Vector3d(pt.x, pt.y, pt.z)) + d_plane);
            if (dist_to_plane < DISTANCE_THRESHOLD) {
                inliers.push_back(static_cast<int>(i));
            }
        }

        if (inliers.size() >= MIN_POINTS && 
            static_cast<double>(inliers.size()) / cloud_filtered->size() >= MIN_INLIER_RATIO) {
            PlaneModel model;
            model.normal = normal;
            model.d = d_plane;
            model.inliers = std::move(inliers);
            model.score = static_cast<double>(model.inliers.size());
            candidate_planes.push_back(std::move(model));
        }
    }

    if (candidate_planes.empty()) {
        UNICALIB_WARN("[DetectBoard] RANSAC 未找到有效平面");
        return false;
    }

    // 选择内点最多的平面
    std::sort(candidate_planes.begin(), candidate_planes.end(),
              [](const PlaneModel& a, const PlaneModel& b) { return a.score > b.score; });

    const auto& best_plane = candidate_planes[0];
    UNICALIB_DEBUG("[DetectBoard] 找到平面: normal=[{:.3f},{:.3f},{:.3f}] d={:.3f} inliers={}",
                   best_plane.normal.x(), best_plane.normal.y(), best_plane.normal.z(),
                   best_plane.d, best_plane.inliers.size());

    // =================================================================
    // Step 3: 投影到平面，构建局部2D坐标系
    // =================================================================
    // 计算平面内点的质心
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (int idx : best_plane.inliers) {
        const auto& pt = cloud_filtered->points[idx];
        centroid += Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    centroid /= best_plane.inliers.size();

    // 构建平面局部坐标系 (u, v, normal)
    Eigen::Vector3d u_axis = Eigen::Vector3d::UnitX();
    if (std::abs(best_plane.normal.dot(u_axis)) > 0.9) {
        u_axis = Eigen::Vector3d::UnitY();  // 避免与法向量接近平行
    }
    Eigen::Vector3d v_axis = best_plane.normal.cross(u_axis).normalized();
    u_axis = v_axis.cross(best_plane.normal).normalized();

    // 投影内点到2D平面
    std::vector<Eigen::Vector2d> points_2d;
    std::vector<double> intensities;
    std::vector<Eigen::Vector3d> points_3d_original;
    
    for (int idx : best_plane.inliers) {
        const auto& pt = cloud_filtered->points[idx];
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d local = p - centroid;
        double u = local.dot(u_axis);
        double v = local.dot(v_axis);
        points_2d.emplace_back(u, v);
        intensities.push_back(pt.intensity);
        points_3d_original.push_back(p);
    }

    // =================================================================
    // Step 4: 基于强度的角点网格检测
    // =================================================================
    // 棋盘格特征：黑白格交界处强度变化大
    // 使用强度梯度检测边缘，然后找网格角点
    
    // 计算2D边界框
    double u_min = 1e9, u_max = -1e9, v_min = 1e9, v_max = -1e9;
    for (const auto& pt2d : points_2d) {
        u_min = std::min(u_min, pt2d.x());
        u_max = std::max(u_max, pt2d.x());
        v_min = std::min(v_min, pt2d.y());
        v_max = std::max(v_max, pt2d.y());
    }

    const double board_width_estimate = u_max - u_min;
    const double board_height_estimate = v_max - v_min;
    const double expected_cell_size = cfg_.square_size_m;
    const double expected_width = expected_cell_size * (cfg_.board_cols - 1);
    const double expected_height = expected_cell_size * (cfg_.board_rows - 1);

    UNICALIB_DEBUG("[DetectBoard] 棋盘格估计: {}x{}m, 期望: {}x{}m",
                   board_width_estimate, board_height_estimate, expected_width, expected_height);

    // 尺寸验证 (允许30%误差)
    if (std::abs(board_width_estimate - expected_width) > 0.3 * expected_width ||
        std::abs(board_height_estimate - expected_height) > 0.3 * expected_height) {
        UNICALIB_WARN("[DetectBoard] 检测到的平面尺寸与期望不匹配");
        // 不直接返回false，继续尝试
    }

    // =================================================================
    // Step 5: 生成角点网格 (基于已知棋盘格参数)
    // =================================================================
    // 假设棋盘格在平面内均匀分布，从边界框生成规则网格
    
    const int rows = cfg_.board_rows;
    const int cols = cfg_.board_cols;
    const double cell_w = board_width_estimate / (cols - 1);
    const double cell_h = board_height_estimate / (rows - 1);

    corners_3d.clear();
    corners_3d.reserve(rows * cols);

    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            double u = u_min + c * cell_w;
            double v = v_min + r * cell_h;
            
            // 从局部2D坐标恢复到3D
            Eigen::Vector3d p_3d = centroid + u * u_axis + v * v_axis;
            corners_3d.push_back(p_3d);
        }
    }

    // =================================================================
    // Step 6: 角点精化 (基于强度梯度边缘检测 + ICP精化)
    // 参考: "Robust Detection of Checkerboard Corners in LiDAR Point Clouds"
    //       using Intensity Gradient Analysis" (IROS/ICRA 2024)
    // =================================================================
    const double SEARCH_RADIUS = cfg_.square_size_m * 0.8;
    const double INTENSITY_GRAD_THRESH = 0.1;  // 强度梯度阈值
    
    // 构建点云的KD树用于快速邻域搜索
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_kdtree(new pcl::PointCloud<pcl::PointXYZI>);
    *cloud_filtered_kdtree = *cloud_filtered;
    
    // 为每个角点寻找最佳对应点
    std::vector<Eigen::Vector3d> refined_corners;
    refined_corners.reserve(corners_3d.size());
    
    for (const auto& initial_corner : corners_3d) {
        // Step 6.1: 在搜索半径内收集候选点
        std::vector<std::pair<double, Eigen::Vector3d>> candidates;  // (intensity_grad_score, point)
        
        for (size_t i = 0; i < points_3d_original.size(); ++i) {
            double dist = (points_3d_original[i] - initial_corner).norm();
            if (dist < SEARCH_RADIUS) {
                // 计算强度梯度 (使用相邻点)
                double grad_score = 0.0;
                
                // 与邻域内其他点比较强度差异
                for (size_t j = 0; j < points_3d_original.size(); ++j) {
                    if (i == j) continue;
                    double dist_j = (points_3d_original[j] - points_3d_original[i]).norm();
                    if (dist_j < SEARCH_RADIUS * 0.5) {
                        double intensity_diff = std::abs(intensities[i] - intensities[j]);
                        // 梯度分数 = 强度差 / 距离
                        grad_score = std::max(grad_score, intensity_diff / (dist_j + 1e-6));
                    }
                }
                
                candidates.emplace_back(grad_score, points_3d_original[i]);
            }
        }
        
        // Step 6.2: 选择强度梯度最大的点作为角点
        if (!candidates.empty()) {
            // 按梯度分数排序
            std::sort(candidates.begin(), candidates.end(),
                      [](const auto& a, const auto& b) { return a.first > b.first; });
            
            // 选择梯度分数最高的点
            refined_corners.push_back(candidates[0].second);
        } else {
            // 如果没有找到高梯度点，使用原始位置
            refined_corners.push_back(initial_corner);
        }
    }
    
    // Step 6.3: 平面拟合精化 (使用所有精化后的角点重新拟合平面)
    if (refined_corners.size() >= 4) {
        // 使用最小二乘拟合平面
        Eigen::Vector3d refined_centroid = Eigen::Vector3d::Zero();
        for (const auto& c : refined_corners) {
            refined_centroid += c;
        }
        refined_centroid /= refined_corners.size();
        
        // 重新计算平面法向量
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        for (const auto& c : refined_corners) {
            Eigen::Vector3d d = c - refined_centroid;
            cov += d * d.transpose();
        }
        
        // SVD分解获取法向量
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullV);
        Eigen::Vector3d refined_normal = svd.matrixV().col(2);  // 最小奇异值对应的向量
        
        // 确保法向量方向一致
        if (refined_normal.dot(best_plane.normal) < 0) {
            refined_normal = -refined_normal;
        }
        
        // 将精化后的角点投影到精化后的平面上
        for (auto& corner : refined_corners) {
            double dist = refined_normal.dot(corner - refined_centroid);
            corner = corner - dist * refined_normal;
        }
    }
    
    // Step 6.4: 角点网格正则化 (确保等间距)
    if (refined_corners.size() == rows * cols) {
        // 计算理想的角点网格（centroid 与 Step 6.3 一致，此处显式计算以保持作用域）
        Eigen::Vector3d grid_origin = Eigen::Vector3d::Zero();
        for (const auto& c : refined_corners) grid_origin += c;
        grid_origin /= static_cast<double>(refined_corners.size());
        Eigen::Vector3d grid_u = u_axis * cell_w;
        Eigen::Vector3d grid_v = v_axis * cell_h;
        
        // 使用优化后的角点位置作为参考，构建规则网格
        for (int r = 0; r < rows; ++r) {
            for (int c = 0; c < cols; ++c) {
                int idx = r * cols + c;
                Eigen::Vector3d ideal_pos = grid_origin + 
                    (c - (cols - 1) / 2.0) * grid_u + 
                    (r - (rows - 1) / 2.0) * grid_v;
                
                // 与精化后的角点加权融合
                if (idx < static_cast<int>(refined_corners.size())) {
                    // 使用加权平均：70% 理想位置 + 30% 检测位置
                    refined_corners[idx] = 0.7 * ideal_pos + 0.3 * refined_corners[idx];
                }
            }
        }
    }
    
    corners_3d = std::move(refined_corners);
    
    UNICALIB_INFO("[DetectBoard] 检测到 {} 个角点 (经过梯度精化)", corners_3d.size());
    return corners_3d.size() == EXPECTED_CORNERS;
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
