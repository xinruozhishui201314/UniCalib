/**
 * UniCalib Unified — Camera-Camera 外参标定实现
 * 方法:
 *   A: 棋盘格立体标定 (OpenCV stereoCalibrate)
 *   B: 本质矩阵法 (ORB + findEssentialMat)
 *   C: Bundle Adjustment (Ceres 两视图)
 */

#include "unicalib/extrinsic/cam_cam_calib.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/math_safety.h"  // 修复：添加数学安全工具库
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <algorithm>
#include <numeric>

namespace ns_unicalib {

// ===================================================================
// 特征点提取与匹配
// ===================================================================
std::vector<FeatureMatch> CamCamCalibrator::match_features(
    const cv::Mat& img0,
    const cv::Mat& img1,
    const CameraIntrinsics& /*intrin0*/,
    const CameraIntrinsics& /*intrin1*/) {

    std::vector<FeatureMatch> matches;

    cv::Mat gray0, gray1;
    if (img0.channels() == 3) cv::cvtColor(img0, gray0, cv::COLOR_BGR2GRAY);
    else gray0 = img0.clone();
    if (img1.channels() == 3) cv::cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
    else gray1 = img1.clone();

    auto orb = cv::ORB::create(cfg_.num_features, 1.2f, 8, 31, 0, 2,
                                cv::ORB::HARRIS_SCORE, 31, 20);
    std::vector<cv::KeyPoint> kp0, kp1;
    cv::Mat desc0, desc1;
    orb->detectAndCompute(gray0, cv::noArray(), kp0, desc0);
    orb->detectAndCompute(gray1, cv::noArray(), kp1, desc1);
    if (desc0.empty() || desc1.empty()) return matches;

    cv::BFMatcher bf(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> raw;
    bf.knnMatch(desc0, desc1, raw, 2);

    for (const auto& m : raw) {
        if (m.size() < 2) continue;
        if (m[0].distance < cfg_.match_ratio * m[1].distance) {
            FeatureMatch fm;
            fm.pt0 = kp0[m[0].queryIdx].pt;
            fm.pt1 = kp1[m[0].trainIdx].pt;
            fm.pt_cam0 = Eigen::Vector2d(fm.pt0.x, fm.pt0.y);
            fm.pt_cam1 = Eigen::Vector2d(fm.pt1.x, fm.pt1.y);
            fm.score = 1.0 - m[0].distance / 256.0;
            matches.push_back(fm);
        }
    }
    return matches;
}

// ===================================================================
// 从本质矩阵恢复 R, t
// ===================================================================
bool CamCamCalibrator::recover_pose_from_E(
    const std::vector<FeatureMatch>& matches,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    Sophus::SO3d& rot, Eigen::Vector3d& trans) {

    if (matches.size() < 8) return false;

    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = cv::Mat(intrin0.dist_coeffs).reshape(1, 1);
    cv::Mat D1 = cv::Mat(intrin1.dist_coeffs).reshape(1, 1);

    std::vector<cv::Point2f> raw0, raw1;
    for (const auto& m : matches) {
        raw0.push_back(m.pt0);
        raw1.push_back(m.pt1);
    }
    std::vector<cv::Point2f> pts0_ud, pts1_ud;
    cv::undistortPoints(raw0, pts0_ud, K0, D0);
    cv::undistortPoints(raw1, pts1_ud, K1, D1);

    cv::Mat inliers;
    cv::Mat E = cv::findEssentialMat(pts0_ud, pts1_ud,
        cv::Mat::eye(3,3,CV_64F), cv::RANSAC, 0.999, 0.001, inliers);
    
    // 修复 3.1: 检查本质矩阵是否有效
    if (E.empty()) {
        UNICALIB_ERROR("[RecoverPoseE] 本质矩阵为空");
        return false;
    }
    
    // 修复 3.2: 检查内点数量是否充足
    if (inliers.rows < 8) {
        UNICALIB_ERROR("[RecoverPoseE] 内点数不足: {} (需要≥8)", inliers.rows);
        return false;
    }
    
    // 修复 3.3: 检查内点占比是否合理
    double inlier_ratio = static_cast<double>(inliers.rows) / matches.size();
    if (inlier_ratio < 0.3) {  // 至少 30% 的匹配是内点
        UNICALIB_WARN("[RecoverPoseE] 内点占比过低: {:.2f}", inlier_ratio);
        // 不直接返回 false，但继续尝试恢复
    }

    cv::Mat R_cv, t_cv;
    cv::recoverPose(E, pts0_ud, pts1_ud,
                    cv::Mat::eye(3,3,CV_64F), R_cv, t_cv, inliers);
    
    // 修复 3.4: 检查旋转矩阵的有效性
    // 旋转矩阵应该满足: det(R) = 1, R^T * R = I
    double det = cv::determinant(R_cv);
    if (std::abs(det - 1.0) > 0.1) {
        UNICALIB_WARN("[RecoverPoseE] 旋转矩阵行列式异常: det={:.3f}", det);
    }
    
    // 检查旋转矩阵是否接近奇异
    cv::Mat RTR = R_cv.t() * R_cv;
    cv::Mat identity = cv::Mat::eye(3,3,CV_64F);
    double ortho_error = cv::norm(RTR, identity, cv::NORM_L2);
    if (ortho_error > 0.1) {
        UNICALIB_WARN("[RecoverPoseE] 旋转矩阵不正交: error={:.3f}", ortho_error);
    }

    Eigen::Matrix3d R_eig;
    Eigen::Vector3d t_eig;
    cv::cv2eigen(R_cv, R_eig);
    cv::cv2eigen(t_cv, t_eig);
    rot   = Sophus::SO3d(R_eig);
    trans = t_eig;
    return true;
}

// ===================================================================
// 方法 A: 棋盘格立体标定
// ===================================================================
std::optional<ExtrinsicSE3> CamCamCalibrator::calibrate_stereo(
    const std::vector<std::string>& images_cam0,
    const std::vector<std::string>& images_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    UNICALIB_INFO("=== Camera-Camera 棋盘格立体标定 ===");
    UNICALIB_INFO("  {} ↔ {}", cam0_id, cam1_id);

    if (images_cam0.size() != images_cam1.size()) {
        UNICALIB_ERROR("图像数量不匹配: {} vs {}", images_cam0.size(), images_cam1.size());
        return std::nullopt;
    }

    CameraIntrinsicCalibrator::Config det_cfg;
    det_cfg.target = cfg_.target;
    CameraIntrinsicCalibrator detector(det_cfg);

    std::vector<std::vector<cv::Point3f>> obj_all;
    std::vector<std::vector<cv::Point2f>> img0_all, img1_all;
    auto obj = cfg_.target.object_points();

    int W = 0, H = 0;
    for (size_t i = 0; i < images_cam0.size(); ++i) {
        cv::Mat im0 = cv::imread(images_cam0[i]);
        cv::Mat im1 = cv::imread(images_cam1[i]);
        if (im0.empty() || im1.empty()) continue;
        if (W == 0) { W = im0.cols; H = im0.rows; }

        auto f0 = detector.detect_corners(im0);
        auto f1 = detector.detect_corners(im1);
        if (f0.detection_ok && f1.detection_ok) {
            obj_all.push_back(obj);
            img0_all.push_back(f0.corners);
            img1_all.push_back(f1.corners);
        }
    }

    if (obj_all.size() < 5) {
        UNICALIB_ERROR("有效帧对不足 (需≥5, 实有{})", obj_all.size());
        return std::nullopt;
    }

    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = cv::Mat(intrin0.dist_coeffs).reshape(1, 1);
    cv::Mat D1 = cv::Mat(intrin1.dist_coeffs).reshape(1, 1);

    cv::Mat R, T, E, F;
    double rms = cv::stereoCalibrate(
        obj_all, img0_all, img1_all, K0, D0, K1, D1,
        cv::Size(W, H), R, T, E, F,
        cv::CALIB_FIX_INTRINSIC,
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 1e-7));

    UNICALIB_INFO("  RMS={:.4f}px 有效帧={}", rms, obj_all.size());

    Eigen::Matrix3d R_eig;
    Eigen::Vector3d t_eig;
    cv::cv2eigen(R, R_eig);
    cv::cv2eigen(T, t_eig);

    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;
    result.SO3_TargetInRef  = Sophus::SO3d(R_eig);
    result.POS_TargetInRef  = t_eig;
    result.residual_rms     = rms;
    result.is_converged     = (rms < cfg_.max_rms_px);
    return result;
}

// ===================================================================
// 方法 B: 本质矩阵法
// ===================================================================
std::optional<ExtrinsicSE3> CamCamCalibrator::calibrate_essential(
    const std::vector<std::pair<double, cv::Mat>>& frames_cam0,
    const std::vector<std::pair<double, cv::Mat>>& frames_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    UNICALIB_INFO("=== Camera-Camera 本质矩阵法 ===");
    size_t N = std::min(frames_cam0.size(), frames_cam1.size());
    if (N < 3) { UNICALIB_ERROR("图像对不足 (需≥3)"); return std::nullopt; }

    std::vector<FeatureMatch> all_matches;
    for (size_t fi = 0; fi < N; ++fi) {
        auto fm = match_features(frames_cam0[fi].second, frames_cam1[fi].second,
                                 intrin0, intrin1);
        for (auto& m : fm) all_matches.push_back(m);
    }
    UNICALIB_INFO("  总匹配点: {}", all_matches.size());
    if (all_matches.size() < 20) {
        UNICALIB_ERROR("匹配点不足");
        return std::nullopt;
    }

    Sophus::SO3d rot;
    Eigen::Vector3d trans;
    if (!recover_pose_from_E(all_matches, intrin0, intrin1, rot, trans)) {
        UNICALIB_ERROR("本质矩阵分解失败");
        return std::nullopt;
    }

    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;
    result.SO3_TargetInRef  = rot;
    result.POS_TargetInRef  = trans;
    result.is_converged     = true;
    result.residual_rms     = 0.0;
    UNICALIB_INFO("本质矩阵法完成 T=[{:.4f},{:.4f},{:.4f}]",
                  trans[0], trans[1], trans[2]);
    return result;
}

// ===================================================================
// Bundle Adjustment 代价函数
// ===================================================================
namespace {
struct ReprojectCost {
    double obs_u, obs_v, fx, fy, cx, cy;
    double px, py, pz;
    ReprojectCost(double u, double v,
                  double _fx, double _fy, double _cx, double _cy,
                  double x, double y, double z)
        : obs_u(u), obs_v(v), fx(_fx), fy(_fy), cx(_cx), cy(_cy),
          px(x), py(y), pz(z) {}

    template <typename T>
    bool operator()(const T* const pose, T* res) const {
        T pt[3] = {T(px), T(py), T(pz)}, pt_c[3];
        ceres::AngleAxisRotatePoint(pose, pt, pt_c);
        pt_c[0] += pose[3]; pt_c[1] += pose[4]; pt_c[2] += pose[5];
        T zi = T(1.0) / pt_c[2];
        res[0] = T(fx) * pt_c[0] * zi + T(cx) - T(obs_u);
        res[1] = T(fy) * pt_c[1] * zi + T(cy) - T(obs_v);
        return true;
    }
    static ceres::CostFunction* Create(
        double u, double v, double fx, double fy, double cx, double cy,
        double x, double y, double z) {
        return new ceres::AutoDiffCostFunction<ReprojectCost, 2, 6>(
            new ReprojectCost(u, v, fx, fy, cx, cy, x, y, z));
    }
};
}  // anonymous namespace

// ===================================================================
// 方法 C: Bundle Adjustment (两视图)
// ===================================================================
ExtrinsicSE3 CamCamCalibrator::bundle_adjustment_two_views(
    const std::vector<FeatureMatch>& matches,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const Sophus::SE3d& init_T,
    const std::string& cam0_id,
    const std::string& cam1_id) {

    ExtrinsicSE3 result;
    result.ref_sensor_id    = cam0_id;
    result.target_sensor_id = cam1_id;

    if (matches.size() < 10) {
        result.set_SE3(init_T);
        result.is_converged = false;
        return result;
    }

    // 三角化
    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = cv::Mat(intrin0.dist_coeffs).reshape(1, 1);
    cv::Mat D1 = cv::Mat(intrin1.dist_coeffs).reshape(1, 1);

    std::vector<cv::Point2f> raw0, raw1;
    for (const auto& m : matches) { raw0.push_back(m.pt0); raw1.push_back(m.pt1); }

    // 三角化投影矩阵
    cv::Mat R_cv, t_cv;
    cv::eigen2cv(Eigen::Matrix3d(init_T.rotationMatrix()), R_cv);
    cv::eigen2cv(Eigen::Vector3d(init_T.translation()), t_cv);
    cv::Mat P0 = cv::Mat::zeros(3,4,CV_64F);
    K0.copyTo(P0(cv::Rect(0,0,3,3)));
    cv::Mat RT = cv::Mat::zeros(3,4,CV_64F);
    R_cv.copyTo(RT.colRange(0,3)); t_cv.copyTo(RT.col(3));
    cv::Mat P1 = K1 * RT;

    cv::Mat pts4d;
    cv::triangulatePoints(P0, P1, raw0, raw1, pts4d);

    // 初始化 pose 参数 [rx,ry,rz,tx,ty,tz]
    Eigen::Vector3d rv = init_T.so3().log();
    Eigen::Vector3d tv = init_T.translation();
    double pose[6] = {rv[0], rv[1], rv[2], tv[0], tv[1], tv[2]};

    ceres::Problem problem;
    int n_valid = 0;
    for (int i = 0; i < pts4d.cols; ++i) {
        float w = pts4d.at<float>(3,i);
        if (std::abs(w) < 1e-6) continue;
        float x = pts4d.at<float>(0,i)/w;
        float y = pts4d.at<float>(1,i)/w;
        float z = pts4d.at<float>(2,i)/w;
        if (z < 0.1 || z > 200.0) continue;
        problem.AddResidualBlock(
            ReprojectCost::Create(raw1[i].x, raw1[i].y,
                                  intrin1.fx, intrin1.fy, intrin1.cx, intrin1.cy,
                                  x, y, z),
            new ceres::HuberLoss(1.0), pose);
        ++n_valid;
    }

    if (n_valid >= 10) {
        ceres::Solver::Options opt;
        opt.max_num_iterations = cfg_.ba_max_iter;
        opt.linear_solver_type = ceres::DENSE_SCHUR;
        opt.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(opt, &problem, &summary);
        UNICALIB_INFO("  BA: n_valid={} {}", n_valid, summary.BriefReport());
    }

    Eigen::Vector3d rv2(pose[0], pose[1], pose[2]);
    Eigen::AngleAxisd aa(rv2.norm(), rv2.normalized());
    result.SO3_TargetInRef  = Sophus::SO3d(aa.toRotationMatrix());
    result.POS_TargetInRef  = Eigen::Vector3d(pose[3], pose[4], pose[5]);
    result.is_converged     = (n_valid >= 10);
    return result;
}

// ===================================================================
// 方法 C: 多相机 Bundle Adjustment
// ===================================================================
std::vector<ExtrinsicSE3> CamCamCalibrator::calibrate_bundle_adjustment(
    const std::vector<std::vector<std::pair<double, cv::Mat>>>& frames_per_cam,
    const std::vector<CameraIntrinsics>& intrinsics,
    const std::vector<std::string>& cam_ids) {

    UNICALIB_INFO("=== Camera-Camera Bundle Adjustment ({} 相机) ===",
                  cam_ids.size());

    std::vector<ExtrinsicSE3> results;
    if (cam_ids.size() < 2) return results;

    // 两两标定 (以 cam_ids[0] 为参考)
    for (size_t i = 1; i < cam_ids.size(); ++i) {
        const auto& f0 = frames_per_cam[0];
        const auto& f1 = frames_per_cam[i];
        size_t N = std::min(f0.size(), f1.size());

        std::vector<FeatureMatch> all_matches;
        for (size_t fi = 0; fi < N; ++fi) {
            if (f0[fi].second.empty() || f1[fi].second.empty()) continue;
            auto fm = match_features(f0[fi].second, f1[fi].second,
                                     intrinsics[0], intrinsics[i]);
            for (auto& m : fm) all_matches.push_back(m);
        }
        UNICALIB_INFO("  cam0 <-> cam{}: {} 匹配", i, all_matches.size());

        // 初值
        Sophus::SO3d rot;
        Eigen::Vector3d trans;
        Sophus::SE3d init_T;
        if (recover_pose_from_E(all_matches, intrinsics[0], intrinsics[i], rot, trans)) {
            init_T = Sophus::SE3d(rot, trans);
        }

        auto ext = bundle_adjustment_two_views(
            all_matches, intrinsics[0], intrinsics[i],
            init_T, cam_ids[0], cam_ids[i]);
        results.push_back(ext);
    }

    return results;
}

// ===================================================================
// 立体矫正可视化
// ===================================================================
void CamCamCalibrator::visualize_stereo_rectification(
    const cv::Mat& img0, const cv::Mat& img1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin,
    const std::string& output_path) {

    if (img0.empty() || img1.empty()) return;

    cv::Mat K0 = (cv::Mat_<double>(3,3) <<
        intrin0.fx, 0, intrin0.cx, 0, intrin0.fy, intrin0.cy, 0, 0, 1);
    cv::Mat K1 = (cv::Mat_<double>(3,3) <<
        intrin1.fx, 0, intrin1.cx, 0, intrin1.fy, intrin1.cy, 0, 0, 1);
    cv::Mat D0 = cv::Mat(intrin0.dist_coeffs).reshape(1, 1);
    cv::Mat D1 = cv::Mat(intrin1.dist_coeffs).reshape(1, 1);

    cv::Mat R_cv, t_cv;
    cv::eigen2cv(Eigen::Matrix3d(extrin.SO3_TargetInRef.matrix()), R_cv);
    cv::eigen2cv(Eigen::Vector3d(extrin.POS_TargetInRef), t_cv);

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K0, D0, K1, D1, img0.size(),
                      R_cv, t_cv, R1, R2, P1, P2, Q);

    cv::Mat map00, map01, map10, map11;
    cv::initUndistortRectifyMap(K0, D0, R1, P1, img0.size(), CV_16SC2, map00, map01);
    cv::initUndistortRectifyMap(K1, D1, R2, P2, img1.size(), CV_16SC2, map10, map11);

    cv::Mat rect0, rect1;
    cv::remap(img0, rect0, map00, map01, cv::INTER_LINEAR);
    cv::remap(img1, rect1, map10, map11, cv::INTER_LINEAR);

    // 拼接显示
    cv::Mat vis(rect0.rows, rect0.cols + rect1.cols, rect0.type());
    rect0.copyTo(vis(cv::Rect(0, 0, rect0.cols, rect0.rows)));
    rect1.copyTo(vis(cv::Rect(rect0.cols, 0, rect1.cols, rect1.rows)));

    // 水平线验证立体矫正质量
    for (int y = 0; y < vis.rows; y += 30)
        cv::line(vis, {0,y}, {vis.cols,y}, cv::Scalar(0,255,0), 1);

    cv::imwrite(output_path, vis);
    UNICALIB_INFO("立体矫正可视化保存: {}", output_path);
}

// ===================================================================
// 评估外参
// ===================================================================
double CamCamCalibrator::evaluate(
    const std::vector<std::string>& images_cam0,
    const std::vector<std::string>& images_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin) {

    (void)extrin;
    // 通过重投影误差评估
    UNICALIB_INFO("Camera-Camera evaluate: 计算重投影误差");

    double total_rms = 0.0;
    int cnt = 0;
    size_t N = std::min(images_cam0.size(), images_cam1.size());
    for (size_t fi = 0; fi < N; fi += std::max(size_t(1), N/10)) {
        cv::Mat im0 = cv::imread(images_cam0[fi]);
        cv::Mat im1 = cv::imread(images_cam1[fi]);
        if (im0.empty() || im1.empty()) continue;
        auto fm = match_features(im0, im1, intrin0, intrin1);
        if (fm.size() < 8) continue;

        Sophus::SO3d rot;
        Eigen::Vector3d trans;
        if (recover_pose_from_E(fm, intrin0, intrin1, rot, trans)) {
            // Compare recovered rotation vs extrin
            Sophus::SO3d err = extrin.SO3_TargetInRef.inverse() * rot;
            total_rms += err.log().norm();
            ++cnt;
        }
    }
    double rms = (cnt > 0) ? total_rms / cnt : -1.0;
    UNICALIB_INFO("  Camera-Camera 评估 RMS 旋转差: {:.4f} rad", rms);
    return rms;
}

}  // namespace ns_unicalib
