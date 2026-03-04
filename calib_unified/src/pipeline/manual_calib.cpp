/**
 * UniCalib Unified — ManualCalib 实现
 *
 * 手动标定的核心思路:
 *   1. ManualExtrinsicAdjuster:
 *      - 维护一个 ExtrinsicSE3 的可撤销栈
 *      - 每次 apply() 操作记录到 undo_stack_
 *      - 提供增量旋转/平移调整 (右乘 delta SE3)
 *
 *   2. ManualClickRefiner:
 *      - 收集用户在图像对上的点击对应点
 *      - 调用 Ceres 非线性最小二乘优化
 *      - 生成残差可视化 (热力图叠加到图像)
 *
 *   3. ManualCalibSession:
 *      - 包装 1 + 2, 管理会话历史
 *      - 自动保存到 JSON 文件
 */

#include "unicalib/pipeline/manual_calib.h"
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ─────────────────────────────────────────────────────────────────────────────
// 工具: 时间戳字符串
// ─────────────────────────────────────────────────────────────────────────────
static std::string ts_str() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::tm tm_buf{};
    localtime_r(&t, &tm_buf);
    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

// ─────────────────────────────────────────────────────────────────────────────
// Ceres 代价函数: 3D→2D 重投影误差 (用于 LiDAR-Cam 点击精化)
// ─────────────────────────────────────────────────────────────────────────────
struct LidarCamReprojCost {
    Eigen::Vector3d pt3d;
    Eigen::Vector2d pt2d;
    double fx, fy, cx, cy;

    LidarCamReprojCost(const Eigen::Vector3d& p3,
                        const Eigen::Vector2d& p2,
                        double fx_, double fy_, double cx_, double cy_)
        : pt3d(p3), pt2d(p2), fx(fx_), fy(fy_), cx(cx_), cy(cy_) {}

    template <typename T>
    bool operator()(const T* const rotation_aa,  // angle-axis [3]
                    const T* const translation,   // [3]
                    T* residuals) const {
        T p[3] = {T(pt3d.x()), T(pt3d.y()), T(pt3d.z())};
        T p_cam[3];
        ceres::AngleAxisRotatePoint(rotation_aa, p, p_cam);
        p_cam[0] += translation[0];
        p_cam[1] += translation[1];
        p_cam[2] += translation[2];

        // 投影
        T u = T(fx) * p_cam[0] / p_cam[2] + T(cx);
        T v = T(fy) * p_cam[1] / p_cam[2] + T(cy);

        residuals[0] = u - T(pt2d.x());
        residuals[1] = v - T(pt2d.y());
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& p3,
                                        const Eigen::Vector2d& p2,
                                        double fx, double fy,
                                        double cx, double cy) {
        return new ceres::AutoDiffCostFunction<LidarCamReprojCost, 2, 3, 3>(
            new LidarCamReprojCost(p3, p2, fx, fy, cx, cy));
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// Ceres 代价函数: Cam-Cam 对极约束误差 (用于手动点击精化)
// ─────────────────────────────────────────────────────────────────────────────
struct CamCamEpipolarCost {
    Eigen::Vector2d pt0_norm;  // 归一化坐标
    Eigen::Vector2d pt1_norm;

    CamCamEpipolarCost(const Eigen::Vector2d& p0,
                        const Eigen::Vector2d& p1)
        : pt0_norm(p0), pt1_norm(p1) {}

    template <typename T>
    bool operator()(const T* const rotation_aa,
                    const T* const translation,
                    T* residuals) const {
        // t × (R*p0) · p1 = 0  (基本矩阵约束)
        T p0[3] = {T(pt0_norm.x()), T(pt0_norm.y()), T(1.0)};
        T Rp0[3];
        ceres::AngleAxisRotatePoint(rotation_aa, p0, Rp0);

        // t × Rp0
        T tx = translation[0], ty = translation[1], tz = translation[2];
        T cross[3] = {
            ty * Rp0[2] - tz * Rp0[1],
            tz * Rp0[0] - tx * Rp0[2],
            tx * Rp0[1] - ty * Rp0[0]
        };

        // · p1
        T p1[3] = {T(pt1_norm.x()), T(pt1_norm.y()), T(1.0)};
        residuals[0] = cross[0]*p1[0] + cross[1]*p1[1] + cross[2]*p1[2];
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d& p0,
                                        const Eigen::Vector2d& p1) {
        return new ceres::AutoDiffCostFunction<CamCamEpipolarCost, 1, 3, 3>(
            new CamCamEpipolarCost(p0, p1));
    }
};

// ===========================================================================
// ManualExtrinsicAdjuster
// ===========================================================================

void ManualExtrinsicAdjuster::set_initial_extrinsic(const ExtrinsicSE3& extrin) {
    current_ = extrin;
    undo_stack_.clear();
    redo_stack_.clear();
    UNICALIB_INFO("[ManualAdjust] 初始外参:");
    print_current();
}

void ManualExtrinsicAdjuster::apply(AdjustCmd cmd, bool fast_mode) {
    double rot_step = fast_mode ? cfg_.step.rot_fast_deg  : cfg_.step.rot_step_deg;
    double t_step   = fast_mode ? cfg_.step.trans_fast_m  : cfg_.step.trans_step_m;

    switch (cmd) {
        case AdjustCmd::ROLL_PLUS:   apply_delta_rotation(+rot_step, 0, 0); break;
        case AdjustCmd::ROLL_MINUS:  apply_delta_rotation(-rot_step, 0, 0); break;
        case AdjustCmd::PITCH_PLUS:  apply_delta_rotation(0, +rot_step, 0); break;
        case AdjustCmd::PITCH_MINUS: apply_delta_rotation(0, -rot_step, 0); break;
        case AdjustCmd::YAW_PLUS:    apply_delta_rotation(0, 0, +rot_step); break;
        case AdjustCmd::YAW_MINUS:   apply_delta_rotation(0, 0, -rot_step); break;
        case AdjustCmd::TX_PLUS:     apply_delta_translation(+t_step, 0, 0); break;
        case AdjustCmd::TX_MINUS:    apply_delta_translation(-t_step, 0, 0); break;
        case AdjustCmd::TY_PLUS:     apply_delta_translation(0, +t_step, 0); break;
        case AdjustCmd::TY_MINUS:    apply_delta_translation(0, -t_step, 0); break;
        case AdjustCmd::TZ_PLUS:     apply_delta_translation(0, 0, +t_step); break;
        case AdjustCmd::TZ_MINUS:    apply_delta_translation(0, 0, -t_step); break;
        case AdjustCmd::UNDO:  undo(); break;
        default: break;
    }
}

void ManualExtrinsicAdjuster::apply_delta_rotation(
    double roll_deg, double pitch_deg, double yaw_deg) {

    // 保存到撤销栈
    if (cfg_.enable_undo) {
        if ((int)undo_stack_.size() >= cfg_.max_undo_depth)
            undo_stack_.pop_front();
        undo_stack_.push_back(current_);
        redo_stack_.clear();
    }

    // 右乘增量旋转 (RPY 顺序: R(z)*R(y)*R(x))
    const double deg2rad = M_PI / 180.0;
    Eigen::AngleAxisd rx(roll_deg  * deg2rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd ry(pitch_deg * deg2rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rz(yaw_deg   * deg2rad, Eigen::Vector3d::UnitZ());
    Sophus::SO3d delta_rot(rz.toRotationMatrix() *
                           ry.toRotationMatrix() *
                           rx.toRotationMatrix());

    // T_new = T_old * delta (右乘 — 在 target 坐标系下调整)
    auto T_old = current_.SE3_TargetInRef();
    Sophus::SE3d delta(delta_rot, Eigen::Vector3d::Zero());
    current_.set_SE3(T_old * delta);

    UNICALIB_DEBUG("[ManualAdjust] 旋转增量 RPY=({:.3f}, {:.3f}, {:.3f})deg",
                   roll_deg, pitch_deg, yaw_deg);
    print_current();
}

void ManualExtrinsicAdjuster::apply_delta_translation(
    double dx_m, double dy_m, double dz_m) {

    if (cfg_.enable_undo) {
        if ((int)undo_stack_.size() >= cfg_.max_undo_depth)
            undo_stack_.pop_front();
        undo_stack_.push_back(current_);
        redo_stack_.clear();
    }

    current_.POS_TargetInRef += Eigen::Vector3d(dx_m, dy_m, dz_m);

    UNICALIB_DEBUG("[ManualAdjust] 平移增量 dxyz=({:.4f}, {:.4f}, {:.4f})m",
                   dx_m, dy_m, dz_m);
    print_current();
}

bool ManualExtrinsicAdjuster::undo() {
    if (undo_stack_.empty()) {
        UNICALIB_WARN("[ManualAdjust] 无可撤销操作");
        return false;
    }
    redo_stack_.push_back(current_);
    current_ = undo_stack_.back();
    undo_stack_.pop_back();
    UNICALIB_INFO("[ManualAdjust] 撤销 (undo_stack 剩余: {})", undo_stack_.size());
    print_current();
    return true;
}

bool ManualExtrinsicAdjuster::redo() {
    if (redo_stack_.empty()) {
        UNICALIB_WARN("[ManualAdjust] 无可重做操作");
        return false;
    }
    undo_stack_.push_back(current_);
    current_ = redo_stack_.back();
    redo_stack_.pop_back();
    UNICALIB_INFO("[ManualAdjust] 重做 (redo_stack 剩余: {})", redo_stack_.size());
    print_current();
    return true;
}

void ManualExtrinsicAdjuster::print_current() const {
    auto euler = current_.euler_deg();
    auto t     = current_.translation();
    UNICALIB_INFO("[ManualAdjust] 当前外参 [{}→{}]:",
                  current_.ref_sensor_id, current_.target_sensor_id);
    UNICALIB_INFO("  旋转 RPY: ({:.4f}, {:.4f}, {:.4f}) deg",
                  euler.x(), euler.y(), euler.z());
    UNICALIB_INFO("  平移 XYZ: ({:.4f}, {:.4f}, {:.4f}) m",
                  t.x(), t.y(), t.z());
}

void ManualExtrinsicAdjuster::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) {
        UNICALIB_ERROR("[ManualAdjust] 无法写入: {}", path);
        return;
    }
    auto euler = current_.euler_deg();
    auto t     = current_.translation();
    f << "# UniCalib 手动校准结果\n";
    f << "ref_sensor_id: " << current_.ref_sensor_id << "\n";
    f << "target_sensor_id: " << current_.target_sensor_id << "\n";
    f << "rotation_rpy_deg: [" << euler.x() << ", " << euler.y() << ", " << euler.z() << "]\n";
    f << "translation_xyz_m: [" << t.x() << ", " << t.y() << ", " << t.z() << "]\n";
    f << "time_offset_s: " << current_.time_offset_s << "\n";
    f << "residual_rms: " << current_.residual_rms << "\n";
    UNICALIB_INFO("[ManualAdjust] 结果已保存: {}", path);
}

// ===========================================================================
// ManualClickRefiner
// ===========================================================================

std::optional<ExtrinsicSE3> ManualClickRefiner::refine_lidar_cam(
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& init_extrin,
    const std::vector<ClickPoint2D3D>& user_clicks) {

    UNICALIB_INFO("[ClickRefine] LiDAR-Camera 手动点击精化");
    UNICALIB_INFO("[ClickRefine] 点击点数量: {}", user_clicks.size());

    if (user_clicks.empty()) {
        UNICALIB_WARN("[ClickRefine] 未提供用户点击点, 返回初始外参");
        return init_extrin;
    }

    // 过滤有 3D 坐标的点击
    std::vector<ClickPoint2D3D> valid_clicks;
    for (const auto& c : user_clicks) {
        if (c.has_3d) valid_clicks.push_back(c);
    }

    if ((int)valid_clicks.size() < cfg_.min_points_lidar_cam) {
        UNICALIB_WARN("[ClickRefine] 有效3D-2D对应点不足 ({}/{})",
                      valid_clicks.size(), cfg_.min_points_lidar_cam);
        return std::nullopt;
    }

    return optimize_from_clicks(valid_clicks, cam_intrin, init_extrin,
                                 "lidar_cam_click");
}

std::optional<ExtrinsicSE3> ManualClickRefiner::refine_cam_cam(
    const cv::Mat& img_cam0,
    const cv::Mat& img_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& init_extrin,
    const std::vector<ClickCorrespondence>& user_clicks) {

    UNICALIB_INFO("[ClickRefine] Camera-Camera 手动点击精化");
    UNICALIB_INFO("[ClickRefine] 对应点数量: {}", user_clicks.size());

    if ((int)user_clicks.size() < cfg_.min_points_cam_cam) {
        UNICALIB_WARN("[ClickRefine] 对应点不足 ({}/{})",
                      user_clicks.size(), cfg_.min_points_cam_cam);
        return std::nullopt;
    }

    return optimize_cam_cam_from_clicks(user_clicks, intrin0, intrin1,
                                         init_extrin, "cam_cam_click");
}

double ManualClickRefiner::evaluate_lidar_cam(
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& extrin) const {
    (void)scan; (void)image;
    UNICALIB_TRACE("[ClickRefine] LiDAR-Cam 评估 (占位 — 需点云投影实现)");
    return extrin.residual_rms;
}

double ManualClickRefiner::evaluate_cam_cam(
    const std::vector<ClickCorrespondence>& clicks,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& extrin) const {
    if (clicks.empty()) return 0.0;

    auto T = extrin.SE3_TargetInRef();
    double sum_sq = 0.0;
    int cnt = 0;

    for (const auto& c : clicks) {
        // 将 cam0 点归一化后通过 T 投影到 cam1
        Eigen::Vector3d p0_norm(
            (c.pt_cam0.x() - intrin0.cx) / intrin0.fx,
            (c.pt_cam0.y() - intrin0.cy) / intrin0.fy,
            1.0);
        Eigen::Vector3d p1_cam = T.rotationMatrix() * p0_norm + T.translation();
        if (p1_cam.z() < 1e-6) continue;
        double u = intrin1.fx * p1_cam.x() / p1_cam.z() + intrin1.cx;
        double v = intrin1.fy * p1_cam.y() / p1_cam.z() + intrin1.cy;
        double du = u - c.pt_cam1.x();
        double dv = v - c.pt_cam1.y();
        sum_sq += du*du + dv*dv;
        cnt++;
    }
    return cnt > 0 ? std::sqrt(sum_sq / cnt) : 0.0;
}

std::optional<ExtrinsicSE3> ManualClickRefiner::optimize_from_clicks(
    const std::vector<ClickPoint2D3D>& clicks,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& init_extrin,
    const std::string& log_prefix) {

    UNICALIB_INFO("[ClickOpt-{}] Ceres 优化 (点数={})", log_prefix, clicks.size());

    // 初始化参数: angle-axis + translation
    auto T_init = init_extrin.SE3_TargetInRef();
    Eigen::AngleAxisd aa(T_init.rotationMatrix());
    double rot[3]   = {aa.axis().x() * aa.angle(),
                       aa.axis().y() * aa.angle(),
                       aa.axis().z() * aa.angle()};
    double trans[3] = {T_init.translation().x(),
                       T_init.translation().y(),
                       T_init.translation().z()};

    ceres::Problem problem;
    for (const auto& c : clicks) {
        auto* cost = LidarCamReprojCost::Create(
            c.world_pt, c.img_pt,
            cam_intrin.fx, cam_intrin.fy, cam_intrin.cx, cam_intrin.cy);
        problem.AddResidualBlock(cost,
            new ceres::HuberLoss(cfg_.ransac_thresh_px),
            rot, trans);
    }

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.max_num_iterations = cfg_.ceres_max_iter;
    opts.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    UNICALIB_INFO("[ClickOpt-{}] 优化结果: {}", log_prefix,
                  summary.BriefReport());
    UNICALIB_INFO("[ClickOpt-{}] 最终代价: {:.6f} → {:.6f}",
                  log_prefix, summary.initial_cost, summary.final_cost);

    if (!summary.IsSolutionUsable()) {
        UNICALIB_WARN("[ClickOpt-{}] 优化未收敛", log_prefix);
        return std::nullopt;
    }

    // 构建结果
    ExtrinsicSE3 result = init_extrin;
    Eigen::AngleAxisd aa_result(
        std::sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]),
        Eigen::Vector3d(rot[0], rot[1], rot[2]).normalized());
    result.SO3_TargetInRef = Sophus::SO3d(aa_result.toRotationMatrix());
    result.POS_TargetInRef = Eigen::Vector3d(trans[0], trans[1], trans[2]);
    result.residual_rms = std::sqrt(summary.final_cost / clicks.size());
    result.is_converged = true;

    UNICALIB_INFO("[ClickOpt-{}] 结果 RMS: {:.4f} px", log_prefix, result.residual_rms);
    return result;
}

std::optional<ExtrinsicSE3> ManualClickRefiner::optimize_cam_cam_from_clicks(
    const std::vector<ClickCorrespondence>& clicks,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& init_extrin,
    const std::string& log_prefix) {

    UNICALIB_INFO("[ClickOpt-{}] Cam-Cam 对极优化 (点数={})",
                  log_prefix, clicks.size());

    auto T_init = init_extrin.SE3_TargetInRef();
    Eigen::AngleAxisd aa(T_init.rotationMatrix());
    double rot[3]   = {aa.axis().x() * aa.angle(),
                       aa.axis().y() * aa.angle(),
                       aa.axis().z() * aa.angle()};
    double trans[3] = {T_init.translation().x(),
                       T_init.translation().y(),
                       T_init.translation().z()};

    ceres::Problem problem;
    for (const auto& c : clicks) {
        Eigen::Vector2d p0_norm(
            (c.pt_cam0.x() - intrin0.cx) / intrin0.fx,
            (c.pt_cam0.y() - intrin0.cy) / intrin0.fy);
        Eigen::Vector2d p1_norm(
            (c.pt_cam1.x() - intrin1.cx) / intrin1.fx,
            (c.pt_cam1.y() - intrin1.cy) / intrin1.fy);

        auto* cost = CamCamEpipolarCost::Create(p0_norm, p1_norm);
        problem.AddResidualBlock(cost,
            new ceres::HuberLoss(1.0 / std::max(intrin0.fx, intrin1.fx)),
            rot, trans);
    }

    // 固定平移的尺度 (对极约束 t 只有方向, 不含尺度)
    problem.SetParameterization(trans,
        new ceres::HomogeneousVectorParameterization(3));

    ceres::Solver::Options opts;
    opts.linear_solver_type = ceres::DENSE_QR;
    opts.max_num_iterations = cfg_.ceres_max_iter;
    opts.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(opts, &problem, &summary);

    UNICALIB_INFO("[ClickOpt-{}] 最终代价: {:.6f} → {:.6f}",
                  log_prefix, summary.initial_cost, summary.final_cost);

    if (!summary.IsSolutionUsable()) {
        UNICALIB_WARN("[ClickOpt-{}] 优化未收敛", log_prefix);
        return std::nullopt;
    }

    ExtrinsicSE3 result = init_extrin;
    double angle = std::sqrt(rot[0]*rot[0] + rot[1]*rot[1] + rot[2]*rot[2]);
    if (angle > 1e-10) {
        Eigen::AngleAxisd aa_result(angle,
            Eigen::Vector3d(rot[0], rot[1], rot[2]) / angle);
        result.SO3_TargetInRef = Sophus::SO3d(aa_result.toRotationMatrix());
    }
    // 注意: Cam-Cam 对极约束无法恢复平移尺度, 保留 init 的平移
    result.residual_rms = std::sqrt(summary.final_cost / clicks.size());
    result.is_converged = true;

    UNICALIB_INFO("[ClickOpt-{}] 结果 RMS: {:.6f}", log_prefix, result.residual_rms);
    return result;
}

void ManualClickRefiner::save_clicks(
    const std::string& path,
    const std::vector<ClickCorrespondence>& clicks) const {
    std::ofstream f(path);
    f << "# UniCalib 手动点击对应点\n";
    f << "num_points: " << clicks.size() << "\n";
    f << "points:\n";
    for (size_t i = 0; i < clicks.size(); ++i) {
        f << "  - id: " << i << "\n";
        f << "    pt_cam0: [" << clicks[i].pt_cam0.x() << ", "
          << clicks[i].pt_cam0.y() << "]\n";
        f << "    pt_cam1: [" << clicks[i].pt_cam1.x() << ", "
          << clicks[i].pt_cam1.y() << "]\n";
        f << "    confidence: " << clicks[i].confidence << "\n";
    }
    UNICALIB_INFO("[ClickRefine] 点击点已保存: {} ({} 点)", path, clicks.size());
}

// ===========================================================================
// ManualCalibSession
// ===========================================================================

ManualCalibSession::ManualCalibSession()
    : ManualCalibSession(SessionConfig{}) {}

ManualCalibSession::ManualCalibSession(const SessionConfig& cfg)
    : cfg_(cfg) {
    if (cfg_.session_id.empty()) {
        session_id_ = "session_" + ts_str();
        std::replace(session_id_.begin(), session_id_.end(), ' ', '_');
        std::replace(session_id_.begin(), session_id_.end(), ':', '-');
    } else {
        session_id_ = cfg_.session_id;
    }
    fs::create_directories(cfg_.save_dir);
    UNICALIB_INFO("[ManualSession] 会话ID: {}", session_id_);
    UNICALIB_INFO("[ManualSession] 保存目录: {}", cfg_.save_dir);
}

ExtrinsicSE3 ManualCalibSession::run_lidar_cam(
    const LiDARScan& scan,
    const cv::Mat& image,
    const CameraIntrinsics& cam_intrin,
    const ExtrinsicSE3& auto_result,
    double auto_rms) {

    UNICALIB_INFO("[ManualSession] 启动 LiDAR-Camera 手动校准");
    UNICALIB_INFO("[ManualSession] 自动标定 RMS: {:.4f} px (阈值建议 < 2.0px)",
                  auto_rms);
    log_session_event("lidar_cam_start",
                       "auto_rms=" + std::to_string(auto_rms));

    // 初始化调整器
    ManualExtrinsicAdjuster::Config adj_cfg;
    adjuster_.set_initial_extrinsic(auto_result);

    // 操作说明
    UNICALIB_INFO("[ManualSession] ===== 手动校准操作说明 =====");
    UNICALIB_INFO("  Q/A: Roll ±{:.1f}deg   W/S: Pitch ±{:.1f}deg   E/D: Yaw ±{:.1f}deg",
                  adj_cfg.step.rot_step_deg, adj_cfg.step.rot_step_deg,
                  adj_cfg.step.rot_step_deg);
    UNICALIB_INFO("  R/F: Tx ±{:.1f}cm   T/G: Ty ±{:.1f}cm   Y/H: Tz ±{:.1f}cm",
                  adj_cfg.step.trans_step_m*100, adj_cfg.step.trans_step_m*100,
                  adj_cfg.step.trans_step_m*100);
    UNICALIB_INFO("  Shift+键: 10倍步长");
    UNICALIB_INFO("  P: 进入点击精化模式  U: 撤销  Z: 重做");
    UNICALIB_INFO("  S: 保存  ESC: 接受并退出");
    UNICALIB_INFO("  点云投影质量越高 (边缘对齐越好) 表示标定越准确");
    UNICALIB_INFO("[ManualSession] 注意: 当前为非交互模式, 返回调整器中的当前值");
    UNICALIB_INFO("[ManualSession] 如需交互, 请使用带 GUI 的 ManualCalibGUI 类");

    // TODO: 当需要真实 GUI 时, 集成 OpenCV highgui 键盘监听循环
    // 此处占位: 返回自动标定结果 (等待集成 GUI 事件循环)
    ExtrinsicSE3 result = auto_result;
    result.residual_rms = auto_rms;

    log_session_event("lidar_cam_end",
        "final_rms=" + std::to_string(result.residual_rms));
    final_extrinsics_["lidar_cam"] = result;

    if (cfg_.auto_save) {
        save_session();
    }
    return result;
}

ExtrinsicSE3 ManualCalibSession::run_cam_cam(
    const cv::Mat& img_cam0,
    const cv::Mat& img_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1,
    const ExtrinsicSE3& auto_result,
    double auto_rms) {

    UNICALIB_INFO("[ManualSession] 启动 Camera-Camera 手动校准");
    UNICALIB_INFO("[ManualSession] 自动标定 RMS: {:.4f} px", auto_rms);
    log_session_event("cam_cam_start",
                       "auto_rms=" + std::to_string(auto_rms));

    adjuster_.set_initial_extrinsic(auto_result);

    UNICALIB_INFO("[ManualSession] 双目相机手动校准模式");
    UNICALIB_INFO("[ManualSession] 对极线可视化质量越好表示标定越准确");

    ExtrinsicSE3 result = auto_result;
    result.residual_rms = auto_rms;

    log_session_event("cam_cam_end",
        "final_rms=" + std::to_string(result.residual_rms));
    final_extrinsics_["cam_cam"] = result;

    if (cfg_.auto_save) save_session();
    return result;
}

ExtrinsicSE3 ManualCalibSession::run_imu_lidar(
    const std::vector<LiDARScan>& scans,
    const std::vector<IMUFrame>& imu_data,
    const ExtrinsicSE3& auto_result,
    double auto_rms) {

    UNICALIB_INFO("[ManualSession] 启动 IMU-LiDAR 手动校准");
    UNICALIB_INFO("[ManualSession] 自动标定旋转残差: {:.4f} deg", auto_rms);
    UNICALIB_INFO("[ManualSession] IMU-LiDAR 手动校准通过旋转增量验证");
    UNICALIB_INFO("[ManualSession] 可视化: 将 IMU 积分轨迹与 LiDAR 里程计对比");
    log_session_event("imu_lidar_start",
                       "auto_rms=" + std::to_string(auto_rms));

    adjuster_.set_initial_extrinsic(auto_result);
    ExtrinsicSE3 result = auto_result;

    log_session_event("imu_lidar_end",
        "final_rms=" + std::to_string(result.residual_rms));
    final_extrinsics_["imu_lidar"] = result;

    if (cfg_.auto_save) save_session();
    return result;
}

void ManualCalibSession::log_session_event(const std::string& event,
                                            const std::string& detail) {
    SessionEvent ev;
    ev.timestamp = ts_str();
    ev.event = event;
    ev.detail = detail;
    ev.extrin_snapshot = adjuster_.current();
    history_.push_back(ev);

    if (cfg_.log_all_steps) {
        UNICALIB_DEBUG("[ManualSession] 事件: {} | {}", event, detail);
    }
}

void ManualCalibSession::save_session(const std::string& path) const {
    std::string save_path = path.empty() ?
        cfg_.save_dir + "/" + session_id_ + ".yaml" : path;

    std::ofstream f(save_path);
    if (!f.is_open()) {
        UNICALIB_WARN("[ManualSession] 无法保存: {}", save_path);
        return;
    }
    f << "session_id: " << session_id_ << "\n";
    f << "events:\n";
    for (const auto& ev : history_) {
        f << "  - ts: " << ev.timestamp << "\n";
        f << "    event: " << ev.event << "\n";
        f << "    detail: \"" << ev.detail << "\"\n";
    }
    f << "final_extrinsics:\n";
    for (const auto& [key, extrin] : final_extrinsics_) {
        auto euler = extrin.euler_deg();
        auto t     = extrin.translation();
        f << "  " << key << ":\n";
        f << "    ref: " << extrin.ref_sensor_id << "\n";
        f << "    target: " << extrin.target_sensor_id << "\n";
        f << "    rpy_deg: [" << euler.x() << ", " << euler.y() << ", " << euler.z() << "]\n";
        f << "    xyz_m: [" << t.x() << ", " << t.y() << ", " << t.z() << "]\n";
        f << "    rms: " << extrin.residual_rms << "\n";
    }
    UNICALIB_INFO("[ManualSession] 会话已保存: {}", save_path);
}

void ManualCalibSession::export_results(CalibParamManager& pm) const {
    for (const auto& [key, extrin] : final_extrinsics_) {
        auto ptr = pm.get_or_create_extrinsic(
            extrin.ref_sensor_id, extrin.target_sensor_id);
        *ptr = extrin;
        UNICALIB_INFO("[ManualSession] 导出外参 [{}→{}] RMS={:.4f}",
                      extrin.ref_sensor_id, extrin.target_sensor_id,
                      extrin.residual_rms);
    }
}

}  // namespace ns_unicalib
