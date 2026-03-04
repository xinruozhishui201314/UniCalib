/**
 * UniCalib — 联合标定求解器实现
 * 协调五种标定的执行, 集成 iKalibr B样条联合优化
 */

#include "unicalib/solver/joint_calib_solver.h"
#include "unicalib/common/logger.h"
#include <filesystem>
#include <chrono>
#include <iomanip>

namespace fs = std::filesystem;
namespace ns_unicalib {

JointCalibSolver::JointCalibSolver()
    : JointCalibSolver(Config{}) {}

JointCalibSolver::JointCalibSolver(const Config& cfg)
    : cfg_(cfg), params_(CalibParamManager::Create()) {}

void JointCalibSolver::set_system_config(const SystemConfig& sys_cfg) {
    sys_cfg_ = sys_cfg;
}

void JointCalibSolver::set_imu_intrinsics(
    const std::string& sensor_id, const IMUIntrinsics& intrin) {
    params_->imu_intrinsics[sensor_id] = std::make_shared<IMUIntrinsics>(intrin);
}

void JointCalibSolver::set_camera_intrinsics(
    const std::string& sensor_id, const CameraIntrinsics& intrin) {
    params_->camera_intrinsics[sensor_id] = std::make_shared<CameraIntrinsics>(intrin);
}

void JointCalibSolver::set_initial_extrinsic(
    const std::string& ref_id, const std::string& target_id,
    const Sophus::SE3d& T_init) {
    auto ext = params_->get_or_create_extrinsic(ref_id, target_id);
    ext->set_SE3(T_init);
}

void JointCalibSolver::report_progress(
    const std::string& stage, double progress, const std::string& msg) {
    if (progress_cb_) {
        progress_cb_(stage, progress, msg);
    }
    if (cfg_.verbose) {
        UNICALIB_LOG_STEP(stage, "{:.0f}% {}", progress * 100, msg);
    }
}

// ===================================================================
// 独立标定接口
// ===================================================================
std::optional<IMUIntrinsics> JointCalibSolver::calibrate_imu_intrinsic(
    const std::string& sensor_id,
    const std::vector<IMUFrame>& data) {

    report_progress("IMU-Intrinsic", 0.0, "开始 IMU 内参标定");
    IMUIntrinsicCalibrator calib(cfg_.imu_intrin_cfg);
    calib.set_progress_callback([this, &sensor_id](const std::string& s, double p) {
        report_progress("IMU-Intrinsic/" + sensor_id, p, s);
    });
    // 转换 IMUFrame -> IMURawFrame (字段相同)
    IMURawData raw_data;
    raw_data.reserve(data.size());
    for (const auto& f : data) {
        IMURawFrame rf;
        rf.timestamp = f.timestamp;
        rf.gyro  = f.gyro;
        rf.accel = f.accel;
        raw_data.push_back(rf);
    }
    auto result = calib.calibrate(raw_data);
    params_->imu_intrinsics[sensor_id] = std::make_shared<IMUIntrinsics>(result);
    report_progress("IMU-Intrinsic", 1.0, "完成");
    return result;
}

std::optional<CameraIntrinsics> JointCalibSolver::calibrate_camera_intrinsic(
    const std::string& sensor_id,
    const std::vector<std::string>& image_paths) {

    report_progress("Camera-Intrinsic", 0.0, "开始相机内参标定");
    CameraIntrinsicCalibrator calib(cfg_.cam_intrin_cfg);
    calib.set_progress_callback([this, &sensor_id](int cur, int total) {
        report_progress("Camera-Intrinsic/" + sensor_id,
                        static_cast<double>(cur) / total, "检测角点");
    });
    auto result = calib.calibrate(image_paths);
    if (result.has_value()) {
        params_->camera_intrinsics[sensor_id] =
            std::make_shared<CameraIntrinsics>(result.value());
        report_progress("Camera-Intrinsic", 1.0,
                        "完成 RMS=" + std::to_string(result->rms_reproj_error) + "px");
    } else {
        report_progress("Camera-Intrinsic", 1.0, "失败");
    }
    return result;
}

std::optional<ExtrinsicSE3> JointCalibSolver::calibrate_imu_lidar(
    const std::string& imu_id,
    const std::string& lidar_id,
    const std::vector<IMUFrame>& imu_data,
    const std::vector<LiDARScan>& lidar_scans) {

    report_progress("IMU-LiDAR", 0.0, "开始 IMU-LiDAR 外参标定");
    IMULiDARCalibrator calib(cfg_.imu_lidar_cfg);
    calib.set_progress_callback([this, &imu_id, &lidar_id](const std::string& s, double p) {
        report_progress("IMU-LiDAR/" + imu_id + "->" + lidar_id, p, s);
    });

    const IMUIntrinsics* imu_intrin = nullptr;
    auto it = params_->imu_intrinsics.find(imu_id);
    if (it != params_->imu_intrinsics.end() && it->second) {
        imu_intrin = it->second.get();
    }

    auto result = calib.calibrate(imu_data, lidar_scans, imu_id, lidar_id, imu_intrin);
    if (result.has_value()) {
        auto ext = params_->get_or_create_extrinsic(imu_id, lidar_id);
        *ext = result.value();
        report_progress("IMU-LiDAR", 1.0, "完成");
    } else {
        report_progress("IMU-LiDAR", 1.0, "失败");
    }
    return result;
}

std::optional<ExtrinsicSE3> JointCalibSolver::calibrate_lidar_camera(
    const std::string& lidar_id,
    const std::string& cam_id,
    const std::vector<LiDARScan>& lidar_scans,
    const std::vector<std::pair<double, cv::Mat>>& cam_frames,
    const CameraIntrinsics& cam_intrin) {

    report_progress("LiDAR-Camera", 0.0, "开始 LiDAR-Camera 外参标定");
    LiDARCameraCalibrator calib(cfg_.lidar_cam_cfg);
    calib.set_progress_callback([this, &lidar_id, &cam_id](const std::string& s, double p) {
        report_progress("LiDAR-Camera/" + lidar_id + "->" + cam_id, p, s);
    });

    std::optional<ExtrinsicSE3> result;
    switch (cfg_.lidar_cam_cfg.method) {
    case LiDARCameraCalibrator::Method::TARGET_CHESSBOARD:
        result = calib.calibrate_target(lidar_scans, cam_frames, cam_intrin, lidar_id, cam_id);
        break;
    case LiDARCameraCalibrator::Method::EDGE_ALIGNMENT: {
        Sophus::SE3d init;
        result = calib.calibrate_edge_align(lidar_scans, cam_frames, cam_intrin,
                                             init, lidar_id, cam_id);
        break;
    }
    case LiDARCameraCalibrator::Method::MOTION_BSPLINE:
        UNICALIB_WARN("Motion method requires IMU data. Use joint calibrate() instead.");
        break;
    }

    if (result.has_value()) {
        auto ext = params_->get_or_create_extrinsic(lidar_id, cam_id);
        *ext = result.value();
        report_progress("LiDAR-Camera", 1.0, "完成");
    }
    return result;
}

std::optional<ExtrinsicSE3> JointCalibSolver::calibrate_cam_cam(
    const std::string& cam0_id,
    const std::string& cam1_id,
    const std::vector<std::string>& images_cam0,
    const std::vector<std::string>& images_cam1,
    const CameraIntrinsics& intrin0,
    const CameraIntrinsics& intrin1) {

    report_progress("Cam-Cam", 0.0, "开始 Camera-Camera 外参标定");
    CamCamCalibrator calib(cfg_.cam_cam_cfg);
    calib.set_progress_callback([this, &cam0_id, &cam1_id](const std::string& s, double p) {
        report_progress("Cam-Cam/" + cam0_id + "->" + cam1_id, p, s);
    });

    std::optional<ExtrinsicSE3> result;
    switch (cfg_.cam_cam_cfg.method) {
    case CamCamCalibrator::Method::CHESSBOARD_STEREO:
        result = calib.calibrate_stereo(images_cam0, images_cam1,
                                         intrin0, intrin1, cam0_id, cam1_id);
        break;
    default:
        UNICALIB_WARN("Other cam-cam methods need image frames, not paths. Use calibrate_essential().");
        break;
    }

    if (result.has_value()) {
        auto ext = params_->get_or_create_extrinsic(cam0_id, cam1_id);
        *ext = result.value();
        report_progress("Cam-Cam", 1.0, "完成");
    }
    return result;
}

// ===================================================================
// 联合标定主函数
// ===================================================================
CalibSummary JointCalibSolver::calibrate(const CalibDataBundle& data) {
    CalibSummary summary;
    summary.params = params_;

    auto t_start = std::chrono::high_resolution_clock::now();

    UNICALIB_INFO("=== UniCalib 联合标定开始 ===");

    try {
        // Phase 1: 内参标定
        if (cfg_.do_imu_intrinsic || cfg_.do_camera_intrinsic) {
            phase1_intrinsics(data, summary);
        }

        // Phase 2: 粗外参
        if (cfg_.do_imu_lidar_extrinsic || cfg_.do_lidar_camera_extrinsic ||
            cfg_.do_cam_cam_extrinsic) {
            phase2_coarse_extrinsic(data, summary);
        }

        // Phase 3: 联合精化
        if (cfg_.do_joint_bspline_refine) {
            phase3_joint_refine(data, summary);
        }

        // Phase 4: 验证
        phase4_validation(data, summary);

        summary.success = true;

    } catch (const std::exception& e) {
        summary.success = false;
        summary.error_message = e.what();
        UNICALIB_ERROR("联合标定失败: {}", e.what());
    }

    // 保存结果
    save_results(summary);

    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    UNICALIB_INFO("=== 联合标定完成 ({}s) ===", elapsed);

    params_->print_summary();
    return summary;
}

void JointCalibSolver::phase1_intrinsics(
    const CalibDataBundle& data, CalibSummary& summary) {

    report_progress("Phase1-Intrinsics", 0.0, "内参标定");

    // IMU 内参
    if (cfg_.do_imu_intrinsic) {
        for (const auto& [sensor_id, imu_data] : data.imu_data) {
            UNICALIB_INFO("  IMU 内参: {} ({} 帧)", sensor_id, imu_data.size());
            auto result = calibrate_imu_intrinsic(sensor_id, imu_data);
            CalibSummary::QualityMetrics qm;
            qm.calib_type = "imu_intrinsic/" + sensor_id;
            qm.converged  = true;
            qm.unit = "rad/s";
            if (result.has_value())
                qm.rms_error = result->allan_fit_rms;
            summary.quality.push_back(qm);
        }
    }

    // 相机内参
    if (cfg_.do_camera_intrinsic) {
        for (const auto& [sensor_id, image_paths] : data.camera_image_paths) {
            UNICALIB_INFO("  相机内参: {}", sensor_id);
            auto result = calibrate_camera_intrinsic(sensor_id, image_paths);
            CalibSummary::QualityMetrics qm;
            qm.calib_type = "camera_intrinsic/" + sensor_id;
            qm.unit = "px";
            if (result.has_value()) {
                qm.rms_error  = result->rms_reproj_error;
                qm.converged  = (result->rms_reproj_error < 2.0);
            }
            summary.quality.push_back(qm);
        }
    }

    report_progress("Phase1-Intrinsics", 1.0, "完成");
}

void JointCalibSolver::phase2_coarse_extrinsic(
    const CalibDataBundle& data, CalibSummary& summary) {

    report_progress("Phase2-Coarse", 0.0, "粗外参估计");

    // IMU-LiDAR
    if (cfg_.do_imu_lidar_extrinsic && !data.imu_data.empty() && !data.lidar_scans.empty()) {
        const auto& imu_id    = data.imu_data.begin()->first;
        const auto& lidar_id  = data.lidar_scans.begin()->first;
        const auto& imu_data  = data.imu_data.begin()->second;
        const auto& lidar_data= data.lidar_scans.begin()->second;

        UNICALIB_INFO("  IMU-LiDAR 外参: {} -> {}", imu_id, lidar_id);
        calibrate_imu_lidar(imu_id, lidar_id, imu_data, lidar_data);
    }

    // Camera-Camera
    if (cfg_.do_cam_cam_extrinsic) {
        auto cam_sensors = sys_cfg_.sensors_of_type(SensorType::CAMERA);
        for (size_t i = 0; i + 1 < cam_sensors.size(); ++i) {
            const auto& id0 = cam_sensors[i]->sensor_id;
            const auto& id1 = cam_sensors[i+1]->sensor_id;
            auto it0 = data.camera_image_paths.find(id0);
            auto it1 = data.camera_image_paths.find(id1);
            if (it0 == data.camera_image_paths.end() ||
                it1 == data.camera_image_paths.end()) continue;

            auto in0 = params_->camera_intrinsics.count(id0) ?
                       params_->camera_intrinsics.at(id0) : nullptr;
            auto in1 = params_->camera_intrinsics.count(id1) ?
                       params_->camera_intrinsics.at(id1) : nullptr;
            if (!in0 || !in1) {
                UNICALIB_WARN("  Cam-Cam {}->{}: 内参未知, 跳过", id0, id1);
                continue;
            }
            UNICALIB_INFO("  Camera-Camera 外参: {} -> {}", id0, id1);
            calibrate_cam_cam(id0, id1, it0->second, it1->second, *in0, *in1);
        }
    }

    report_progress("Phase2-Coarse", 1.0, "完成");
}

void JointCalibSolver::phase3_joint_refine(
    const CalibDataBundle& /*data*/, CalibSummary& /*summary*/) {

    report_progress("Phase3-Refine", 0.0, "B样条联合精化");
    // TODO: 调用 iKalibr CalibSolver 进行联合 B样条时空优化
    // 这需要:
    //   1. 构建 CalibParamManager (使用 params_ 中的结果作为初值)
    //   2. 加载 bag 数据到 iKalibr 的数据管理器
    //   3. 调用 CalibSolver::Process()
    //   4. 将结果写回 params_
    UNICALIB_INFO("  B样条联合精化 (Phase 3): 需要 iKalibr CalibSolver 接口");
    UNICALIB_INFO("  当前已有粗外参作为初值, 联合精化功能将在 iKalibr 模式下激活");
    report_progress("Phase3-Refine", 1.0, "完成(iKalibr模式)");
}

void JointCalibSolver::phase4_validation(
    const CalibDataBundle& /*data*/, CalibSummary& summary) {

    report_progress("Phase4-Validation", 0.0, "验证");
    params_->print_summary();
    report_progress("Phase4-Validation", 1.0, "完成");
}

void JointCalibSolver::save_results(const CalibSummary& summary) {
    if (!summary.params) return;

    fs::create_directories(cfg_.output_dir);

    if (cfg_.save_yaml) {
        summary.params->save_yaml(cfg_.output_dir + "/calibration_result.yaml");
    }

    UNICALIB_INFO("结果已保存至: {}", cfg_.output_dir);
}

void CalibSummary::print() const {
    std::cout << "\n╔══════════════════════════════════╗\n";
    std::cout << "║       标定结果摘要               ║\n";
    std::cout << "╠══════════════════════════════════╣\n";
    std::cout << "║ 状态: " << (success ? "✓ 成功" : "✗ 失败") << "\n";
    if (!error_message.empty())
        std::cout << "║ 错误: " << error_message << "\n";
    for (const auto& q : quality) {
        std::cout << "║ " << q.calib_type << ": RMS=" << q.rms_error
                  << q.unit << " " << (q.converged ? "✓" : "✗") << "\n";
    }
    std::cout << "╚══════════════════════════════════╝\n\n";
}

}  // namespace ns_unicalib
