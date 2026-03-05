/**
 * UniCalib Unified — 统一标定可视化系统
 *
 * 完整实现:
 *   1. IMU-LiDAR 标定可视化 (LiDAR里程计、旋转对、B样条收敛)
 *   2. LiDAR-Camera 投影验证 (边缘对齐、互信息)
 *   3. 通用可视化 (收敛曲线、时间偏移图、误差分布)
 *   4. Pangolin 3D 可视化（交互式显示）
 */

#include "unicalib/viz/calib_visualizer.h"
#include "unicalib/common/logger.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <numeric>

#ifdef UNICALIB_WITH_PANGOLIN
#include <pangolin/pangolin.h>
#include <pangolin/plot/plotter.h>
#include <pangolin/gl/gldraw.h>
#endif  // UNICALIB_WITH_PANGOLIN

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===========================================================================
// 构造/析构
// ===========================================================================
CalibVisualizer::CalibVisualizer(const VizConfig& config)
    : config_(config), stopped_(false), id_counter_(0) {
    
    // 创建日志器
    logger_ = spdlog::get("unicalib");
    if (!logger_) {
        logger_ = spdlog::stdout_color_mt("unicalib");
    }

    // 创建报告生成器
    report_generator_ = std::make_shared<ReportGenerator>();

    // 创建截图目录
    if (!config_.screenshot_dir.empty()) {
        fs::create_directories(config_.screenshot_dir);
    }

    UNICALIB_INFO("[CalibVisualizer] 初始化完成");
}

CalibVisualizer::~CalibVisualizer() {
    stopped_ = true;
    if (cloud_viewer_) {
        cloud_viewer_.reset();
    }
}

// ===========================================================================
// 3D 可视化接口
// ===========================================================================
CloudViewer::Ptr CalibVisualizer::get_cloud_viewer() {
    if (!cloud_viewer_) {
        cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
    }
    return cloud_viewer_;
}

// ===========================================================================
// IMU-LiDAR 标定可视化
// ===========================================================================
void CalibVisualizer::show_imu_lidar_result(const IMULiDARVizData& data, bool block) {
    (void)block;
    if (!cloud_viewer_) {
        cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
    }

    // 清除之前的可视化
    cloud_viewer_->clear();

    // 1. 显示 LiDAR 里程计轨迹 (add_trajectory 需要 vector<SE3d>)
    if (!data.lidar_odom_poses.empty()) {
        std::vector<Sophus::SE3d> poses_se3;
        poses_se3.reserve(data.lidar_odom_poses.size());
        for (const auto& p : data.lidar_odom_poses)
            poses_se3.push_back(p.second);
        cloud_viewer_->add_trajectory(poses_se3, "lidar_odom", config_.lidar_trajectory_color, 3.0f);
        for (size_t i = 0; i < data.lidar_odom_poses.size(); i += 10) {
            cloud_viewer_->add_coordinate_system(data.lidar_odom_poses[i].second, "odom_" + std::to_string(i), 0.3f);
        }
    }

    // 2. 显示旋转对可视化
    if (!data.rotation_pairs.empty()) {
        show_rotation_pairs(data.rotation_pairs, true);
    }

    // 3. 显示 B样条收敛曲线
    if (!data.optimization_history.empty()) {
        show_bspline_optimization(data.optimization_history, config_.enable_animation);
    }

    // 4. 显示最终外参坐标系
    if (data.coarse_result.has_value() || data.fine_result.has_value()) {
        const auto& result = data.fine_result.has_value() ? data.fine_result : data.coarse_result;
        cloud_viewer_->add_coordinate_system(Sophus::SE3d(result->SO3_TargetInRef, result->POS_TargetInRef),
                                              "extrinsic_final", 0.5f);
    }

    // 5. 保存可视化
    save_plots(config_.screenshot_dir + "/imu_lidar_result");

    report_progress("IMU-LiDAR", "visualization_complete", 1.0, "Visualization complete");
}

void CalibVisualizer::show_rotation_pairs(const std::vector<IMULiDARVizData::RotationPair>& pairs, bool show_imu_traj) {
    if (!cloud_viewer_ || pairs.empty()) return;

    // 清除旋转对可视化
    cloud_viewer_->clear("rot_pair");

    // 创建 IMU 轨迹
    std::vector<Sophus::SE3d> imu_poses;
    std::vector<Sophus::SE3d> lidar_poses;
    for (const auto& pair : pairs) {
        imu_poses.emplace_back(pair.rot_imu, Eigen::Vector3d::Zero());
        lidar_poses.emplace_back(pair.rot_lidar, Eigen::Vector3d::Zero());
    }

    // 添加轨迹
    if (show_imu_traj && !imu_poses.empty()) {
        cloud_viewer_->add_trajectory(imu_poses, "imu_traj_rot", config_.imu_trajectory_color, 2.0f);
    }
    cloud_viewer_->add_trajectory(lidar_poses, "lidar_traj_rot", config_.lidar_trajectory_color, 2.0f);

    // 为每对旋转添加可视化
    for (size_t i = 0; i < pairs.size(); ++i) {
        const auto& pair = pairs[i];

        // IMU 旋转轴 (红色)
        Eigen::Vector3d axis_imu(1, 0, 0);
        cloud_viewer_->add_coordinate_system(Sophus::SE3d(pair.rot_imu, Sophus::SO3d::exp(axis_imu * 0.2)),
                                             "imu_axis_" + std::to_string(i), 0.2f);

        // LiDAR 旋转轴 (蓝色)
        Eigen::Vector3d axis_lidar(1, 0, 0);
        Sophus::SE3d T_imu_lidar(pair.rot_lidar * pair.rot_imu.inverse(), Eigen::Vector3d::Zero());
        cloud_viewer_->add_coordinate_system(Sophus::SE3d(pair.rot_lidar, Sophus::SO3d::exp(axis_lidar * 0.2)),
                                             "lidar_axis_" + std::to_string(i), 0.2f);
    }

    if (config_.enable_animation) {
        cloud_viewer_->spin_once(100);
    }
}

void CalibVisualizer::show_lidar_odometry(const std::vector<Sophus::SE3d>& poses) {
    if (!cloud_viewer_) {
        cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
    }

    cloud_viewer_->clear();
    cloud_viewer_->add_trajectory(poses, "lidar_odom", config_.lidar_trajectory_color, 3.0f);

    // 添加坐标轴
    for (size_t i = 0; i < poses.size(); i += 5) {
        cloud_viewer_->add_coordinate_system(poses[i], "odom_" + std::to_string(i), 0.5f);
    }

    save_plots(config_.screenshot_dir + "/lidar_odom");
}

void CalibVisualizer::show_bspline_optimization(const std::vector<IMULiDARVizData::OptimizationLog>& history, bool show_animation) {
#ifdef UNICALIB_WITH_PANGOLIN
    if (history.empty()) return;

    try {
        // ───────────────────────────────────────────────────────────
        // 创建 Pangolin 窗口
        // ───────────────────────────────────────────────────────────
        std::string window_name = "UniCalib - B-spline Optimization";
        pangolin::CreateWindowAndBind(window_name, 1400, 900);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 3D 视图配置
        pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1400, 600, 500, 500, 700, 300, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -3, 0, 0, 0, pangolin::AxisZ));
        
        pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.0, 0.65)
            .SetHandler(new pangolin::Handler3D(s_cam));

        // 图表窗口
        pangolin::View& d_plot = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, 0.65, 1.0);

        // ───────────────────────────────────────────────────────────
        // 渲染循环
        // ───────────────────────────────────────────────────────────
        while (!pangolin::ShouldQuit()) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            // 3D 场景
            d_cam.Activate(s_cam);
            glEnable(GL_DEPTH_TEST);
            glColorMask(true, true, true, true);

            // 绘制坐标系
            pangolin::glDrawCoordinateFrame(1.0);

            // 可视化轨迹点（如果有可用的位姿信息）
            glPointSize(2.0f);
            glBegin(GL_POINTS);
            glColor3f(0, 1, 0);  // 绿色
            for (const auto& log : history) {
                // 这里可扩展显示轨迹
            }
            glEnd();

            // 收敛曲线窗口
            d_plot.Activate();
            
            // 提取优化数据
            std::vector<double> iterations;
            std::vector<double> costs;
            
            for (size_t i = 0; i < history.size(); ++i) {
                iterations.push_back(static_cast<double>(i));
                costs.push_back(history[i].cost);
            }

            // 绘制收敛曲线
            if (!costs.empty()) {
                double max_cost = *std::max_element(costs.begin(), costs.end());
                double min_cost = *std::min_element(costs.begin(), costs.end());
                
                glClear(GL_COLOR_BUFFER_BIT);
                glMatrixMode(GL_PROJECTION);
                glPushMatrix();
                glLoadIdentity();
                glOrtho(0, iterations.size(), min_cost * 0.9, max_cost * 1.1, -1, 1);
                glMatrixMode(GL_MODELVIEW);
                glPushMatrix();
                glLoadIdentity();

                // 绘制网格
                glColor3f(0.3f, 0.3f, 0.3f);
                glBegin(GL_LINES);
                for (int i = 0; i <= 10; ++i) {
                    double y = min_cost + (max_cost - min_cost) * i / 10.0;
                    glVertex2f(0, y);
                    glVertex2f(iterations.size(), y);
                }
                glEnd();

                // 绘制曲线
                glColor3f(1, 0, 0);
                glLineWidth(2.0f);
                glBegin(GL_LINE_STRIP);
                for (size_t i = 0; i < costs.size(); ++i) {
                    glVertex2f(i, costs[i]);
                }
                glEnd();

                // 恢复投影
                glPopMatrix();
                glMatrixMode(GL_PROJECTION);
                glPopMatrix();
                glMatrixMode(GL_MODELVIEW);
            }

            pangolin::FinishFrame();

            if (!show_animation) break;
        }

        UNICALIB_INFO("[CalibVisualizer] B-spline optimization visualization completed");

    } catch (const std::exception& e) {
        UNICALIB_ERROR("[CalibVisualizer] Pangolin visualization error: {}", e.what());
    }

#else
    // 无 Pangolin 时，输出到文件
    UNICALIB_WARN("[CalibVisualizer] Pangolin not available, using OpenCV visualization");
    
    if (cloud_viewer_ || history.empty()) return;

    // 提取收敛数据
    std::vector<double> iterations;
    std::vector<double> costs;
    std::vector<double> time_offsets;

    for (const auto& log : history) {
        iterations.push_back(static_cast<double>(log.iterations.size()));
        costs.push_back(log.cost);
        for (const auto& ts : log.time_offsets) {
            time_offsets.push_back(ts);
        }
    }

    // 绘制收敛曲线（OpenCV 版本）
    cv::Mat conv_curve = draw_optimization_convergence_curve(iterations, costs, "B-spline Optimization",
                                                              config_.plot_width, config_.plot_height);
    save_plots(config_.screenshot_dir + "/bspline_convergence");

    // 绘制时间偏移曲线
    if (!time_offsets.empty()) {
        cv::Mat time_curve = plot_time_offset_convergence_curve(time_offsets, costs, "Time Offset Estimation");
        save_plots(config_.screenshot_dir + "/time_offset");
    }

    if (show_animation && cloud_viewer_) {
        cloud_viewer_->spin_once(100);
    }

#endif  // UNICALIB_WITH_PANGOLIN
}

// ===========================================================================
// LiDAR-Camera 投影可视化
// ===========================================================================
void CalibVisualizer::show_lidar_camera_projection(const std::vector<LiDARScan>& lidar_scans,
                                                    const std::vector<CameraFrame>& camera_frames,
                                                    const ExtrinsicSE3& extrinsic,
                                                    const CameraIntrinsics& intrinsics,
                                                    bool save_images) {
    if (lidar_scans.empty() || camera_frames.empty()) return;

    fs::create_directories(config_.screenshot_dir + "/lidar_cam_projection");

    Sophus::SE3d T_lidar_cam = extrinsic.SE3_TargetInRef().inverse();

    for (size_t i = 0; i < std::min(lidar_scans.size(), camera_frames.size()); ++i) {
        auto& scan = lidar_scans[i];
        auto& frame = camera_frames[i];

        if (!scan.cloud || scan.cloud->empty()) continue;

        // 投影点云到图像
        cv::Mat proj_img = LiDARProjectionViz::project_to_image(
            *scan.cloud, frame.image, T_lidar_cam,
            intrinsics.fx, intrinsics.fy, intrinsics.cx, intrinsics.cy,
            intrinsics.width, intrinsics.height, 0.5, 50.0);

        if (save_images) {
            std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count());
            std::string proj_path = config_.screenshot_dir + "/lidar_cam_projection/" +
                                     std::to_string(i) + "_" + timestamp + ".png";
            cv::imwrite(proj_path, proj_img);
        }
    }

    report_progress("LiDAR-Camera", "projection_complete", 1.0, "Projection visualization complete");
}

void CalibVisualizer::show_image_alignment(const std::vector<CameraFrame>& camera_frames,
                                           const ExtrinsicSE3& extrinsic,
                                           const CameraIntrinsics& intrinsics) {
    if (camera_frames.empty()) return;

    fs::create_directories(config_.screenshot_dir + "/image_alignment");

    Sophus::SE3d T_lidar_cam = extrinsic.SE3_TargetInRef().inverse();

    for (size_t i = 0; i < camera_frames.size(); ++i) {
        auto& frame = camera_frames[i];

        // 绘制角点重投影
        std::vector<Eigen::Vector2d> corners_2d;
        std::vector<Eigen::Vector2d> corners_reprojected;
        // TODO: 根据实际数据填充

        cv::Mat alignment_img = draw_camera_reprojection_errors(corners_2d, corners_reprojected,
                                                                  "Image Alignment");

        std::string img_path = config_.screenshot_dir + "/image_alignment/frame_" + std::to_string(i) + ".png";
        cv::imwrite(img_path, alignment_img);
    }
}

// ===========================================================================
// 点云对齐可视化
// ===========================================================================
void CalibVisualizer::show_cloud_alignment(const std::vector<LiDARScan>& scans_before,
                                            const std::vector<LiDARScan>& scans_after,
                                            const ExtrinsicSE3& extrinsic) {
    if (!cloud_viewer_) {
        cloud_viewer_ = std::make_shared<CloudViewer>(config_.window_title, config_.enable_background_thread);
    }

    cloud_viewer_->clear();

    Sophus::SE3d T_extrinsic = extrinsic.SE3_TargetInRef();

    // 变换并显示对齐前的点云 (灰色)
    for (size_t i = 0; i < std::min(scans_before.size(), scans_after.size()); ++i) {
        auto& scan_before = scans_before[i];
        auto& scan_after = scans_after[i];

        if (!scan_before.cloud || !scan_after.cloud) continue;

        // 变换后的点云
        auto cloud_transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::transformPointCloud(*scan_after.cloud, *cloud_transformed, Eigen::Affine3d(T_extrinsic.matrix()));

        // 添加到可视化器
        cloud_viewer_->add_cloud(cloud_transformed, "cloud_aligned", config_.point_cloud_color, 1.5f);
    }

    // 添加坐标系
    cloud_viewer_->add_coordinate_system(Sophus::SE3d(), "imu_origin", 0.5f);
    cloud_viewer_->add_coordinate_system(extrinsic, "extrinsic_frame", 0.5f);

    cloud_viewer_->spin_once(100);

    save_plots(config_.screenshot_dir + "/cloud_comparison");
}

// ===========================================================================
// 保存和报告
// ===========================================================================
void CalibVisualizer::save_calibration_results(const CalibParamManager& params, const std::string& output_dir, const std::string& format) {
    fs::create_directories(output_dir);

    // 生成报告
    std::string report_path = output_dir + "/calibration_report." + format;
    report_generator_->generate(report_path);

    // 保存可视化数据
    for (const auto& [key, data] : viz_data_) {
        std::string data_path = output_dir + "/" + key + "_viz_data.json";
        std::ofstream ofs(data_path);
        ofs << "{\n";
        ofs << "  \"key\": \"" << key << "\",\n";
        ofs << "  \"data\": " << data.to_json() << "\n";
        ofs << "}\n";
    }
}

void CalibVisualizer::generate_report(const std::string& output_path) {
    report_generator_->generate(output_path);
}

void CalibVisualizer::save_plots(const std::string& output_path) {
    if (config_.auto_save_plots && !output_path.empty() && cloud_viewer_) {
        std::string timestamp = std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
        std::string filepath = output_path + "_" + timestamp + ".png";
        cloud_viewer_->save_screenshot(filepath);
        UNICALIB_INFO("Saved plot: {}", filepath);
    }
}

// ===========================================================================
// 2D 绘图工具
// ===========================================================================
cv::Mat CalibVisualizer::draw_optimization_convergence_curve(const std::vector<double>& iterations,
                                                              const std::vector<double>& costs,
                                                              const std::string& title,
                                                              int width, int height) {
    if (iterations.size() != costs.size() || iterations.empty()) {
        return cv::Mat(height, width, CV_8UC3, cv::Scalar(255, 255, 255));
    }

    const int W = width > 0 ? width : config_.plot_width;
    const int H = height > 0 ? height : config_.plot_height;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    // 绘制标题
    cv::putText(img, title, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 计算范围
    double min_cost = *std::min_element(costs.begin(), costs.end());
    double max_cost = *std::max_element(costs.begin(), costs.end());
    int max_iter = static_cast<int>(*std::max_element(iterations.begin(), iterations.end()));

    // 绘制坐标轴
    const int margin = 60;
    const int plot_w = W - 2 * margin;
    const int plot_h = H - 2 * margin;
    cv::line(img, cv::Point(margin, margin), cv::Point(margin, H - margin), cv::Scalar(0, 0, 0), 1);
    cv::line(img, cv::Point(margin, H - margin), cv::Point(W - margin, H - margin), cv::Scalar(0, 0, 0), 1);

    // 轴标签
    cv::putText("Iteration", cv::Point(W / 2 - 40, H - margin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText("Cost", cv::Point(margin - 5, margin + 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // 绘制曲线
    double cost_range = max_cost - min_cost;
    if (cost_range < 1e-10) cost_range = 1.0;

    for (size_t i = 1; i < iterations.size(); ++i) {
        int x1 = margin + static_cast<int>(iterations[i - 1] / max_iter * plot_w);
        int y1 = H - margin - static_cast<int>((costs[i - 1] - min_cost) / cost_range * plot_h);
        int x2 = margin + static_cast<int>(iterations[i] / max_iter * plot_w);
        int y2 = H - margin - static_cast<int>((costs[i] - min_cost) / cost_range * plot_h);
        cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(60, 160, 60), 2);
        cv::circle(img, cv::Point(x2, y2), 4, cv::Scalar(200, 60, 60), -1);
    }

    return img;
}

cv::Mat CalibVisualizer::plot_time_offset_convergence_curve(const std::vector<double>& time_offsets,
                                                             const std::vector<double>& costs,
                                                             const std::string& title) {
    if (time_offsets.empty() || costs.empty()) return cv::Mat();

    std::sort(time_offsets.begin(), time_offsets.end());
    double min_time_offset = time_offsets.front();
    double max_time_offset = time_offsets.back();

    const int W = config_.plot_width;
    const int H = config_.plot_height;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    const int margin = 50;
    const int plot_w = W - 2 * margin;
    const int plot_h = H - 2 * margin;

    // 绘制坐标轴
    cv::line(img, cv::Point(margin, margin), cv::Point(margin, H - margin), cv::Scalar(0, 0, 0), 1);
    cv::line(img, cv::Point(margin, H - margin), cv::Point(W - margin, H - margin), cv::Scalar(0, 0, 0), 1);

    // 轴标签
    cv::putText("Time Offset (s)", cv::Point(W / 2 - 50, H - margin + 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText("Cost", cv::Point(margin - 5, margin + 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // 绘制数据点
    if (time_offsets.size() != costs.size()) return img;

    double time_range = max_time_offset - min_time_offset;
    double max_cost = *std::max_element(costs.begin(), costs.end());
    double min_cost = *std::min_element(costs.begin(), costs.end());
    double cost_range = max_cost - min_cost;
    if (cost_range < 1e-10) cost_range = 1.0;

    for (size_t i = 0; i < time_offsets.size(); ++i) {
        int x = margin + static_cast<int>((time_offsets[i] - min_time_offset) / time_range * plot_w);
        int y = H - margin - static_cast<int>((costs[i] - min_cost) / cost_range * plot_h);
        cv::circle(img, cv::Point(x, y), 3, cv::Scalar(200, 50, 50), -1);
    }

    return img;
}

cv::Mat CalibVisualizer::draw_imu_lidar_result_summary(const ExtrinsicSE3& coarse,
                                                       const ExtrinsicSE3& fine,
                                                       const std::vector<double>& rot_errors,
                                                       const std::vector<double>& trans_errors) {
    const int W = 800, H = 400;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(255, 255, 255));

    // 绘制标题
    cv::putText("IMU-LiDAR Calibration Results", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 计算变化量
    Eigen::Vector3d trans_diff = fine.POS_TargetInRef - coarse.POS_TargetInRef;
    Eigen::Vector3d rpy_diff = fine.euler_deg() - coarse.euler_deg();

    // 绘制结果
    cv::putText("Coarse -> Fine Changes:", cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(cv::format("  Translation: [{:.4f}, {:.4f}, {:.4f}] -> [{:.4f}, {:.4f}, {:.4f}] m",
                  trans_diff.x(), trans_diff.y(), trans_diff.z(),
                  fine.POS_TargetInRef.x(), fine.POS_TargetInRef.y(), fine.POS_TargetInRef.z()),
                cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(cv::format("  Rotation: [{:.4f}, {:.4f}, {:.4f}] -> [{:.4f}, {:.4f}, {:.4f}] deg",
                  rpy_diff.x(), rpy_diff.y(), rpy_diff.z(),
                  fine.euler_deg().x(), fine.euler_deg().y(), fine.euler_deg().z()),
                cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    cv::putText(cv::format("  Time Offset: {:.4f} -> {:.4f} ms",
                  coarse.time_offset_s * 1000, fine.time_offset_s * 1000),
                cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

    // 绘制误差统计
    if (!rot_errors.empty()) {
        cv::putText("Rotation Error Distribution", cv::Point(10, 200), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        // 绘制误差柱状图
        double max_err = *std::max_element(rot_errors.begin(), rot_errors.end());
        int margin = 60;
        int bar_width = 680;
        int bar_height = 150;
        for (size_t i = 0; i < rot_errors.size(); ++i) {
            double err_normalized = rot_errors[i] / max_err;
            int x = margin + static_cast<int>(i * bar_width / rot_errors.size());
            int bar_h = static_cast<int>(err_normalized * bar_height);
            int color = err_normalized < 0.3 ? static_cast<int>((1.0 - err_normalized) * 255) : static_cast<int>((1.0 - err_normalized) * 255);
            cv::rectangle(img, cv::Point(x, H - margin - bar_h - 10),
                          cv::Point(x + bar_width - 5, H - margin),
                          cv::Scalar(color, color, 0), cv::FILLED);
        }
    }

    if (!trans_errors.empty()) {
        cv::putText("Translation Error Distribution", cv::Point(10, 340), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
        // 绘制误差柱状图
        double max_err = *std::max_element(trans_errors.begin(), trans_errors.end());
        int margin = 60;
        int bar_width = 680;
        int bar_height = 150;
        for (size_t i = 0; i < trans_errors.size(); ++i) {
            double err_normalized = trans_errors[i] / max_err;
            int x = margin + static_cast<int>(i * bar_width / trans_errors.size());
            int bar_h = static_cast<int>(err_normalized * bar_height);
            int color = err_normalized < 0.3 ? static_cast<int>((1.0 - err_normalized) * 255) : static_cast<int>((1.0 - err_normalized) * 255);
            cv::rectangle(img, cv::Point(x, H - margin - bar_h - 10),
                          cv::Point(x + bar_width - 5, H - margin),
                          cv::Scalar(color, color, 0), cv::FILLED);
        }
    }

    return img;
}

// ===========================================================================
// 添加可视化数据
// ===========================================================================
void CalibVisualizer::add_visualization_data(const std::string& key, const VisualizationData& data) {
    std::lock_guard<std::mutex> lock(mtx_);
    viz_data_[key] = data;
}

}  // namespace ns_unicalib
