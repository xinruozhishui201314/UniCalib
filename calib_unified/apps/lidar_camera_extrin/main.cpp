/**
 * UniCalib — LiDAR-Camera 外参标定应用
 *
 * 两阶段流程:
 *   Stage 1 粗标定: MIAS-LCEC AI 模型 (可选, --coarse)
 *   Stage 2 精标定: 无目标边缘对齐 (默认) / 棋盘格 / B样条运动法
 *   手动校准:       --manual 标志触发 6-DOF 交互式调整
 *
 * 用法:
 *   unicalib_lidar_camera --config <config.yaml>
 *   unicalib_lidar_camera --config <config.yaml> --data-dir /path/to/data
 *   unicalib_lidar_camera --config <config.yaml> --coarse --ai-root /path/to/ai
 *   unicalib_lidar_camera --config <config.yaml> --manual
 *   unicalib_lidar_camera --config <config.yaml> --method edge|target|motion
 *
 * 数据路径: 支持两种配置方式
 *   1) 统一格式 (推荐): config 中 data.lidar.<id> / data.camera.<id>.images_dir，配合 CALIB_DATA_DIR 或 --data-dir 作为基准路径
 *   2) 扁平键: config 中 lidar_data_dir / camera_images_dir (可为相对上述基准路径)
 */
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/pipeline/calib_pipeline.h"
#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/pipeline/manual_calib.h"
#include "unicalib/extrinsic/lidar_camera_calib.h"
#include "unicalib/io/yaml_io.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <cstdlib>

namespace fs = std::filesystem;
using namespace ns_unicalib;

// 解析数据路径：若 path 为相对路径则与 base 拼接；若为占位符 /path/to/... 则视为相对 base 的路径；否则绝对路径直接返回
static std::string resolve_data_path(const std::string& base, const std::string& path) {
    if (path.empty()) return "";
    std::string work = path;
    // 占位符绝对路径：/path/to/xxx -> 视为 base + xxx，便于 --dataset 与示例配置配合
    const char prefix[] = "/path/to/";
    if (base.size() > 0 && work.size() > sizeof(prefix) - 1 &&
        work.compare(0, sizeof(prefix) - 1, prefix) == 0) {
        work = work.substr(sizeof(prefix) - 1);
    }
    if (base.empty()) return path;  // 未改过则原样返回
    fs::path p(work);
    if (p.is_absolute()) return path;
    fs::path b(base);
    return (b / p).lexically_normal().string();
}

static void print_banner() {
    std::cout << R"(
 ╔═══════════════════════════════════════════════════════╗
 ║   UniCalib — LiDAR-Camera 外参标定                    ║
 ║   两阶段: AI粗标定(MIAS-LCEC) → 无目标边缘精标定      ║
 ║   手动校准: --manual 触发 6-DOF 交互式调整             ║
 ╚═══════════════════════════════════════════════════════╝
)" << '\n';
}

static void print_help() {
    std::cout <<
        "用法: unicalib_lidar_camera [选项]\n\n"
        "必选:\n"
        "  --config/-c <file>      YAML 配置文件\n\n"
        "可选:\n"
        "  --coarse                启用 AI 粗标定 (MIAS-LCEC)\n"
        "  --ai-root <dir>         AI 工程根目录 (默认: ../)\n"
        "  --manual                精标定后启用手动校准\n"
        "  --method <m>            精标定方法: edge(默认)|target|motion\n"
        "  --no-targetfree         禁用无目标优先 (允许使用棋盘格)\n"
        "  --log-level <l>         日志级别: trace|debug|info|warn|error\n"
        "  --output-dir <dir>      输出目录 (默认: ./calib_output)\n"
        "  --data-dir <dir>        数据根目录 (与 config 中相对路径拼接；也可用环境变量 CALIB_DATA_DIR)\n"
        "  --help/-h               显示此帮助\n\n"
        "配置文件: 支持统一格式 data.lidar.<id>/data.camera.<id>.images_dir 或扁平键 lidar_data_dir/camera_images_dir\n"
        "  lidar_data_dir:         PCD 文件目录\n"
        "  camera_images_dir:      图像目录\n"
        "  camera_intrinsic_file:  相机内参 YAML\n"
        "  method:                 edge|target|motion\n"
        "  board_cols/rows:        棋盘格尺寸 (target 方法)\n"
        "  square_size_m:          棋盘格格子大小 [m]\n"
        "  edge_canny_low/high:    Canny 边缘检测阈值\n"
        "  coarse_rms_threshold:   粗标定通过阈值 [px]\n"
        "  fine_rms_threshold:     精标定通过阈值 [px]\n"
        "  manual_rms_threshold:   触发手动校准的阈值 [px] (默认 2.0)\n\n";
}

int main(int argc, char** argv) {
    print_banner();

    // ─────────────────────────────────────────────────────────────────
    // 解析命令行
    // ─────────────────────────────────────────────────────────────────
    std::string config_file, method_str = "edge";
    std::string ai_root = "../";
    std::string output_dir = "./calib_output/lidar_camera";
    std::string log_level  = "info";
    std::string data_dir;   // 数据根目录，空则用环境变量 CALIB_DATA_DIR
    bool do_coarse   = false;
    bool do_manual   = false;
    bool prefer_tf   = true;   // prefer target-free
    
    // ROS2 参数
    bool use_ros2_bag = false;
    bool use_ros2_topics = false;
    std::string ros2_bag_file;
    std::string lidar_ros2_topic;
    std::string camera_ros2_topic;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "--config" || a == "-c") && i+1 < argc)
            config_file = argv[++i];
        else if (a == "--coarse")                 do_coarse = true;
        else if (a == "--manual")                 do_manual = true;
        else if (a == "--no-targetfree")          prefer_tf = false;
        else if (a == "--method" && i+1 < argc)   method_str = argv[++i];
        else if (a == "--ai-root" && i+1 < argc)  ai_root = argv[++i];
        else if (a == "--output-dir" && i+1 < argc) output_dir = argv[++i];
        else if (a == "--data-dir" && i+1 < argc)  data_dir = argv[++i];
        else if (a == "--log-level" && i+1 < argc) log_level = argv[++i];
        else if (a == "--ros2-bag" && i+1 < argc) {
            use_ros2_bag = true;
            ros2_bag_file = argv[++i];
        }
        else if (a == "--ros2-topic") {
            use_ros2_topics = true;
        }
        else if (a == "--lidar-topic" && i+1 < argc) {
            lidar_ros2_topic = argv[++i];
        }
        else if (a == "--camera-topic" && i+1 < argc) {
            camera_ros2_topic = argv[++i];
        }
        else if (a == "--help" || a == "-h") { print_help(); return 0; }
    }

    if (config_file.empty()) {
        std::cerr << "[Error] 未指定 --config 文件\n";
        print_help();
        return 1;
    }

    UNICALIB_MAIN_TRY_BEGIN

    // ─────────────────────────────────────────────────────────────────
    // 初始化日志（写入 logs 目录，文件名带时间戳）
    // ─────────────────────────────────────────────────────────────────
    std::string logs_dir = resolve_logs_dir(output_dir);
    std::string log_file = logs_dir + "/lidar_camera_" + log_timestamp_filename() + ".log";
    Logger::init("LiDAR-Camera",
                 log_file,
                 log_level == "debug"   ? spdlog::level::debug   :
                 log_level == "trace"   ? spdlog::level::trace   :
                 log_level == "warn"    ? spdlog::level::warn    :
                 log_level == "error"   ? spdlog::level::err     :
                                          spdlog::level::info);

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("配置文件: {}", config_file);
    UNICALIB_INFO("输出目录: {}", output_dir);
    UNICALIB_INFO("精标定方法: {} (无目标优先={})", method_str, prefer_tf);
    UNICALIB_INFO("AI粗标定: {} | 手动校准: {}", do_coarse, do_manual);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─────────────────────────────────────────────────────────────────
    // 加载配置
    // ─────────────────────────────────────────────────────────────────
    YAML::Node cfg;
    try {
        cfg = YAML::LoadFile(config_file);
    } catch (const std::exception& e) {
        UNICALIB_ERROR("配置文件加载失败: {}", e.what());
        return 1;
    }

    // 从配置文件覆盖命令行默认值
    if (cfg["method"])      method_str = cfg["method"].as<std::string>();
    if (cfg["output_dir"])  output_dir = cfg["output_dir"].as<std::string>();
    if (cfg["prefer_targetfree"]) prefer_tf = cfg["prefer_targetfree"].as<bool>();

    double manual_rms_thresh = 2.0;
    if (cfg["manual_rms_threshold"]) manual_rms_thresh = cfg["manual_rms_threshold"].as<double>();

    // ─────────────────────────────────────────────────────────────────
    // 数据路径解析 (统一格式 data.lidar / data.camera 或扁平键 lidar_data_dir / camera_images_dir)
    // ─────────────────────────────────────────────────────────────────
    std::string base_data_dir = data_dir.empty() ? (std::getenv("CALIB_DATA_DIR") ? std::getenv("CALIB_DATA_DIR") : "") : data_dir;
    std::string lidar_dir_raw;
    std::string camera_dir_raw;

    if (cfg["lidar_data_dir"])
        lidar_dir_raw = cfg["lidar_data_dir"].as<std::string>();
    if (cfg["camera_images_dir"])
        camera_dir_raw = cfg["camera_images_dir"].as<std::string>();

    if (lidar_dir_raw.empty() || camera_dir_raw.empty()) {
        try {
            SystemConfig sys_cfg = YamlIO::load_system_config(config_file);
            if (lidar_dir_raw.empty() && !sys_cfg.lidar_data_paths.empty())
                lidar_dir_raw = sys_cfg.lidar_data_paths.begin()->second;
            if (camera_dir_raw.empty() && !sys_cfg.camera_images_dirs.empty())
                camera_dir_raw = sys_cfg.camera_images_dirs.begin()->second;
        } catch (const std::exception& e) {
            UNICALIB_DEBUG("使用扁平键读取数据路径 (load_system_config 未用): {}", e.what());
        }
    }

    std::string lidar_dir_resolved  = resolve_data_path(base_data_dir, lidar_dir_raw);
    std::string camera_dir_resolved = resolve_data_path(base_data_dir, camera_dir_raw);

    // 若目录不存在则自动创建，便于首次运行或挂载卷为空时先建目录再放入数据
    if (!lidar_dir_resolved.empty() && !fs::exists(lidar_dir_resolved)) {
        std::error_code ec;
        fs::create_directories(lidar_dir_resolved, ec);
        if (ec)
            UNICALIB_WARN("自动创建 LiDAR 目录失败: {} — {}", lidar_dir_resolved, ec.message());
        else
            UNICALIB_INFO("已自动创建 LiDAR 数据目录: {}", lidar_dir_resolved);
    }
    if (!camera_dir_resolved.empty() && !fs::exists(camera_dir_resolved)) {
        std::error_code ec;
        fs::create_directories(camera_dir_resolved, ec);
        if (ec)
            UNICALIB_WARN("自动创建相机图像目录失败: {} — {}", camera_dir_resolved, ec.message());
        else
            UNICALIB_INFO("已自动创建相机图像目录: {}", camera_dir_resolved);
    }

    // 数据路径来源（便于排查：来自扁平键还是 data.lidar/data.camera）
    bool from_flat = cfg["lidar_data_dir"] && cfg["camera_images_dir"];
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ [数据路径] ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("  数据路径来源: {}",
                  from_flat ? "扁平键 (lidar_data_dir / camera_images_dir)" : "统一格式 (data.lidar.<id> / data.camera.<id>.images_dir)");
    UNICALIB_INFO("  配置原始值 — lidar:  {}",
                  lidar_dir_raw.empty() ? "(未设置)" : lidar_dir_raw);
    UNICALIB_INFO("  配置原始值 — camera: {}",
                  camera_dir_raw.empty() ? "(未设置)" : camera_dir_raw);
    UNICALIB_INFO("  base_data_dir:  {}",
                  base_data_dir.empty() ? "(未设置，可设 CALIB_DATA_DIR 或 --data-dir)" : base_data_dir);
    if (!base_data_dir.empty()) {
        UNICALIB_INFO("  base 来源: {}",
                      data_dir.empty() ? "环境变量 CALIB_DATA_DIR" : "命令行 --data-dir");
    }
    bool lidar_absolute = !lidar_dir_raw.empty() && fs::path(lidar_dir_raw).is_absolute();
    bool cam_absolute   = !camera_dir_raw.empty() && fs::path(camera_dir_raw).is_absolute();
    UNICALIB_INFO("  lidar_dir:      {}",
                  lidar_dir_resolved.empty() ? "(未设置)" : lidar_dir_resolved);
    if (!lidar_dir_resolved.empty()) {
        UNICALIB_INFO("    → {}",
                      lidar_absolute ? "配置为绝对路径，未与 base 拼接（若为 /path/to/... 占位符则不会指向实际数据）"
                                    : "相对路径，已与 base 拼接");
        bool exists = fs::exists(lidar_dir_resolved);
        UNICALIB_INFO("    → 存在: {}", exists ? "是" : "否");
        if (!exists)
            UNICALIB_WARN("    LiDAR 数据目录不存在，精标定将无法加载点云");
    }
    UNICALIB_INFO("  camera_dir:     {}",
                  camera_dir_resolved.empty() ? "(未设置)" : camera_dir_resolved);
    if (!camera_dir_resolved.empty()) {
        UNICALIB_INFO("    → {}",
                      cam_absolute ? "配置为绝对路径，未与 base 拼接（若为 /path/to/... 占位符则不会指向实际数据）"
                                  : "相对路径，已与 base 拼接");
        bool exists = fs::exists(camera_dir_resolved);
        UNICALIB_INFO("    → 存在: {}", exists ? "是" : "否");
        if (!exists)
            UNICALIB_WARN("    相机图像目录不存在，精标定将无法加载图像");
    }

    bool data_ready = !lidar_dir_resolved.empty() && !camera_dir_resolved.empty() &&
                      fs::exists(lidar_dir_resolved) && fs::exists(camera_dir_resolved);
    std::string skip_reason;
    if (!data_ready) {
        if (base_data_dir.empty())
            skip_reason = "未设置数据根目录（请设置环境变量 CALIB_DATA_DIR 或命令行 --data-dir）";
        else if (lidar_dir_raw.find("/path/to") != std::string::npos || camera_dir_raw.find("/path/to") != std::string::npos)
            skip_reason = "配置中 data.lidar / data.camera 为占位符 /path/to/... 且为绝对路径，未与数据根目录拼接；请改为相对路径（如 pcd、images）或填写实际路径";
        else if (lidar_dir_resolved.empty() || camera_dir_resolved.empty())
            skip_reason = "配置中未提供 LiDAR 或相机数据路径（请填写 data.lidar.<id> 与 data.camera.<id>.images_dir）";
        else if (!fs::exists(lidar_dir_resolved))
            skip_reason = "LiDAR 目录不存在: " + lidar_dir_resolved;
        else
            skip_reason = "相机图像目录不存在: " + camera_dir_resolved;
    }
    UNICALIB_INFO("  【数据就绪】{} — {}",
                  data_ready ? "是" : "否",
                  data_ready ? "点云与图像目录均存在，将执行精标定" : ("原因: " + skip_reason));
    UNICALIB_INFO("  说明: 使用 --dataset 时请将 data.lidar/data.camera 设为相对 base 的路径（如 pcd、images），不要使用 /path/to/... 占位符");
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    // ─────────────────────────────────────────────────────────────────
    // 构建流水线配置
    // ─────────────────────────────────────────────────────────────────
    PipelineConfig pipe_cfg;
    pipe_cfg.tasks               = CalibTaskType::LIDAR_CAM_EXTRIN;
    pipe_cfg.enable_coarse_lidar_cam = do_coarse;
    pipe_cfg.allow_manual_fallback   = do_manual;
    pipe_cfg.prefer_targetfree       = prefer_tf;
    pipe_cfg.lidar_cam_rms_threshold = manual_rms_thresh;
    pipe_cfg.output_dir              = output_dir;
    pipe_cfg.ai_models_root          = ai_root;
    pipe_cfg.log_level               = log_level;
    
    // ─── 数据源配置 (文件或 ROS2) ─────────────────────────────
    // 命令行 ROS2 标志与话题
    bool use_ros2_bag = false;
    bool use_ros2_topics = false;
    std::string ros2_bag_file;
    std::string lidar_ros2_topic;
    std::string camera_ros2_topic;
    
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--ros2-bag" && i+1 < argc) {
            use_ros2_bag = true;
            ros2_bag_file = argv[++i];
        } else if (a == "--ros2-topic") {
            use_ros2_topics = true;
        } else if (a == "--lidar-topic" && i+1 < argc) {
            lidar_ros2_topic = argv[++i];
        } else if (a == "--camera-topic" && i+1 < argc) {
            camera_ros2_topic = argv[++i];
        }
    }
    
    // 未从命令行指定时，从配置文件 ros2 / sensors 读取默认话题
    const YAML::Node ros2_node = cfg["ros2"];
    if (ros2_node) {
        if (!use_ros2_bag && ros2_node["use_ros2_bag"] && ros2_node["use_ros2_bag"].as<bool>()) {
            use_ros2_bag = true;
            if (ros2_bag_file.empty() && ros2_node["ros2_bag_file"])
                ros2_bag_file = ros2_node["ros2_bag_file"].as<std::string>();
        }
        if (!use_ros2_topics && ros2_node["use_ros2_topics"] && ros2_node["use_ros2_topics"].as<bool>())
            use_ros2_topics = true;
        if (lidar_ros2_topic.empty() && ros2_node["lidar_topic"])
            lidar_ros2_topic = ros2_node["lidar_topic"].as<std::string>();
        if (camera_ros2_topic.empty() && ros2_node["camera_topic"])
            camera_ros2_topic = ros2_node["camera_topic"].as<std::string>();
    }
    if (lidar_ros2_topic.empty() || camera_ros2_topic.empty()) {
        try {
            SystemConfig sys_cfg = YamlIO::load_system_config(config_file);
            for (const auto& s : sys_cfg.sensors) {
                if (s.type == SensorType::LiDAR && lidar_ros2_topic.empty() && !s.topic.empty()) {
                    lidar_ros2_topic = s.topic;
                    pipe_cfg.lidar_id = s.sensor_id;
                }
                if (s.type == SensorType::CAMERA && camera_ros2_topic.empty() && !s.topic.empty()) {
                    camera_ros2_topic = s.topic;
                    pipe_cfg.camera_id = s.sensor_id;
                }
            }
        } catch (const std::exception& e) {
            UNICALIB_DEBUG("从系统配置读取 ROS2 话题失败，沿用命令行或 ros2 段: {}", e.what());
        }
    }
    
    // 设置数据源配置
    if (use_ros2_bag && !ros2_bag_file.empty()) {
        pipe_cfg.use_ros2_bag = true;
        pipe_cfg.ros2_bag_file = ros2_bag_file;
        pipe_cfg.lidar_ros2_topic = lidar_ros2_topic;
        pipe_cfg.camera_ros2_topic = camera_ros2_topic;
        
        UNICALIB_INFO("配置: 使用 ROS2 bag 模式");
        UNICALIB_INFO("  Bag 文件: {}", ros2_bag_file);
        UNICALIB_INFO("  LiDAR 话题: {}", pipe_cfg.lidar_ros2_topic.empty() ? "(未设置)" : pipe_cfg.lidar_ros2_topic);
        UNICALIB_INFO("  相机话题: {}", pipe_cfg.camera_ros2_topic.empty() ? "(未设置)" : pipe_cfg.camera_ros2_topic);
    } else if (use_ros2_topics) {
        pipe_cfg.use_ros2_topics = true;
        pipe_cfg.lidar_ros2_topic = lidar_ros2_topic;
        pipe_cfg.camera_ros2_topic = camera_ros2_topic;
        
        UNICALIB_INFO("配置: 使用 ROS2 实时话题模式");
        UNICALIB_INFO("  LiDAR 话题: {}", pipe_cfg.lidar_ros2_topic.empty() ? "(未设置)" : pipe_cfg.lidar_ros2_topic);
        UNICALIB_INFO("  相机话题: {}", pipe_cfg.camera_ros2_topic.empty() ? "(未设置)" : pipe_cfg.camera_ros2_topic);
    } else {
        pipe_cfg.lidar_data_dir    = lidar_dir_resolved;
        pipe_cfg.camera_images_dir = camera_dir_resolved;
    }

    CalibPipeline pipeline(pipe_cfg);

    // ─────────────────────────────────────────────────────────────────
    // Stage 1: AI 粗标定 (可选)
    // ─────────────────────────────────────────────────────────────────
    std::optional<Sophus::SE3d> coarse_init;
    if (do_coarse) {
        UNICALIB_INFO("▶ Stage 1: AI 粗标定 (MIAS-LCEC)");

        AICoarseCalibManager::Config ai_cfg;
        ai_cfg.ai_root    = ai_root;
        ai_cfg.python_exe = cfg["python_exe"] ?
            cfg["python_exe"].as<std::string>() : "python3";

        AICoarseCalibManager ai_mgr(ai_cfg);
        ai_mgr.print_availability();

        std::string pcd_file   = cfg["coarse_pcd_file"] ?
            cfg["coarse_pcd_file"].as<std::string>() : "";
        std::string image_file = cfg["coarse_image_file"] ?
            cfg["coarse_image_file"].as<std::string>() : "";

        // 内参 (从文件加载)
        CameraIntrinsics cam_intrin;
        if (cfg["camera_intrinsic_file"]) {
            try {
                auto intrin_node = YAML::LoadFile(
                    cfg["camera_intrinsic_file"].as<std::string>());
                cam_intrin.fx = intrin_node["fx"].as<double>();
                cam_intrin.fy = intrin_node["fy"].as<double>();
                cam_intrin.cx = intrin_node["cx"].as<double>();
                cam_intrin.cy = intrin_node["cy"].as<double>();
                cam_intrin.width  = intrin_node["width"].as<int>();
                cam_intrin.height = intrin_node["height"].as<int>();
                UNICALIB_INFO("  内参: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}",
                              cam_intrin.fx, cam_intrin.fy,
                              cam_intrin.cx, cam_intrin.cy);
            } catch (...) {
                UNICALIB_WARN("  内参文件加载失败, 使用默认值");
            }
        }

        if (!pcd_file.empty() && !image_file.empty()) {
            auto coarse_result = ai_mgr.coarse_lidar_cam(
                pcd_file, image_file, cam_intrin);
            if (coarse_result.has_value()) {
                coarse_init = coarse_result->SE3_TargetInRef();
                UNICALIB_INFO("  ✓ AI粗标定成功: t=[{:.3f},{:.3f},{:.3f}]m",
                              coarse_init->translation().x(),
                              coarse_init->translation().y(),
                              coarse_init->translation().z());
            } else {
                UNICALIB_WARN("  ✗ AI粗标定失败, 使用 identity 初始化");
            }
        } else {
            UNICALIB_WARN("  未提供 coarse_pcd_file/coarse_image_file, 跳过AI粗标定");
        }
    }

    // ─────────────────────────────────────────────────────────────────
    // Stage 2: 精标定
    // ─────────────────────────────────────────────────────────────────
    UNICALIB_INFO("▶ Stage 2: 精标定 (方法={})", method_str);

    LiDARCameraCalibrator::Config calib_cfg;
    if      (method_str == "edge")   calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
    else if (method_str == "target") calib_cfg.method = LiDARCameraCalibrator::Method::TARGET_CHESSBOARD;
    else if (method_str == "motion") calib_cfg.method = LiDARCameraCalibrator::Method::MOTION_BSPLINE;
    else {
        UNICALIB_WARN("未知方法 '{}', 使用 edge", method_str);
        calib_cfg.method = LiDARCameraCalibrator::Method::EDGE_ALIGNMENT;
    }

    if (cfg["board_cols"])    calib_cfg.board_cols = cfg["board_cols"].as<int>();
    if (cfg["board_rows"])    calib_cfg.board_rows = cfg["board_rows"].as<int>();
    if (cfg["square_size_m"]) calib_cfg.square_size_m = cfg["square_size_m"].as<double>();
    if (cfg["edge_canny_low"])  calib_cfg.edge_canny_low  = cfg["edge_canny_low"].as<int>();
    if (cfg["edge_canny_high"]) calib_cfg.edge_canny_high = cfg["edge_canny_high"].as<int>();
    if (cfg["ceres_max_iter"])  calib_cfg.ceres_max_iter  = cfg["ceres_max_iter"].as<int>();
    calib_cfg.verbose = (log_level == "debug" || log_level == "trace");

    LiDARCameraCalibrator calibrator(calib_cfg);
    calibrator.set_progress_callback([](const std::string& step, double prog) {
        if (prog >= 0)
            UNICALIB_INFO("  [进度] {}: {:.1f}%", step, prog * 100.0);
    });

    if (!data_ready) {
        UNICALIB_WARN("数据路径未就绪，当前运行将仅执行流水线占位阶段，不进行实际标定优化。详见上方【数据就绪】原因。");
    }

    // ─────────────────────────────────────────────────────────────────
    // Stage 3: 手动校准 (可选, 当精标定质量不足时)
    // ─────────────────────────────────────────────────────────────────
    if (do_manual) {
        UNICALIB_INFO("▶ Stage 3: 手动校准 (manual_threshold={:.2f}px)",
                      manual_rms_thresh);
        UNICALIB_INFO("  使用 ManualCalibSession 进行 6-DOF 交互式调整");
        UNICALIB_INFO("  调用示例:");
        UNICALIB_INFO("    ManualCalibSession::SessionConfig sess_cfg;");
        UNICALIB_INFO("    sess_cfg.save_dir = \"{}/manual_sessions\";", output_dir);
        UNICALIB_INFO("    ManualCalibSession session(sess_cfg);");
        UNICALIB_INFO("    auto result = session.run_lidar_cam(scan, image, intrin, auto_result, auto_rms);");
    }

    // ─────────────────────────────────────────────────────────────────
    // 运行流水线日志摘要
    // ─────────────────────────────────────────────────────────────────
    auto report = pipeline.run();
    report.print_summary();

    bool any_placeholder = false;
    for (const auto& r : report.stage_results) {
        if (r.message.find("占位") != std::string::npos) { any_placeholder = true; break; }
    }
    if (any_placeholder) {
        UNICALIB_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
        UNICALIB_WARN("本次未执行实际标定: 精标定阶段为占位实现，未加载点云/图像也未调用标定器。");
        UNICALIB_WARN("请确保: 1) 配置 data.lidar.<id> 与 data.camera.<id>.images_dir (或扁平键); 2) 设置 CALIB_DATA_DIR 或 --data-dir; 3) 数据目录存在且含 PCD/图像。");
        UNICALIB_WARN("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    }

    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_INFO("LiDAR-Camera 标定流程完成");
    UNICALIB_INFO("结果目录: {}", output_dir);
    UNICALIB_INFO("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    UNICALIB_MAIN_TRY_END(0)
}
