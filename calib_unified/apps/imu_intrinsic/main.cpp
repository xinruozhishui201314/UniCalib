/**
 * UniCalib — IMU 内参标定应用
 * 使用: unicalib_imu_intrinsic --config <config.yaml>
 *       unicalib_imu_intrinsic --data_dir <dir> --sensor_id <id>
 *
 * 输出:
 *   - 标定结果 YAML/JSON
 *   - Allan 偏差图 PNG
 *   - HTML 报告
 */

#include "unicalib/common/logger.h"
#include "unicalib/common/sensor_types.h"
#include "unicalib/common/calib_param.h"
#include "unicalib/intrinsic/imu_intrinsic_calib.h"
#include "unicalib/viz/report_gen.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <chrono>
#include <iomanip>

namespace fs = std::filesystem;

// ===================================================================
// 配置解析
// ===================================================================
struct AppConfig {
    std::string sensor_id = "imu_0";
    std::string data_file;          // CSV 文件: timestamp,gx,gy,gz,ax,ay,az
    std::string data_dir;           // 或目录(含 imu.csv)
    std::string output_dir = "./calib_output/imu_intrinsic";
    ns_unicalib::IMUIntrinsicCalibrator::Config calib_cfg;
};

AppConfig parse_config(int argc, char** argv) {
    AppConfig cfg;

    // 简单参数解析
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) {
            // 从 YAML 文件加载
            YAML::Node node = YAML::LoadFile(argv[++i]);
            if (node["sensor_id"])  cfg.sensor_id  = node["sensor_id"].as<std::string>();
            if (node["data_file"])  cfg.data_file  = node["data_file"].as<std::string>();
            if (node["data_dir"])   cfg.data_dir   = node["data_dir"].as<std::string>();
            if (node["output_dir"]) cfg.output_dir = node["output_dir"].as<std::string>();
            if (node["calib"]) {
                auto cc = node["calib"];
                if (cc["static_gyro_thresh"]) cfg.calib_cfg.static_gyro_threshold = cc["static_gyro_thresh"].as<double>();
                if (cc["min_static_frames"])  cfg.calib_cfg.min_static_frames = cc["min_static_frames"].as<int>();
            }
        } else if (arg == "--sensor_id" && i + 1 < argc) {
            cfg.sensor_id = argv[++i];
        } else if (arg == "--data_file" && i + 1 < argc) {
            cfg.data_file = argv[++i];
        } else if (arg == "--data_dir" && i + 1 < argc) {
            cfg.data_dir = argv[++i];
        } else if (arg == "--output_dir" && i + 1 < argc) {
            cfg.output_dir = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: unicalib_imu_intrinsic [options]\n"
                      << "  --config <yaml>      配置文件\n"
                      << "  --sensor_id <id>     传感器ID (default: imu_0)\n"
                      << "  --data_file <csv>    IMU数据CSV文件\n"
                      << "  --data_dir <dir>     数据目录(含 imu.csv)\n"
                      << "  --output_dir <dir>   输出目录\n"
                      << "\nCSV 格式: timestamp[s],gx,gy,gz[rad/s],ax,ay,az[m/s^2]\n";
            exit(0);
        }
    }

    if (cfg.data_file.empty() && !cfg.data_dir.empty()) {
        cfg.data_file = cfg.data_dir + "/imu.csv";
    }

    return cfg;
}

// ===================================================================
// 加载 IMU CSV 数据
// ===================================================================
ns_unicalib::IMURawData load_imu_csv(const std::string& csv_path) {
    ns_unicalib::IMURawData data;

    std::ifstream f(csv_path);
    if (!f.is_open()) {
        UNICALIB_ERROR("Cannot open IMU data file: {}", csv_path);
        return data;
    }

    std::string line;
    // 跳过注释和表头
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        if (line.find("timestamp") != std::string::npos ||
            line.find("time") != std::string::npos) continue;

        std::istringstream ss(line);
        double ts, gx, gy, gz, ax, ay, az;
        char comma;
        if (ss >> ts >> comma >> gx >> comma >> gy >> comma >> gz
               >> comma >> ax >> comma >> ay >> comma >> az) {
            ns_unicalib::IMURawFrame frame;
            frame.timestamp = ts;
            frame.gyro = {gx, gy, gz};
            frame.accel = {ax, ay, az};
            data.push_back(frame);
        }
    }

    UNICALIB_INFO("Loaded {} IMU frames from {}", data.size(), csv_path);
    return data;
}

// ===================================================================
// 主函数
// ===================================================================
int main(int argc, char** argv) {
    // 初始化日志
    ns_unicalib::Logger::init("UniCalib-IMU-Intrinsic");

    std::cout << R"(
 ╔══════════════════════════════════════════════╗
 ║    UniCalib — IMU 内参标定 (Allan + 六面法)  ║
 ╚══════════════════════════════════════════════╝
)" << std::endl;

    // 解析配置
    auto cfg = parse_config(argc, argv);

    if (cfg.data_file.empty()) {
        UNICALIB_ERROR("No data file specified. Use --data_file or --config");
        return 1;
    }

    // 创建输出目录
    fs::create_directories(cfg.output_dir);

    // 加载 IMU 数据
    UNICALIB_INFO("Loading IMU data from: {}", cfg.data_file);
    auto imu_data = load_imu_csv(cfg.data_file);

    if (imu_data.size() < 1000) {
        UNICALIB_ERROR("Insufficient IMU data: {} frames (need ≥ 1000)", imu_data.size());
        return 1;
    }

    double duration = imu_data.back().timestamp - imu_data.front().timestamp;
    UNICALIB_INFO("Duration: {:.1f} s | Frames: {}", duration, imu_data.size());

    // 标定
    ns_unicalib::IMUIntrinsicCalibrator calibrator(cfg.calib_cfg);

    calibrator.set_progress_callback([](const std::string& stage, double prog) {
        static std::string last_stage;
        if (stage != last_stage) {
            std::cout << "\r  [" << stage << "] " << std::flush;
            last_stage = stage;
        }
        int bar_width = 30;
        int filled = static_cast<int>(prog * bar_width);
        std::cout << "\r  [" << stage << "] [";
        for (int i = 0; i < bar_width; ++i)
            std::cout << (i < filled ? "█" : "░");
        std::cout << "] " << std::fixed << std::setprecision(0)
                  << prog * 100 << "%" << std::flush;
    });

    auto t_start = std::chrono::high_resolution_clock::now();
    ns_unicalib::IMUIntrinsics intrin = calibrator.calibrate(imu_data);
    auto t_end = std::chrono::high_resolution_clock::now();
    std::cout << std::endl;

    double elapsed = std::chrono::duration<double>(t_end - t_start).count();
    UNICALIB_INFO("Calibration took {:.1f} s", elapsed);

    // 保存 Allan 偏差图
    std::string allan_plot_path = cfg.output_dir + "/allan_gyro.png";
    auto allan_result = calibrator.analyze_allan(imu_data);
    calibrator.save_allan_plot(allan_result, allan_plot_path);

    // 保存标定结果
    YAML::Node result_node;
    result_node["sensor_id"] = cfg.sensor_id;
    result_node["timestamp"] = std::to_string(
        std::chrono::system_clock::now().time_since_epoch().count());
    result_node["noise_gyro"]        = intrin.noise_gyro;
    result_node["bias_instab_gyro"]  = intrin.bias_instab_gyro;
    result_node["noise_accel"]       = intrin.noise_acce;
    result_node["bias_instab_accel"] = intrin.bias_instab_acce;
    result_node["bias_gyro"] = std::vector<double>{
        intrin.bias_gyro[0], intrin.bias_gyro[1], intrin.bias_gyro[2]};
    result_node["bias_accel"] = std::vector<double>{
        intrin.bias_acce[0], intrin.bias_acce[1], intrin.bias_acce[2]};

    std::string yaml_path = cfg.output_dir + "/imu_intrinsic_" + cfg.sensor_id + ".yaml";
    {
        std::ofstream yaml_out(yaml_path);
        yaml_out << result_node;
        UNICALIB_INFO("Results saved to: {}", yaml_path);
    }

    // 打印终端摘要
    std::cout << "\n";
    std::cout << "┌────────────────────────────────────────────────────┐\n";
    std::cout << "│              IMU 内参标定结果                      │\n";
    std::cout << "├──────────────────────┬─────────────────────────────┤\n";
    std::cout << "│ 传感器 ID            │ " << std::setw(27) << cfg.sensor_id << " │\n";
    std::cout << "│ 数据帧数             │ " << std::setw(27) << imu_data.size() << " │\n";
    std::cout << "│ 数据时长 (s)         │ " << std::setw(27) << std::fixed
              << std::setprecision(1) << duration << " │\n";
    std::cout << "├──────────────────────┼─────────────────────────────┤\n";
    std::cout << "│ 陀螺噪声 (rad/s/√Hz) │ " << std::setw(27) << std::scientific
              << std::setprecision(4) << intrin.noise_gyro << " │\n";
    std::cout << "│ 陀螺零偏不稳 (rad/s) │ " << std::setw(27) << intrin.bias_instab_gyro << " │\n";
    std::cout << "│ 加速计噪声 (m/s²/√Hz)│ " << std::setw(27) << intrin.noise_acce << " │\n";
    std::cout << "│ 加速计零偏 (m/s²)    │ " << std::setw(27) << intrin.bias_instab_acce << " │\n";
    std::cout << "└──────────────────────┴─────────────────────────────┘\n";
    std::cout << "\n结果保存至: " << cfg.output_dir << "\n\n";

    return 0;
}
