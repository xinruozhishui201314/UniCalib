/**
 * UniCalib Unified — YAML/JSON 配置读写实现
 * 支持:
 *   - 读取 SystemConfig (传感器/标定对配置)
 *   - 读取各标定器 Config
 *   - 写入 CalibParamManager 到 YAML
 *   - IMU CSV 数据加载
 */

#include "unicalib/io/yaml_io.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/sensor_types.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ns_unicalib {

// ===================================================================
// YAML 辅助宏
// ===================================================================
#define YAML_GET_OR(node, key, default_val) \
    ((node)[key] ? (node)[key].as<decltype(default_val)>() : (default_val))

// ===================================================================
// 读取 SystemConfig
// ===================================================================
SystemConfig YamlIO::load_system_config(const std::string& yaml_path) {
    UNICALIB_INFO("读取系统配置: {}", yaml_path);
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML 解析失败: " + yaml_path + "\n" + e.what());
    }

    SystemConfig cfg;

    // 传感器列表
    if (root["sensors"]) {
        for (const auto& s : root["sensors"]) {
            SensorDesc desc;
            desc.sensor_id = YAML_GET_OR(s, "id", std::string("unknown"));

            std::string type_str = YAML_GET_OR(s, "type", std::string("camera"));
            desc.type = sensor_type_from_str(type_str);

            std::string lidar_str = YAML_GET_OR(s, "lidar_type", std::string("spinning"));
            SensorDesc::LiDARParams lp;
            lp.lidar_type = (lidar_str == "solid_state") ?
                LidarType::SOLID_STATE : LidarType::SPINNING;
            lp.scan_lines = YAML_GET_OR(s, "scan_lines", 16);
            lp.rate_hz    = YAML_GET_OR(s, "rate_hz", 10.0);
            if (desc.type == SensorType::LiDAR) desc.lidar_params = lp;

            std::string cam_str = YAML_GET_OR(s, "camera_model", std::string("pinhole"));
            SensorDesc::CameraParams cp;
            cp.model   = (cam_str == "fisheye") ? CameraModel::FISHEYE :
                          (cam_str == "omni")   ? CameraModel::OMNI : CameraModel::PINHOLE;
            cp.width   = YAML_GET_OR(s, "width", 0);
            cp.height  = YAML_GET_OR(s, "height", 0);
            cp.fps     = YAML_GET_OR(s, "fps", 30.0);
            if (desc.type == SensorType::CAMERA) desc.camera_params = cp;

            SensorDesc::IMUParams ip;
            ip.rate_hz = YAML_GET_OR(s, "rate_hz", 200.0);
            if (desc.type == SensorType::IMU) desc.imu_params = ip;

            desc.topic = YAML_GET_OR(s, "topic", std::string(""));
            cfg.sensors.push_back(desc);
        }
    }

    // 参考传感器
    cfg.reference_imu = YAML_GET_OR(root, "reference_sensor", std::string(""));

    // 标定对
    if (root["calib_pairs"]) {
        for (const auto& p : root["calib_pairs"]) {
            CalibPair pair;
            pair.reference_sensor = YAML_GET_OR(p, "sensor_a", std::string(""));
            pair.target_sensor    = YAML_GET_OR(p, "sensor_b", std::string(""));

            std::string t_str = YAML_GET_OR(p, "type", std::string("imu_lidar"));
            if      (t_str == "imu_intrinsic")   pair.calib_type = CalibType::IMU_INTRINSIC;
            else if (t_str == "camera_intrinsic") pair.calib_type = CalibType::CAMERA_INTRINSIC;
            else if (t_str == "imu_lidar")        pair.calib_type = CalibType::IMU_LIDAR_EXTRINSIC;
            else if (t_str == "lidar_camera")     pair.calib_type = CalibType::LIDAR_CAMERA_EXTRINSIC;
            else if (t_str == "cam_cam")          pair.calib_type = CalibType::CAM_CAM_EXTRINSIC;
            else                                   pair.calib_type = CalibType::JOINT;

            pair.initial_time_offset_s = YAML_GET_OR(p, "initial_time_offset", 0.0);
            cfg.calib_pairs.push_back(pair);
        }
    }

    // 输出目录
    cfg.output_dir   = YAML_GET_OR(root, "output_dir", std::string("./calib_output"));
    cfg.data_dir     = YAML_GET_OR(root, "data_dir", std::string("./data"));
    cfg.bag_file     = YAML_GET_OR(root, "bag_file", std::string(""));

    UNICALIB_INFO("  传感器数: {} 标定对: {}",
                  cfg.sensors.size(), cfg.calib_pairs.size());
    return cfg;
}

// ===================================================================
// 保存标定结果为 YAML
// ===================================================================
void YamlIO::save_calib_results(
    const CalibParamManager& params,
    const std::string& yaml_path) {

    params.save_yaml(yaml_path);
}

// ===================================================================
// IMU CSV 数据加载
// 格式: timestamp_s, gx, gy, gz, ax, ay, az  (逗号或空格分隔)
// ===================================================================
IMURawData YamlIO::load_imu_csv(const std::string& csv_path) {
    UNICALIB_INFO("读取 IMU CSV: {}", csv_path);
    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        throw std::runtime_error("无法打开 IMU CSV 文件: " + csv_path);
    }

    IMURawData data;
    std::string line;
    bool header_skipped = false;
    int  skip_count = 0;

    while (std::getline(ifs, line)) {
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') continue;

        // 自动检测并跳过列标题行
        if (!header_skipped &&
            (line.find("timestamp") != std::string::npos ||
             line.find("time") != std::string::npos)) {
            header_skipped = true;
            continue;
        }
        header_skipped = true;

        // 替换逗号为空格
        for (char& c : line) if (c == ',') c = ' ';

        std::istringstream ss(line);
        IMURawFrame f;
        if (!(ss >> f.timestamp >> f.gyro[0] >> f.gyro[1] >> f.gyro[2]
                 >> f.accel[0] >> f.accel[1] >> f.accel[2])) {
            ++skip_count;
            continue;
        }
        data.push_back(f);
    }

    UNICALIB_INFO("  加载 {} 帧 (跳过 {} 行)",
                  data.size(), skip_count);

    if (data.size() > 1) {
        double dur = data.back().timestamp - data.front().timestamp;
        double dt  = dur / (data.size() - 1);
        UNICALIB_INFO("  时长: {:.2f} s, 平均采样率: {:.1f} Hz",
                      dur, 1.0/dt);
    }

    return data;
}

// ===================================================================
// 读取相机内参 YAML
// ===================================================================
CameraIntrinsics YamlIO::load_camera_intrinsics(const std::string& yaml_path) {
    UNICALIB_INFO("读取相机内参: {}", yaml_path);
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("YAML 解析失败: " + yaml_path + "\n" + e.what());
    }

    CameraIntrinsics I;
    I.width  = YAML_GET_OR(root, "width",  0);
    I.height = YAML_GET_OR(root, "height", 0);
    I.fx = YAML_GET_OR(root, "fx", 0.0);
    I.fy = YAML_GET_OR(root, "fy", 0.0);
    I.cx = YAML_GET_OR(root, "cx", 0.0);
    I.cy = YAML_GET_OR(root, "cy", 0.0);

    std::string model_str = YAML_GET_OR(root, "model", std::string("pinhole"));
    if (model_str == "fisheye") I.model = CameraIntrinsics::Model::FISHEYE;
    else                         I.model = CameraIntrinsics::Model::PINHOLE;

    if (root["dist_coeffs"]) {
        for (const auto& v : root["dist_coeffs"]) {
            I.dist_coeffs.push_back(v.as<double>());
        }
    }

    UNICALIB_INFO("  fx={:.2f} fy={:.2f} cx={:.2f} cy={:.2f} dist={}",
                  I.fx, I.fy, I.cx, I.cy, I.dist_coeffs.size());
    return I;
}

// ===================================================================
// 保存相机内参到 YAML
// ===================================================================
void YamlIO::save_camera_intrinsics(
    const CameraIntrinsics& intrin,
    const std::string& yaml_path) {

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "model"
        << YAML::Value
        << (intrin.model == CameraIntrinsics::Model::FISHEYE ? "fisheye" : "pinhole");
    out << YAML::Key << "width"  << YAML::Value << intrin.width;
    out << YAML::Key << "height" << YAML::Value << intrin.height;
    out << YAML::Key << "fx" << YAML::Value << intrin.fx;
    out << YAML::Key << "fy" << YAML::Value << intrin.fy;
    out << YAML::Key << "cx" << YAML::Value << intrin.cx;
    out << YAML::Key << "cy" << YAML::Value << intrin.cy;
    out << YAML::Key << "rms_reproj_error" << YAML::Value << intrin.rms_reproj_error;
    out << YAML::Key << "num_images_used"  << YAML::Value << intrin.num_images_used;

    if (!intrin.dist_coeffs.empty()) {
        out << YAML::Key << "dist_coeffs" << YAML::Value
            << YAML::Flow << intrin.dist_coeffs;
    }

    out << YAML::EndMap;

    std::ofstream ofs(yaml_path);
    if (!ofs.is_open())
        throw std::runtime_error("无法写入文件: " + yaml_path);
    ofs << out.c_str() << "\n";
    UNICALIB_INFO("相机内参已保存: {}", yaml_path);
}

// ===================================================================
// 保存 IMU 内参到 YAML
// ===================================================================
void YamlIO::save_imu_intrinsics(
    const IMUIntrinsics& intrin,
    const std::string& yaml_path) {

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "gyroscope_noise_density"
        << YAML::Value << intrin.noise_gyro;
    out << YAML::Key << "gyroscope_random_walk"
        << YAML::Value << intrin.bias_instab_gyro;
    out << YAML::Key << "accelerometer_noise_density"
        << YAML::Value << intrin.noise_acce;
    out << YAML::Key << "accelerometer_random_walk"
        << YAML::Value << intrin.bias_instab_acce;

    out << YAML::Key << "gyroscope_bias_init"
        << YAML::Value << YAML::Flow
        << std::vector<double>{intrin.bias_gyro[0], intrin.bias_gyro[1], intrin.bias_gyro[2]};
    out << YAML::Key << "accelerometer_bias_init"
        << YAML::Value << YAML::Flow
        << std::vector<double>{intrin.bias_acce[0], intrin.bias_acce[1], intrin.bias_acce[2]};

    out << YAML::Key << "num_samples_used" << YAML::Value << intrin.num_samples_used;
    out << YAML::Key << "allan_fit_rms"    << YAML::Value << intrin.allan_fit_rms;
    out << YAML::EndMap;

    std::ofstream ofs(yaml_path);
    if (!ofs.is_open())
        throw std::runtime_error("无法写入文件: " + yaml_path);
    ofs << out.c_str() << "\n";
    UNICALIB_INFO("IMU 内参已保存: {}", yaml_path);
}

}  // namespace ns_unicalib
