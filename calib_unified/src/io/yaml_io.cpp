/**
 * UniCalib Unified — YAML/CSV I/O 实现
 * 
 * 工程化改造:
 *   - 使用 Status/ErrorCode 替代 throw std::runtime_error
 *   - 统一错误处理和日志
 */

#include "unicalib/io/yaml_io.h"
#include "unicalib/common/logger.h"
#include "unicalib/common/exception.h"
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace ns_unicalib {

// YAML 安全读取：若 key 存在则 as<T>()，否则返回 default_val
#define YAML_GET_OR(node, key, default_val) \
    ((node)[key] ? (node)[key].as<std::decay_t<decltype(default_val)>>() : (default_val))

// ===================================================================
// 读取系统配置 (YAML)
// ===================================================================
SystemConfig YamlIO::load_system_config(const std::string& yaml_path) {
    UNICALIB_INFO("读取系统配置: {}", yaml_path);
    
    if (!fs::exists(yaml_path)) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_NOT_FOUND, 
                           "配置文件不存在: " + yaml_path);
    }
    
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::DATA_PARSE_ERROR,
                           "YAML 解析失败: " + yaml_path + "\n" + e.what());
    }
    
    SystemConfig cfg;
    cfg.config_file = yaml_path;
    
    // IMU 传感器列表 -> cfg.sensors (SensorDesc, type=IMU)
    if (root["imu_sensors"]) {
        for (const auto& imu : root["imu_sensors"]) {
            SensorDesc d;
            d.sensor_id = YAML_GET_OR(imu, "id", std::string(""));
            d.topic = YAML_GET_OR(imu, "topic", std::string(""));
            d.type = SensorType::IMU;
            d.imu_params = SensorDesc::IMUParams{};
            d.imu_params->rate_hz = YAML_GET_OR(imu, "rate_hz", 200.0);
            cfg.sensors.push_back(d);
        }
    }
    
    // LiDAR 传感器列表
    if (root["lidar_sensors"]) {
        for (const auto& lidar : root["lidar_sensors"]) {
            SensorDesc d;
            d.sensor_id = YAML_GET_OR(lidar, "id", std::string(""));
            d.topic = YAML_GET_OR(lidar, "topic", std::string(""));
            d.type = SensorType::LiDAR;
            d.lidar_params = SensorDesc::LiDARParams{};
            cfg.sensors.push_back(d);
        }
    }
    
    // 相机传感器列表
    if (root["camera_sensors"]) {
        for (const auto& cam : root["camera_sensors"]) {
            SensorDesc d;
            d.sensor_id = YAML_GET_OR(cam, "id", std::string(""));
            d.topic = YAML_GET_OR(cam, "topic", std::string(""));
            d.type = SensorType::CAMERA;
            d.camera_params = SensorDesc::CameraParams{};
            cfg.sensors.push_back(d);
        }
    }
    
    size_t n_imu = 0, n_lidar = 0, n_cam = 0;
    for (const auto& s : cfg.sensors) {
        if (s.type == SensorType::IMU) ++n_imu;
        else if (s.type == SensorType::LiDAR) ++n_lidar;
        else if (s.type == SensorType::CAMERA) ++n_cam;
    }
    UNICALIB_INFO("  IMU: {}, LiDAR: {}, Camera: {}",
                  n_imu, n_lidar, n_cam);
    return cfg;
}

// ===================================================================
// 保存标定结果 (YAML)
// ===================================================================
void YamlIO::save_calib_results(
    const CalibParamManager& params,
    const std::string& yaml_path) {
    
    params.save_yaml(yaml_path);
}

// ===================================================================
// IMU CSV 数据加载
// 格式: timestamp_s, gx, gy, gz, ax, ay, az (逗号或空格分隔)
// ===================================================================
IMURawData YamlIO::load_imu_csv(const std::string& csv_path) {
    UNICALIB_INFO("读取 IMU CSV: {}", csv_path);
    
    if (!fs::exists(csv_path)) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_NOT_FOUND,
                           "IMU CSV 文件不存在: " + csv_path);
    }
    
    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_READ_ERROR,
                           "无法打开 IMU CSV 文件: " + csv_path);
    }
    
    IMURawData data;
    std::string line;
    bool header_skipped = false;
    int skip_count = 0;
    
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
    
    if (!fs::exists(yaml_path)) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_NOT_FOUND,
                           "相机内参文件不存在: " + yaml_path);
    }
    
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const YAML::Exception& e) {
        UNICALIB_THROW_DATA(ErrorCode::DATA_PARSE_ERROR,
                           "YAML 解析失败: " + yaml_path + "\n" + e.what());
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
    
    // 确保目录存在
    fs::path p(yaml_path);
    if (p.has_parent_path() && !fs::exists(p.parent_path())) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
        if (ec) {
            UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                               "无法创建目录: " + p.parent_path().string());
        }
    }
    
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
    if (!ofs.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                           "无法写入文件: " + yaml_path);
    }
    ofs << out.c_str() << "\n";
    UNICALIB_INFO("相机内参已保存: {}", yaml_path);
}

// ===================================================================
// 保存 IMU 内参到 YAML (Kalibr / iKalibr 兼容格式)
// ===================================================================
void YamlIO::save_imu_intrinsics(
    const IMUIntrinsics& intrin,
    const std::string& yaml_path) {
    
    // 确保目录存在
    fs::path p(yaml_path);
    if (p.has_parent_path() && !fs::exists(p.parent_path())) {
        std::error_code ec;
        fs::create_directories(p.parent_path(), ec);
        if (ec) {
            UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                               "无法创建目录: " + p.parent_path().string());
        }
    }
    
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
    if (!ofs.is_open()) {
        UNICALIB_THROW_DATA(ErrorCode::FILE_WRITE_ERROR,
                           "无法写入文件: " + yaml_path);
    }
    ofs << out.c_str() << "\n";
    UNICALIB_INFO("IMU 内参已保存: {}", yaml_path);
}

}  // namespace ns_unicalib
