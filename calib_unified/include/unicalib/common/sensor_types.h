#pragma once
/**
 * UniCalib Unified — 传感器类型与基础配置
 * 统一枚举: SensorType / LidarType / CameraModel / CalibType
 * 支持 magic_enum 反射
 */

#include <string>
#include <vector>
#include <optional>
#include <memory>
#include <map>

namespace ns_unicalib {

// ===================================================================
// 传感器类型枚举
// ===================================================================
enum class SensorType {
    IMU     = 0,    // 惯性测量单元
    LiDAR   = 1,    // 激光雷达 (旋转/固态)
    CAMERA  = 2,    // 相机 (针孔/鱼眼/畸变)
    RADAR   = 3,    // 毫米波雷达
    RGBD    = 4,    // RGB-D 深度相机
    EVENT   = 5,    // 事件相机
};

enum class LidarType {
    SPINNING,       // 旋转式: Velodyne / Ouster / Robosense
    SOLID_STATE,    // 固态: Livox Avia/Mid360/HAP
};

enum class CameraModel {
    PINHOLE,        // 针孔 + 径向/切向畸变 (OpenCV)
    FISHEYE,        // 鱼眼等距 (kb4 model, OpenCV fisheye)
    OMNI,           // 全向 (MEI model)
};

enum class IMUModel {
    SCALE_MISALIGNMENT,         // 比例因子 + 交轴误差 + 偏置
    SCALE_ONLY,                 // 仅偏置 + 比例
    BIAS_ONLY,                  // 仅偏置
};

// ===================================================================
// 标定类型
// ===================================================================
enum class CalibType {
    IMU_INTRINSIC           = 0,    // IMU 内参 (噪声/偏置/比例/交轴)
    CAMERA_INTRINSIC        = 1,    // 相机内参 (焦距/主点/畸变)
    IMU_LIDAR_EXTRINSIC     = 2,    // IMU-LiDAR 外参 + 时间偏移
    LIDAR_CAMERA_EXTRINSIC  = 3,    // LiDAR-Camera 外参 + 时间偏移
    CAM_CAM_EXTRINSIC       = 4,    // Camera-Camera 外参
    JOINT                   = 5,    // 联合标定 (多传感器同时)
};

// ===================================================================
// 传感器配置描述符
// ===================================================================
struct SensorDesc {
    std::string sensor_id;      // 唯一 ID, e.g. "imu_0", "lidar_front"
    SensorType  type;

    // 可选: ROS topic (若使用 ROS bag 读取)
    std::string topic;

    // 可选: 相机特定参数
    struct CameraParams {
        CameraModel model = CameraModel::PINHOLE;
        int width  = 0;
        int height = 0;
        double fps = 30.0;
        bool is_rolling_shutter = false;
    };
    std::optional<CameraParams> camera_params;

    // 可选: IMU 特定参数
    struct IMUParams {
        double rate_hz = 200.0;
        IMUModel model = IMUModel::SCALE_MISALIGNMENT;
    };
    std::optional<IMUParams> imu_params;

    // 可选: LiDAR 特定参数
    struct LiDARParams {
        LidarType lidar_type = LidarType::SPINNING;
        int scan_lines = 16;
        double rate_hz = 10.0;
    };
    std::optional<LiDARParams> lidar_params;

    bool is_imu()    const { return type == SensorType::IMU; }
    bool is_lidar()  const { return type == SensorType::LiDAR; }
    bool is_camera() const { return type == SensorType::CAMERA; }
    bool is_radar()  const { return type == SensorType::RADAR; }
};

// ===================================================================
// 标定对 — 描述两个传感器之间要标定的关系
// ===================================================================
struct CalibPair {
    std::string reference_sensor;   // 参考传感器 ID (通常是 IMU)
    std::string target_sensor;      // 待标定传感器 ID
    CalibType   calib_type;
    int         priority = 0;       // 执行优先级 (越大越先)

    // 针对联合标定的额外配置
    double initial_time_offset_s = 0.0;     // 初始时间偏移估计
    bool   fix_extrinsic = false;           // 是否固定外参（仅优化其他量）
    bool   fix_time_offset = false;         // 是否固定时间偏移
};

// ===================================================================
// 多传感器系统配置
// ===================================================================
struct SystemConfig {
    std::string config_file;        // 来源配置文件路径
    std::string data_dir;           // 数据目录
    std::string output_dir;         // 输出目录
    std::string bag_file;           // ROS bag 路径 (可选)

    std::string reference_imu;      // 参考 IMU ID (Br)

    double calib_time_window_s = 60.0;  // 标定数据时间窗口
    double gravity_magnitude = 9.81;    // 重力加速度

    std::vector<SensorDesc> sensors;
    std::vector<CalibPair>  calib_pairs;

    // 按 ID 查找传感器
    const SensorDesc* find_sensor(const std::string& id) const;
    std::vector<const SensorDesc*> sensors_of_type(SensorType t) const;
};

// ===================================================================
// 工具函数 — string 转换
// ===================================================================
std::string sensor_type_to_str(SensorType t);
SensorType  sensor_type_from_str(const std::string& s);
std::string calib_type_to_str(CalibType t);
std::string lidar_type_to_str(LidarType t);
std::string camera_model_to_str(CameraModel m);

}  // namespace ns_unicalib
