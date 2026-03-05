/**
 * UniCalib Unified — ROS2 数据源实现
 * 
 * 实现两种模式:
 *   1. ROS2 bag 文件读取 (使用 rosbag2_cpp)
 *   2. ROS2 实时话题订阅 (使用 rclcpp)
 */

#include "unicalib/io/ros2_data_source.h"
#include "unicalib/common/logger.h"
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <chrono>
#include <thread>

#ifdef UNICALIB_WITH_ROS2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#endif

namespace fs = std::filesystem;

namespace ns_unicalib {

// ===========================================================================
// Ros2BagDataSource 实现
// ===========================================================================

Ros2BagDataSource::Ros2BagDataSource(const RosDataSourceConfig& cfg)
    : RosDataSourceBase(cfg) {
    status_msg_ = "未初始化";
}

Ros2BagDataSource::~Ros2BagDataSource() {
    bag_reader_.reset();
}

double Ros2BagDataSource::stamp_to_sec(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

bool Ros2BagDataSource::read_metadata_yaml(const std::string& bag_file) {
    // 尝试在同目录下查找 metadata.yaml
    fs::path bag_path(bag_file);
    fs::path metadata_path = bag_path.parent_path() / "metadata.yaml";
    
    if (!fs::exists(metadata_path)) {
        UNICALIB_INFO("[Ros2BagDataSource] 未找到 metadata.yaml: {}", metadata_path.string());
        return false;
    }
    
    UNICALIB_INFO("[Ros2BagDataSource] 正在读取 metadata.yaml: {}", metadata_path.string());
    
    try {
        YAML::Node root = YAML::LoadFile(metadata_path.string());
        
        // 查找 topics_with_message_count 节
        if (!root["topics_with_message_count"]) {
            UNICALIB_WARN("[Ros2BagDataSource] metadata.yaml 中缺少 topics_with_message_count 节");
            return false;
        }
        
        // 收集所有话题及其消息数量
        std::map<std::string, size_t> topic_msg_counts;
        for (const auto& topic_node : root["topics_with_message_count"]) {
            std::string topic_name = topic_node["name"].as<std::string>();
            size_t msg_count = topic_node["message_count"].as<size_t>();
            topic_msg_counts[topic_name] = msg_count;
            
            UNICALIB_DEBUG("[Ros2BagDataSource] 找到话题: {} (消息数: {})", 
                         topic_name, msg_count);
        }
        
        // 自动检测话题类型
        auto_detect_topics(topic_msg_counts);
        
        return true;
        
    } catch (const std::exception& e) {
        UNICALIB_ERROR("[Ros2BagDataSource] 读取 metadata.yaml 失败: {}", e.what());
        return false;
    }
}

void Ros2BagDataSource::auto_detect_topics(
    const std::map<std::string, size_t>& topic_msg_counts) {
    
    // 1) 优先使用配置中的单话题（来自配置文件 sensors[].topic 或 ros2.lidar_topic 等）
    if (!cfg_.lidar_ros2_topic.empty()) {
        topic_mapping_.lidar_topic = cfg_.lidar_ros2_topic;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 LiDAR 话题: {}", cfg_.lidar_ros2_topic);
    } else if (!cfg_.lidar_topics.empty()) {
        topic_mapping_.lidar_topic = cfg_.lidar_topics.begin()->second;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 LiDAR 话题 (来自 sensors): {}", topic_mapping_.lidar_topic);
    } else {
        std::string detected_lidar = find_best_lidar_topic(topic_msg_counts);
        if (!detected_lidar.empty()) {
            topic_mapping_.lidar_topic = detected_lidar;
            UNICALIB_INFO("[Ros2BagDataSource] 自动检测 LiDAR 话题: {}", detected_lidar);
        }
    }
    
    if (!cfg_.camera_ros2_topic.empty()) {
        topic_mapping_.camera_topic = cfg_.camera_ros2_topic;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的相机话题: {}", cfg_.camera_ros2_topic);
    } else if (!cfg_.camera_topics.empty()) {
        topic_mapping_.camera_topic = cfg_.camera_topics.begin()->second;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的相机话题 (来自 sensors): {}", topic_mapping_.camera_topic);
    } else {
        std::string detected_camera = find_best_camera_topic(topic_msg_counts);
        if (!detected_camera.empty()) {
            topic_mapping_.camera_topic = detected_camera;
            UNICALIB_INFO("[Ros2BagDataSource] 自动检测相机话题: {}", detected_camera);
        }
    }
    
    if (!cfg_.imu_ros2_topic.empty()) {
        topic_mapping_.imu_topic = cfg_.imu_ros2_topic;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 IMU 话题: {}", cfg_.imu_ros2_topic);
    } else if (!cfg_.imu_topics.empty()) {
        topic_mapping_.imu_topic = cfg_.imu_topics.begin()->second;
        UNICALIB_INFO("[Ros2BagDataSource] 使用配置的 IMU 话题 (来自 sensors): {}", topic_mapping_.imu_topic);
    } else {
        std::string detected_imu = find_best_imu_topic(topic_msg_counts);
        if (!detected_imu.empty()) {
            topic_mapping_.imu_topic = detected_imu;
            UNICALIB_INFO("[Ros2BagDataSource] 自动检测 IMU 话题: {}", detected_imu);
        }
    }
}

std::string Ros2BagDataSource::find_best_lidar_topic(
    const std::map<std::string, size_t>& topic_msg_counts) {
    
    // LiDAR 话题关键词
    std::vector<std::string> lidar_keywords = {
        "pointcloud", "lidar", "velodyne", "ouster", "livox", "pandar",
        "points", "scan", "os_cloud"
    };
    
    // 优先级关键词（更具体）
    std::vector<std::string> priority_keywords = {
        "velodyne_points", "os_cloud_node1/points", "os_cloud_node2/points",
        "ouster/points", "livox/lidar"
    };
    
    // 查找匹配的话题
    for (const auto& [topic, count] : topic_msg_counts) {
        std::string topic_lower = topic;
        std::transform(topic_lower.begin(), topic_lower.end(), topic_lower.begin(), ::tolower);
        
        // 首先检查优先级关键词（更准确）
        for (const auto& keyword : priority_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配高优先级 LiDAR 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
        
        // 然后检查通用 LiDAR 关键词
        for (const auto& keyword : lidar_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配 LiDAR 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
    }
    
    // 如果没有找到精确匹配，返回空
    return "";
}

std::string Ros2BagDataSource::find_best_camera_topic(
    const std::map<std::string, size_t>& topic_msg_counts) {
    
    // 相机话题关键词
    std::vector<std::string> camera_keywords = {
        "image", "camera", "left", "right", "front", "rear",
        "rgb", "ir", "mono", "stereo", "color"
    };
    
    // 优先级关键词（更具体）
    std::vector<std::string> priority_keywords = {
        "left/image_raw", "right/image_raw",
        "/cam_left/image_raw", "/cam_right/image_raw",
        "left/image_raw/compressed", "right/image_raw/compressed"
    };
    
    // 查找匹配的话题
    for (const auto& [topic, count] : topic_msg_counts) {
        std::string topic_lower = topic;
        std::transform(topic_lower.begin(), topic_lower.end(), topic_lower.begin(), ::tolower);
        
        // 首先检查优先级关键词（更准确）
        for (const auto& keyword : priority_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                // 优先选择包含 "left" 的相机话题（假设为左相机）
                if (topic_lower.find("left") != std::string::npos) {
                    UNICALIB_DEBUG("[Ros2BagDataSource] 匹配高优先级左相机话题: {} (关键词: {})", 
                                 topic, keyword);
                    return topic;
                }
            }
        }
        
        // 然后检查通用相机关键词
        for (const auto& keyword : camera_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配相机话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
    }
    
    return "";
}

std::string Ros2BagDataSource::find_best_imu_topic(
    const std::map<std::string, size_t>& topic_msg_counts) {
    
    // IMU 话题关键词
    std::vector<std::string> imu_keywords = {
        "imu", "gyro", "accel", "magnetic", "angular", "linear",
        "attitude", "orientation", "rotation"
    };
    
    // 优先级关键词（常见 IMU 话题名称）
    std::vector<std::string> priority_keywords = {
        "/imu/data", "/dji_sdk/imu", "/imu/imu",
        "/imu/magnetic_field", "/imu/temperature"
    };
    
    // 查找匹配的话题
    for (const auto& [topic, count] : topic_msg_counts) {
        std::string topic_lower = topic;
        std::transform(topic_lower.begin(), topic_lower.end(), topic_lower.begin(), ::tolower);
        
        // 首先检查优先级关键词（更准确）
        for (const auto& keyword : priority_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配高优先级 IMU 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
        
        // 然后检查通用 IMU 关键词
        for (const auto& keyword : imu_keywords) {
            if (topic_lower.find(keyword) != std::string::npos) {
                UNICALIB_DEBUG("[Ros2BagDataSource] 匹配 IMU 话题: {} (关键词: {})", 
                             topic, keyword);
                return topic;
            }
        }
    }
    
    return "";
}

LiDARScanRos Ros2BagDataSource::parse_point_cloud2(
    const std::shared_ptr<sensor_msgs::msg::PointCloud2>& msg) {
    
    LiDARScanRos scan;
    scan.timestamp = stamp_to_sec(msg->header.stamp);
    
    // 转换为 PCL 点云
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    
    // 过滤点
    pcl::PointCloud<pcl::PointXYZI> filtered;
    filtered.reserve(cloud->size());
    
    for (const auto& pt : cloud->points) {
        // 过滤 NaN
        if (cfg_.filter_nan && (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))) {
            continue;
        }
        
        // 距离过滤
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (dist < cfg_.min_range || dist > cfg_.max_range) {
            continue;
        }
        
        filtered.push_back(pt);
    }
    
    scan.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>(filtered));
    
    return scan;
}

CameraFrameRos Ros2BagDataSource::parse_image(
    const std::shared_ptr<sensor_msgs::msg::Image>& msg) {
    
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        return CameraFrameRos(stamp_to_sec(msg->header.stamp), cv_ptr->image);
    } catch (const cv_bridge::Exception& e) {
        UNICALIB_WARN("图像转换失败: {}", e.what());
        return CameraFrameRos(stamp_to_sec(msg->header.stamp), cv::Mat());
    }
}

IMUFrameRos Ros2BagDataSource::parse_imu(
    const std::shared_ptr<sensor_msgs::msg::Imu>& msg) {
    
    IMUFrameRos frame;
    frame.timestamp = stamp_to_sec(msg->header.stamp);
    
    if (msg->angular_velocity.x == msg->angular_velocity.x &&
        msg->linear_acceleration.x == msg->linear_acceleration.x) {
        // 检查是否为 NaN
        frame.gyro[0] = 0.0; frame.gyro[1] = 0.0; frame.gyro[2] = 0.0;
        frame.accel[0] = 0.0; frame.accel[1] = 0.0; frame.accel[2] = 0.0;
    } else {
        frame.gyro[0] = msg->angular_velocity.x;
        frame.gyro[1] = msg->angular_velocity.y;
        frame.gyro[2] = msg->angular_velocity.z;
        frame.accel[0] = msg->linear_acceleration.x;
        frame.accel[1] = msg->linear_acceleration.y;
        frame.accel[2] = msg->linear_acceleration.z;
    }
    
    return frame;
}

bool Ros2BagDataSource::load() {
    status_msg_ = "正在加载 bag...";
    
    if (cfg_.bag_file.empty()) {
        status_msg_ = "未指定 bag 文件";
        return false;
    }
    
    if (!fs::exists(cfg_.bag_file)) {
        status_msg_ = "bag 文件不存在: " + cfg_.bag_file;
        return false;
    }
    
    UNICALIB_INFO("[Ros2BagDataSource] 打开 bag 文件: {}", cfg_.bag_file);
    
    // ─── 步骤 1: 从 metadata.yaml 自动检测话题 ─────────
    if (!read_metadata_yaml(cfg_.bag_file)) {
        status_msg_ = "读取 metadata.yaml 失败";
        return false;
    }
    
    // ─── 步骤 2: 打开 bag 文件 ────────────────────────────────────────
    try {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = cfg_.bag_file;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        bag_reader_ = std::make_unique<rosbag2_cpp::SequentialReader>();
        bag_reader_->open(storage_options, converter_options);
        
        UNICALIB_INFO("[Ros2BagDataSource] Bag 已打开，开始读取消息...");
        
        // 获取所有话题信息
        const auto& topics = bag_reader_->get_all_topics_and_types();
        UNICALIB_INFO("[Ros2BagDataSource] 找到 {} 个话题", topics.size());
        for (const auto& [topic, type] : topics) {
            UNICALIB_DEBUG("  话题: {} (类型: {})", topic, type);
        }
        
        // 读取消息
        bool time_window_set = (cfg_.time_window_end > cfg_.time_window_start);
        double last_time = 0.0;
        
        while (bag_reader_->has_next()) {
            auto msg = bag_reader_->read_next();
            double msg_time = stamp_to_sec(msg->get_time());
            
            // 时间窗口过滤
            if (time_window_set) {
                if (msg_time < cfg_.time_window_start) continue;
                if (msg_time > cfg_.time_window_end) break;
            }
            
                last_time = std::max(last_time, msg_time);
            
            // 判断话题类型
            std::string topic = msg->get_topic_name();
            
            // LiDAR 数据
            for (const auto& [sensor_id, lidar_topic] : cfg_.lidar_topics) {
                if (topic == lidar_topic) {
                    auto cloud_msg = msg->instantiate<sensor_msgs::msg::PointCloud2>();
                    if (cloud_msg) {
                        lidar_data_[sensor_id].push_back(parse_point_cloud2(cloud_msg));
                        UNICALIB_TRACE("  收集 LiDAR {} 帧数: {}", 
                                     sensor_id, lidar_data_[sensor_id].size());
                    }
                }
            }
            
            // 相机数据
            for (const auto& [sensor_id, camera_topic] : cfg_.camera_topics) {
                if (topic == camera_topic) {
                    auto image_msg = msg->instantiate<sensor_msgs::msg::Image>();
                    if (image_msg) {
                        camera_data_[sensor_id].push_back(parse_image(image_msg));
                        UNICALIB_TRACE("  收集相机 {} 帧数: {}", 
                                     sensor_id, camera_data_[sensor_id].size());
                    }
                }
            }
            
            // IMU 数据
            for (const auto& [sensor_id, imu_topic] : cfg_.imu_topics) {
                if (topic == imu_topic) {
                    auto imu_msg = msg->instantiate<sensor_msgs::msg::Imu>();
                    if (imu_msg) {
                        imu_data_[sensor_id].push_back(parse_imu(imu_msg));
                        UNICALIB_TRACE("  收集 IMU {} 帧数: {}", 
                                     sensor_id, imu_data_[sensor_id].size());
                    }
                }
            }
        }
        
        // 记录时间范围
        if (!lidar_data_.empty()) {
            const auto& scans = lidar_data_.begin()->second;
            if (!scans.empty()) {
                start_time_ = scans.front().timestamp;
                end_time_ = scans.back().timestamp;
            }
        }
        
        UNICALIB_INFO("[Ros2BagDataSource] 加载完成:");
        for (const auto& [sensor_id, data] : lidar_data_) {
            UNICALIB_INFO("  LiDAR {}: {} 帧", sensor_id, data.size());
        }
        for (const auto& [sensor_id, data] : camera_data_) {
            UNICALIB_INFO("  相机 {}: {} 帧", sensor_id, data.size());
        }
        for (const auto& [sensor_id, data] : imu_data_) {
            UNICALIB_INFO("  IMU {}: {} 帧", sensor_id, data.size());
        }
        UNICALIB_INFO("  时间范围: {:.3f} - {:.3f} 秒", start_time_, end_time_);
        
        // 采样和帧数限制
        if (cfg_.max_frames > 0 || cfg_.sample_interval > 0.0) {
            UNICALIB_INFO("[Ros2BagDataSource] 应用采样/帧数限制...");
            for (auto& [sensor_id, data] : lidar_data_) {
                if (cfg_.max_frames > 0 && data.size() > cfg_.max_frames) {
                    // 均匀采样
                    size_t step = data.size() / cfg_.max_frames;
                    std::vector<LiDARScanRos> sampled;
                    for (size_t i = 0; i < data.size() && sampled.size() < cfg_.max_frames; i += step) {
                        sampled.push_back(data[i]);
                    }
                    data = std::move(sampled);
                    UNICALIB_INFO("  LiDAR {} 采样后: {} 帧", sensor_id, data.size());
                }
            }
            for (auto& [sensor_id, data] : camera_data_) {
                if (cfg_.max_frames > 0 && data.size() > cfg_.max_frames) {
                    size_t step = data.size() / cfg_.max_frames;
                    std::vector<CameraFrameRos> sampled;
                    for (size_t i = 0; i < data.size() && sampled.size() < cfg_.max_frames; i += step) {
                        sampled.push_back(data[i]);
                    }
                    data = std::move(sampled);
                    UNICALIB_INFO("  相机 {} 采样后: {} 帧", sensor_id, data.size());
                }
            }
        }
        
        loaded_ = true;
        status_msg_ = "加载成功";
        return true;
        
    } catch (const std::exception& e) {
        status_msg_ = "加载失败: " + std::string(e.what());
        UNICALIB_ERROR("[Ros2BagDataSource] {}", status_msg_);
        return false;
    }
}

std::vector<LiDARScanRos> Ros2BagDataSource::get_lidar_scans(
    const std::string& sensor_id) const {
    auto it = lidar_data_.find(sensor_id);
    return (it != lidar_data_.end()) ? it->second : std::vector<LiDARScanRos>();
}

std::vector<CameraFrameRos> Ros2BagDataSource::get_camera_frames(
    const std::string& sensor_id) const {
    auto it = camera_data_.find(sensor_id);
    return (it != camera_data_.end()) ? it->second : std::vector<CameraFrameRos>();
}

std::vector<IMUFrameRos> Ros2BagDataSource::get_imu_frames(
    const std::string& sensor_id) const {
    auto it = imu_data_.find(sensor_id);
    return (it != imu_data_.end()) ? it->second : std::vector<IMUFrameRos>();
}

std::vector<std::string> Ros2BagDataSource::get_lidar_ids() const {
    std::vector<std::string> ids;
    for (const auto& [id, _] : lidar_data_) ids.push_back(id);
    return ids;
}

std::vector<std::string> Ros2BagDataSource::get_camera_ids() const {
    std::vector<std::string> ids;
    for (const auto& [id, _] : camera_data_) ids.push_back(id);
    return ids;
}

std::vector<std::string> Ros2BagDataSource::get_imu_ids() const {
    std::vector<std::string> ids;
    for (const auto& [id, _] : imu_data_) ids.push_back(id);
    return ids;
}

// ===========================================================================
// Ros2RealtimeDataSource 实现
// ===========================================================================

Ros2RealtimeDataSource::Ros2RealtimeDataSource(const RosDataSourceConfig& cfg)
    : RosDataSourceBase(cfg) {
    status_msg_ = "未启动";
}

Ros2RealtimeDataSource::~Ros2RealtimeDataSource() {
    stop();
}

bool Ros2RealtimeDataSource::start() {
    if (running_) {
        UNICALIB_WARN("[Ros2RealtimeDataSource] 已在运行中");
        return true;
    }
    
    UNICALIB_INFO("[Ros2RealtimeDataSource] 启动 ROS2 节点...");
    
    try {
        // 创建 ROS2 节点
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("unicalib_data_loader");
        
        // 创建订阅者
        for (const auto& [sensor_id, topic] : cfg_.lidar_topics) {
            auto sub = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
                topic, 10,
                [this, sensor_id](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    this->lidar_callback(sensor_id, msg);
                });
            subscriptions_[sensor_id] = sub;
            UNICALIB_INFO("  订阅 LiDAR {}: {}", sensor_id, topic);
        }
        
        for (const auto& [sensor_id, topic] : cfg_.camera_topics) {
            auto sub = node_->create_subscription<sensor_msgs::msg::Image>(
                topic, 10,
                [this, sensor_id](const sensor_msgs::msg::Image::SharedPtr msg) {
                    this->camera_callback(sensor_id, msg);
                });
            subscriptions_[sensor_id] = sub;
            UNICALIB_INFO("  订阅相机 {}: {}", sensor_id, topic);
        }
        
        for (const auto& [sensor_id, topic] : cfg_.imu_topics) {
            auto sub = node_->create_subscription<sensor_msgs::msg::Imu>(
                topic, 100,
                [this, sensor_id](const sensor_msgs::msg::Imu::SharedPtr msg) {
                    this->imu_callback(sensor_id, msg);
                });
            subscriptions_[sensor_id] = sub;
            UNICALIB_INFO("  订阅 IMU {}: {}", sensor_id, topic);
        }
        
        // 创建执行器
        executor_ = std::make_unique<rclcpp::exec::SingleThreadedExecutor>();
        executor_->add_node(node_);
        
        // 在后台线程中运行
        stop_requested_ = false;
        std::thread exec_thread([this]() {
            while (rclcpp::ok() && !stop_requested_) {
                executor_->spin_once(std::chrono::milliseconds(10));
            }
        });
        exec_thread.detach();
        
        running_ = true;
        status_msg_ = "运行中";
        UNICALIB_INFO("[Ros2RealtimeDataSource] 已启动，等待数据...");
        return true;
        
    } catch (const std::exception& e) {
        status_msg_ = "启动失败: " + std::string(e.what());
        UNICALIB_ERROR("[Ros2RealtimeDataSource] {}", status_msg_);
        return false;
    }
}

void Ros2RealtimeDataSource::stop() {
    if (!running_) return;
    
    UNICALIB_INFO("[Ros2RealtimeDataSource] 停止...");
    stop_requested_ = true;
    
    // 清理订阅
    subscriptions_.clear();
    
    // 清理执行器和节点
    executor_.reset();
    node_.reset();
    
    // 清理 ROS2
    rclcpp::shutdown();
    
    running_ = false;
    status_msg_ = "已停止";
}

bool Ros2RealtimeDataSource::load() {
    return start();
}

bool Ros2RealtimeDataSource::wait_for_data(double timeout_sec) {
    UNICALIB_INFO("[Ros2RealtimeDataSource] 等待数据就绪 (超时: {:.1f} 秒)...", 
                  timeout_sec);
    
    auto start = std::chrono::steady_clock::now();
    
    while (running_ && !stop_requested_) {
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            
            // 检查是否已收集足够数据
            bool has_lidar = !cfg_.lidar_topics.empty();
            bool has_camera = !cfg_.camera_topics.empty();
            bool has_imu = !cfg_.imu_topics.empty();
            
            for (const auto& [sensor_id, _] : cfg_.lidar_topics) {
                if (!lidar_data_[sensor_id].empty()) has_lidar = true;
            }
            for (const auto& [sensor_id, _] : cfg_.camera_topics) {
                if (!camera_data_[sensor_id].empty()) has_camera = true;
            }
            for (const auto& [sensor_id, _] : cfg_.imu_topics) {
                if (!imu_data_[sensor_id].empty()) has_imu = true;
            }
            
            if ((has_lidar || cfg_.lidar_topics.empty()) &&
                (has_camera || cfg_.camera_topics.empty()) &&
                (has_imu || cfg_.imu_topics.empty())) {
                UNICALIB_INFO("[Ros2RealtimeDataSource] 数据已就绪");
                data_cv_.notify_all();
                return true;
            }
        }
        
        // 等待数据到达或超时
        std::unique_lock<std::mutex> lock(data_mutex_);
        if (data_cv_.wait_for(lock, std::chrono::milliseconds(100)) == 
            std::cv_status::timeout) {
            // 检查超时
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(
                now - start).count();
            if (elapsed > timeout_sec) {
                UNICALIB_WARN("[Ros2RealtimeDataSource] 等待超时");
                status_msg_ = "等待超时";
                return false;
            }
        }
    }
    
    return running_;
}

LiDARScanRos Ros2RealtimeDataSource::convert_point_cloud2(
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    
    LiDARScanRos scan;
    scan.timestamp = stamp_to_sec(msg->header.stamp);
    
    // 转换为 PCL 点云
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    
    // 过滤点
    pcl::PointCloud<pcl::PointXYZI> filtered;
    filtered.reserve(cloud->size());
    
    for (const auto& pt : cloud->points) {
        if (cfg_.filter_nan && (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z))) {
            continue;
        }
        
        double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
        if (dist < cfg_.min_range || dist > cfg_.max_range) {
            continue;
        }
        
        filtered.push_back(pt);
    }
    
    scan.cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>(filtered));
    
    return scan;
}

void Ros2RealtimeDataSource::lidar_callback(
    const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    
    auto scan = convert_point_cloud2(msg);
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        lidar_data_[sensor_id].push_back(scan);
        lidar_queues_[sensor_id].push(scan);
        
        // 限制队列大小
        while (lidar_queues_[sensor_id].size() > 1000) {
            lidar_queues_[sensor_id].pop();
        }
        
        UNICALIB_DEBUG("[LidarCallback] 收集 LiDAR {} 数据, 帧数: {}", 
                      sensor_id, lidar_data_[sensor_id].size());
    }
    
    data_cv_.notify_all();
    
    if (data_callback_) {
        data_callback_(sensor_id);
    }
}

void Ros2RealtimeDataSource::camera_callback(
    const std::string& sensor_id,
    const sensor_msgs::msg::Image::SharedPtr msg) {
    
    CameraFrameRos frame;
    frame.timestamp = stamp_to_sec(msg->header.stamp);
    
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        frame.image = cv_ptr->image.clone();
    } catch (const cv_bridge::Exception& e) {
        UNICALIB_WARN("图像转换失败: {}", e.what());
        frame.image = cv::Mat();
    }
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        camera_data_[sensor_id].push_back(frame);
        camera_queues_[sensor_id].push(frame);
        
        while (camera_queues_[sensor_id].size() > 1000) {
            camera_queues_[sensor_id].pop();
        }
        
        UNICALIB_DEBUG("[CameraCallback] 收集相机 {} 数据, 帧数: {}", 
                      sensor_id, camera_data_[sensor_id].size());
    }
    
    data_cv_.notify_all();
    
    if (data_callback_) {
        data_callback_(sensor_id);
    }
}

void Ros2RealtimeDataSource::imu_callback(
    const std::string& sensor_id,
    const sensor_msgs::msg::Imu::SharedPtr msg) {
    
    IMUFrameRos frame;
    frame.timestamp = stamp_to_sec(msg->header.stamp);
    
    frame.gyro[0] = msg->angular_velocity.x;
    frame.gyro[1] = msg->angular_velocity.y;
    frame.gyro[2] = msg->angular_velocity.z;
    frame.accel[0] = msg->linear_acceleration.x;
    frame.accel[1] = msg->linear_acceleration.y;
    frame.accel[2] = msg->linear_acceleration.z;
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        imu_data_[sensor_id].push_back(frame);
        imu_queues_[sensor_id].push(frame);
        
        while (imu_queues_[sensor_id].size() > 10000) {
            imu_queues_[sensor_id].pop();
        }
        
        UNICALIB_DEBUG("[IMUCallback] 收集 IMU {} 数据, 帧数: {}", 
                      sensor_id, imu_data_[sensor_id].size());
    }
    
    data_cv_.notify_all();
    
    if (data_callback_) {
        data_callback_(sensor_id);
    }
}

std::vector<LiDARScanRos> Ros2RealtimeDataSource::get_lidar_scans(
    const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = lidar_data_.find(sensor_id);
    return (it != lidar_data_.end()) ? it->second : std::vector<LiDARScanRos>();
}

std::vector<CameraFrameRos> Ros2RealtimeDataSource::get_camera_frames(
    const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = camera_data_.find(sensor_id);
    return (it != camera_data_.end()) ? it->second : std::vector<CameraFrameRos>();
}

std::vector<IMUFrameRos> Ros2RealtimeDataSource::get_imu_frames(
    const std::string& sensor_id) const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    auto it = imu_data_.find(sensor_id);
    return (it != imu_data_.end()) ? it->second : std::vector<IMUFrameRos>();
}

std::vector<std::string> Ros2RealtimeDataSource::get_lidar_ids() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<std::string> ids;
    for (const auto& [id, _] : lidar_data_) ids.push_back(id);
    return ids;
}

std::vector<std::string> Ros2RealtimeDataSource::get_camera_ids() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<std::string> ids;
    for (const auto& [id, _] : camera_data_) ids.push_back(id);
    return ids;
}

std::vector<std::string> Ros2RealtimeDataSource::get_imu_ids() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<std::string> ids;
    for (const auto& [id, _] : imu_data_) ids.push_back(id);
    return ids;
}

double Ros2RealtimeDataSource::get_start_time() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    double min_time = 1e9;
    for (const auto& [_, data] : lidar_data_) {
        if (!data.empty()) min_time = std::min(min_time, data.front().timestamp);
    }
    for (const auto& [_, data] : camera_data_) {
        if (!data.empty()) min_time = std::min(min_time, data.front().timestamp);
    }
    return (min_time < 1e9) ? min_time : 0.0;
}

double Ros2RealtimeDataSource::get_end_time() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    double max_time = 0.0;
    for (const auto& [_, data] : lidar_data_) {
        if (!data.empty()) max_time = std::max(max_time, data.back().timestamp);
    }
    for (const auto& [_, data] : camera_data_) {
        if (!data.empty()) max_time = std::max(max_time, data.back().timestamp);
    }
    return max_time;
}

// ===========================================================================
// 工厂函数
// ===========================================================================

RosDataSourceBase::Ptr create_ros_data_source(const RosDataSourceConfig& cfg) {
    if (cfg.bag_file.empty() && cfg.realtime_mode) {
        // 实时模式
        return std::make_shared<Ros2RealtimeDataSource>(cfg);
    } else if (!cfg.bag_file.empty()) {
        // Bag 模式
        return std::make_shared<Ros2BagDataSource>(cfg);
    } else {
        UNICALIB_ERROR("无效的 ROS2 数据源配置: 未指定 bag 文件或实时模式");
        return nullptr;
    }
}

// ===========================================================================
// UnifiedDataLoader 实现
// ===========================================================================

UnifiedDataLoader::UnifiedDataLoader(const Config& cfg) : cfg_(cfg) {
    status_msg_ = "未加载";
}

UnifiedDataLoader::~UnifiedDataLoader() {
}

bool UnifiedDataLoader::load_from_files() {
    status_msg_ = "从文件加载数据...";
    
    UNICALIB_INFO("[UnifiedDataLoader] 从文件加载 LiDAR: {}", cfg_.lidar_data_dir);
    UNICALIB_INFO("[UnifiedDataLoader] 从文件加载相机: {}", cfg_.camera_images_dir);
    
    try {
        // 加载 LiDAR 点云
        if (!cfg_.lidar_data_dir.empty() && fs::exists(cfg_.lidar_data_dir)) {
            std::vector<fs::path> pcd_files;
            for (const auto& entry : fs::directory_iterator(cfg_.lidar_data_dir)) {
                if (entry.path().extension() == ".pcd") {
                    pcd_files.push_back(entry.path());
                }
            }
            std::sort(pcd_files.begin(), pcd_files.end());
            
            for (const auto& pcd_path : pcd_files) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
                    new pcl::PointCloud<pcl::PointXYZI>);
                if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path.string(), *cloud) == 0) {
                    double ts;
                    try {
                        ts = std::stod(pcd_path.stem().string());
                    } catch (...) {
                        ts = static_cast<double>(file_lidar_data_["lidar_front"].size()) * 0.1;
                    }
                    file_lidar_data_["lidar_front"].push_back(LiDARScanRos(ts, cloud));
                }
            }
            UNICALIB_INFO("  加载 {} 帧 LiDAR 点云", 
                          file_lidar_data_["lidar_front"].size());
        }
        
        // 加载相机图像
        if (!cfg_.camera_images_dir.empty() && fs::exists(cfg_.camera_images_dir)) {
            std::vector<fs::path> img_files;
            for (const auto& entry : fs::directory_iterator(cfg_.camera_images_dir)) {
                std::string ext = entry.path().extension().string();
                std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
                if (ext == ".png" || ext == ".jpg" || ext == ".jpeg") {
                    img_files.push_back(entry.path());
                }
            }
            std::sort(img_files.begin(), img_files.end());
            
            for (const auto& img_path : img_files) {
                cv::Mat img = cv::imread(img_path.string());
                if (!img.empty()) {
                    double ts;
                    try {
                        ts = std::stod(img_path.stem().string());
                    } catch (...) {
                        ts = static_cast<double>(file_camera_data_["cam_left"].size()) * 0.1;
                    }
                    file_camera_data_["cam_left"].push_back(CameraFrameRos(ts, img));
                }
            }
            UNICALIB_INFO("  加载 {} 帧相机图像", 
                          file_camera_data_["cam_left"].size());
        }
        
        // 加载相机内参
        if (!cfg_.camera_intrinsic_file.empty() && 
            fs::exists(cfg_.camera_intrinsic_file)) {
            try {
                auto intrin = YamlIO::load_camera_intrinsics(cfg_.camera_intrinsic_file);
                camera_intrinsics_["cam_left"] = intrin;
                UNICALIB_INFO("  加载相机内参: fx={:.1f} fy={:.1f}", 
                              intrin.fx, intrin.fy);
            } catch (const std::exception& e) {
                UNICALIB_WARN("内参加载失败: {}", e.what());
            }
        }
        
        loaded_ = true;
        status_msg_ = "文件加载成功";
        return true;
        
    } catch (const std::exception& e) {
        status_msg_ = "文件加载失败: " + std::string(e.what());
        UNICALIB_ERROR("[UnifiedDataLoader] {}", status_msg_);
        return false;
    }
}

bool UnifiedDataLoader::load_from_ros() {
    status_msg_ = "从 ROS2 加载数据...";
    
    UNICALIB_INFO("[UnifiedDataLoader] 从 ROS2 加载数据...");
    UNICALIB_INFO("  Bag 文件: {}", cfg_.ros_config.bag_file);
    UNICALIB_INFO("  实时模式: {}", cfg_.ros_config.realtime_mode);
    
    ros_source_ = create_ros_data_source(cfg_.ros_config);
    if (!ros_source_) {
        status_msg_ = "创建 ROS2 数据源失败";
        return false;
    }
    
    if (!ros_source_->load()) {
        status_msg_ = ros_source_->get_status_message();
        return false;
    }
    
    if (cfg_.ros_config.realtime_mode) {
        // 实时模式: 等待数据就绪
        auto* realtime_source = dynamic_cast<Ros2RealtimeDataSource*>(ros_source_.get());
        if (realtime_source && !realtime_source->wait_for_data(cfg_.ros_config.realtime_timeout)) {
            status_msg_ = "等待 ROS2 数据超时";
            return false;
        }
    }
    
    loaded_ = true;
    status_msg_ = "ROS2 数据加载成功";
    return true;
}

bool UnifiedDataLoader::load() {
    if (cfg_.source_type == SourceType::FILES) {
        return load_from_files();
    } else if (cfg_.source_type == SourceType::ROS2_BAG || 
               cfg_.source_type == SourceType::ROS2_TOPIC) {
        return load_from_ros();
    } else {
        status_msg_ = "未知的数据源类型";
        return false;
    }
}

std::vector<LiDARScanRos> UnifiedDataLoader::get_lidar_scans(
    const std::string& sensor_id) const {
    
    if (cfg_.source_type == SourceType::FILES) {
        auto it = file_lidar_data_.find(sensor_id);
        return (it != file_lidar_data_.end()) ? it->second : std::vector<LiDARScanRos>();
    } else {
        return ros_source_ ? ros_source_->get_lidar_scans(sensor_id) : std::vector<LiDARScanRos>();
    }
}

std::vector<CameraFrameRos> UnifiedDataLoader::get_camera_frames(
    const std::string& sensor_id) const {
    
    if (cfg_.source_type == SourceType::FILES) {
        auto it = file_camera_data_.find(sensor_id);
        return (it != file_camera_data_.end()) ? it->second : std::vector<CameraFrameRos>();
    } else {
        return ros_source_ ? ros_source_->get_camera_frames(sensor_id) : std::vector<CameraFrameRos>();
    }
}

std::vector<IMUFrameRos> UnifiedDataLoader::get_imu_frames(
    const std::string& sensor_id) const {
    
    if (cfg_.source_type == SourceType::FILES) {
        return {};  // 文件模式暂不支持 IMU
    } else {
        return ros_source_ ? ros_source_->get_imu_frames(sensor_id) : std::vector<IMUFrameRos>();
    }
}

std::vector<LiDARScan> UnifiedDataLoader::to_lidar_scans(
    const std::string& sensor_id) const {
    
    std::vector<LiDARScan> scans;
    auto ros_scans = get_lidar_scans(sensor_id);
    
    for (const auto& ros_scan : ros_scans) {
        LiDARScan scan;
        scan.timestamp = ros_scan.timestamp;
        scan.cloud = ros_scan.cloud;
        scans.push_back(scan);
    }
    
    return scans;
}

std::vector<std::pair<double, cv::Mat>> UnifiedDataLoader::to_camera_frames(
    const std::string& sensor_id) const {
    
    std::vector<std::pair<double, cv::Mat>> frames;
    auto ros_frames = get_camera_frames(sensor_id);
    
    for (const auto& ros_frame : ros_frames) {
        frames.emplace_back(ros_frame.timestamp, ros_frame.image);
    }
    
    return frames;
}

IMURawData UnifiedDataLoader::to_imu_raw_data(
    const std::string& sensor_id) const {
    
    IMURawData data;
    auto ros_frames = get_imu_frames(sensor_id);
    
    for (const auto& ros_frame : ros_frames) {
        IMURawFrame frame;
        frame.timestamp = ros_frame.timestamp;
        frame.gyro[0] = ros_frame.gyro[0];
        frame.gyro[1] = ros_frame.gyro[1];
        frame.gyro[2] = ros_frame.gyro[2];
        frame.accel[0] = ros_frame.accel[0];
        frame.accel[1] = ros_frame.accel[1];
        frame.accel[2] = ros_frame.accel[2];
        data.push_back(frame);
    }
    
    return data;
}

std::optional<CameraIntrinsics> UnifiedDataLoader::get_camera_intrinsics(
    const std::string& sensor_id) const {
    
    auto it = camera_intrinsics_.find(sensor_id);
    if (it != camera_intrinsics_.end()) {
        return it->second;
    }
    return std::nullopt;
}

bool UnifiedDataLoader::is_ready() const {
    return loaded_;
}

std::string UnifiedDataLoader::get_status_message() const {
    return status_msg_;
}

}  // namespace ns_unicalib
