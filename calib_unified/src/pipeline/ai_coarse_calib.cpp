/**
 * UniCalib Unified — AI 粗标定适配器实现
 *
 * 核心实现思路:
 *   - 通过 popen/system 调用 Python 子进程
 *   - 在 work_dir 下创建临时输入/输出文件
 *   - 解析 YAML/JSON 输出提取标定结果
 *   - 完整日志记录每次调用的命令行、耗时、返回码
 *
 * 安全注意:
 *   - 所有命令行参数都需要转义 (此处使用 std::filesystem::path 规范化)
 *   - 超时通过 timeout 命令实现
 */

#include "unicalib/pipeline/ai_coarse_calib.h"
#include "unicalib/common/logger.h"
#include <filesystem>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include <chrono>
#include <stdexcept>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <unistd.h>
#include <sys/wait.h>
#include <signal.h>
#include <fcntl.h>

namespace fs = std::filesystem;

namespace ns_unicalib {

// ---------------------------------------------------------------------------
// 工具: 执行命令并捕获输出
// ---------------------------------------------------------------------------
struct ExecResult {
    int exit_code;
    std::string stdout_str;
    double elapsed_ms;
};

// 安全执行: 无 shell，参数列表直接传 execvp，避免命令注入
static ExecResult exec_cmd_safe(std::vector<std::string> argv, int timeout_sec) {
    ExecResult r{};
    if (argv.empty()) {
        r.exit_code = -1;
        r.stdout_str = "exec_cmd_safe: argv empty";
        return r;
    }
    auto t0 = std::chrono::steady_clock::now();

    int pipe_fd[2];
    if (pipe(pipe_fd) != 0) {
        r.exit_code = -1;
        r.stdout_str = "pipe() failed";
        return r;
    }
    pid_t pid = fork();
    if (pid < 0) {
        close(pipe_fd[0]);
        close(pipe_fd[1]);
        r.exit_code = -1;
        r.stdout_str = "fork() failed";
        return r;
    }
    if (pid == 0) {
        close(pipe_fd[0]);
        dup2(pipe_fd[1], STDOUT_FILENO);
        dup2(pipe_fd[1], STDERR_FILENO);
        close(pipe_fd[1]);
        std::vector<char*> ptrs;
        for (auto& s : argv) ptrs.push_back(const_cast<char*>(s.c_str()));
        ptrs.push_back(nullptr);
        execvp(argv[0].c_str(), ptrs.data());
        _exit(127);
    }
    close(pipe_fd[1]);
    std::thread timeout_thread([pid, timeout_sec]() {
        std::this_thread::sleep_for(std::chrono::seconds(timeout_sec));
        kill(pid, SIGKILL);
    });
    char buf[4096];
    ssize_t n;
    while ((n = read(pipe_fd[0], buf, sizeof(buf))) > 0)
        r.stdout_str.append(buf, static_cast<size_t>(n));
    close(pipe_fd[0]);
    int status = 0;
    waitpid(pid, &status, 0);
    timeout_thread.join();
    r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : (WIFSIGNALED(status) ? 128 + WTERMSIG(status) : -1);
    auto t1 = std::chrono::steady_clock::now();
    r.elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
    UNICALIB_DEBUG("[AI-Exec] 退出码={} 耗时={:.0f}ms", r.exit_code, r.elapsed_ms);
    return r;
}

static ExecResult exec_cmd(const std::string& cmd, int timeout_sec) {
    // 兼容旧调用: 将单字符串拆成 argv (仅空格分隔，路径含空格会失败)
    std::vector<std::string> argv;
    std::istringstream iss(cmd);
    std::string tok;
    while (iss >> tok) argv.push_back(tok);
    if (argv.empty()) {
        ExecResult r;
        r.exit_code = -1;
        r.stdout_str = "exec_cmd: empty command";
        return r;
    }
    return exec_cmd_safe(std::move(argv), timeout_sec);
}

// ---------------------------------------------------------------------------
// 工具: 检查 Python 模块是否可用
// ---------------------------------------------------------------------------
static bool check_python_module(const std::string& python_exe,
                                  const std::string& module) {
    std::string cmd = python_exe + " -c \"import " + module + "\" 2>/dev/null";
    return system(cmd.c_str()) == 0;
}

// ---------------------------------------------------------------------------
// 工具: 序列化 IMU 数据为 NumPy npy 格式 (通过 YAML 中间文件)
// ---------------------------------------------------------------------------
static std::string serialize_imu_to_yaml(
    const std::vector<IMUFrame>& data, const std::string& path) {

    std::ofstream f(path);
    if (!f.is_open()) throw std::runtime_error("Cannot write " + path);

    f << "num_samples: " << data.size() << "\n";
    f << "imu_data:\n";
    for (const auto& d : data) {
        f << "  - ts: " << std::fixed << std::setprecision(9) << d.timestamp << "\n";
        f << "    gyro: [" << d.gyro.x() << ", " << d.gyro.y() << ", " << d.gyro.z() << "]\n";
        f << "    accel: [" << d.accel.x() << ", " << d.accel.y() << ", " << d.accel.z() << "]\n";
    }
    return path;
}

// ===========================================================================
// DMCalibAdapter
// ===========================================================================

bool DMCalibAdapter::is_available() const {
    if (!fs::exists(cfg_.repo_dir)) {
        UNICALIB_DEBUG("[DM-Calib] repo_dir 不存在: {}", cfg_.repo_dir);
        return false;
    }
    if (!fs::exists(cfg_.repo_dir + "/" + cfg_.infer_script)) {
        UNICALIB_DEBUG("[DM-Calib] 推理脚本不存在: {}/{}", cfg_.repo_dir, cfg_.infer_script);
        return false;
    }
    return check_python_module(cfg_.python_exe, "torch");
}

std::string DMCalibAdapter::build_cmd(const std::string& input_dir,
                                       const std::string& output_dir) const {
    std::ostringstream oss;
    oss << cfg_.python_exe << " " << cfg_.repo_dir << "/" << cfg_.infer_script
        << " --pretrained_model_path " << cfg_.model_path
        << " --input_dir "  << input_dir
        << " --output_dir " << output_dir
        << " --domain " << cfg_.domain;
    if (cfg_.scale_10)       oss << " --scale_10";
    if (cfg_.domain_specify) oss << " --domain_specify";
    oss << " --seed 666";
    return oss.str();
}

DMCalibResult DMCalibAdapter::estimate(const std::string& image_path,
                                        const std::string& output_dir_arg) const {
    DMCalibResult result;
    result.model_name = "DM-Calib";
    result.success = false;

    if (!is_available()) {
        result.error_msg = "DM-Calib 不可用 (检查 repo_dir 和 torch 安装)";
        UNICALIB_WARN("[DM-Calib] {}", result.error_msg);
        return result;
    }

    try {
        fs::create_directories(cfg_.work_dir);
        std::string input_dir  = cfg_.work_dir + "/input";
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(input_dir);
        fs::create_directories(output_dir);

        // 软链接或复制图像到 input_dir
        std::string link_path = input_dir + "/" + fs::path(image_path).filename().string();
        if (!fs::exists(link_path)) {
            fs::copy_file(image_path, link_path,
                          fs::copy_options::overwrite_existing);
        }

        // 获取图像尺寸
        // 此处简化: 通过 Python 获取, 也可用 OpenCV
        int img_w = 0, img_h = 0;
        {
            std::string size_cmd = cfg_.python_exe +
                " -c \"from PIL import Image; img=Image.open('" + image_path +
                "'); print(img.width, img.height)\" 2>/dev/null";
            FILE* p = popen(size_cmd.c_str(), "r");
            if (p) {
                if (fscanf(p, "%d %d", &img_w, &img_h) != 2) {
                    img_w = img_h = 0;
                }
                pclose(p);
            }
        }
        if (img_w == 0 || img_h == 0) {
            result.error_msg = "无法读取图像尺寸: " + image_path;
            UNICALIB_WARN("[DM-Calib] {}", result.error_msg);
            return result;
        }

        std::string cmd = build_cmd(input_dir, output_dir);
        UNICALIB_INFO("[DM-Calib] 运行推理: 超时={}s", cfg_.timeout_sec);
        UNICALIB_DEBUG("[DM-Calib] cmd: {}", cmd);

        auto exec_r = exec_cmd(cmd, cfg_.timeout_sec);
        result.elapsed_ms = exec_r.elapsed_ms;
        log_call(cmd, exec_r.exit_code, exec_r.elapsed_ms);

        if (exec_r.exit_code != 0) {
            result.error_msg = "DM-Calib 推理失败 (exit=" +
                                std::to_string(exec_r.exit_code) + ")";
            UNICALIB_ERROR("[DM-Calib] {}", result.error_msg);
            UNICALIB_DEBUG("[DM-Calib] stdout: {}", exec_r.stdout_str.substr(0, 500));
            return result;
        }

        // 解析输出
        std::string out_json = output_dir + "/intrinsics.json";
        if (!fs::exists(out_json)) {
            // 尝试 YAML 格式
            out_json = output_dir + "/intrinsics.yaml";
        }
        if (fs::exists(out_json)) {
            result.coarse_intrin = parse_output(out_json, img_w, img_h);
            result.success    = true;
            result.confidence = 0.75;  // DM-Calib 论文声称良好精度
            UNICALIB_INFO("[DM-Calib] 粗估内参: fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f}",
                          result.coarse_intrin.fx, result.coarse_intrin.fy,
                          result.coarse_intrin.cx, result.coarse_intrin.cy);
        } else {
            result.error_msg = "未找到输出文件: " + out_json;
            UNICALIB_WARN("[DM-Calib] {}", result.error_msg);
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("DM-Calib 异常: ") + e.what();
        UNICALIB_ERROR("[DM-Calib] {}", result.error_msg);
    }
    return result;
}

DMCalibResult DMCalibAdapter::estimate_multi(
    const std::vector<std::string>& image_paths,
    const std::string& output_dir) const {

    UNICALIB_INFO("[DM-Calib] 批量推理 {} 张图像", image_paths.size());

    std::vector<CameraIntrinsics> results;
    for (const auto& img : image_paths) {
        auto r = estimate(img, output_dir);
        if (r.success) results.push_back(r.coarse_intrin);
    }

    if (results.empty()) {
        DMCalibResult r;
        r.model_name = "DM-Calib";
        r.error_msg  = "所有图像推理失败";
        return r;
    }

    // 取中位数
    DMCalibResult final_r;
    final_r.model_name = "DM-Calib";
    std::sort(results.begin(), results.end(),
              [](const CameraIntrinsics& a, const CameraIntrinsics& b) {
                  return a.fx < b.fx;
              });
    final_r.coarse_intrin = results[results.size() / 2];
    final_r.success    = true;
    final_r.confidence = 0.8;
    UNICALIB_INFO("[DM-Calib] 批量中位数: fx={:.1f}", final_r.coarse_intrin.fx);
    return final_r;
}

CameraIntrinsics DMCalibAdapter::parse_output(const std::string& path,
                                               int img_w, int img_h) const {
    CameraIntrinsics intrin;
    intrin.width  = img_w;
    intrin.height = img_h;
    intrin.model  = CameraIntrinsics::Model::PINHOLE;

    try {
        auto ends_with = [](const std::string& s, const std::string& suffix) {
            return s.size() >= suffix.size() &&
                   s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
        };
        if (ends_with(path, ".yaml") || ends_with(path, ".yml")) {
            YAML::Node node = YAML::LoadFile(path);
            intrin.fx = node["fx"] ? node["fx"].as<double>() : 0.0;
            intrin.fy = node["fy"] ? node["fy"].as<double>() : intrin.fx;
            intrin.cx = node["cx"] ? node["cx"].as<double>() : img_w * 0.5;
            intrin.cy = node["cy"] ? node["cy"].as<double>() : img_h * 0.5;
        } else {
            // 简单 JSON 解析
            std::ifstream f(path);
            std::string content((std::istreambuf_iterator<char>(f)),
                                 std::istreambuf_iterator<char>());
            auto find_val = [&](const std::string& key) -> double {
                size_t pos = content.find("\"" + key + "\"");
                if (pos == std::string::npos) return 0.0;
                pos = content.find(':', pos);
                if (pos == std::string::npos) return 0.0;
                return std::stod(content.substr(pos + 1));
            };
            intrin.fx = find_val("fx");
            intrin.fy = find_val("fy");
            intrin.cx = find_val("cx");
            intrin.cy = find_val("cy");
        }
    } catch (const std::exception& e) {
        UNICALIB_WARN("[DM-Calib] 解析输出异常: {}", e.what());
    }

    if (intrin.cx == 0.0) intrin.cx = img_w * 0.5;
    if (intrin.cy == 0.0) intrin.cy = img_h * 0.5;
    if (intrin.fy == 0.0) intrin.fy = intrin.fx;

    return intrin;
}

void DMCalibAdapter::log_call(const std::string& cmd,
                               int exit_code, double elapsed_ms) const {
    UNICALIB_INFO("[DM-Calib] 调用完成 exit={} 耗时={:.0f}ms", exit_code, elapsed_ms);
    UNICALIB_TRACE("[DM-Calib] cmd={}", cmd);
}

// ===========================================================================
// MIASLCECAdapter
// ===========================================================================

bool MIASLCECAdapter::is_available() const {
    if (!fs::exists(cfg_.repo_dir)) return false;
    std::string script = cfg_.repo_dir + "/" + cfg_.calib_script;
    if (!fs::exists(script)) {
        // MIAS-LCEC 使用二进制或不同路径
        UNICALIB_DEBUG("[MIAS-LCEC] 脚本不存在: {}", script);
        return false;
    }
    return true;
}

std::string MIASLCECAdapter::write_sensor_config(
    const CameraIntrinsics& intrin,
    const std::string& work_dir) const {
    std::string cfg_path = work_dir + "/sensor_config.yaml";
    std::ofstream f(cfg_path);
    f << "camera:\n";
    f << "  model: " << (intrin.model == CameraIntrinsics::Model::FISHEYE ?
                          "fisheye" : "pinhole") << "\n";
    f << "  width: " << intrin.width << "\n";
    f << "  height: " << intrin.height << "\n";
    f << "  fx: " << intrin.fx << "\n";
    f << "  fy: " << intrin.fy << "\n";
    f << "  cx: " << intrin.cx << "\n";
    f << "  cy: " << intrin.cy << "\n";
    f << "  dist_coeffs: [";
    for (size_t i = 0; i < intrin.dist_coeffs.size(); ++i) {
        f << intrin.dist_coeffs[i];
        if (i + 1 < intrin.dist_coeffs.size()) f << ", ";
    }
    f << "]\n";
    f << "lidar:\n  type: " << cfg_.lidar_type << "\n";
    return cfg_path;
}

MIASLCECResult MIASLCECAdapter::estimate(
    const std::string& pcd_file,
    const std::string& image_file,
    const CameraIntrinsics& cam_intrin,
    const std::string& output_dir_arg) const {

    MIASLCECResult result;
    result.model_name = "MIAS-LCEC";

    if (!is_available()) {
        result.error_msg = "MIAS-LCEC 不可用";
        UNICALIB_WARN("[MIAS-LCEC] {}", result.error_msg);
        return result;
    }

    try {
        fs::create_directories(cfg_.work_dir);
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(output_dir);

        std::string sensor_cfg = write_sensor_config(cam_intrin, cfg_.work_dir);

        std::ostringstream cmd;
        cmd << cfg_.python_exe << " " << cfg_.repo_dir << "/" << cfg_.calib_script
            << " --pcd " << pcd_file
            << " --image " << image_file
            << " --sensor_config " << sensor_cfg
            << " --output_dir " << output_dir;

        UNICALIB_INFO("[MIAS-LCEC] 运行跨模态掩码匹配标定");
        auto exec_r = exec_cmd(cmd.str(), cfg_.timeout_sec);
        result.elapsed_ms = exec_r.elapsed_ms;

        if (exec_r.exit_code != 0) {
            result.error_msg = "MIAS-LCEC 失败 (exit=" +
                                std::to_string(exec_r.exit_code) + ")";
            UNICALIB_ERROR("[MIAS-LCEC] {}", result.error_msg);
            return result;
        }

        std::string out_yaml = output_dir + "/extrinsic_result.yaml";
        if (fs::exists(out_yaml)) {
            result.coarse_extrin = parse_output(out_yaml);
            result.success    = true;
            result.confidence = 0.7;
            UNICALIB_INFO("[MIAS-LCEC] 粗估外参: t=[{:.3f}, {:.3f}, {:.3f}]m",
                          result.coarse_extrin.POS_TargetInRef.x(),
                          result.coarse_extrin.POS_TargetInRef.y(),
                          result.coarse_extrin.POS_TargetInRef.z());
        } else {
            result.error_msg = "未找到输出: " + out_yaml;
            UNICALIB_WARN("[MIAS-LCEC] {}", result.error_msg);
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("MIAS-LCEC 异常: ") + e.what();
        UNICALIB_ERROR("[MIAS-LCEC] {}", result.error_msg);
    }
    return result;
}

ExtrinsicSE3 MIASLCECAdapter::parse_output(const std::string& path) const {
    ExtrinsicSE3 extrin;
    try {
        YAML::Node node = YAML::LoadFile(path);
        if (node["rotation_matrix"]) {
            Eigen::Matrix3d R;
            auto rm = node["rotation_matrix"];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R(i, j) = rm[i][j].as<double>();
            extrin.SO3_TargetInRef = Sophus::SO3d(R);
        }
        if (node["translation"]) {
            auto t = node["translation"];
            extrin.POS_TargetInRef = Eigen::Vector3d(
                t[0].as<double>(), t[1].as<double>(), t[2].as<double>());
        }
        if (node["lidar_id"])  extrin.ref_sensor_id    = node["lidar_id"].as<std::string>();
        if (node["camera_id"]) extrin.target_sensor_id = node["camera_id"].as<std::string>();
    } catch (const std::exception& e) {
        UNICALIB_WARN("[MIAS-LCEC] 解析输出异常: {}", e.what());
    }
    return extrin;
}

// ===========================================================================
// TransformerIMUAdapter
// ===========================================================================

bool TransformerIMUAdapter::is_available() const {
    if (!fs::exists(cfg_.repo_dir)) return false;
    std::string script = cfg_.repo_dir + "/" + cfg_.eval_script;
    return fs::exists(script) && check_python_module(cfg_.python_exe, "torch");
}

TransformerIMUResult TransformerIMUAdapter::estimate(
    const std::vector<IMUFrame>& imu_data,
    const std::string& output_dir_arg) const {

    TransformerIMUResult result;
    result.model_name = "Transformer-IMU-Calibrator";

    if (!is_available()) {
        result.error_msg = "Transformer-IMU 不可用";
        UNICALIB_WARN("[TransformerIMU] {}", result.error_msg);
        return result;
    }

    UNICALIB_INFO("[TransformerIMU] 从 {} 帧 IMU 数据估计内参", imu_data.size());

    try {
        fs::create_directories(cfg_.work_dir);
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(output_dir);

        // 序列化 IMU 数据
        std::string imu_yaml = cfg_.work_dir + "/imu_data.yaml";
        serialize_imu_to_yaml(imu_data, imu_yaml);

        std::ostringstream cmd;
        cmd << cfg_.python_exe
            << " " << cfg_.repo_dir << "/" << cfg_.eval_script
            << " --imu_data " << imu_yaml
            << " --weights " << cfg_.repo_dir << "/" << cfg_.model_weights
            << " --output_dir " << output_dir;

        auto exec_r = exec_cmd(cmd.str(), cfg_.timeout_sec);
        result.elapsed_ms = exec_r.elapsed_ms;

        if (exec_r.exit_code != 0) {
            result.error_msg = "Transformer-IMU 失败";
            UNICALIB_ERROR("[TransformerIMU] {}", result.error_msg);
            return result;
        }

        std::string out_json = output_dir + "/imu_intrinsics.json";
        if (fs::exists(out_json)) {
            result.coarse_intrin = parse_output(out_json);
            result.success    = true;
            result.confidence = 0.6;
            UNICALIB_INFO("[TransformerIMU] 粗估 bias_gyro=[{:.6f}, {:.6f}, {:.6f}]",
                          result.coarse_intrin.bias_gyro.x(),
                          result.coarse_intrin.bias_gyro.y(),
                          result.coarse_intrin.bias_gyro.z());
        } else {
            result.error_msg = "未找到输出: " + out_json;
            UNICALIB_WARN("[TransformerIMU] {}", result.error_msg);
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("TransformerIMU 异常: ") + e.what();
        UNICALIB_ERROR("[TransformerIMU] {}", result.error_msg);
    }
    return result;
}

IMUIntrinsics TransformerIMUAdapter::parse_output(const std::string& path) const {
    IMUIntrinsics intrin;
    try {
        std::ifstream f(path);
        std::string content((std::istreambuf_iterator<char>(f)),
                             std::istreambuf_iterator<char>());
        // 简单解析 JSON
        auto find_arr3 = [&](const std::string& key) -> Eigen::Vector3d {
            size_t pos = content.find("\"" + key + "\"");
            if (pos == std::string::npos) return Eigen::Vector3d::Zero();
            size_t lb = content.find('[', pos);
            size_t rb = content.find(']', lb);
            if (lb == std::string::npos || rb == std::string::npos)
                return Eigen::Vector3d::Zero();
            std::string arr_str = content.substr(lb+1, rb-lb-1);
            std::istringstream iss(arr_str);
            double x, y, z; char comma;
            iss >> x >> comma >> y >> comma >> z;
            return Eigen::Vector3d(x, y, z);
        };
        intrin.bias_gyro  = find_arr3("bias_gyro");
        intrin.bias_acce  = find_arr3("bias_acce");
    } catch (const std::exception& e) {
        UNICALIB_WARN("[TransformerIMU] 解析异常: {}", e.what());
    }
    return intrin;
}

std::string TransformerIMUAdapter::serialize_imu_data(
    const std::vector<IMUFrame>& data, const std::string& path) const {
    return serialize_imu_to_yaml(data, path);
}

// ===========================================================================
// L2CalibAdapter
// ===========================================================================

bool L2CalibAdapter::is_available() const {
    if (!fs::exists(cfg_.repo_dir)) return false;
    std::string script = cfg_.repo_dir + "/" + cfg_.train_script;
    return fs::exists(script) && check_python_module(cfg_.python_exe, "torch");
}

L2CalibResult L2CalibAdapter::estimate_from_bag(
    const std::string& bag_path,
    const std::string& output_dir_arg) const {

    L2CalibResult result;
    result.model_name = "L2Calib";

    if (!is_available()) {
        result.error_msg = "L2Calib 不可用";
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    if (!fs::exists(bag_path)) {
        result.error_msg = "Bag 文件不存在: " + bag_path;
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    UNICALIB_INFO("[L2Calib] RL强化学习 IMU-LiDAR 外参粗估");
    UNICALIB_INFO("[L2Calib] bag: {} epochs: {}", bag_path, cfg_.num_epochs);

    try {
        fs::create_directories(cfg_.work_dir);
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(output_dir);

        // 写 FastLIO 配置 (如果没有提供)
        std::string fastlio_cfg = cfg_.fastlio_config;
        if (fastlio_cfg.empty()) {
            fastlio_cfg = cfg_.work_dir + "/fastlio_config.yaml";
            std::ofstream f(fastlio_cfg);
            f << "common:\n  imu_topic: " << cfg_.imu_topic << "\n";
        }

        std::ostringstream cmd;
        cmd << "cd " << cfg_.repo_dir << " && "
            << "export PYTHONPATH=" << cfg_.repo_dir << "/rl_solver:$PYTHONPATH && "
            << cfg_.python_exe << " " << cfg_.train_script
            << " --lio-config " << fastlio_cfg
            << " --bag-dir " << fs::path(bag_path).parent_path().string()
            << " --alg " << cfg_.alg
            << " --SO3-distribution " << cfg_.so3_dist
            << " --num-epochs " << cfg_.num_epochs
            << " --min " << (-cfg_.rough_trans_m)
            << " --max " << cfg_.rough_trans_m
            << " --output_dir " << output_dir;

        auto exec_r = exec_cmd(cmd.str(), cfg_.timeout_sec);
        result.elapsed_ms = exec_r.elapsed_ms;
        result.num_epochs  = cfg_.num_epochs;

        if (exec_r.exit_code != 0) {
            result.error_msg = "L2Calib 失败 (exit=" +
                                std::to_string(exec_r.exit_code) + ")";
            UNICALIB_ERROR("[L2Calib] {}", result.error_msg);
            UNICALIB_DEBUG("[L2Calib] stdout: {}", exec_r.stdout_str.substr(0, 500));
            return result;
        }

        std::string out_yaml = output_dir + "/calibration_result.yaml";
        if (fs::exists(out_yaml)) {
            result.coarse_extrin = parse_output(out_yaml);
            result.success    = true;
            result.confidence = 0.65;
            UNICALIB_INFO("[L2Calib] 粗估外参: t=[{:.3f}, {:.3f}, {:.3f}]m",
                          result.coarse_extrin.POS_TargetInRef.x(),
                          result.coarse_extrin.POS_TargetInRef.y(),
                          result.coarse_extrin.POS_TargetInRef.z());
        } else {
            result.error_msg = "未找到输出: " + out_yaml;
            UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("L2Calib 异常: ") + e.what();
        UNICALIB_ERROR("[L2Calib] {}", result.error_msg);
    }
    return result;
}

L2CalibResult L2CalibAdapter::estimate_from_trajectories(
    const std::string& lidar_traj_path,
    const std::string& imu_traj_path,
    const std::string& output_dir_arg) const {

    L2CalibResult result;
    result.model_name = "L2Calib";

    if (!is_available()) {
        result.error_msg = "L2Calib 不可用";
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    if (!fs::exists(lidar_traj_path) || !fs::exists(imu_traj_path)) {
        result.error_msg = "轨迹文件不存在: " + lidar_traj_path + " / " + imu_traj_path;
        UNICALIB_WARN("[L2Calib] {}", result.error_msg);
        return result;
    }

    try {
        fs::create_directories(cfg_.work_dir);
        std::string output_dir = output_dir_arg.empty() ?
            cfg_.work_dir + "/output" : output_dir_arg;
        fs::create_directories(output_dir);

        std::ostringstream cmd;
        cmd << "cd " << cfg_.repo_dir << " && "
            << "export PYTHONPATH=" << cfg_.repo_dir << "/rl_solver:$PYTHONPATH && "
            << cfg_.python_exe << " " << cfg_.train_script
            << " --lidar-traj " << lidar_traj_path
            << " --imu-traj " << imu_traj_path
            << " --alg " << cfg_.alg
            << " --SO3-distribution " << cfg_.so3_dist
            << " --num-epochs " << cfg_.num_epochs
            << " --output_dir " << output_dir;

        auto exec_r = exec_cmd(cmd.str(), cfg_.timeout_sec);
        result.elapsed_ms = exec_r.elapsed_ms;
        result.num_epochs = cfg_.num_epochs;

        if (exec_r.exit_code != 0) {
            result.error_msg = "L2Calib (轨迹模式) 失败 exit=" + std::to_string(exec_r.exit_code);
            UNICALIB_WARN("[L2Calib] {}", result.error_msg);
            return result;
        }

        std::string out_yaml = output_dir + "/calibration_result.yaml";
        if (fs::exists(out_yaml)) {
            result.coarse_extrin = parse_output(out_yaml);
            result.success    = true;
            result.confidence = 0.65;
        } else {
            result.error_msg = "未找到输出: " + out_yaml;
        }
    } catch (const std::exception& e) {
        result.error_msg = std::string("L2Calib 异常: ") + e.what();
        UNICALIB_ERROR("[L2Calib] {}", result.error_msg);
    }
    return result;
}

ExtrinsicSE3 L2CalibAdapter::parse_output(const std::string& path) const {
    ExtrinsicSE3 extrin;
    extrin.ref_sensor_id    = "imu_0";
    extrin.target_sensor_id = "lidar_0";
    try {
        YAML::Node node = YAML::LoadFile(path);
        if (node["rotation_matrix"]) {
            Eigen::Matrix3d R;
            auto rm = node["rotation_matrix"];
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R(i, j) = rm[i][j].as<double>();
            extrin.SO3_TargetInRef = Sophus::SO3d(R);
        } else if (node["rotation_euler_deg"]) {
            auto e = node["rotation_euler_deg"];
            double r = e[0].as<double>() * M_PI / 180.0;
            double p = e[1].as<double>() * M_PI / 180.0;
            double y = e[2].as<double>() * M_PI / 180.0;
            Eigen::AngleAxisd rx(r, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd ry(p, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd rz(y, Eigen::Vector3d::UnitZ());
            extrin.SO3_TargetInRef = Sophus::SO3d(
                (rz * ry * rx).toRotationMatrix());
        }
        if (node["translation"]) {
            auto t = node["translation"];
            extrin.POS_TargetInRef = Eigen::Vector3d(
                t[0].as<double>(), t[1].as<double>(), t[2].as<double>());
        }
        if (node["time_offset_s"])
            extrin.time_offset_s = node["time_offset_s"].as<double>();
    } catch (const std::exception& e) {
        UNICALIB_WARN("[L2Calib] 解析输出异常: {}", e.what());
    }
    return extrin;
}

// ===========================================================================
// AICoarseCalibManager
// ===========================================================================

AICoarseCalibManager::AICoarseCalibManager(const Config& cfg) : cfg_(cfg) {
    setup_adapters();
}

void AICoarseCalibManager::setup_adapters() {
    // DM-Calib
    DMCalibAdapter::Config dm_cfg = cfg_.dm_calib;
    if (dm_cfg.repo_dir.empty())
        dm_cfg.repo_dir = cfg_.ai_root + "/DM-Calib";
    dm_cfg.python_exe = cfg_.python_exe;
    dm_calib_ = std::make_unique<DMCalibAdapter>(dm_cfg);

    // MIAS-LCEC
    MIASLCECAdapter::Config mias_cfg = cfg_.mias_lcec;
    if (mias_cfg.repo_dir.empty())
        mias_cfg.repo_dir = cfg_.ai_root + "/MIAS-LCEC";
    mias_cfg.python_exe = cfg_.python_exe;
    mias_lcec_ = std::make_unique<MIASLCECAdapter>(mias_cfg);

    // Transformer-IMU
    TransformerIMUAdapter::Config ti_cfg = cfg_.transformer_imu;
    if (ti_cfg.repo_dir.empty())
        ti_cfg.repo_dir = cfg_.ai_root + "/Transformer-IMU-Calibrator";
    ti_cfg.python_exe = cfg_.python_exe;
    transformer_imu_ = std::make_unique<TransformerIMUAdapter>(ti_cfg);

    // L2Calib
    L2CalibAdapter::Config l2_cfg = cfg_.l2calib;
    if (l2_cfg.repo_dir.empty())
        l2_cfg.repo_dir = cfg_.ai_root + "/learn-to-calibrate";
    l2_cfg.python_exe = cfg_.python_exe;
    l2calib_ = std::make_unique<L2CalibAdapter>(l2_cfg);
}

bool AICoarseCalibManager::check_dm_calib() const        { return dm_calib_->is_available(); }
bool AICoarseCalibManager::check_mias_lcec() const       { return mias_lcec_->is_available(); }
bool AICoarseCalibManager::check_transformer_imu() const { return transformer_imu_->is_available(); }
bool AICoarseCalibManager::check_l2calib() const         { return l2calib_->is_available(); }

void AICoarseCalibManager::print_availability() const {
    UNICALIB_INFO("[AI-Calib] ===== AI 模型可用性检查 =====");
    UNICALIB_INFO("  DM-Calib (相机内参):      {}",
                  check_dm_calib()        ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("  MIAS-LCEC (LiDAR-Cam):   {}",
                  check_mias_lcec()       ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("  Transformer-IMU (IMU内参): {}",
                  check_transformer_imu() ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("  L2Calib (IMU-LiDAR):     {}",
                  check_l2calib()         ? "✓ 可用" : "✗ 不可用");
    UNICALIB_INFO("[AI-Calib] ================================");
}

std::optional<CameraIntrinsics> AICoarseCalibManager::coarse_cam_intrinsic(
    const std::vector<std::string>& image_paths,
    int img_w, int img_h,
    const std::string& cam_id) const {

    UNICALIB_INFO("[AI-Calib] 相机内参粗估 (DM-Calib) cam_id={}", cam_id);

    if (!dm_calib_->is_available()) {
        UNICALIB_WARN("[AI-Calib] DM-Calib 不可用, 跳过粗估");
        return std::nullopt;
    }

    DMCalibResult r;
    if (image_paths.size() == 1) {
        r = dm_calib_->estimate(image_paths[0]);
    } else {
        r = dm_calib_->estimate_multi(image_paths);
    }

    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] DM-Calib 置信度不足 ({:.2f} < {:.2f})",
                      r.confidence, cfg_.min_confidence_threshold);
        if (!cfg_.fallback_on_failure) return std::nullopt;
        UNICALIB_INFO("[AI-Calib] 降级: 使用默认内参初始值");
        CameraIntrinsics fallback;
        fallback.width = img_w; fallback.height = img_h;
        fallback.fx = fallback.fy = std::max(img_w, img_h) * 0.8;
        fallback.cx = img_w * 0.5; fallback.cy = img_h * 0.5;
        return fallback;
    }

    r.coarse_intrin.width  = img_w;
    r.coarse_intrin.height = img_h;
    return r.coarse_intrin;
}

std::optional<IMUIntrinsics> AICoarseCalibManager::coarse_imu_intrinsic(
    const std::vector<IMUFrame>& imu_data,
    const std::string& imu_id) const {

    UNICALIB_INFO("[AI-Calib] IMU 内参粗估 (Transformer-IMU) imu_id={}", imu_id);

    if (!transformer_imu_->is_available()) {
        UNICALIB_WARN("[AI-Calib] Transformer-IMU 不可用, 跳过粗估");
        return std::nullopt;
    }

    auto r = transformer_imu_->estimate(imu_data);
    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] Transformer-IMU 置信度不足");
        return std::nullopt;
    }
    return r.coarse_intrin;
}

std::optional<ExtrinsicSE3> AICoarseCalibManager::coarse_imu_lidar(
    const std::string& bag_path,
    const std::string& imu_id,
    const std::string& lidar_id) const {

    UNICALIB_INFO("[AI-Calib] IMU-LiDAR 粗估 (L2Calib) {}→{}", imu_id, lidar_id);

    if (!l2calib_->is_available()) {
        UNICALIB_WARN("[AI-Calib] L2Calib 不可用, 跳过粗估");
        return std::nullopt;
    }

    auto r = l2calib_->estimate_from_bag(bag_path);
    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] L2Calib 置信度不足");
        return std::nullopt;
    }

    r.coarse_extrin.ref_sensor_id    = imu_id;
    r.coarse_extrin.target_sensor_id = lidar_id;
    return r.coarse_extrin;
}

std::optional<ExtrinsicSE3> AICoarseCalibManager::coarse_lidar_cam(
    const std::string& pcd_file,
    const std::string& image_file,
    const CameraIntrinsics& cam_intrin,
    const std::string& lidar_id,
    const std::string& cam_id) const {

    UNICALIB_INFO("[AI-Calib] LiDAR-Cam 粗估 (MIAS-LCEC) {}→{}", lidar_id, cam_id);

    if (!mias_lcec_->is_available()) {
        UNICALIB_WARN("[AI-Calib] MIAS-LCEC 不可用, 跳过粗估");
        return std::nullopt;
    }

    auto r = mias_lcec_->estimate(pcd_file, image_file, cam_intrin);
    if (!r.is_reliable(cfg_.min_confidence_threshold)) {
        UNICALIB_WARN("[AI-Calib] MIAS-LCEC 置信度不足");
        return std::nullopt;
    }

    r.coarse_extrin.ref_sensor_id    = lidar_id;
    r.coarse_extrin.target_sensor_id = cam_id;
    return r.coarse_extrin;
}

}  // namespace ns_unicalib
