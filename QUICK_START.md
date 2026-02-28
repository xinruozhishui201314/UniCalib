# UniCalib 快速开始指南

## 🚀 一键配置和运行

### 方式1：完全自动化（推荐）

```bash
# 1. 自动检测并配置所有工具路径
./auto_config_env.sh

# 2. 一键编译和运行
./build_and_run.sh
```

### 方式2：手动配置

```bash
# 1. 检查配置状态
./verify_config.sh

# 2. 根据输出手动设置环境变量
export UNICALIB_DM_CALIB=/path/to/DM-Calib
export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate
export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC

# 3. 运行
./build_and_run.sh
```

---

## 📋 配置验证

### 使用验证脚本

```bash
# 基本验证
./verify_config.sh

# 自动修复（检测路径并更新配置）
./verify_config.sh --fix-auto
```

### 验证内容

- ✅ 配置文件存在性
- ✅ 环境变量设置
- ✅ 配置文件中的路径
- ✅ 工具目录自动检测
- ✅ 配置建议和修复提示

---

## 🔧 手动配置

### 方式1：环境变量

```bash
# 在 ~/.bashrc 中添加
export UNICALIB_DM_CALIB="${HOME}/calibration/DM-Calib"
export UNICALIB_LEARN_TO_CALIB="${HOME}/calibration/learn-to-calibrate"
export UNICALIB_MIAS_LCEC="${HOME}/calibration/MIAS-LCEC"
export UNICALIB_IKALIBR="${HOME}/calibration/iKalibr"
export UNICALIB_CLICK_CALIB="${HOME}/calibration/click_calib"
export UNICALIB_TRANSFORMER_IMU="${HOME}/calibration/Transformer-IMU-Calibrator"

# 重新加载
source ~/.bashrc
```

### 方式2：配置文件

编辑 `unicalib_C_plus_plus/config/sensors.yaml`:

```yaml
third_party:
  dm_calib: "/path/to/DM-Calib"
  learn_to_calibrate: "/path/to/learn-to-calibrate"
  mias_lcec: "/path/to/MIAS-LCEC"
  ikalibr: "/path/to/iKalibr"
  click_calib: "/path/to/click_calib"
  transformer_imu: "/path/to/Transformer-IMU-Calibrator"
```

---

## 📦 第三方工具说明

### DM-Calib（必需）
- **作用**: 针孔相机内参标定，无需标定板
- **路径**: 检查点：`DMCalib/tools/infer.py`
- **环境变量**: `UNICALIB_DM_CALIB`
- **配置键**: `third_party.dm_calib`

### learn-to-calibrate（必需）
- **作用**: IMU-LiDAR粗外参标定，基于强化学习
- **路径**: 检查点：`rl_solver/calib_rl.py`
- **环境变量**: `UNICALIB_LEARN_TO_CALIB`
- **配置键**: `third_party.learn_to_calibrate`

### MIAS-LCEC（可选）
- **作用**: LiDAR-Camera外参标定，粗/精两阶段
- **路径**: 检查点：`model/pretrained_overlap_transformer.pth.tar`
- **环境变量**: `UNICALIB_MIAS_LCEC`
- **配置键**: `third_party.mias_lcec`

### iKalibr（可选）
- **作用**: 多传感器联合优化，需要ROS2
- **路径**: 检查点：`package.xml`
- **环境变量**: `UNICALIB_IKALIBR`
- **配置键**: `third_party.ikalibr`

### click_calib（可选）
- **作用**: Camera-Camera束调整和验证
- **路径**: 检查点：`source/optimize.py`
- **环境变量**: `UNICALIB_CLICK_CALIB`
- **配置键**: `third_party.click_calib`

### Transformer-IMU-Calibrator（可选）
- **作用**: IMU内参备选方案，适用于运动捕捉
- **路径**: 检查点：`checkpoint/TIC_13.pth`
- **环境变量**: `UNICALIB_TRANSFORMER_IMU`
- **配置键**: `third_party.transformer_imu`

---

## 🎯 运行流程

### 完整四阶段标定

```bash
# 使用默认数据目录
./build_and_run.sh

# 使用自定义数据目录
CALIB_DATA_DIR=/path/to/data \
CALIB_RESULTS_DIR=/path/to/results \
./build_and_run.sh
```

### 分阶段运行

```bash
# 进入容器
./build_and_run.sh --shell

# 进入构建目录
cd unicalib_C_plus_plus/build

# 仅Stage1：内参标定
./unicalib_example ../config/sensors.yaml /path/to/data --stage intrinsic

# 仅Stage2：粗外参
./unicalib_example ../config/sensors.yaml /path/to/data --stage coarse

# 仅Stage3：精外参
./unicalib_example ../config/sensors.yaml /path/to/data --stage fine

# 仅Stage4：验证
./unicalib_example ../config/sensors.yaml /path/to/data --stage validate
```

---

## 🔍 常见问题

### Q1: DM-Calib not configured

**错误信息**:
```
[WARN] DM-Calib not configured. Set 'third_party.dm_calib' in config.yaml or set UNICALIB_DM_CALIB environment variable.
```

**解决方案**:
```bash
# 方式1：运行自动配置
./auto_config_env.sh

# 方式2：手动设置
export UNICALIB_DM_CALIB=/path/to/DM-Calib
```

### Q2: learn-to-calibrate not configured

**错误信息**:
```
[WARN] learn-to-calibrate not configured. Set 'third_party.learn_to_calibrate' in config.yaml or set UNICALIB_LEARN_TO_CALIB environment variable.
```

**解决方案**:
```bash
# 方式1：运行自动配置
./auto_config_env.sh

# 方式2：手动设置
export UNICALIB_LEARN_TO_CALIB=/path/to/learn-to-calibrate
```

### Q3: MIAS-LCEC not configured

**错误信息**:
```
[WARN] MIAS-LCEC not configured for coarse extrinsic calibration.
```

**解决方案**:
```bash
# 方式1：运行自动配置
./auto_config_env.sh

# 方式2：手动设置
export UNICALIB_MIAS_LCEC=/path/to/MIAS-LCEC

# 方式3：如果不需要LiDAR-Camera标定，可以在sensors.yaml中移除对应传感器对
```

### Q4: 模型文件不存在

**检查**:
```bash
# 验证模型文件
./verify_models.sh

# 检查工具路径
./verify_config.sh
```

**解决方案**:
```bash
# 下载模型
cd DM-Calib
# 参考DM-Calib/README.md下载模型

cd ../MIAS-LCEC
# 模型已包含在项目中

cd ../learn-to-calibrate
# 模型已包含在项目中
```

---

## 📚 详细文档

- [主README](README.md) - 项目概述
- [编译和运行指南](BUILD_AND_RUN_GUIDE.md) - 详细构建说明
- [快速参考](QUICK_REFERENCE.md) - 常用命令
- [集成分析](unicalib_C_plus_plus/INTEGRATION_ANALYSIS.md) - 架构和集成说明

---

## 🆘 获取帮助

### 验证工具

```bash
# 验证模型
./verify_models.sh

# 验证配置
./verify_config.sh

# 自动配置
./auto_config_env.sh
```

### 日志和调试

```bash
# 查看构建日志
./build_and_run.sh --build-only 2>&1 | tee build.log

# 查看运行日志
cat calib_results/calibration.log
```

---

## ✅ 检查清单

运行前检查：

- [ ] 已运行 `./auto_config_env.sh` 或手动配置了环境变量
- [ ] 已运行 `./verify_config.sh` 确认配置正确
- [ ] 已运行 `./verify_models.sh` 确认模型文件存在
- [ ] 数据目录已准备：`/path/to/data`
- [ ] Docker已安装并运行：`docker info`
- [ ] GPU可用（可选但推荐）：`nvidia-smi`

配置文件检查：

- [ ] `unicalib_C_plus_plus/config/sensors.yaml` 存在
- [ ] 传感器配置正确（sensor_id, topic, frame_id）
- [ ] 必需工具路径已设置（DM-Calib, learn-to-calibrate）
- [ ] 可选工具路径已按需设置（MIAS-LCEC, iKalibr等）

---

**提示**: 首次使用建议先运行 `./auto_config_env.sh` 自动检测和配置所有工具路径！
