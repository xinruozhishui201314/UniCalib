# =============================================================================
# UniCalib Unified — 多传感器统一标定系统
# 五种标定: IMU内参 / 相机内参 / IMU-LiDAR外参 / LiDAR-Camera外参 / Camera-Camera外参
# 独立标定 + 联合标定
# 无 tiny-viewer 依赖 / 无 ROS 强制依赖 (可选 ROS2 bag 读取)
# =============================================================================
cmake_minimum_required(VERSION 3.20)
project(UniCalib VERSION 2.0.0 LANGUAGES C CXX)

# ---------------------------------------------------------------------------
# C++ 标准与编译选项
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_C_FLAGS_RELEASE   "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_C_FLAGS_DEBUG   "-g -O1")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O1")

# 打开额外优化标志（仅 Release）
if(CMAKE_BUILD_TYPE STREQUAL "Release")
  add_compile_options(-march=native -funroll-loops)
endif()

# 输出目录
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 模块路径
list(APPEND CMAKE_MODULE_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus/cmake_modules
)

# ---------------------------------------------------------------------------
# 选项
# ---------------------------------------------------------------------------
option(UNICALIB_BUILD_APPS      "Build application executables"        ON)
option(UNICALIB_WITH_ROS2       "Enable optional ROS2 bag reading"     OFF)
option(UNICALIB_WITH_PCL_VIZ    "Enable PCL visualizer"                ON)
option(UNICALIB_WITH_OPENMP     "Enable OpenMP parallelism"            ON)
option(UNICALIB_WITH_PYTHON_BRIDGE "Enable AI coarse calib (Python)"   ON)
option(UNICALIB_BUILD_TESTS      "Build unit tests"                     OFF)
# iKalibr B-spline engine: requires velodyne_msgs, full ROS2, ufomap, opengv.
# Default ON; set OFF for quick lean builds (e.g. -DUNICALIB_WITH_IKALIBR=OFF).
option(UNICALIB_WITH_IKALIBR    "Build iKalibr B-spline calibration engine" ON)
option(BUILD_TESTING            "Build tests"                          OFF)

# ---------------------------------------------------------------------------
# 第三方库: Eigen3 (header-only, from system or bundled)
# ---------------------------------------------------------------------------
find_package(Eigen3 QUIET)
if(NOT Eigen3_FOUND)
  # 尝试从系统 /usr/include/eigen3
  find_path(EIGEN3_INCLUDE_DIR NAMES Eigen/Core
    HINTS /usr/include/eigen3 /usr/local/include/eigen3 /opt/ros/humble/include/eigen3)
  if(EIGEN3_INCLUDE_DIR)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
    message(STATUS "Eigen3 found at: ${EIGEN3_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "Eigen3 not found. Install: sudo apt install libeigen3-dev")
  endif()
else()
  message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")
endif()

# ---------------------------------------------------------------------------
# 第三方库: Sophus (header-only, from thirdparty/Sophus)
# ---------------------------------------------------------------------------
set(SOPHUS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus)
if(EXISTS ${SOPHUS_SOURCE_DIR}/CMakeLists.txt)
  set(SOPHUS_INSTALL OFF CACHE BOOL "" FORCE)
  set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "" FORCE)
  set(BUILD_SOPHUS_EXAMPLES OFF CACHE BOOL "" FORCE)
  add_subdirectory(${SOPHUS_SOURCE_DIR} thirdparty_build/Sophus EXCLUDE_FROM_ALL)
  message(STATUS "Sophus: using bundled source")
else()
  find_package(Sophus REQUIRED)
  message(STATUS "Sophus: found in system")
endif()

# ---------------------------------------------------------------------------
# 第三方库: magic_enum (header-only)
# ---------------------------------------------------------------------------
add_library(magic_enum INTERFACE)
target_include_directories(magic_enum INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/magic_enum/include)
add_library(magic_enum::magic_enum ALIAS magic_enum)

# ---------------------------------------------------------------------------
# 第三方库: cereal (header-only)
# ---------------------------------------------------------------------------
add_library(cereal_unicalib INTERFACE)
target_include_directories(cereal_unicalib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/include
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal_eigen_include)
add_library(cereal::cereal ALIAS cereal_unicalib)

# ---------------------------------------------------------------------------
# 第三方库: veta-impl (自包含 SfM 数据结构, 无外部依赖)
# 完全替代外部 veta 库; 头文件位于 thirdparty/veta-impl/include/
# ---------------------------------------------------------------------------
set(VETA_IMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/veta-impl/include)
add_library(veta_impl INTERFACE)
target_include_directories(veta_impl INTERFACE ${VETA_IMPL_DIR})
target_link_libraries(veta_impl INTERFACE Eigen3::Eigen)
add_library(veta::veta   ALIAS veta_impl)
add_library(ns_veta::veta ALIAS veta_impl)
message(STATUS "veta: using self-contained veta-impl at ${VETA_IMPL_DIR}")

# ---------------------------------------------------------------------------
# 第三方库: tiny-viewer stubs (no-op, 仅提供 ns_viewer:: 骨架类型)
# ---------------------------------------------------------------------------
add_library(tiny_viewer_stub INTERFACE)
target_include_directories(tiny_viewer_stub INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/ikalibr  # viewer_types.h is in viewer/ subdir
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/tiny-viewer-stub
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj_full/tiny-viewer_stub)
add_library(tiny_viewer::stub ALIAS tiny_viewer_stub)

# ---------------------------------------------------------------------------
# 第三方库: Pangolin 0.9.0 (C++ 可视化, 替代 tiny-viewer)
# 从 thirdparty/Pangolin/Pangolin-0.9.0 源码编译
# ---------------------------------------------------------------------------
set(PANGOLIN_SOURCE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Pangolin/Pangolin-0.9.0)
if(EXISTS ${PANGOLIN_SOURCE_DIR}/CMakeLists.txt)
  set(BUILD_EXAMPLES    OFF CACHE BOOL "Pangolin: disable examples"    FORCE)
  set(BUILD_TOOLS       OFF CACHE BOOL "Pangolin: disable tools"       FORCE)
  set(BUILD_PANGOLIN_PYPANGOLIN OFF CACHE BOOL "Pangolin: no Python"   FORCE)
  set(BUILD_ASAN        OFF CACHE BOOL "Pangolin: no ASan"             FORCE)
  add_subdirectory(${PANGOLIN_SOURCE_DIR}
                   ${CMAKE_BINARY_DIR}/thirdparty_build/Pangolin
                   EXCLUDE_FROM_ALL)
  set(UNICALIB_WITH_PANGOLIN ON)
  add_compile_definitions(UNICALIB_WITH_PANGOLIN=1)
  message(STATUS "Pangolin: building from ${PANGOLIN_SOURCE_DIR}")
else()
  set(UNICALIB_WITH_PANGOLIN OFF)
  add_compile_definitions(UNICALIB_WITH_PANGOLIN=0)
  message(WARNING "Pangolin source not found at ${PANGOLIN_SOURCE_DIR}, using no-op stub")
  # include/ikalibr/pangolin/display/display.h 提供无操作占位
  # (include/ikalibr 已在 target_include_directories 中, 不需要额外添加路径)
endif()

# ---------------------------------------------------------------------------
# 第三方库: basalt-headers + ctraj_basalt_compat (替代 ctraj，B-Spline 轨迹)
# ---------------------------------------------------------------------------
set(BASALT_HEADERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers/include)
if(EXISTS "${BASALT_HEADERS_DIR}/basalt/spline/se3_spline.h")
  message(STATUS "basalt-headers: using ${BASALT_HEADERS_DIR}")
  set(BASALT_HEADERS_FOUND TRUE)
else()
  message(FATAL_ERROR "basalt-headers not found at ${BASALT_HEADERS_DIR}")
  set(BASALT_HEADERS_FOUND FALSE)
endif()
set(CTRAJ_BASALT_COMPAT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj_basalt_compat/include)
if(NOT EXISTS "${CTRAJ_BASALT_COMPAT_DIR}/ctraj/core/spline_bundle.h")
  message(FATAL_ERROR "ctraj_basalt_compat not found at ${CTRAJ_BASALT_COMPAT_DIR}")
endif()
message(STATUS "ctraj: using basalt-headers + ctraj_basalt_compat (replacing ctraj_full)")

# ---------------------------------------------------------------------------
# 系统依赖: Ceres Solver
# ---------------------------------------------------------------------------
find_package(Ceres REQUIRED)
message(STATUS "Ceres: ${CERES_VERSION}")

# ---------------------------------------------------------------------------
# 系统依赖: OpenCV (4.x preferred)
# ---------------------------------------------------------------------------
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc calib3d highgui features2d video)
message(STATUS "OpenCV: ${OpenCV_VERSION}")

# ---------------------------------------------------------------------------
# 系统依赖: PCL
# ---------------------------------------------------------------------------
find_package(PCL REQUIRED COMPONENTS
    common io filters kdtree search segmentation registration features visualization)
message(STATUS "PCL: ${PCL_VERSION}")
# PCL 不能与 -march=native 同时使用时需要禁用
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

# ---------------------------------------------------------------------------
# 系统依赖: spdlog
# ---------------------------------------------------------------------------
find_package(spdlog QUIET)
if(NOT spdlog_FOUND)
  message(STATUS "spdlog not found via find_package, using FetchContent")
  include(FetchContent)
  FetchContent_Declare(spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG        v1.13.0
    GIT_SHALLOW    TRUE)
  FetchContent_MakeAvailable(spdlog)
else()
  message(STATUS "spdlog: ${spdlog_VERSION}")
endif()

# ---------------------------------------------------------------------------
# 系统依赖: yaml-cpp
# ---------------------------------------------------------------------------
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  find_package(PkgConfig QUIET)
  if(PkgConfig_FOUND)
    pkg_check_modules(YAML_CPP yaml-cpp)
  endif()
  if(NOT YAML_CPP_FOUND)
    message(STATUS "yaml-cpp not found via find_package, using FetchContent")
    include(FetchContent)
    FetchContent_Declare(yaml-cpp
      GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
      GIT_TAG        yaml-cpp-0.8.0
      GIT_SHALLOW    TRUE)
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(yaml-cpp)
  endif()
else()
  message(STATUS "yaml-cpp: ${yaml-cpp_VERSION}")
endif()

# ---------------------------------------------------------------------------
# OpenMP (可选)
# ---------------------------------------------------------------------------
if(UNICALIB_WITH_OPENMP)
  find_package(OpenMP)
  if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP: found, version ${OpenMP_CXX_VERSION}")
  else()
    message(STATUS "OpenMP: not found, parallel processing disabled")
  endif()
endif()

# ---------------------------------------------------------------------------
# ROS2 (自动探测 — iKalibr 头文件通过 ros.h shim 封装)
# ---------------------------------------------------------------------------
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(rosbag2_cpp QUIET)
find_package(sensor_msgs QUIET)
if(rclcpp_FOUND)
  message(STATUS "ROS2: found rclcpp, enabling full ROS2 integration")
  add_compile_definitions(UNICALIB_WITH_ROS2=1)
  set(UNICALIB_WITH_ROS2 ON)
else()
  message(STATUS "ROS2: rclcpp not found, using lightweight stub (ros.h stub)")
  add_compile_definitions(UNICALIB_WITH_ROS2=0)
  set(UNICALIB_WITH_ROS2 OFF)
endif()

# ---------------------------------------------------------------------------
# ufomap (必须使用包含 surfel/octree 新 API 的版本, 优先 iKalibr 预编译)
# 旧版 thirdparty/ufomap 无 surfel_map.h / octree/node.h, 不满足要求
# ---------------------------------------------------------------------------
set(UFOMAP_INSTALL_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../iKalibr/thirdparty-install/ufomap-install)

# 仅接受含 surfel_map.h 的新版 ufomap; 先从 iKalibr 预编译找, 再从外部 ufomap_DIR
set(UFOMAP_SEARCH_HINTS
    ${UFOMAP_INSTALL_DIR}/lib/cmake/ufomap)
if(DEFINED ufomap_DIR AND NOT ufomap_DIR STREQUAL "")
  list(PREPEND UFOMAP_SEARCH_HINTS ${ufomap_DIR})
endif()

# 清除旧版缓存条目, 防止本地旧 thirdparty ufomap 被复用
unset(ufomap_DIR CACHE)
find_package(ufomap QUIET HINTS ${UFOMAP_SEARCH_HINTS} NO_DEFAULT_PATH)

# 验证找到的 ufomap 是否含新 API (surfel_map.h)
if(ufomap_FOUND)
  get_target_property(_ufo_inc UFO::Map INTERFACE_INCLUDE_DIRECTORIES)
  if(EXISTS "${_ufo_inc}/ufo/map/surfel_map.h")
    message(STATUS "ufomap: found new API (surfel_map) at ${_ufo_inc}")
  else()
    message(STATUS "ufomap: found but OLD API (no surfel_map.h), disabling")
    set(ufomap_FOUND FALSE)
  endif()
endif()

if(NOT ufomap_FOUND)
  message(STATUS "ufomap: not found or outdated, LiDAR surfel map disabled")
  add_compile_definitions(UNICALIB_NO_UFOMAP=1)
endif()

# ---------------------------------------------------------------------------
# opengv (来自 iKalibr 预编译安装)
# 使用 iKalibr 预编译头文件; find_package 仅用于链接, 失败时直接用 include 路径
# ---------------------------------------------------------------------------
set(OPENGV_INSTALL_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../iKalibr/thirdparty-install/opengv-install)
unset(opengv_DIR CACHE)
find_package(opengv QUIET
    HINTS ${OPENGV_INSTALL_DIR}/lib/cmake/opengv-1.0
    NO_DEFAULT_PATH)
if(opengv_FOUND)
  message(STATUS "opengv: found prebuilt via cmake config")
  add_compile_definitions(UNICALIB_WITH_OPENGV=1)
elseif(EXISTS "${OPENGV_INSTALL_DIR}/include/opengv/types.hpp")
  # cmake config not usable, but headers exist → add include path + import lib manually
  message(STATUS "opengv: using prebuilt headers at ${OPENGV_INSTALL_DIR}/include")
  add_compile_definitions(UNICALIB_WITH_OPENGV=1)
  include_directories(BEFORE ${OPENGV_INSTALL_DIR}/include)
  # Import the opengv library target so we can link it later
  find_library(OPENGV_LIB NAMES opengv
      PATHS ${OPENGV_INSTALL_DIR}/lib NO_DEFAULT_PATH)
  if(OPENGV_LIB)
    add_library(opengv STATIC IMPORTED GLOBAL)
    set_target_properties(opengv PROPERTIES IMPORTED_LOCATION "${OPENGV_LIB}"
        INTERFACE_INCLUDE_DIRECTORIES "${OPENGV_INSTALL_DIR}/include")
    message(STATUS "opengv library: ${OPENGV_LIB}")
  else()
    message(WARNING "opengv: headers found but library not found; link may fail")
  endif()
else()
  # Fall back to bundled opengv headers (copied from iKalibr prebuilt, header-only mode)
  set(_OPENGV_STUB ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/opengv-stub)
  if(EXISTS "${_OPENGV_STUB}/opengv/types.hpp")
    message(STATUS "opengv: using bundled headers at ${_OPENGV_STUB}")
    add_compile_definitions(UNICALIB_WITH_OPENGV=1)
    include_directories(BEFORE ${_OPENGV_STUB})
    # opengv in header-only mode (no .so needed for the SAC template classes)
    add_library(opengv INTERFACE IMPORTED GLOBAL)
    set_target_properties(opengv PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_OPENGV_STUB}")
  else()
    message(STATUS "opengv: not found, disabling opengv-dependent features")
    add_compile_definitions(UNICALIB_NO_OPENGV=1)
  endif()
endif()

# ===========================================================================
# 选项: 是否构建 Python 桥接 (AI 粗标定依赖)
# ===========================================================================
option(UNICALIB_WITH_PYTHON_BRIDGE  "Enable Python subprocess bridge for AI coarse calib" ON)
if(UNICALIB_WITH_PYTHON_BRIDGE)
  message(STATUS "Python bridge: enabled (DM-Calib/MIAS-LCEC/Transformer-IMU/L2Calib)")
  add_compile_definitions(UNICALIB_WITH_PYTHON_BRIDGE=1)
endif()

# ===========================================================================
# 核心标定库: libunicalib
# ===========================================================================

# 收集 iKalibr 核心源文件 (非 ROS-强依赖模块)
set(IKALIBR_CORE_SOURCES
    # 工具函数
    src/ikalibr/util/utils.cpp
    src/ikalibr/util/circlesgrid.cpp
    # 传感器数据结构
    src/ikalibr/sensor/imu.cpp
    src/ikalibr/sensor/lidar.cpp
    src/ikalibr/sensor/camera.cpp
    src/ikalibr/sensor/radar.cpp
    src/ikalibr/sensor/rgbd.cpp
    src/ikalibr/sensor/event.cpp
    src/ikalibr/sensor/sensor_model.cpp
    # 配置解析
    src/ikalibr/config/configor.cpp
    # 标定参数管理
    src/ikalibr/calib/calib_param_manager.cpp
    src/ikalibr/calib/calib_data_manager.cpp
    src/ikalibr/calib/estimator.cpp
    src/ikalibr/calib/ceres_callback.cpp
    src/ikalibr/calib/spat_temp_priori.cpp
    # 核心算法
    src/ikalibr/core/rotation_estimator.cpp
    src/ikalibr/core/lidar_odometer.cpp
    src/ikalibr/core/scan_undistortion.cpp
    src/ikalibr/core/pts_association.cpp
    src/ikalibr/core/feature_tracking.cpp
    src/ikalibr/core/optical_flow_trace.cpp
    src/ikalibr/core/visual_distortion.cpp
    src/ikalibr/core/visual_reproj_association.cpp
    src/ikalibr/core/visual_velocity_estimator.cpp
    src/ikalibr/core/visual_velocity_sac.cpp
    src/ikalibr/core/rot_only_vo.cpp
    src/ikalibr/core/colmap_data_io.cpp
    src/ikalibr/core/radar_velocity_sac.cpp
    src/ikalibr/core/event_preprocessing.cpp
    src/ikalibr/core/event_trace_sac.cpp
    src/ikalibr/core/ev_circle_tracking.cpp
    src/ikalibr/core/haste_data_io.cpp
    # SfM (条件编译, 依赖 veta/opengv)
    src/ikalibr/core/vision_only_sfm.cpp
    # 因子
    src/ikalibr/factor/data_correspondence.cpp
    src/ikalibr/factor/factor_instantiation.cpp
    # 求解器 (分块实现)
    src/ikalibr/solver/calib_solver_common_impl.cpp
    src/ikalibr/solver/calib_solver_proc_impl.cpp
    src/ikalibr/solver/calib_solver_da_impl.cpp
    src/ikalibr/solver/calib_solver_bo_impl.cpp
    src/ikalibr/solver/calib_solver_init_so3_spline_impl.cpp
    src/ikalibr/solver/calib_solver_init_scale_spline_impl.cpp
    src/ikalibr/solver/calib_solver_init_sen_inertial_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_li_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ri_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_pci_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_vci_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_di_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_circle_based_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ei_align_line_based_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_ii_align_impl.cpp
    src/ikalibr/solver/calib_solver_init_prep_bo_impl.cpp
    src/ikalibr/solver/calib_solver_io.cpp
    # 可视化 (viewer.cpp + viewer_pangolin.cpp 仅在 UNICALIB_WITH_PANGOLIN 时加入，见下方)
    src/ikalibr/viewer/visual_ang_vel_drawer.cpp
    src/ikalibr/viewer/visual_colorized_cloud_map.cpp
    src/ikalibr/viewer/visual_gravity.cpp
    src/ikalibr/viewer/visual_lidar_covisibility.cpp
    src/ikalibr/viewer/visual_lin_vel_drawer.cpp
)

# 数据加载器 (含 ROS2 可选支持)
set(IKALIBR_LOADER_SOURCES
    src/ikalibr/sensor/imu_data_loader.cpp
    src/ikalibr/sensor/lidar_data_loader.cpp
    src/ikalibr/sensor/camera_data_loader.cpp
    src/ikalibr/sensor/radar_data_loader.cpp
    src/ikalibr/sensor/depth_data_loader.cpp
    src/ikalibr/sensor/event_data_loader.cpp
)

# UniCalib 新增统一接口源文件
set(UNICALIB_NEW_SOURCES
    # 公共数据结构
    src/common/sensor_types.cpp
    src/common/calib_param.cpp
    # 内参标定
    src/intrinsic/allan_variance.cpp
    src/intrinsic/camera_pinhole_calib.cpp
    # 外参标定
    src/extrinsic/imu_lidar_calib.cpp
    src/extrinsic/lidar_camera_calib.cpp
    src/extrinsic/cam_cam_calib.cpp
    # 联合求解
    src/solver/joint_calib_solver.cpp
    # ─── 两阶段流水线 (新增) ─────────────────────────────────────────
    src/pipeline/calib_pipeline.cpp
    src/pipeline/manual_calib.cpp
    src/pipeline/ai_coarse_calib.cpp
    # ─────────────────────────────────────────────────────────────────
    # 可视化 (PCL + OpenCV, 替换 tiny-viewer)
    src/viz/cloud_viewer.cpp
    src/viz/report_gen.cpp
    # I/O
    src/io/yaml_io.cpp
)

# iKalibr sources are optional (requires full ROS2 + velodyne_msgs + ufomap + opengv)
if(UNICALIB_WITH_IKALIBR)
    add_compile_definitions(UNICALIB_WITH_IKALIBR=1)
    message(STATUS "iKalibr: enabled (full B-spline engine)")
    set(_UNICALIB_SOURCES ${IKALIBR_CORE_SOURCES} ${IKALIBR_LOADER_SOURCES} ${UNICALIB_NEW_SOURCES})
    if(UNICALIB_WITH_PANGOLIN)
      list(APPEND _UNICALIB_SOURCES
        src/ikalibr/viewer/viewer.cpp
        src/ikalibr/viewer/viewer_pangolin.cpp
      )
    endif()
    add_library(unicalib SHARED ${_UNICALIB_SOURCES})
else()
    message(STATUS "iKalibr: disabled (lean build — use -DUNICALIB_WITH_IKALIBR=ON to enable B-spline engine)")
    add_compile_definitions(UNICALIB_WITH_IKALIBR=0)
    add_library(unicalib SHARED
        ${UNICALIB_NEW_SOURCES}
    )
endif()

# ---------------------------------------------------------------------------
# 编译选项: IKALIBR_VIEWER_DISABLED (无 Pangolin 时使用 stub，不编 viewer.cpp)
# ---------------------------------------------------------------------------
if(UNICALIB_WITH_PANGOLIN)
  target_compile_definitions(unicalib PUBLIC IKALIBR_VIEWER_DISABLED=0)
  message(STATUS "iKalibr viewer: Pangolin backend (viewer.cpp + viewer_pangolin.cpp)")
else()
  target_compile_definitions(unicalib PUBLIC
      IKALIBR_VIEWER_DISABLED=TRUE
      NS_IKALIBR_VIEWER_STUB=1
  )
  message(STATUS "iKalibr viewer: stub (no GUI)")
endif()

# ---------------------------------------------------------------------------
# Include 路径
# ---------------------------------------------------------------------------
target_include_directories(unicalib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ikalibr>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/unicalib>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/unicalib/pipeline>
    # 第三方头文件
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/magic_enum/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal_eigen_include>
    # tiny-viewer stubs (ns_viewer:: 骨架类型)
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/tiny-viewer-stub>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/ctraj_full/tiny-viewer_stub>
    # ctraj compat (basalt-headers backend)
    $<BUILD_INTERFACE:${CTRAJ_BASALT_COMPAT_DIR}>
    $<BUILD_INTERFACE:${BASALT_HEADERS_DIR}>
    # veta (veta_impl self-contained headers)
    $<BUILD_INTERFACE:${VETA_IMPL_DIR}>
    # PCL
    ${PCL_INCLUDE_DIRS}
    $<INSTALL_INTERFACE:include>
)

# veta: 使用 veta_impl（自包含头文件库，include 路径由 INTERFACE 自动传递）
target_link_libraries(unicalib PUBLIC veta_impl)

# ufomap: UFO::Map INTERFACE_INCLUDE_DIRECTORIES 自动传递; 额外显式添加保证鲁棒性
if(ufomap_FOUND)
  target_include_directories(unicalib PUBLIC
      $<BUILD_INTERFACE:${UFOMAP_INSTALL_DIR}/include>)
endif()

# ROS2 include paths (rclcpp, sensor_msgs, pcl_conversions, etc.)
# In ROS2 ament, each package has headers at /opt/ros/<distro>/include/<pkg>/<pkg>/
# We add the package-level dirs so that '#include <pkg/header.h>' works.
if(UNICALIB_WITH_ROS2 AND rclcpp_FOUND)
    set(_ROS2_INCLUDE_BASE "/opt/ros/humble/include")
    if(EXISTS "${_ROS2_INCLUDE_BASE}")
        # Collect all package-level include dirs (ament convention)
        file(GLOB _ROS2_PKG_INCLUDES LIST_DIRECTORIES true "${_ROS2_INCLUDE_BASE}/*")
        target_include_directories(unicalib PUBLIC
            ${_ROS2_INCLUDE_BASE}        # top-level (for some old-style includes)
            ${_ROS2_PKG_INCLUDES}        # per-package dirs (new ament convention)
        )
    endif()
    if(rclcpp_INCLUDE_DIRS)
        target_include_directories(unicalib PUBLIC ${rclcpp_INCLUDE_DIRS})
    endif()
endif()

# ---------------------------------------------------------------------------
# 链接库
# ---------------------------------------------------------------------------
target_link_libraries(unicalib PUBLIC
    # 数学/几何
    Eigen3::Eigen
    Sophus::Sophus
    # 优化
    Ceres::ceres
    # B-Spline: basalt-headers + ctraj_basalt_compat (header-only, no ctraj lib)
    # 图像
    ${OpenCV_LIBS}
    # 点云
    ${PCL_LIBRARIES}
    # 序列化 & 配置
    yaml-cpp
    # 日志
    spdlog::spdlog
    # 枚举
    magic_enum::magic_enum
    # filesystem (GCC < 9 需要显式链接)
    $<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_LESS:$<CXX_COMPILER_VERSION>,9.0>>:stdc++fs>
)

# opengv: link if target exists (found via cmake config, prebuilt lib, or bundled headers)
if(TARGET opengv)
  target_link_libraries(unicalib PUBLIC opengv)
endif()

# ufomap (可选): 使用正确的 CMake target 名 UFO::Map (而非 ufomap)
if(ufomap_FOUND)
  # UFO::Map INTERFACE_LINK_LIBRARIES 要求 lz4 和 tbb 作为 cmake target 或 linker flag
  # 若系统只有运行时库 (liblz4.so.1 无 .so 符号链接), 手动创建 IMPORTED target
  if(NOT TARGET lz4)
    find_library(LZ4_LIBRARY NAMES lz4
        PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
        PATH_SUFFIXES "" NO_DEFAULT_PATH)
    if(NOT LZ4_LIBRARY)
      # 回退: 搜索带版本号的 .so
      find_library(LZ4_LIBRARY NAMES liblz4.so.1 lz4
          PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
    endif()
    if(LZ4_LIBRARY)
      add_library(lz4 SHARED IMPORTED)
      set_target_properties(lz4 PROPERTIES IMPORTED_LOCATION "${LZ4_LIBRARY}")
      message(STATUS "lz4 library: ${LZ4_LIBRARY}")
    else()
      message(WARNING "liblz4 not found; UFO::Map may fail to link")
    endif()
  endif()
  target_link_libraries(unicalib PUBLIC UFO::Map)
endif()

# Pangolin (可视化): pango_display 包含 pangolin/display/display.h 所需的所有 headers
if(UNICALIB_WITH_PANGOLIN)
  target_link_libraries(unicalib PUBLIC pango_display)
endif()

# OpenMP (可选)
if(OpenMP_CXX_FOUND AND UNICALIB_WITH_OPENMP)
  target_link_libraries(unicalib PUBLIC OpenMP::OpenMP_CXX)
endif()

# ROS2 (可选)
if(UNICALIB_WITH_ROS2)
  target_link_libraries(unicalib PUBLIC
      rclcpp::rclcpp
      rosbag2_cpp::rosbag2_cpp)
endif()

# ===========================================================================
# 可执行程序
# ===========================================================================
if(UNICALIB_BUILD_APPS)
  # 辅助函数: 添加标定应用
  function(add_calib_app target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE unicalib)
    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  endfunction()

  # IMU 内参标定
  add_calib_app(unicalib_imu_intrinsic
      apps/imu_intrinsic/main.cpp)

  # 相机内参标定 (针孔 + 鱼眼)
  add_calib_app(unicalib_camera_intrinsic
      apps/camera_intrinsic/main.cpp)

  # IMU-LiDAR 外参标定
  add_calib_app(unicalib_imu_lidar
      apps/imu_lidar_extrin/main.cpp)

  # LiDAR-Camera 外参标定
  add_calib_app(unicalib_lidar_camera
      apps/lidar_camera_extrin/main.cpp)

  # Camera-Camera 外参标定
  add_calib_app(unicalib_cam_cam
      apps/cam_cam_extrin/main.cpp)

  # 联合标定 (全传感器)
  add_calib_app(unicalib_joint
      apps/joint_calib/main.cpp)

  # iKalibr 主程序和 IMU 内参工具仅在 UNICALIB_WITH_IKALIBR=ON 时构建
  if(UNICALIB_WITH_IKALIBR)
    add_executable(unicalib_ikalibr_prog
        src/ikalibr/exe/solver/main.cpp)
    target_link_libraries(unicalib_ikalibr_prog PRIVATE unicalib)

    add_executable(unicalib_imu_intri_tool
        src/ikalibr/nofree/imu_intri_calib.cpp)
    target_link_libraries(unicalib_imu_intri_tool PRIVATE unicalib)
  endif()
endif()

# ===========================================================================
# 测试 (可选)
# ===========================================================================
if(UNICALIB_BUILD_TESTS)
    find_package(GTest REQUIRED)
    enable_testing()
    include(GoogleTest)
    add_subdirectory(tests)
endif()

# ===========================================================================
# 安装规则
# ===========================================================================
include(GNUInstallDirs)
install(TARGETS unicalib
    EXPORT UniCalibTargets
    LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(DIRECTORY config/
    DESTINATION ${CMAKE_INSTALL_DATADIR}/unicalib/config)

# ===========================================================================
# 构建摘要
# ===========================================================================
message(STATUS "")
message(STATUS "========= UniCalib v${PROJECT_VERSION} 构建配置 =========")
message(STATUS "  Build type:    ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ standard:  ${CMAKE_CXX_STANDARD}")
message(STATUS "  Eigen3:        ${EIGEN3_VERSION_STRING}")
message(STATUS "  OpenCV:        ${OpenCV_VERSION}")
message(STATUS "  PCL:           ${PCL_VERSION}")
message(STATUS "  Ceres:         ${CERES_VERSION}")
message(STATUS "  OpenMP:        ${OpenMP_CXX_FOUND}")
message(STATUS "  opengv:        bundled headers (${_OPENGV_STUB})")
message(STATUS "  ufomap:        ${ufomap_FOUND}")
message(STATUS "  veta:          self-contained (veta-impl)")
message(STATUS "  Pangolin:      ${UNICALIB_WITH_PANGOLIN}")
message(STATUS "  ROS2:          ${UNICALIB_WITH_ROS2}")
message(STATUS "  Build apps:    ${UNICALIB_BUILD_APPS}")
message(STATUS "  PCL viz:       ${UNICALIB_WITH_PCL_VIZ}")
message(STATUS "=======================================================")
