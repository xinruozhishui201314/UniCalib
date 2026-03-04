# UniCalib - 单元测试 CMake 配置

cmake_minimum_required(VERSION 3.14)
project(unicalib_tests)

# 查找依赖
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTest REQUIRED)

# 设置包含路径
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../include
)

# 测试可执行文件
add_executable(test_math_safety test_math_safety.cpp)

# 链接库
target_link_libraries(test_math_safety
    ${OpenCV_LIBS}
    GTest::gtest
    GTest::gtest_main
)

# 添加测试
enable_testing()
include(GoogleTest)
add_test(NAME MathSafety COMMAND test_math_safety)

# 编译选项
target_compile_options(test_math_safety PRIVATE
    -Wall -Wextra -Wpedantic
    -Wno-unused-parameter
)

# 调试信息
target_compile_options(test_math_safety PRIVATE $<$<CONFIG:Debug>:-g -O0>)
target_compile_options(test_math_safety PRIVATE $<$<CONFIG:Release>:-O3>)

# 定义测试启用宏
add_definitions(-DENABLE_TESTING)
