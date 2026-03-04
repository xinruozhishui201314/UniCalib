#pragma once
/**
 * UniCalib Unified — 统一日志系统 (spdlog 包装)
 * 日志级别: TRACE / DEBUG / INFO / WARN / ERROR / CRITICAL
 * 同时输出到控制台 (带颜色) 和文件
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/fmt/ostr.h>
#include <memory>
#include <string>

namespace ns_unicalib {

class Logger {
public:
    static void init(const std::string& name = "UniCalib",
                     const std::string& log_file = "",
                     spdlog::level::level_enum level = spdlog::level::info) {
        std::vector<spdlog::sink_ptr> sinks;
        // 彩色控制台
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(level);
        console_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] [%n] %v");
        sinks.push_back(console_sink);
        // 文件 (可选)
        if (!log_file.empty()) {
            auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
                log_file, true);
            file_sink->set_level(spdlog::level::trace);
            file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%n] %v");
            sinks.push_back(file_sink);
        }
        auto logger = std::make_shared<spdlog::logger>(name, sinks.begin(), sinks.end());
        logger->set_level(level);
        spdlog::set_default_logger(logger);
    }

    static void set_level(spdlog::level::level_enum level) {
        spdlog::set_level(level);
    }

    static spdlog::logger& get() { return *spdlog::default_logger(); }
};

}  // namespace ns_unicalib

// ===================================================================
// 便利宏
// ===================================================================
#define UNICALIB_TRACE(...)    spdlog::trace(__VA_ARGS__)
#define UNICALIB_DEBUG(...)    spdlog::debug(__VA_ARGS__)
#define UNICALIB_INFO(...)     spdlog::info(__VA_ARGS__)
#define UNICALIB_WARN(...)     spdlog::warn(__VA_ARGS__)
#define UNICALIB_ERROR(...)    spdlog::error(__VA_ARGS__)
#define UNICALIB_CRITICAL(...) spdlog::critical(__VA_ARGS__)

// 进度日志 (带模块名)
#define UNICALIB_LOG_STEP(step_name, msg, ...) \
    spdlog::info("[{}] " msg, step_name, ##__VA_ARGS__)
