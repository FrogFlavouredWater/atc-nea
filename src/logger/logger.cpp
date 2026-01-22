#include "logger.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iostream>

namespace {
    std::shared_ptr<spdlog::logger> get_logger() {
        static auto logger = spdlog::stdout_color_mt("console");
        logger->set_pattern("%v");
        return logger;
    }
}

void Logger::debug(const std::string& msg) {
    get_logger()->info("\033[90m[DEBUG]\033[0m      {}", msg); // Grey
}

void Logger::info(const std::string& msg) {
    get_logger()->info("\033[36m[INFO]\033[0m       {}", msg); // Cyan
}

void Logger::warn(const std::string& msg) {
    get_logger()->warn("\033[33m[WARN]\033[0m       {}", msg); // Yellow
}

void Logger::error(const std::string& msg) {
    get_logger()->error("\033[91m[ERROR]\033[0m      {}", msg); // Red
}

void Logger::critical(const std::string& msg) {
    get_logger()->critical("\033[1;31m[CRITICAL]\033[0m   {}", msg); // Bold Red
}

void Logger::fatal(const std::string& msg) {
    get_logger()->critical("\033[1;4;31m[FATAL]\033[0m      {}", msg); // Bold Underline Red
}

void Logger::success(const std::string& msg) {
    get_logger()->info("\033[32m[SUCCESS]\033[0m    {}", msg); // Green
}

void Logger::command(const std::string& msg) {
    get_logger()->info("\033[34m[COMMAND]\033[0m    {}", msg); // Blue
}

void Logger::ok(const std::string& msg) {
    get_logger()->info("\033[32m[OK]\033[0m         {}", msg); // Green
}

void Logger::fail(const std::string& msg) {
    get_logger()->error("\033[91m[FAIL]\033[0m       {}", msg); // Bright Red
}

void Logger::spacer() {
    std::cout << "------------------------------------------" << std::endl;
}

void Logger::test() {
        Logger::info("Info message");
        Logger::debug("Debug message");
        Logger::warn("Warning message");
        Logger::error("Error message");
        Logger::critical("Critical message");
        Logger::fatal("Fatal message");
        Logger::command("Command message");
        Logger::ok("OK message");
        Logger::success("Success message");
        Logger::fail("Fail message");
        Logger::spacer();
}