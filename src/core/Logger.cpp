#include "core/Logger.h"

#include <iostream>

// These log helpers write ANSI-coloured console output when the terminal
// supports it. The simulator only relies on the text, not the colour.
void Logger::debug(const std::string& msg) {
    std::cout << "\033[90m[DEBUG]\033[0m      " << msg << std::endl;
}

void Logger::info(const std::string& msg) {
    std::cout << "\033[36m[INFO]\033[0m       " << msg << std::endl;
}

void Logger::warn(const std::string& msg) {
    std::cout << "\033[33m[WARN]\033[0m       " << msg << std::endl;
}

void Logger::error(const std::string& msg) {
    std::cout << "\033[91m[ERROR]\033[0m      " << msg << std::endl;
}

void Logger::critical(const std::string& msg) {
    std::cout << "\033[1;31m[CRITICAL]\033[0m   " << msg << std::endl;
}

void Logger::fatal(const std::string& msg) {
    std::cout << "\033[1;4;31m[FATAL]\033[0m      " << msg << std::endl;
}

void Logger::success(const std::string& msg) {
    std::cout << "\033[32m[SUCCESS]\033[0m    " << msg << std::endl;
}

void Logger::command(const std::string& msg) {
    std::cout << "\033[34m[COMMAND]\033[0m    " << msg << std::endl;
}

void Logger::ok(const std::string& msg) {
    std::cout << "\033[32m[OK]\033[0m         " << msg << std::endl;
}

void Logger::fail(const std::string& msg) {
    std::cout << "\033[91m[FAIL]\033[0m       " << msg << std::endl;
}

void Logger::spacer() {
    std::cout << "------------------------------------------" << std::endl;
}

void Logger::test() {
    // Handy for checking log formatting while developing locally.
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
