#include "core/Logger.h"

#include <iostream>

namespace {
void write(const char* tag, const char* color, const char* pad, const std::string& msg) {
    std::cout << color << tag << "\033[0m" << pad << msg << std::endl;
}
}

// These log helpers write ANSI-coloured console output when the terminal
// supports it. The simulator only relies on the text, not the colour.
void Logger::debug(const std::string& msg) {
    write("[DEBUG]", "\033[90m", "      ", msg);
}

void Logger::info(const std::string& msg) {
    write("[INFO]", "\033[36m", "       ", msg);
}

void Logger::warn(const std::string& msg) {
    write("[WARN]", "\033[33m", "       ", msg);
}

void Logger::error(const std::string& msg) {
    write("[ERROR]", "\033[91m", "      ", msg);
}

void Logger::critical(const std::string& msg) {
    write("[CRITICAL]", "\033[1;31m", "   ", msg);
}

void Logger::fatal(const std::string& msg) {
    write("[FATAL]", "\033[1;4;31m", "      ", msg);
}

void Logger::success(const std::string& msg) {
    write("[SUCCESS]", "\033[32m", "    ", msg);
}

void Logger::command(const std::string& msg) {
    write("[COMMAND]", "\033[34m", "    ", msg);
}

void Logger::ok(const std::string& msg) {
    write("[OK]", "\033[32m", "         ", msg);
}

void Logger::fail(const std::string& msg) {
    write("[FAIL]", "\033[91m", "       ", msg);
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
