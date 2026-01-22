#pragma once
#include <string>

class Logger {
public:
    static void debug(const std::string& msg);
    static void info(const std::string& msg);
    static void warn(const std::string& msg);
    static void error(const std::string& msg);
    static void critical(const std::string& msg);
    static void fatal(const std::string& msg);
    static void success(const std::string& msg);
    static void command(const std::string& msg);
    static void ok(const std::string& msg);
    static void fail(const std::string& msg);
    static void spacer();

    static void test();
};