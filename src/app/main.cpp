#include "app/engine.h"
#include "common/logger.h"
#include <exception>

int main() {
    try {
        Logger::info("Launching ATC Simulator");

        Engine& engine = Engine::getInstance();
        engine.init();
        engine.run();

        Logger::info("ATC Simulator exited successfully");
        return 0;
    } catch (const std::exception& exception) {
        Logger::fatal(std::string("Unhandled exception: ") + exception.what());
    } catch (...) {
        Logger::fatal("Unhandled unknown exception");
    }

    return 1;
}
