#include "app/engine.h"

#include "core/Logger.h"
#include <exception>

// Tiny bootstrap. easier to spot startup/shutdown failures.
int main() {
    try {
        Logger::info("Launching ATC Simulator");

        Engine& engine = Engine::getInstance();
        engine.init();
        engine.run();

        Logger::info("ATC Simulator exited successfully");
        return 0;
    } catch (const std::exception& exception) {
        // Final log, then non-zero exit. lets scripts see startup failed.
        Logger::fatal(std::string("Unhandled exception: ") + exception.what());
    } catch (...) {
        Logger::fatal("Unhandled unknown exception");
    }

    return 1;
}
