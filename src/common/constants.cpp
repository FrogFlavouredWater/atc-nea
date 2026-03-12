#include "common/constants.h"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

const Constants CONSTANTS = []() {
    // Try to find the constants file in several locations
    std::vector<std::string> searchPaths = {
        "src/data/constants.json",             // Relative to project root
        "../src/data/constants.json",          // Relative to build dir
        "./src/data/constants.json"            // Relative to current dir
    };

    std::ifstream f;
    for (const auto& path : searchPaths) {
        f.open(path);
        if (f.is_open()) break;
    }

    if (!f.is_open()) {
        throw std::runtime_error("Failed to open constants.json from any search path");
    }
    
    json data = json::parse(f);
    
    DisplaySettings display{
        data["display"]["SCREEN_WIDTH"].get<int32_t>(),
        data["display"]["SCREEN_HEIGHT"].get<int32_t>(),
        data["display"]["FULLSCREEN"].get<bool>(),
        data["display"]["TARGET_FPS"].get<int32_t>()
    };
    
    GameSettings game{
        data["game"]["MAX_AIRCRAFT"].get<int32_t>(),
        data["game"]["AIRCRAFT_SIZE"].get<int32_t>(),
        data["game"]["PIXELS_PER_NM"].get<double>(),
        data["game"]["SIMULATION_SPEED"].get<double>()
    };
    
    AudioSettings audio{
        data["audio"]["MASTER_VOLUME"].get<double>(),
        data["audio"]["EFFECTS_ENABLED"].get<bool>()
    };
    
    return Constants(display, game, audio);
}();