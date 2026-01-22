#include "constants.h"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

const Constants CONSTANTS = []() {
    std::ifstream f("src/data/constants.json");
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open constants.json");
    }
    
    json data = json::parse(f);
    
    DisplaySettings display{
        data["display"]["SCREEN_WIDTH"].get<int32_t>(),
        data["display"]["SCREEN_HEIGHT"].get<int32_t>(),
        data["display"]["FULLSCREEN"].get<bool>(),
        data["display"]["TARGET_FPS"].get<int32_t>()
    };
    
    GameSettings game{
        data["game"]["AIRCRAFT_SPEED"].get<float>(),
        data["game"]["MAX_AIRCRAFT"].get<int32_t>()
    };
    
    AudioSettings audio{
        data["audio"]["MASTER_VOLUME"].get<float>(),
        data["audio"]["EFFECTS_ENABLED"].get<bool>()
    };
    
    return Constants(display, game, audio);
}();