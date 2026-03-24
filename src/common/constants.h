#pragma once

#include <cstdint>

enum class ScreenMode {
    WINDOWED,
    BORDERLESS_WINDOWED,
    FULLSCREEN
};

struct DisplaySettings {
    int32_t screenWidth;
    int32_t screenHeight;
    ScreenMode screenMode;
    int32_t targetFps;
};

struct SimSettings {
    int32_t maxAircraft;
    int32_t aircraftSize;
    double pixelsPerNm;
    double simulationSpeed;
    double minXNm;
    double maxXNm;
    double minYNm;
    double maxYNm;
};

struct AppSettings {
    DisplaySettings display;
    SimSettings sim;
};

namespace DisplayConfig {
    inline constexpr DisplaySettings DEFAULTS{
        1536,
        864,
        ScreenMode::WINDOWED,
        120
    };
}

namespace SimConfig {
    inline constexpr SimSettings DEFAULTS{
        5,
        12,
        10.5,
        5.0,
        -70.0,
        70.0,
        -40.0,
        40.0
    };
}

namespace AppConfig {
    inline constexpr AppSettings DEFAULTS{
        DisplayConfig::DEFAULTS,
        SimConfig::DEFAULTS
    };
}

enum class GameState {
    MENU,
    RUNNING,
    PAUSED,
    GAME_OVER,
    EXIT
};
