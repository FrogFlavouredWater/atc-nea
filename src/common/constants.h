#pragma once

#include <cstdint>

namespace DisplayConfig {
inline constexpr int32_t SCREEN_WIDTH = 1280;
inline constexpr int32_t SCREEN_HEIGHT = 720;
inline constexpr bool FULLSCREEN = false;
inline constexpr int32_t TARGET_FPS = 60;
inline constexpr int32_t HALF_WIDTH = SCREEN_WIDTH / 2;
inline constexpr int32_t HALF_HEIGHT = SCREEN_HEIGHT / 2;
}

namespace SimConfig {
inline constexpr int32_t MAX_AIRCRAFT = 5;
inline constexpr int32_t AIRCRAFT_SIZE = 12;
inline constexpr double PIXELS_PER_NM = 8.0;
inline constexpr double SIMULATION_SPEED = 30.0;
}

enum class GameState {
    MENU,
    RUNNING,
    PAUSED,
    GAME_OVER,
    EXIT
};
