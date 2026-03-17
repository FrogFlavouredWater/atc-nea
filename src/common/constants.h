#pragma once
#include <cstdint>
#include <string>

struct DisplaySettings
{
    static constexpr int32_t SCREEN_WIDTH = 1280;
    static constexpr int32_t SCREEN_HEIGHT = 720;
    bool FULLSCREEN;
    const int32_t TARGET_FPS;
    const int HALF_WIDTH = SCREEN_WIDTH / 2;
    static constexpr int HALF_HEIGHT = SCREEN_HEIGHT / 2;
};

struct GameSettings
{
    static constexpr int32_t MAX_AIRCRAFT = 5;
    static constexpr int32_t AIRCRAFT_SIZE = 12;
    const double PIXELS_PER_NM;
    const double SIMULATION_SPEED;
};

enum class GameState
{
    MENU,
    RUNNING,
    PAUSED,
    GAME_OVER,
    EXIT
};

struct AudioSettings
{
    const double MASTER_VOLUME;
    const bool EFFECTS_ENABLED;
};

struct Constants
{
    const DisplaySettings display;
    const GameSettings game;
    const AudioSettings audio;

    Constants(const DisplaySettings& d, const GameSettings& g, const AudioSettings& a) : display(d),
        game(g),
        audio(a)
    {
    };
};

extern const Constants CONSTANTS;