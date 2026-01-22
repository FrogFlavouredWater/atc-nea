#pragma once
#include <cstdint>
#include <string>

struct DisplaySettings
{
    const int32_t SCREEN_WIDTH;
    const int32_t SCREEN_HEIGHT;
    const bool FULLSCREEN;
    const int32_t TARGET_FPS;
    const int HALF_WIDTH = SCREEN_WIDTH / 2;
    const int HALF_HEIGHT = SCREEN_HEIGHT / 2;
};

struct GameSettings
{
    const float AIRCRAFT_SPEED;
    const int32_t MAX_AIRCRAFT;
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
    const float MASTER_VOLUME;
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
