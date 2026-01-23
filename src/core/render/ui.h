#pragma once
#include "raylib.h"
#include "constants/constants.h"

class UI {
public:
    UI() = default;

    void DrawMainMenu(GameState& currentState);
    void DrawSimulationHUD(int aircraftCount);
    void DrawPauseMenu(GameState& currentState);
    void DrawBackground();
};