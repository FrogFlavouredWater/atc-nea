#pragma once
#include "raylib.h"
#include "constants/constants.h"

// Forward declare or include your GameState enum here
// #include "core/types.h" 

class UI {
public:
    UI() = default;

    // Pass the state by reference so the UI can change it (e.g., Start Button -> RUNNING)
    void DrawMainMenu(GameState& currentState);
    
    // Pass data needed for the game HUD (e.g., score, plane count)
    void DrawGameHUD(int aircraftCount);
    void DrawPauseMenu(GameState& currentState);

    void DrawBackground();
};