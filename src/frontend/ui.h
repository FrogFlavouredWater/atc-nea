#pragma once
#include "raylib.h"
#include "common/constants.h"
#include "backend/Simulation.h"

class UI {
public:
    UI() = default;

    void DrawMainMenu(GameState& currentState);
    void DrawSimulationHUD(int aircraftCount, bool& debugEnabled);
    void DrawPauseMenu(GameState& currentState);
    void DrawBackground();
    
    void DrawSimulation(const Simulation& sim, bool debugEnabled, Aircraft* selectedAircraft);
    
    // Units conversion
    static Vector2 NMToPixels(Vec2 nmPos);
    static double NMToPixels(double nmDistance);
    static Vec2 PixelsToNM(Vector2 pixelPos);
private:
    void DrawAircraft(const Aircraft& aircraft, bool debugEnabled, bool selected);
    void DrawAirport(const Airport& airport);
};