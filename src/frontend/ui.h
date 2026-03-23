#pragma once
#include "raylib.h"
#include "common/constants.h"
#include "backend/Simulation.h"

enum class MainMenuAction {
    NONE,
    START,
    OPEN_SETTINGS,
    EXIT
};

struct SettingsMenuResult {
    bool applyRequested = false;
    bool closeRequested = false;
};

class UI {
public:
    UI() = default;

    MainMenuAction DrawMainMenu();
    SettingsMenuResult DrawSettingsMenu(AppSettings& settings);
    void DrawSimulationHUD(int aircraftCount, int outOfBoundsCount, double simulationSpeed, bool& debugEnabled);
    void DrawPauseMenu(GameState& currentState);
    void DrawBackground();
    void DrawRangeRings(Vec2 airportPos, const SimSettings& simSettings);

    void DrawSimulation(const Simulation& sim, const AppSettings& settings, bool& debugEnabled, Aircraft* selectedAircraft);
    
    // Units conversion
    static Vector2 NMToPixels(Vec2 nmPos, const SimSettings& simSettings);
    static double NMToPixels(double nmDistance, const SimSettings& simSettings);
    static Vec2 PixelsToNM(Vector2 pixelPos, const SimSettings& simSettings);
private:
    void DrawAircraft(const Aircraft& aircraft, const SimSettings& simSettings, bool selected);
    void DrawAirport(const Airport& airport, const SimSettings& simSettings);
};
