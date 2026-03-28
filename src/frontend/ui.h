#pragma once
#include "raylib.h"
#include "backend/navigation/GuidancePreview.h"
#include "common/constants.h"
#include "backend/simulation/Simulation.h"

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

struct SimulationViewResult {
    bool spawnRequested = false;
};

class UI {
public:
    UI() = default;

    MainMenuAction DrawMainMenu();
    SettingsMenuResult DrawSettingsMenu(AppSettings& settings);
    SimulationViewResult DrawSimulationHUD(int aircraftCount,
                                           int outOfBoundsCount,
                                           int landedCount,
                                           int hullLossCount,
                                           int predictedConflictCount,
                                           double simulationSpeed,
                                           bool& debugEnabled,
                                           const SpawnRequestResult& spawnResult);
    void DrawPauseMenu(GameState& currentState);
    void DrawBackground();
    void DrawRangeRings(Vec2 airportPos, const SimSettings& simSettings);

    SimulationViewResult DrawSimulation(const Simulation& sim,
                                        const AppSettings& settings,
                                        bool& debugEnabled,
                                        Aircraft* selectedAircraft);
    
    // Units conversion
    static Vector2 NMToPixels(Vec2 nmPos, const SimSettings& simSettings);
    static double NMToPixels(double nmDistance, const SimSettings& simSettings);
    static Vec2 PixelsToNM(Vector2 pixelPos, const SimSettings& simSettings);
private:
    void DrawAircraftTrail(const Aircraft& aircraft, const SimSettings& simSettings, bool selected);
    void DrawAircraft(const Aircraft& aircraft, const SimSettings& simSettings, bool selected);
    void DrawAirport(const Airport& airport, const SimSettings& simSettings);
    void DrawGuidancePreview(const GuidancePreview& preview, const SimSettings& simSettings, bool debugEnabled);
};
