#pragma once

#include "core/Types.h"
#include "raylib.h"

class Aircraft;
class Simulation;

// UI only renders and reports lightweight user intent back to Engine.
enum class MainMenuAction {
    NONE,
    START,
    OPEN_SETTINGS,
    EXIT
};

struct SettingsMenuResult {
    // Apply and close stay separate. Engine can tell save from dismiss.
    bool applyRequested = false;
    bool closeRequested = false;
};

enum class PauseMenuAction {
    NONE,
    RESUME,
    RETURN_TO_MENU
};

struct SimulationViewResult {
    // UI reports intent only. Engine decides if requests can run.
    bool spawnRequested = false;
    bool toggleDebugRequested = false;
};

class UI {
public:
    UI() = default;

    // Each draw call renders one screen, returns a compact action result.
    MainMenuAction DrawMainMenu();
    SettingsMenuResult DrawSettingsMenu(AppSettings& settings);
    PauseMenuAction DrawPauseMenu();

    SimulationViewResult DrawSimulation(const Simulation& sim,
                                        const AppSettings& settings,
                                        bool showDebug,
                                        const Aircraft* selected);

    static Vector2 NMToPixels(Vec2 nmPos, const SimSettings& sim);
    static double NMToPixels(double nmDistance, const SimSettings& sim);
    // Radar clicks are converted back into sim-space before Engine asks the
    // Simulation to hit-test or issue commands.
    static Vec2 PixelsToNM(Vector2 pixelPos, const SimSettings& sim);
};
