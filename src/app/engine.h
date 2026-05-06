#pragma once

#include "sim/Simulation.h"
#include "core/Config.h"
#include "frontend/ui.h"

// Engine owns the top-level app flow: window lifecycle, state transitions,
// input dispatch, and coordination between UI and Simulation.
class Engine {
    private:
        GameState state = GameState::MENU;
        UI ui;
        Simulation sim;
        // Borrowed pointer into sim-owned aircraft. clearInvalidSelection() drops stale targets.
        Aircraft* selected{};
        AppSettings settings = AppConfig::DEFAULTS;
        // Settings edits stay here until the user presses Apply.
        AppSettings pending = AppConfig::DEFAULTS;
        bool showSettings = false;

        // Debug rendering flag only. UI concern, Engine just carries it.
        bool showDebug = true;

        Engine();
        void createWindow();
        void applySettings();

        // Small handlers around the loop. keeps run() on lifecycle work.
        void handleGlobalInput();
        void handleSimulationInput();
        void handleMainMenuAction(MainMenuAction action);
        void handleSettingsMenu(const SettingsMenuResult& result);
        void handleSimView(const SimulationViewResult& result);
        void handlePauseMenuAction(PauseMenuAction action);
        void updateRunning(double dt);
        void clearInvalidSelection();
        void renderMenu();
        void renderRunning();
        void renderPaused();
        void spawnInitialTraffic();

    public:
        static Engine& getInstance();

        Engine(const Engine&) = delete;
        void operator=(const Engine&) = delete;

        void init();
        void update(double dt);
        void render();
        void run();

        bool shouldClose();
};
