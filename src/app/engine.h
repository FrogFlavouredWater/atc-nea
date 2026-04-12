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
        Aircraft* selected{};
        AppSettings settings = AppConfig::DEFAULTS;
        // Settings edits stay here until the user presses Apply.
        AppSettings pending = AppConfig::DEFAULTS;
        bool showSettings = false;

        bool showDebug = true;

        Engine();
        void createWindow();
        void applySettings();

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
