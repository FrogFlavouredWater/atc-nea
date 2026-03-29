#pragma once

#include "sim/Simulation.h"
#include "core/Config.h"
#include "frontend/ui.h"

// Engine owns the top-level app flow: window lifecycle, state transitions,
// input dispatch, and coordination between UI and Simulation.
class Engine {
    private:
        GameState currentState;
        UI ui;
        Simulation sim;
        Aircraft* selectedAircraft{};
        AppSettings settings = AppConfig::DEFAULTS;
        // Settings edits stay here until the user presses Apply.
        AppSettings pendingSettings = AppConfig::DEFAULTS;
        bool showingSettings = false;

        bool debugEnabled = true;

        Engine();
        void constructWindow();
        void applySettings();

        void handleGlobalInput();
        void handleSimulationInput();
        void handleMainMenuAction(MainMenuAction action);
        void handleSettingsMenuResult(const SettingsMenuResult& result);
        void handleSimulationViewResult(const SimulationViewResult& result);
        void handlePauseMenuAction(PauseMenuAction action);
        void updateRunning(double deltaTime);
        void validateSelection();
        void renderMenu();
        void renderRunning();
        void renderPaused();
        void spawnInitialTraffic();

    public:
        static Engine& getInstance();

        Engine(const Engine&) = delete;
        void operator=(const Engine&) = delete;

        void init();
        void update(double deltaTime);
        void render();
        void run();

        bool shouldClose();
};
