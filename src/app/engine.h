#pragma once
#include "common/constants.h"
#include "frontend/ui.h"
#include "backend/Simulation.h"
#include <memory>

class Engine {
    private:
        static Engine instance;
        GameState currentState;
        UI ui;
        Simulation sim;
        Aircraft* selectedAircraft{};

        bool debugEnabled = true;

        Engine();
        static void constructWindow();

        void handleInput();
        void spawnAircraft();

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