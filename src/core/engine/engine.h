#pragma once
#include "constants/constants.h"
#include "../render/ui.h"

class Engine {
    private:
        static Engine instance;
        GameState currentState;
        UI ui;

        Engine();
        void constructWindow();

    public:
        static Engine& getInstance();

        Engine(const Engine&) = delete;
        void operator=(const Engine&) = delete;

        void init();
        void update(float deltaTime);
        void render();
        void run();
        bool shouldClose();
};
