#pragma once
#include "constants/constants.h"
#include "../render/ui.h"
#include "../aircraft/Aircraft.h"
#include <vector>
#include <memory>
#include "../world/Airport.h"


class Engine {
    private:
        static Engine instance;
        GameState currentState;
        UI ui;

        //aircraft
        std::vector<std::unique_ptr<Aircraft>> aircraft{}; //dynamic array; 1 unqptr owns 1 aircraft obj;
        Aircraft* selectedAircraft{}; //raw ptr for selected aircraft

        //airports
        std::vector<Airport> airports;


        bool debugEnabled = true;

        Engine();
        static void constructWindow();

        //sim logic
        void updateSimulation(double deltaTime); //sim separate since menu doesnt need aircraft updates
        void renderSimulation();
        void handleInput();
        void spawnAircraft();
        void detectConflicts() const;


    public:
        static Engine& getInstance();

        Engine(const Engine&) = delete;
        void operator=(const Engine&) = delete;

        void init();
        void update(double deltaTime); //main update
        void render();
        void run();

        bool shouldClose();
};