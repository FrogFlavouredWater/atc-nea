#include "app/engine.h"
#include "common/constants.h"
#include "frontend/ui.h"
#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <stdexcept>

Engine Engine::instance;

Engine::Engine() : currentState(GameState::MENU), ui{}, sim{}, selectedAircraft(nullptr) {}

Engine& Engine::getInstance() {
    return instance;
}

void Engine::constructWindow() {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(CONSTANTS.display.SCREEN_WIDTH, 
               CONSTANTS.display.SCREEN_HEIGHT, 
               "ATC Simulator");
    SetTargetFPS(CONSTANTS.display.TARGET_FPS);
}

void Engine::handleInput() {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 mousePos = GetMousePosition();
        Vec2 nmMousePos = UI::PixelsToNM(mousePos);
        // We need 30px in NM. PixelsToNM(Vector2) exists.
        Vec2 radiusVector = UI::PixelsToNM(Vector2{30.0f, 0.0f});
        double selectionRadiusNm = radiusVector.x; 

        selectedAircraft = sim.getAircraftAt(nmMousePos, selectionRadiusNm);
        
        for (auto& plane : sim.getAircraft()) {
            plane->setSelected(plane.get() == selectedAircraft);
        }
    }

    if (selectedAircraft) {
        double targetHeading = selectedAircraft->getTargetHeading();
        double targetSpeed = selectedAircraft->getTargetSpeed();

        if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_A)) {
            selectedAircraft->setHeading(targetHeading - 30);
        }
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) {
            selectedAircraft->setHeading(targetHeading + 30);
        }
        if (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_W)) {
            selectedAircraft->setSpeed(targetSpeed + 10);
        }
        if (IsKeyPressed(KEY_DOWN) || IsKeyPressed(KEY_S)) {
            selectedAircraft->setSpeed(targetSpeed - 10);
        }
    }
}

void Engine::spawnAircraft() {
    sim.spawnAircraft({10.0, 20.0}, 180.0, 210.0, 3000, "AA123");
    sim.spawnAircraft({30.0, 40.0}, 0.0, 250.0, 3500, "BA456");
    sim.spawnAircraft({50.0, 10.0}, 270.0, 180.0, 3200, "UA789");
}

void Engine::init() {
    constructWindow();
    sim.addAirport({"LHR",{40.0, 30.0}, 90.0});
    spawnAircraft();
}

void Engine::update(double deltaTime) {
    if (IsKeyPressed(KEY_P)) {
        if (currentState == GameState::RUNNING) currentState = GameState::PAUSED;
        else if (currentState == GameState::PAUSED) currentState = GameState::RUNNING;
    }

    if (currentState == GameState::RUNNING) {
        handleInput();
        sim.update(deltaTime);
    }
}

void Engine::render() {
    BeginDrawing();
    ClearBackground(GRAY);

    switch (currentState) {
    case GameState::MENU:
        ui.DrawMainMenu(currentState);
        break;

    case GameState::RUNNING:
        ui.DrawSimulation(sim, debugEnabled, selectedAircraft);
        break;

    case GameState::PAUSED:
        ui.DrawSimulation(sim, debugEnabled, selectedAircraft);
        ui.DrawPauseMenu(currentState);
        break;
    
    case GameState::GAME_OVER:
        break;
            
    default:
        throw std::runtime_error("Unhandled Mode enum");
    }

    EndDrawing();
}

void Engine::run() {
    while (!shouldClose()) {
        double deltaTime = static_cast<double>(GetFrameTime());
        update(deltaTime);
        render();
    }
    CloseWindow();
}

bool Engine::shouldClose() {
    return WindowShouldClose() || currentState == GameState::EXIT;
}