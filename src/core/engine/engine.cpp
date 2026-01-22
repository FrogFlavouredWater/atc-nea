#include "engine.h"
#include "../../constants/constants.h"
#include <raylib.h>
#include <iostream>

using std::cout, std::endl;

Engine Engine::instance;

Engine::Engine() : currentState(GameState::MENU) {}

Engine& Engine::getInstance() {
    return instance;
}

void Engine::constructWindow() {
    InitWindow(CONSTANTS.display.SCREEN_WIDTH, 
               CONSTANTS.display.SCREEN_HEIGHT, 
               "ATC Simulator");
    SetTargetFPS(CONSTANTS.display.TARGET_FPS);
}

void Engine::init() {
    constructWindow();
}


void Engine::render() {
    BeginDrawing();
    ClearBackground(RAYWHITE);

    switch (currentState) {
    case GameState::MENU:
        ui.DrawMainMenu(currentState);
        break;

    case GameState::RUNNING:
        ui.DrawBackground();
        ui.DrawGameHUD(10); 
        break;

    case GameState::PAUSED:
        // renderWorld();
        ui.DrawPauseMenu(currentState);
        break;
    
    case GameState::GAME_OVER:
        // ui.DrawGameOver(currentState);
        break;
    }

    EndDrawing();
}

void Engine::update(float deltaTime) {
    //toggle state for testing
    if (IsKeyPressed(KEY_P)) {
        if (currentState == GameState::RUNNING) currentState = GameState::PAUSED;
        else if (currentState == GameState::PAUSED) currentState = GameState::RUNNING;
    }
}

bool Engine::shouldClose() {
    //handle raylib exit or user exit
    return WindowShouldClose() || currentState == GameState::EXIT;
}

void Engine::run() {
    while (!shouldClose()) {
        float deltaTime = GetFrameTime();
        update(deltaTime);
        render();
    }
    CloseWindow();
}
