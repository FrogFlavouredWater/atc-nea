#include "engine.h"
#include "constants/constants.h"
#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <stdexcept>
#include "../render/ui.h"

using std::cout, std::endl;

Engine Engine::instance;

Engine::Engine() : currentState(GameState::MENU), ui{}, aircraft{}, selectedAircraft(nullptr) {}

Engine& Engine::getInstance() {
    return instance;
}

void Engine::constructWindow() {
    // Enable antialiasing BEFORE initializing window
    SetConfigFlags(FLAG_MSAA_4X_HINT);  // Enable 4x MSAA antialiasing
    
    InitWindow(CONSTANTS.display.SCREEN_WIDTH, 
               CONSTANTS.display.SCREEN_HEIGHT, 
               "ATC Simulator");
    SetTargetFPS(CONSTANTS.display.TARGET_FPS);
}

void Engine::updateSimulation(float deltaTime) {
    handleInput();

    for (auto& plane : aircraft) {
        plane->update(deltaTime);
    }

    detectConflicts();
}

void Engine::renderSimulation() {
    ui.DrawBackground();

    for (auto& plane : aircraft) {
        plane->render();
    }

    ui.DrawSimulationHUD(aircraft.size());

    if (selectedAircraft) {
        DrawText("Selected: ", 10, 40, 20, WHITE);  // Increased font size from 16 to 20
        DrawText(selectedAircraft->getCallsign().c_str(), 110, 40, 20, YELLOW);
        DrawText("Controls: WASD/Arrows = Vector/Speed", 10, 65, 16, GRAY);
    } else {
        DrawText("Click aircraft to vector", 10, 40, 20, WHITE);  // Increased font size
    }
}

//TODO: TEMPORARY <REMOVE THIS>
void Engine::handleInput() {
    // Aircraft selection with mouse
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 mousePos = GetMousePosition();
        selectedAircraft = nullptr;

        for (auto& plane : aircraft) {
            Vector2 planePos = plane->getPosition();
            if (Vector2Distance(mousePos, planePos) < 30.0f) {
                selectedAircraft = plane.get();
                plane->setSelected(true);
            } else {
                plane->setSelected(false);
            }
        }
    }

    // Control selected aircraft with keyboard
    if (selectedAircraft) {
        float currentHeading = selectedAircraft->getHeading();
        float currentSpeed = selectedAircraft->getSpeed();

        // Heading controls
        if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_A)) {
            selectedAircraft->setHeading(currentHeading - 30);
        }
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) {
            selectedAircraft->setHeading(currentHeading + 30);
        }

        // Speed controls
        if (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_W)) {
            selectedAircraft->setSpeed(currentSpeed + 25);
        }
        if (IsKeyPressed(KEY_DOWN) || IsKeyPressed(KEY_S)) {
            selectedAircraft->setSpeed(currentSpeed - 25);
        }
    }
}

void Engine::spawnAircraft() {
    //TODO: TEMPORARY <REMOVE THIS>
    aircraft.push_back(std::make_unique<Aircraft>(
        Vector2{100, 200}, 90.0f, 5.0f, "AA123"
    ));

    aircraft.push_back(std::make_unique<Aircraft>(
        Vector2{300, 400}, 270.0f, 6.0f, "BA456"
    ));

    aircraft.push_back(std::make_unique<Aircraft>(
        Vector2{500, 100}, 180.0f, 5.5f, "UA789"
    ));
}

void Engine::detectConflicts() {
    //reset all aircraft to non conflict state
    for (auto& plane : aircraft) {
        if (plane->getState() == AircraftState::CONFLICT) {
            plane->setState(AircraftState::APPROACH); //reset to default state
        }
    }

    // Check for conflicts & set state
    for (size_t i = 0; i < aircraft.size(); i++) {
        for (size_t j = i + 1; j < aircraft.size(); j++) {
            if (aircraft[i]->collidesWith(*aircraft[j])) {
                // Set both aircraft to conflict state
                aircraft[i]->setState(AircraftState::CONFLICT);
                aircraft[j]->setState(AircraftState::CONFLICT);
                
                //handle separation conflict
                cout << "SEPARATION CONFLICT: " << aircraft[i]->getCallsign()
                     << " and " << aircraft[j]->getCallsign() << endl;
            }
        }
    }
}

void Engine::init() {
    constructWindow();
    spawnAircraft();
}

void Engine::update(float deltaTime) {
    //toggle state for testing
    if (IsKeyPressed(KEY_P)) {
        if (currentState == GameState::RUNNING) currentState = GameState::PAUSED;
        else if (currentState == GameState::PAUSED) currentState = GameState::RUNNING;
    }

    //update sim when running
    if (currentState == GameState::RUNNING) {
        updateSimulation(deltaTime);
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
        renderSimulation();
        break;

    case GameState::PAUSED:
        renderSimulation();
        ui.DrawPauseMenu(currentState);
        break;
    
    case GameState::GAME_OVER:
        // ui.DrawGameOver(currentState);
        break;
            
    default:
        throw std::runtime_error("Unhandled Mode enum"); //literally should be impossible to trigger
    }

    EndDrawing();
}

void Engine::run() {
    while (!shouldClose()) {
        float deltaTime = GetFrameTime();
        update(deltaTime);
        render();
    }
    CloseWindow();
}

bool Engine::shouldClose() {
    //handle raylib exit or user exit
    return WindowShouldClose() || currentState == GameState::EXIT;
}