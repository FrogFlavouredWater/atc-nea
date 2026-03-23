#include "app/engine.h"
#include "common/constants.h"
#include "frontend/ui.h"
#include <raylib.h>
#include <raymath.h>
#include <iostream>
#include <stdexcept>

Engine Engine::instance;

namespace {
void applyDisplaySettings(const DisplaySettings& displaySettings) {
    if (IsWindowFullscreen() && displaySettings.screenMode != ScreenMode::FULLSCREEN) {
        ToggleFullscreen();
    }

    switch (displaySettings.screenMode) {
    case ScreenMode::WINDOWED:
        ClearWindowState(FLAG_WINDOW_UNDECORATED);
        SetWindowSize(displaySettings.screenWidth, displaySettings.screenHeight);
        SetWindowPosition(
            (GetMonitorWidth(0) - displaySettings.screenWidth) / 2,
            (GetMonitorHeight(0) - displaySettings.screenHeight) / 2
        );
        break;

    case ScreenMode::BORDERLESS_WINDOWED:
        SetWindowState(FLAG_WINDOW_UNDECORATED);
        SetWindowPosition(0, 0);
        SetWindowSize(GetMonitorWidth(0), GetMonitorHeight(0));
        break;

    case ScreenMode::FULLSCREEN:
        ClearWindowState(FLAG_WINDOW_UNDECORATED);
        if (!IsWindowFullscreen()) {
            ToggleFullscreen();
        }
        break;

    default:
        throw std::runtime_error("Unhandled ScreenMode enum");
    }

    SetTargetFPS(displaySettings.targetFps);
}
}

Engine::Engine() : currentState(GameState::MENU), ui{}, sim{}, selectedAircraft(nullptr) {}

Engine& Engine::getInstance() {
    return instance;
}

void Engine::constructWindow() {
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(settings.display.screenWidth,
               settings.display.screenHeight,
               "ATC Simulator");
    applySettings();
}

void Engine::applySettings() {
    applyDisplaySettings(settings.display);
    sim.applySettings(settings.sim);
}

void Engine::handleInput() {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 mousePos = GetMousePosition();
        Vec2 nmMousePos = UI::PixelsToNM(mousePos, settings.sim);
        double selectionRadiusNm = 30.0 / settings.sim.pixelsPerNm;

        selectedAircraft = sim.getAircraftAt(nmMousePos, selectionRadiusNm);
    }

    if (selectedAircraft) {
        AircraftCommand command = selectedAircraft->getCommand();
        bool commandChanged = false;

        if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_A)) {
            command.targetHeading -= 30.0;
            commandChanged = true;
        }
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) {
            command.targetHeading += 30.0;
            commandChanged = true;
        }
        if (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_W)) {
            command.targetSpeed += 10.0;
            commandChanged = true;
        }
        if (IsKeyPressed(KEY_DOWN) || IsKeyPressed(KEY_S)) {
            command.targetSpeed -= 10.0;
            commandChanged = true;
        }

        if (commandChanged) {
            command.source = AircraftControlMode::MANUAL;
            sim.issueCommand(selectedAircraft, command);
        }
    }
}

void Engine::spawnAircraft() {
    sim.spawnAircraft({-30.0, -20.0}, 135.0, 210.0, 3000, "AA123");
    sim.spawnAircraft({40.0, 30.0}, 225.0, 250.0, 3500, "BA456");
    sim.spawnAircraft({-10.0, 40.0}, 45.0, 180.0, 3200, "UA789");
}

void Engine::init() {
    constructWindow();
    sim.addAirport({"LHR",{0.0, 0.0}, 90.0});
    spawnAircraft();
}

void Engine::update(double deltaTime) {
    if (IsKeyPressed(KEY_P)) {
        if (currentState == GameState::RUNNING) currentState = GameState::PAUSED;
        else if (currentState == GameState::PAUSED) currentState = GameState::RUNNING;
    }

    if (currentState == GameState::RUNNING) {
        handleInput();
        double scaledDeltaTime = deltaTime * settings.sim.simulationSpeed;
        sim.update(scaledDeltaTime);

        if (selectedAircraft && !sim.containsAircraft(selectedAircraft)) {
            selectedAircraft = nullptr;
        }
    }
}

void Engine::render() {
    BeginDrawing();
    ClearBackground(GRAY);

    switch (currentState) {
    case GameState::MENU:
        if (showingSettings) {
            SettingsMenuResult result = ui.DrawSettingsMenu(pendingSettings);

            if (result.applyRequested) {
                settings = pendingSettings;
                applySettings();
            }

            if (result.closeRequested) {
                showingSettings = false;
                pendingSettings = settings;
            }
        } else {
            switch (ui.DrawMainMenu()) {
            case MainMenuAction::START:
                currentState = GameState::RUNNING;
                break;

            case MainMenuAction::OPEN_SETTINGS:
                pendingSettings = settings;
                showingSettings = true;
                break;

            case MainMenuAction::EXIT:
                currentState = GameState::EXIT;
                break;

            case MainMenuAction::NONE:
                break;

            default:
                throw std::runtime_error("Unhandled MainMenuAction enum");
            }
        }
        break;

    case GameState::RUNNING:
        ui.DrawSimulation(sim, settings, debugEnabled, selectedAircraft);
        break;

    case GameState::PAUSED:
        ui.DrawSimulation(sim, settings, debugEnabled, selectedAircraft);
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
