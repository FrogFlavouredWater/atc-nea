#include "app/engine.h"
#include "common/constants.h"
#include "common/logger.h"
#include "frontend/ui.h"
#include <raylib.h>
#include <raymath.h>
#include <algorithm>
#include <sstream>
#include <stdexcept>

Engine Engine::instance;

namespace {
constexpr double kSimulationSpeedStep = 1.0;
constexpr double kSimulationSpeedMin = 1.0;
constexpr double kSimulationSpeedMax = 120.0;

const char* screenModeName(ScreenMode mode) {
    switch (mode) {
    case ScreenMode::WINDOWED:
        return "WINDOWED";
    case ScreenMode::BORDERLESS_WINDOWED:
        return "BORDERLESS_WINDOWED";
    case ScreenMode::FULLSCREEN:
        return "FULLSCREEN";
    default:
        return "UNKNOWN";
    }
}

const char* gameStateName(GameState state) {
    switch (state) {
    case GameState::MENU:
        return "MENU";
    case GameState::RUNNING:
        return "RUNNING";
    case GameState::PAUSED:
        return "PAUSED";
    case GameState::GAME_OVER:
        return "GAME_OVER";
    case GameState::EXIT:
        return "EXIT";
    default:
        return "UNKNOWN";
    }
}

std::string describeDisplaySettings(const DisplaySettings& displaySettings) {
    std::ostringstream stream;
    stream << displaySettings.screenWidth << "x" << displaySettings.screenHeight
           << ", mode=" << screenModeName(displaySettings.screenMode)
           << ", target_fps=" << displaySettings.targetFps;
    return stream.str();
}

std::string describeSimSettings(const SimSettings& simSettings) {
    std::ostringstream stream;
    stream << "speed=" << simSettings.simulationSpeed
           << "x, max_aircraft=" << simSettings.maxAircraft
           << ", aircraft_size=" << simSettings.aircraftSize
           << ", pixels_per_nm=" << simSettings.pixelsPerNm;
    return stream.str();
}

std::string formatSimulationSpeed(double simulationSpeed) {
    std::ostringstream stream;
    stream.setf(std::ios::fixed);
    stream.precision(1);
    stream << simulationSpeed << "x";
    return stream.str();
}

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
    Logger::info("Creating application window");
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(settings.display.screenWidth,
               settings.display.screenHeight,
               "ATC Simulator");
    applySettings();
    Logger::success("Window initialized");
}

void Engine::applySettings() {
    Logger::info("Applying display settings: " + describeDisplaySettings(settings.display));
    Logger::info("Applying simulation settings: " + describeSimSettings(settings.sim));
    applyDisplaySettings(settings.display);
    sim.applySettings(settings.sim);
}

void Engine::handleInput() {
    if (IsKeyPressed(KEY_COMMA)) {
        settings.sim.simulationSpeed = std::clamp(settings.sim.simulationSpeed - kSimulationSpeedStep,
                                                  kSimulationSpeedMin,
                                                  kSimulationSpeedMax);
        Logger::info("Simulation speed set to " + formatSimulationSpeed(settings.sim.simulationSpeed));
    }
    if (IsKeyPressed(KEY_PERIOD)) {
        settings.sim.simulationSpeed = std::clamp(settings.sim.simulationSpeed + kSimulationSpeedStep,
                                                  kSimulationSpeedMin,
                                                  kSimulationSpeedMax);
        Logger::info("Simulation speed set to " + formatSimulationSpeed(settings.sim.simulationSpeed));
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Aircraft* previousSelection = selectedAircraft;
        Vector2 mousePos = GetMousePosition();
        Vec2 nmMousePos = UI::PixelsToNM(mousePos, settings.sim);
        double selectionRadiusNm = 30.0 / settings.sim.pixelsPerNm;

        selectedAircraft = sim.getAircraftAt(nmMousePos, selectionRadiusNm);

        if (selectedAircraft != previousSelection) {
            if (selectedAircraft) {
                Logger::info("Selected aircraft " + selectedAircraft->getCallsign());
            } else if (previousSelection) {
                Logger::info("Cleared aircraft selection");
            }
        }
    }

    if (selectedAircraft && IsKeyPressed(KEY_I)) {
        sim.toggleApproachClearance(selectedAircraft);
    }

    if (selectedAircraft
        && selectedAircraft->getControlMode() != AircraftControlMode::ILS
        && IsKeyPressed(KEY_H)) {
        if (selectedAircraft->getInstructionType() == AircraftInstructionType::HOLD) {
            sim.releaseHold(selectedAircraft);
        } else {
            sim.issueHoldAtCurrentPosition(selectedAircraft);
        }
    }

    if (selectedAircraft && selectedAircraft->getControlMode() != AircraftControlMode::ILS) {
        const AircraftCommand currentCommand = selectedAircraft->getCommand();
        AircraftInstruction instruction{
            AircraftInstructionType::VECTOR,
            currentCommand.targetHeading,
            currentCommand.targetSpeed,
            currentCommand.targetAltitude,
            AircraftControlMode::MANUAL
        };
        bool instructionChanged = false;

        if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_A)) {
            instruction.targetHeading -= 10.0;
            instructionChanged = true;
        }
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) {
            instruction.targetHeading += 10.0;
            instructionChanged = true;
        }
        if (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_W)) {
            instruction.targetSpeed += 10.0;
            instructionChanged = true;
        }
        if (IsKeyPressed(KEY_DOWN) || IsKeyPressed(KEY_S)) {
            instruction.targetSpeed -= 10.0;
            instructionChanged = true;
        }
        if (IsKeyPressed(KEY_Q) || IsKeyPressed(KEY_PAGE_UP)) {
            instruction.targetAltitude += 1000;
            instructionChanged = true;
        }
        if (IsKeyPressed(KEY_E) || IsKeyPressed(KEY_PAGE_DOWN)) {
            instruction.targetAltitude -= 1000;
            instructionChanged = true;
        }

        if (instructionChanged) {
            sim.issueInstruction(selectedAircraft, instruction);
        }
    }
}

void Engine::spawnInitialTraffic() {
    for (int i = 0; i < 3; ++i) {
        sim.requestRandomSpawn();
    }
    Logger::info("Initial traffic seeded with " + std::to_string(sim.getAircraftCount()) + " aircraft");
    sim.clearLastSpawnResult();
}

void Engine::init() {
    Logger::info("Initializing engine");
    constructWindow();
    sim.addAirport({"LHR", {0.0, 0.0}, 90.0, 2.0, {}});
    spawnInitialTraffic();
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
            Logger::info("Selected aircraft left the simulation");
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
                Logger::info("Closing settings menu");
                showingSettings = false;
                pendingSettings = settings;
            }
        } else {
            switch (ui.DrawMainMenu()) {
            case MainMenuAction::START:
                currentState = GameState::RUNNING;
                break;

            case MainMenuAction::OPEN_SETTINGS:
                Logger::info("Opening settings menu");
                pendingSettings = settings;
                showingSettings = true;
                break;

            case MainMenuAction::EXIT:
                Logger::info("Exit requested from main menu");
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
        {
        const bool debugWasEnabled = debugEnabled;
        if (const SimulationViewResult result = ui.DrawSimulation(sim, settings, debugEnabled, selectedAircraft);
            result.spawnRequested) {
            const SpawnRequestResult spawnResult = sim.requestRandomSpawn();
            if (spawnResult.success) {
                selectedAircraft = spawnResult.aircraft;
            }
        }
        if (debugEnabled != debugWasEnabled) {
            Logger::info(std::string("Debug overlay ") + (debugEnabled ? "enabled" : "disabled"));
        }
        }
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
    Logger::info("Entering main loop");
    GameState previousState = currentState;
    while (!shouldClose()) {
        double deltaTime = static_cast<double>(GetFrameTime());
        update(deltaTime);
        render();

        if (currentState != previousState) {
            Logger::info(std::string("Game state changed from ")
                         + gameStateName(previousState)
                         + " to "
                         + gameStateName(currentState));
            previousState = currentState;
        }
    }
    Logger::info("Closing application window");
    CloseWindow();
}

bool Engine::shouldClose() {
    return WindowShouldClose() || currentState == GameState::EXIT;
}
