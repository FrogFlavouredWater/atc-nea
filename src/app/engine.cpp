#include "app/engine.h"

#include "core/Config.h"
#include "core/Logger.h"
#include <raylib.h>
#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace {
constexpr double kSimulationSpeedStep = 1.0;

void applyDisplay(const DisplaySettings& display) {

    switch (display.screenMode) {
    case ScreenMode::WINDOWED:
        ClearWindowState(FLAG_WINDOW_UNDECORATED);
        SetWindowSize(display.screenWidth, display.screenHeight);
        SetWindowPosition(
            (GetMonitorWidth(0) - display.screenWidth) / 2,
            (GetMonitorHeight(0) - display.screenHeight) / 2
        );
        break;

    case ScreenMode::BORDERLESS_WINDOWED:
        SetWindowState(FLAG_WINDOW_UNDECORATED);
        SetWindowPosition(0, 0);
        SetWindowSize(GetMonitorWidth(0), GetMonitorHeight(0));
        break;

    default:
        throw std::runtime_error("Unhandled ScreenMode enum");
    }

    SetTargetFPS(display.targetFps);
}
}

Engine::Engine() = default;

Engine& Engine::getInstance() {
    static Engine instance;
    return instance;
}

void Engine::createWindow() {
    Config::clampAppSettings(settings);
    pending = settings;

    Logger::info("Creating application window");
    SetConfigFlags(FLAG_MSAA_4X_HINT);
    InitWindow(settings.display.screenWidth,
               settings.display.screenHeight,
               "ATC Simulator");
    applySettings();
    Logger::success("Window initialized");
}

void Engine::applySettings() {
    Config::clampAppSettings(settings);
    {
        std::ostringstream stream;
        stream << "Applying display settings: "
               << settings.display.screenWidth << "x" << settings.display.screenHeight
               << ", mode=" << toString(settings.display.screenMode)
               << ", target_fps=" << settings.display.targetFps;
        Logger::info(stream.str());
    }
    {
        std::ostringstream stream;
        stream << "Applying simulation settings: "
               << "speed=" << settings.sim.simulationSpeed
               << "x, max_aircraft=" << settings.sim.maxAircraft
               << ", aircraft_size=" << settings.sim.aircraftSize
               << ", pixels_per_nm=" << settings.sim.pixelsPerNm;
        Logger::info(stream.str());
    }
    applyDisplay(settings.display);
    sim.applySettings(settings.sim);
}

void Engine::handleSimulationInput() {
    if (IsKeyPressed(KEY_COMMA)) {
        settings.sim.simulationSpeed = std::clamp(settings.sim.simulationSpeed - kSimulationSpeedStep,
                                                  SimConfig::MIN_SIMULATION_SPEED,
                                                  SimConfig::MAX_SIMULATION_SPEED);
        {
            std::ostringstream stream;
            stream.setf(std::ios::fixed);
            stream.precision(1);
            stream << "Simulation speed set to " << settings.sim.simulationSpeed << "x";
            Logger::info(stream.str());
        }
    }
    if (IsKeyPressed(KEY_PERIOD)) {
        settings.sim.simulationSpeed = std::clamp(settings.sim.simulationSpeed + kSimulationSpeedStep,
                                                  SimConfig::MIN_SIMULATION_SPEED,
                                                  SimConfig::MAX_SIMULATION_SPEED);
        {
            std::ostringstream stream;
            stream.setf(std::ios::fixed);
            stream.precision(1);
            stream << "Simulation speed set to " << settings.sim.simulationSpeed << "x";
            Logger::info(stream.str());
        }
    }

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        // Selection is done in sim-space, so convert the screen click back into
        // nautical miles before asking the simulation for a hit test.
        Aircraft* prevSelected = selected;
        Vector2 mousePos = GetMousePosition();
        Vec2 mouseNm = UI::PixelsToNM(mousePos, settings.sim);
        double pickRadiusNm = 30.0 / settings.sim.pixelsPerNm;

        selected = sim.getAircraftAt(mouseNm, pickRadiusNm);

        if (selected != prevSelected) {
            if (selected) {
                Logger::info("Selected aircraft " + selected->getCallsign());
            } else if (prevSelected) {
                Logger::info("Cleared aircraft selection");
            }
        }
    }

    if (selected && IsKeyPressed(KEY_I)) {
        sim.toggleApproachClearance(selected);
    }

    if (selected
        && selected->getControlMode() != AircraftControlMode::ILS
        && IsKeyPressed(KEY_H)) {
        if (selected->getInstructionType() == AircraftInstructionType::HOLD) {
            sim.releaseHold(selected);
        } else {
            sim.issueHoldAtCurrentPosition(selected);
        }
    }

    if (selected && selected->getControlMode() != AircraftControlMode::ILS) {
        // Manual vectoring edits the currently commanded targets, then sends the
        // whole instruction back to the simulation in one go.
        const AircraftCommand cmd = selected->getCommand();
        AircraftInstruction instruction{
            AircraftInstructionType::VECTOR,
            cmd.targetHeading,
            cmd.targetSpeed,
            cmd.targetAltitude,
            AircraftControlMode::MANUAL
        };
        bool changed = false;

        if (IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_A)) {
            instruction.targetHeading -= 10.0;
            changed = true;
        }
        if (IsKeyPressed(KEY_RIGHT) || IsKeyPressed(KEY_D)) {
            instruction.targetHeading += 10.0;
            changed = true;
        }
        if (IsKeyPressed(KEY_UP) || IsKeyPressed(KEY_W)) {
            instruction.targetSpeed += 10.0;
            changed = true;
        }
        if (IsKeyPressed(KEY_DOWN) || IsKeyPressed(KEY_S)) {
            instruction.targetSpeed -= 10.0;
            changed = true;
        }
        if (IsKeyPressed(KEY_Q) || IsKeyPressed(KEY_PAGE_UP)) {
            instruction.targetAltitude += 1000;
            changed = true;
        }
        if (IsKeyPressed(KEY_E) || IsKeyPressed(KEY_PAGE_DOWN)) {
            instruction.targetAltitude -= 1000;
            changed = true;
        }

        if (changed) {
            sim.issueInstruction(selected, instruction);
        }
    }
}

void Engine::handleGlobalInput() {
    if (IsKeyPressed(KEY_P)) {
        if (state == GameState::RUNNING) {
            state = GameState::PAUSED;
        } else if (state == GameState::PAUSED) {
            state = GameState::RUNNING;
        }
    }
}

void Engine::handleMainMenuAction(MainMenuAction action) {
    switch (action) {
    case MainMenuAction::START:
        state = GameState::RUNNING;
        break;

    case MainMenuAction::OPEN_SETTINGS:
        Logger::info("Opening settings menu");
        pending = settings;
        showSettings = true;
        break;

    case MainMenuAction::EXIT:
        Logger::info("Exit requested from main menu");
        state = GameState::EXIT;
        break;

    case MainMenuAction::NONE:
        break;

    default:
        throw std::runtime_error("Unhandled MainMenuAction enum");
    }
}

void Engine::handleSettingsMenu(const SettingsMenuResult& result) {
    if (result.applyRequested) {
        settings = pending;
        applySettings();
    }

    if (result.closeRequested) {
        Logger::info("Closing settings menu");
        showSettings = false;
        pending = settings;
    }
}

void Engine::handleSimView(const SimulationViewResult& result) {
    if (result.spawnRequested) {
        const SpawnRequestResult spawn = sim.requestRandomSpawn();
        if (spawn.success) {
            selected = spawn.aircraft;
        }
    }

    if (result.toggleDebugRequested) {
        showDebug = !showDebug;
        Logger::info(std::string("Debug overlay ") + (showDebug ? "enabled" : "disabled"));
    }
}

void Engine::handlePauseMenuAction(PauseMenuAction action) {
    switch (action) {
    case PauseMenuAction::NONE:
        break;

    case PauseMenuAction::RESUME:
        state = GameState::RUNNING;
        break;

    case PauseMenuAction::RETURN_TO_MENU:
        state = GameState::MENU;
        break;

    default:
        throw std::runtime_error("Unhandled PauseMenuAction enum");
    }
}

void Engine::updateRunning(double dt) {
    handleSimulationInput();
    sim.update(dt * settings.sim.simulationSpeed, dt);
    clearInvalidSelection();
}

void Engine::clearInvalidSelection() {
    if (selected && !sim.containsAircraft(selected)) {
        Logger::info("Selected aircraft left the simulation");
        selected = nullptr;
    }
}

void Engine::renderMenu() {
    if (showSettings) {
        handleSettingsMenu(ui.DrawSettingsMenu(pending));
        return;
    }

    handleMainMenuAction(ui.DrawMainMenu());
}

void Engine::renderRunning() {
    handleSimView(ui.DrawSimulation(sim, settings, showDebug, selected));
}

void Engine::renderPaused() {
    ui.DrawSimulation(sim, settings, showDebug, selected);
    handlePauseMenuAction(ui.DrawPauseMenu());
}

void Engine::spawnInitialTraffic() {
    // Seed a little traffic so the simulator starts with something to control.
    for (int i = 0; i < 3; ++i) {
        sim.requestRandomSpawn();
    }
    Logger::info("Initial traffic seeded with " + std::to_string(sim.getAircraftCount()) + " aircraft");
    sim.clearLastSpawnResult();
}

void Engine::init() {
    Logger::info("Initializing engine");
    createWindow();
    sim.addAirport({"LHR", {0.0, 0.0}, 90.0, 2.0, {}});
    spawnInitialTraffic();
}

void Engine::update(double dt) {
    handleGlobalInput();

    switch (state) {
    case GameState::RUNNING:
        updateRunning(dt);
        break;

    case GameState::MENU:
    case GameState::PAUSED:
    case GameState::GAME_OVER:
    case GameState::EXIT:
        break;

    default:
        throw std::runtime_error("Unhandled GameState enum");
    }
}

void Engine::render() {
    BeginDrawing();
    ClearBackground(GRAY);

    switch (state) {
    case GameState::MENU:
        renderMenu();
        break;

    case GameState::RUNNING:
        renderRunning();
        break;

    case GameState::PAUSED:
        renderPaused();
        break;

    case GameState::GAME_OVER:
    case GameState::EXIT:
        break;

    default:
        throw std::runtime_error("Unhandled GameState enum");
    }

    EndDrawing();
}

void Engine::run() {
    Logger::info("Entering main loop");
    GameState prevState = state;
    while (!shouldClose()) {
        const double dt = GetFrameTime();
        update(dt);
        render();

        if (state != prevState) {
            Logger::info(std::string("Game state changed from ")
                         + toString(prevState)
                         + " to "
                         + toString(state));
            prevState = state;
        }
    }
    Logger::info("Closing application window");
    CloseWindow();
}

bool Engine::shouldClose() {
    return WindowShouldClose() || state == GameState::EXIT;
}
