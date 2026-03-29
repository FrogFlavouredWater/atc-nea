#pragma once

#include <cstdint>

// The simulator uses nautical miles for lateral distances in world space.
struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

// Plain data carriers only. Startup defaults and allowed limits live in core/Config.h.

enum class ScreenMode {
    WINDOWED,
    BORDERLESS_WINDOWED
};

struct DisplaySettings {
    int32_t screenWidth = 0;
    int32_t screenHeight = 0;
    ScreenMode screenMode = ScreenMode::WINDOWED;
    int32_t targetFps = 0;
};

struct SimSettings {
    int32_t maxAircraft = 0;
    int32_t aircraftSize = 0;
    double pixelsPerNm = 0.0;
    double simulationSpeed = 0.0;
    double minXNm = 0.0;
    double maxXNm = 0.0;
    double minYNm = 0.0;
    double maxYNm = 0.0;
};

struct AppSettings {
    DisplaySettings display{};
    SimSettings sim{};
};

enum class GameState {
    MENU,
    RUNNING,
    PAUSED,
    GAME_OVER,
    EXIT
};

enum class AircraftControlMode {
    AUTONOMOUS,
    MANUAL,
    ILS
};

enum class AircraftInstructionType {
    MAINTAIN,
    VECTOR,
    HOLD,
    ILS_INTERCEPT,
    CONFLICT_RESOLUTION
};

enum class FlightPhase {
    ARRIVAL,
    VECTORING,
    ON_FINAL,
    LANDING,
    EXITED
};

struct AircraftCommand {
    double targetHeading = 0.0;
    double targetSpeed = 0.0;
    int targetAltitude = 0;
    AircraftControlMode source = AircraftControlMode::AUTONOMOUS;
};

struct AircraftInstruction {
    AircraftInstructionType type = AircraftInstructionType::MAINTAIN;
    double targetHeading = 0.0;
    double targetSpeed = 0.0;
    int targetAltitude = 0;
    AircraftControlMode controlMode = AircraftControlMode::AUTONOMOUS;
    // Hold metadata is only meaningful when type == HOLD.
    Vec2 holdEntryPosition{};
    double holdLegLengthNm = 0.0;
    double holdTurnRadiusNm = 0.0;
    int holdTurnDirection = 1;
};

inline constexpr const char* toString(ScreenMode mode) {
    switch (mode) {
        case ScreenMode::WINDOWED: return "WINDOWED";
        case ScreenMode::BORDERLESS_WINDOWED: return "BORDERLESS_WINDOWED";
        default: return "UNKNOWN";
    }
}

inline constexpr const char* toString(GameState state) {
    switch (state) {
        case GameState::MENU: return "MENU";
        case GameState::RUNNING: return "RUNNING";
        case GameState::PAUSED: return "PAUSED";
        case GameState::GAME_OVER: return "GAME_OVER";
        case GameState::EXIT: return "EXIT";
        default: return "UNKNOWN";
    }
}

inline constexpr const char* toString(AircraftControlMode mode) {
    switch (mode) {
        case AircraftControlMode::AUTONOMOUS: return "AUTONOMOUS";
        case AircraftControlMode::MANUAL: return "MANUAL";
        case AircraftControlMode::ILS: return "ILS";
        default: return "UNKNOWN";
    }
}

inline constexpr const char* toString(AircraftInstructionType type) {
    switch (type) {
        case AircraftInstructionType::MAINTAIN: return "MAINTAIN";
        case AircraftInstructionType::VECTOR: return "VECTOR";
        case AircraftInstructionType::HOLD: return "HOLD";
        case AircraftInstructionType::ILS_INTERCEPT: return "ILS_INTERCEPT";
        case AircraftInstructionType::CONFLICT_RESOLUTION: return "CONFLICT_RESOLUTION";
        default: return "UNKNOWN";
    }
}

inline constexpr const char* toString(FlightPhase phase) {
    switch (phase) {
        case FlightPhase::ARRIVAL: return "ARRIVAL";
        case FlightPhase::VECTORING: return "VECTORING";
        case FlightPhase::ON_FINAL: return "ON_FINAL";
        case FlightPhase::LANDING: return "LANDING";
        case FlightPhase::EXITED: return "EXITED";
        default: return "UNKNOWN";
    }
}
