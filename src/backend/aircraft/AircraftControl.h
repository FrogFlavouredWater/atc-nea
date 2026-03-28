#pragma once

#include "common/utils.h"

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
    Vec2 holdEntryPosition{};
    double holdLegLengthNm = 0.0;
    double holdTurnRadiusNm = 0.0;
    int holdTurnDirection = 1;
};

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
