#include "backend/Aircraft.h"
#include "common/constants.h"
#include "backend/AircraftMotion.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kCommandEpsilon = 0.01;
constexpr double kTrailSampleDistanceNm = 0.45;
constexpr size_t kTrailMaxPoints = 2400;

double distanceNm(Vec2 from, Vec2 to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    return std::sqrt(dx * dx + dy * dy);
}

AircraftInstruction instructionFromCommand(const AircraftCommand& command) {
    return AircraftInstruction{
        command.source == AircraftControlMode::ILS
            ? AircraftInstructionType::ILS_INTERCEPT
            : AircraftInstructionType::VECTOR,
        command.targetHeading,
        command.targetSpeed,
        command.targetAltitude,
        command.source
    };
}
}

Aircraft::Aircraft(Vec2 startPos,
                    double initialHeading,
                    double initialSpeed,
                    int initialAltitude,
                    const std::string &id):
                        performance{},
                        motion{},
                        activeInstruction{},
                        command{initialHeading, initialSpeed, initialAltitude, AircraftControlMode::AUTONOMOUS},
                        callsign(id),
                        phase(FlightPhase::ARRIVAL),
                        conflictAlert(false) {
    motion.position = startPos;
    motion.heading = initialHeading;
    motion.speed = initialSpeed;
    motion.altitude = initialAltitude;

    initializeAircraftMotion(motion, performance);

    activeInstruction = AircraftInstruction{
        AircraftInstructionType::MAINTAIN,
        motion.heading,
        motion.speed,
        getAltitude(),
        AircraftControlMode::AUTONOMOUS
    };
    syncCommandToInstruction();

    recordTrailPoint();
}

double Aircraft::distanceTo(const Aircraft& other) const {
    double dx = motion.position.x - other.motion.position.x;
    double dy = motion.position.y - other.motion.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

double Aircraft::altitudeDifferenceTo(const Aircraft& other) const {
    return std::abs(motion.altitude - other.motion.altitude);
}

bool Aircraft::breachesSeparationWith(const Aircraft& other) const {
    return distanceTo(other) < SeparationRules::HORIZONTAL_NM
        && altitudeDifferenceTo(other) < SeparationRules::VERTICAL_FT;
}

bool Aircraft::collidesWith(const Aircraft& other) const {
    return breachesSeparationWith(other);
}

void Aircraft::update(double deltaTime) {
    trailElapsedSeconds += deltaTime;
    stepAircraftMotion(motion, command, performance, deltaTime);

    if (trailPoints.empty() || distanceNm(trailPoints.back().position, motion.position) >= kTrailSampleDistanceNm) {
        recordTrailPoint();
    }
}

void Aircraft::recordTrailPoint() {
    trailPoints.push_back(AircraftTrailPoint{motion.position, trailElapsedSeconds});
    trimTrailPoints();
}

void Aircraft::trimTrailPoints() {
    while (trailPoints.size() > kTrailMaxPoints) {
        trailPoints.pop_front();
    }
}

void Aircraft::syncCommandToInstruction() {
    command.source = activeInstruction.controlMode;

    switch (activeInstruction.type) {
        case AircraftInstructionType::MAINTAIN:
            command.targetHeading = motion.heading;
            command.targetSpeed = motion.speed;
            command.targetAltitude = getAltitude();
            return;
        case AircraftInstructionType::VECTOR:
        case AircraftInstructionType::HOLD:
        case AircraftInstructionType::ILS_INTERCEPT:
        case AircraftInstructionType::CONFLICT_RESOLUTION:
            command.targetHeading = normalizeAngle(activeInstruction.targetHeading);
            command.targetSpeed = std::clamp(activeInstruction.targetSpeed,
                                             performance.minSpeedKts,
                                             performance.maxSpeedKts);
            command.targetAltitude = std::max(0, activeInstruction.targetAltitude);
            return;
    }
}

void Aircraft::updatePhaseFromInstruction() {
    if (phase == FlightPhase::LANDING || phase == FlightPhase::EXITED) {
        return;
    }

    if (activeInstruction.type == AircraftInstructionType::ILS_INTERCEPT) {
        phase = FlightPhase::ON_FINAL;
        return;
    }

    const bool hasVectoringCommand =
        std::abs(getShortestAngleDiff(command.targetHeading, motion.heading)) > kCommandEpsilon
        || std::abs(command.targetSpeed - motion.speed) > kCommandEpsilon
        || std::abs(static_cast<double>(command.targetAltitude) - motion.altitude) > performance.altitudeCaptureToleranceFt;

    if (activeInstruction.type == AircraftInstructionType::HOLD
        || activeInstruction.type == AircraftInstructionType::CONFLICT_RESOLUTION
        || (activeInstruction.type == AircraftInstructionType::VECTOR && hasVectoringCommand)) {
        phase = FlightPhase::VECTORING;
    } else if (phase == FlightPhase::VECTORING) {
        phase = FlightPhase::ARRIVAL;
    }
}

void Aircraft::applyInstruction(const AircraftInstruction& newInstruction) {
    activeInstruction.type = newInstruction.type;
    activeInstruction.controlMode = newInstruction.controlMode;
    activeInstruction.targetHeading = normalizeAngle(newInstruction.targetHeading);
    activeInstruction.targetSpeed = std::clamp(newInstruction.targetSpeed,
                                               performance.minSpeedKts,
                                               performance.maxSpeedKts);
    activeInstruction.targetAltitude = std::max(0, newInstruction.targetAltitude);

    syncCommandToInstruction();
    updatePhaseFromInstruction();
}

void Aircraft::applyCommand(const AircraftCommand& newCommand) {
    applyInstruction(instructionFromCommand(newCommand));
}
