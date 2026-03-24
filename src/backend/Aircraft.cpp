#include "backend/Aircraft.h"
#include "common/constants.h"
#include "backend/AircraftMotion.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kHorizontalSeparationNm = 3.0;
constexpr double kVerticalSeparationFt = 1000.0;
constexpr double kCommandEpsilon = 0.01;
constexpr double kTrailSampleDistanceNm = 0.45;
constexpr size_t kTrailMaxPoints = 2400;

double distanceNm(Vec2 from, Vec2 to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    return std::sqrt(dx * dx + dy * dy);
}
}

Aircraft::Aircraft(Vec2 startPos,
                    double initialHeading,
                    double initialSpeed,
                    int initialAltitude,
                    const std::string &id):
                        performance{},
                        motion{},
                        command{initialHeading, initialSpeed, initialAltitude, AircraftControlMode::AUTONOMOUS},
                        callsign(id),
                        phase(FlightPhase::ARRIVAL),
                        controlMode(AircraftControlMode::AUTONOMOUS),
                        conflictAlert(false) {
    motion.position = startPos;
    motion.heading = initialHeading;
    motion.speed = initialSpeed;
    motion.altitude = initialAltitude;

    initializeAircraftMotion(motion, performance);

    command.targetHeading = motion.heading;
    command.targetSpeed = motion.speed;
    command.targetAltitude = getAltitude();

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
    return distanceTo(other) < kHorizontalSeparationNm
        && altitudeDifferenceTo(other) < kVerticalSeparationFt;
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

void Aircraft::applyCommand(const AircraftCommand& newCommand) {
    command.targetHeading = normalizeAngle(newCommand.targetHeading);
    command.targetSpeed = std::clamp(newCommand.targetSpeed, performance.minSpeedKts, performance.maxSpeedKts);
    command.targetAltitude = std::max(0, newCommand.targetAltitude);
    command.source = newCommand.source;
    controlMode = newCommand.source;

    if (controlMode == AircraftControlMode::ILS && phase != FlightPhase::LANDING && phase != FlightPhase::EXITED) {
        phase = FlightPhase::ON_FINAL;
        return;
    }

    const bool hasVectoringCommand =
        std::abs(getShortestAngleDiff(command.targetHeading, motion.heading)) > kCommandEpsilon
        || std::abs(command.targetSpeed - motion.speed) > kCommandEpsilon
        || std::abs(static_cast<double>(command.targetAltitude) - motion.altitude) > performance.altitudeCaptureToleranceFt;

    if (hasVectoringCommand && phase != FlightPhase::LANDING && phase != FlightPhase::EXITED) {
        phase = FlightPhase::VECTORING;
    } else if (phase == FlightPhase::VECTORING && phase != FlightPhase::LANDING && phase != FlightPhase::EXITED) {
        phase = FlightPhase::ARRIVAL;
    }
}
