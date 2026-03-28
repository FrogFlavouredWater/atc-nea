#include "backend/aircraft/Aircraft.h"
#include "common/constants.h"
#include "backend/aircraft/AircraftMotion.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kCommandEpsilon = 0.01;
constexpr double kTrailSampleDistanceNm = 0.45;
constexpr size_t kTrailMaxPoints = 2400;
constexpr double kHoldTurnCompletionHeadingToleranceDeg = 8.0;
constexpr double kHoldTurnCompletionLateralToleranceNm = 0.2;
constexpr double kHoldTurnMinimumRadiusNm = 0.25;

double distanceNm(Vec2 from, Vec2 to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    return std::sqrt(dx * dx + dy * dy);
}

Vec2 directionVectorForHeading(double headingDeg) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;
    return Vec2{
        std::cos(kDegToRad * (headingDeg - 90.0)),
        std::sin(kDegToRad * (headingDeg - 90.0))
    };
}

Vec2 rightNormalForHeading(double headingDeg) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;
    return Vec2{
        std::cos(kDegToRad * headingDeg),
        std::sin(kDegToRad * headingDeg)
    };
}

double dot(Vec2 first, Vec2 second) {
    return first.x * second.x + first.y * second.y;
}

double bearingDeg(Vec2 from, Vec2 to) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kRadToDeg = 180.0 / kPi;
    return normalizeAngle(std::atan2(to.y - from.y, to.x - from.x) * kRadToDeg + 90.0);
}

AircraftInstruction instructionFromCommand(const AircraftCommand& command) {
    return AircraftInstruction{
        command.source == AircraftControlMode::ILS
            ? AircraftInstructionType::ILS_INTERCEPT
            : AircraftInstructionType::VECTOR,
        command.targetHeading,
        command.targetSpeed,
        command.targetAltitude,
        command.source,
        {},
        0.0,
        0.0,
        1
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
    if (activeInstruction.type == AircraftInstructionType::HOLD) {
        updateHoldCommand();
    }
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

void Aircraft::updateHoldCommand() {
    const double legLengthNm = std::max(activeInstruction.holdLegLengthNm, 0.5);
    const double turnRadiusNm = std::max(activeInstruction.holdTurnRadiusNm, kHoldTurnMinimumRadiusNm);
    const double outboundHeading = normalizeAngle(activeInstruction.targetHeading);
    const double inboundHeading = normalizeAngle(outboundHeading + 180.0);
    const double turnDirection = activeInstruction.holdTurnDirection >= 0 ? 1.0 : -1.0;
    const Vec2 axis = directionVectorForHeading(outboundHeading);
    const Vec2 lateralAxis = Vec2{
        rightNormalForHeading(outboundHeading).x * turnDirection,
        rightNormalForHeading(outboundHeading).y * turnDirection
    };
    const Vec2 outboundEnd{
        activeInstruction.holdEntryPosition.x + axis.x * legLengthNm,
        activeInstruction.holdEntryPosition.y + axis.y * legLengthNm
    };
    const Vec2 inboundOffset{
        lateralAxis.x * turnRadiusNm * 2.0,
        lateralAxis.y * turnRadiusNm * 2.0
    };
    const Vec2 firstTurnCenter{
        outboundEnd.x + lateralAxis.x * turnRadiusNm,
        outboundEnd.y + lateralAxis.y * turnRadiusNm
    };
    const Vec2 secondTurnCenter{
        activeInstruction.holdEntryPosition.x + lateralAxis.x * turnRadiusNm,
        activeInstruction.holdEntryPosition.y + lateralAxis.y * turnRadiusNm
    };
    const Vec2 relativePosition{
        motion.position.x - activeInstruction.holdEntryPosition.x,
        motion.position.y - activeInstruction.holdEntryPosition.y
    };
    const double longitudinalNm = dot(relativePosition, axis);
    const double lateralNm = dot(relativePosition, lateralAxis);
    const double lateralTargetNm = 2.0 * turnRadiusNm;
    const Vec2 firstTurnRelative{
        motion.position.x - firstTurnCenter.x,
        motion.position.y - firstTurnCenter.y
    };
    const Vec2 secondTurnRelative{
        motion.position.x - secondTurnCenter.x,
        motion.position.y - secondTurnCenter.y
    };
    const double firstTurnLateralNm = dot(firstTurnRelative, lateralAxis);
    const double secondTurnLateralNm = dot(secondTurnRelative, lateralAxis);

    switch (holdPhase) {
        case HoldPhase::OUTBOUND:
            if (longitudinalNm >= legLengthNm) {
                holdPhase = HoldPhase::TURN_INBOUND;
            }
            break;
        case HoldPhase::TURN_INBOUND:
            if (std::abs(getShortestAngleDiff(inboundHeading, motion.heading)) <= kHoldTurnCompletionHeadingToleranceDeg
                && firstTurnLateralNm >= turnRadiusNm - std::max(kHoldTurnCompletionLateralToleranceNm, turnRadiusNm * 0.3)) {
                holdPhase = HoldPhase::INBOUND;
            }
            break;
        case HoldPhase::INBOUND:
            if (longitudinalNm <= 0.0) {
                holdPhase = HoldPhase::TURN_OUTBOUND;
            }
            break;
        case HoldPhase::TURN_OUTBOUND:
            if (std::abs(getShortestAngleDiff(outboundHeading, motion.heading)) <= kHoldTurnCompletionHeadingToleranceDeg
                && secondTurnLateralNm <= -turnRadiusNm + std::max(kHoldTurnCompletionLateralToleranceNm, turnRadiusNm * 0.3)) {
                holdPhase = HoldPhase::OUTBOUND;
            }
            break;
    }

    if (holdPhase == HoldPhase::OUTBOUND) {
        command.targetHeading = outboundHeading;
    } else if (holdPhase == HoldPhase::INBOUND) {
        command.targetHeading = inboundHeading;
    } else if (holdPhase == HoldPhase::TURN_INBOUND) {
        const double radialBearingDeg = bearingDeg(firstTurnCenter, motion.position);
        command.targetHeading = normalizeAngle(radialBearingDeg + turnDirection * 90.0);
    } else {
        const double radialBearingDeg = bearingDeg(secondTurnCenter, motion.position);
        command.targetHeading = normalizeAngle(radialBearingDeg + turnDirection * 90.0);
    }

    if (holdPhase == HoldPhase::INBOUND && lateralNm < lateralTargetNm - turnRadiusNm) {
        command.targetHeading = bearingDeg(motion.position, Vec2{
            activeInstruction.holdEntryPosition.x + inboundOffset.x,
            activeInstruction.holdEntryPosition.y + inboundOffset.y
        });
    }
    command.targetSpeed = std::clamp(activeInstruction.targetSpeed,
                                     performance.minSpeedKts,
                                     performance.maxSpeedKts);
    command.targetAltitude = std::max(0, activeInstruction.targetAltitude);
    command.source = activeInstruction.controlMode;
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
    const bool enteringHold = newInstruction.type == AircraftInstructionType::HOLD;

    activeInstruction.type = newInstruction.type;
    activeInstruction.controlMode = newInstruction.controlMode;
    activeInstruction.targetHeading = normalizeAngle(newInstruction.targetHeading);
    activeInstruction.targetSpeed = std::clamp(newInstruction.targetSpeed,
                                               performance.minSpeedKts,
                                               performance.maxSpeedKts);
    activeInstruction.targetAltitude = std::max(0, newInstruction.targetAltitude);
    activeInstruction.holdEntryPosition = newInstruction.holdEntryPosition;
    activeInstruction.holdLegLengthNm = std::max(0.0, newInstruction.holdLegLengthNm);
    activeInstruction.holdTurnRadiusNm = std::max(0.0, newInstruction.holdTurnRadiusNm);
    activeInstruction.holdTurnDirection = newInstruction.holdTurnDirection >= 0 ? 1 : -1;

    if (enteringHold) {
        holdPhase = HoldPhase::OUTBOUND;
    }

    syncCommandToInstruction();
    updatePhaseFromInstruction();
}

void Aircraft::applyCommand(const AircraftCommand& newCommand) {
    applyInstruction(instructionFromCommand(newCommand));
}
