#include "backend/AircraftMotion.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kCommandEpsilon = 0.01;
constexpr double kMinutesPerSecond = 1.0 / 60.0;

void updateTurnDynamics(AircraftMotionState& motion, const AircraftPerformance& performance) {
    motion.turnRateDegPerSec = calculateTurnRateDegPerSec(motion.speed, performance);
    motion.turnRadiusNm = calculateTurnRadiusNm(motion.speed, motion.turnRateDegPerSec);
}

void updateVelocity(AircraftMotionState& motion) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kDegToRad = kPi / 180.0;

    const double speedInNmPerSec = motion.speed / 3600.0;
    motion.velocity.x = std::cos(kDegToRad * (motion.heading - 90.0)) * speedInNmPerSec;
    motion.velocity.y = std::sin(kDegToRad * (motion.heading - 90.0)) * speedInNmPerSec;
}

void updateSpeed(AircraftMotionState& motion,
                 const AircraftCommand& command,
                 const AircraftPerformance& performance,
                 double deltaTime) {
    const double speedDiff = command.targetSpeed - motion.speed;
    if (std::abs(speedDiff) <= kCommandEpsilon) {
        motion.speed = command.targetSpeed;
        return;
    }

    const double speedStep = performance.accelerationKtsPerSec * deltaTime;
    const double actualStep = std::min(speedStep, std::abs(speedDiff));
    motion.speed += speedDiff > 0.0 ? actualStep : -actualStep;
    motion.speed = std::clamp(motion.speed, performance.minSpeedKts, performance.maxSpeedKts);
}

void updateHeading(AircraftMotionState& motion, const AircraftCommand& command, double deltaTime) {
    const double headingDiff = getShortestAngleDiff(command.targetHeading, motion.heading);
    if (std::abs(headingDiff) <= kCommandEpsilon) {
        motion.heading = normalizeAngle(command.targetHeading);
        return;
    }

    const double turnAmount = motion.turnRateDegPerSec * deltaTime;
    const double actualTurn = std::min(turnAmount, std::abs(headingDiff));
    motion.heading += headingDiff > 0.0 ? actualTurn : -actualTurn;
    motion.heading = normalizeAngle(motion.heading);
}

void updateAltitude(AircraftMotionState& motion,
                    const AircraftCommand& command,
                    const AircraftPerformance& performance,
                    double deltaTime) {
    const double altitudeDiff = static_cast<double>(command.targetAltitude) - motion.altitude;
    if (std::abs(altitudeDiff) <= performance.altitudeCaptureToleranceFt) {
        motion.altitude = static_cast<double>(command.targetAltitude);
        motion.verticalSpeedFpm = 0.0;
        return;
    }

    const double maxVerticalRateFpm = altitudeDiff > 0.0
        ? performance.climbRateFpm
        : performance.descentRateFpm;
    const double altitudeStepFt = maxVerticalRateFpm * kMinutesPerSecond * deltaTime;
    const double actualStepFt = std::min(altitudeStepFt, std::abs(altitudeDiff));

    motion.altitude += altitudeDiff > 0.0 ? actualStepFt : -actualStepFt;

    if (deltaTime > 0.0) {
        const double actualVerticalRateFpm = actualStepFt / deltaTime / kMinutesPerSecond;
        motion.verticalSpeedFpm = altitudeDiff > 0.0 ? actualVerticalRateFpm : -actualVerticalRateFpm;
    } else {
        motion.verticalSpeedFpm = 0.0;
    }

    if (std::abs(static_cast<double>(command.targetAltitude) - motion.altitude) <= performance.altitudeCaptureToleranceFt) {
        motion.altitude = static_cast<double>(command.targetAltitude);
        motion.verticalSpeedFpm = 0.0;
    }
}
}

void initializeAircraftMotion(AircraftMotionState& motion, const AircraftPerformance& performance) {
    motion.heading = normalizeAngle(motion.heading);
    motion.speed = std::clamp(motion.speed, performance.minSpeedKts, performance.maxSpeedKts);
    motion.altitude = std::max(0.0, motion.altitude);
    motion.verticalSpeedFpm = 0.0;
    updateTurnDynamics(motion, performance);
    updateVelocity(motion);
}

void stepAircraftMotion(AircraftMotionState& motion,
                        const AircraftCommand& command,
                        const AircraftPerformance& performance,
                        double deltaTime) {
    updateSpeed(motion, command, performance, deltaTime);
    updateTurnDynamics(motion, performance);
    updateHeading(motion, command, deltaTime);
    updateAltitude(motion, command, performance, deltaTime);
    updateVelocity(motion);

    motion.position.x += motion.velocity.x * deltaTime;
    motion.position.y += motion.velocity.y * deltaTime;
}
