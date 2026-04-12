#include "backend/aircraft/AircraftMotion.h"

#include "core/MathUtils.h"
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
    motion.velocity.x = std::cos(kDegToRad * (motion.heading - 90.0)) * speedInNmPerSec; // -90 since 0 north in sim, but 0 east in trig
    motion.velocity.y = std::sin(kDegToRad * (motion.heading - 90.0)) * speedInNmPerSec;
}

void updateSpeed(AircraftMotionState& motion,
                 const AircraftCommand& command,
                 const AircraftPerformance& performance,
                 double dt) {
    const double speedDiff = command.targetSpeed - motion.speed;
    if (std::abs(speedDiff) <= kCommandEpsilon) {
        motion.speed = command.targetSpeed;
        return;
    }

    const double speedStep = performance.accelerationKtsPerSec * dt;
    const double actualStep = std::min(speedStep, std::abs(speedDiff));
    motion.speed += speedDiff > 0.0 ? actualStep : -actualStep;
    motion.speed = std::clamp(motion.speed, performance.minSpeedKts, performance.maxSpeedKts);
}

void updateHeading(AircraftMotionState& motion, const AircraftCommand& command, double dt) {
    const double headingDiff = getShortestAngleDiff(command.targetHeading, motion.heading);
    if (std::abs(headingDiff) <= kCommandEpsilon) {
        motion.heading = normalizeAngle(command.targetHeading);
        return;
    }

    const double turnAmount = motion.turnRateDegPerSec * dt;
    const double actualTurn = std::min(turnAmount, std::abs(headingDiff));
    motion.heading += headingDiff > 0.0 ? actualTurn : -actualTurn;
    motion.heading = normalizeAngle(motion.heading);
}

void updateAltitude(AircraftMotionState& motion,
                    const AircraftCommand& command,
                    const AircraftPerformance& performance,
                    double dt) {
    const double altitudeDiff = static_cast<double>(command.targetAltitude) - motion.altitude;
    if (std::abs(altitudeDiff) <= performance.altitudeCaptureToleranceFt) {
        motion.altitude = static_cast<double>(command.targetAltitude);
        motion.verticalSpeedFpm = 0.0;
        return;
    }

    const double maxVerticalRateFpm = altitudeDiff > 0.0
        ? performance.climbRateFpm
        : performance.descentRateFpm;
    const double altitudeStepFt = maxVerticalRateFpm * kMinutesPerSecond * dt;
    const double actualStepFt = std::min(altitudeStepFt, std::abs(altitudeDiff));

    motion.altitude += altitudeDiff > 0.0 ? actualStepFt : -actualStepFt;

    if (dt > 0.0) {
        const double actualVerticalRateFpm = actualStepFt / dt / kMinutesPerSecond;
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
    // Normalize the initial state once so later stepping can assume sane values.
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
                        double dt) {
    // Update order matters here: speed affects turn dynamics, then heading and
    // altitude respond to the command, then velocity/position are integrated.
    updateSpeed(motion, command, performance, dt);
    updateTurnDynamics(motion, performance);
    updateHeading(motion, command, dt);
    updateAltitude(motion, command, performance, dt);
    updateVelocity(motion);

    motion.position.x += motion.velocity.x * dt;
    motion.position.y += motion.velocity.y * dt;
}
