#include "backend/Aircraft.h"
#include "common/constants.h"
#include "common/utils.h"
#include <cmath>
#include <algorithm>

#ifndef PI
#define PI 3.14159265358979323846
#endif

#ifndef DEG2RAD
#define DEG2RAD (PI / 180.0)
#endif

namespace {
constexpr double kCommandEpsilon = 0.01;
}

Aircraft::Aircraft(Vec2 startPos,
                    double initialHeading,
                    double initialSpeed,
                    int initialAltitude,
                    const std::string &id):
                        position(startPos),
                        heading(initialHeading),
                        speed(initialSpeed),
                        altitude(initialAltitude),
                        command{initialHeading, initialSpeed, initialAltitude, AircraftControlMode::AUTONOMOUS},
                        callsign(id),
                        phase(FlightPhase::ARRIVAL),
                        controlMode(AircraftControlMode::AUTONOMOUS),
                        conflictAlert(false) {

    // Velocity in NM per hour
    double speedInNmPerSec = speed / 3600.0;
    velocity.x = cos(DEG2RAD * (heading - 90.0)) * speedInNmPerSec;
    velocity.y = sin(DEG2RAD * (heading - 90.0)) * speedInNmPerSec;

}

double Aircraft::distanceTo(const Aircraft& other) const {
    double dx = position.x - other.position.x;
    double dy = position.y - other.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

bool Aircraft::collidesWith(const Aircraft& other) const {
    return distanceTo(other) < 3.0;  // 3.0 NM collision radius
}


void Aircraft::update(double deltaTime) {

    double headingDiff = getShortestAngleDiff(command.targetHeading, heading);

    //smooth heading change
    if (fabs(headingDiff) > 0.0) {
        double turnAmount = turnRate * deltaTime;
        double actualTurn = std::min(turnAmount, std::abs(headingDiff));

        if (headingDiff > 0) {
            heading += actualTurn;
        } else {
            heading -= actualTurn;
        }

        heading = normalizeAngle(heading);
    }

    //smooth speed change
    if (fabs(speed - command.targetSpeed) > 0.0) {
        if (speed < command.targetSpeed) {
            speed += acceleration * deltaTime;
            if (speed > command.targetSpeed) speed = command.targetSpeed;
        } else {
            speed -= acceleration * deltaTime;
            if (speed < command.targetSpeed) speed = command.targetSpeed;
        }
    }

    // Velocity in NM per hour, deltaTime is in seconds, so divide by 3600
    double speedInNmPerSec = speed / 3600.0;
    velocity.x = cos(DEG2RAD * (heading - 90.0)) * speedInNmPerSec;
    velocity.y = sin(DEG2RAD * (heading - 90.0)) * speedInNmPerSec;

    position.x += velocity.x * deltaTime;
    position.y += velocity.y * deltaTime;

}

void Aircraft::applyCommand(const AircraftCommand& newCommand) {
    command.targetHeading = normalizeAngle(newCommand.targetHeading);
    command.targetSpeed = std::max(0.0, newCommand.targetSpeed);
    command.targetAltitude = std::max(0, newCommand.targetAltitude);
    command.source = newCommand.source;
    controlMode = newCommand.source;

    if (controlMode == AircraftControlMode::ILS && phase != FlightPhase::LANDING && phase != FlightPhase::EXITED) {
        phase = FlightPhase::ON_FINAL;
        return;
    }

    const bool hasVectoringCommand =
        std::abs(getShortestAngleDiff(command.targetHeading, heading)) > kCommandEpsilon
        || std::abs(command.targetSpeed - speed) > kCommandEpsilon
        || command.targetAltitude != altitude;

    if (hasVectoringCommand && phase != FlightPhase::LANDING && phase != FlightPhase::EXITED) {
        phase = FlightPhase::VECTORING;
    }
}
