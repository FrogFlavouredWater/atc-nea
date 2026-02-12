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

Aircraft::Aircraft(Vec2 startPos,
                    double initialHeading,
                    double initialSpeed,
                    int initialAltitude,
                    const std::string &id):
                        position(startPos),
                        heading(initialHeading),
                        targetHeading(initialHeading),
                        speed(initialSpeed),
                        targetSpeed(initialSpeed),
                        altitude(initialAltitude),
                        targetAltitude(initialAltitude),
                        callsign(id),
                        state(AircraftState::APPROACH),
                        selected(false) {

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

    double headingDiff = getShortestAngleDiff(targetHeading, heading);

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
    if (fabs(speed - targetSpeed) > 0.0) {
        if (speed < targetSpeed) {
            speed += acceleration * deltaTime;
            if (speed > targetSpeed) speed = targetSpeed; //pos.overshoot prot (i bet harry will complain)
        } else {
            speed -= acceleration * deltaTime;
            if (speed < targetSpeed) speed = targetSpeed; //neg.overshoot prot
        }
    }

    // Velocity in NM per hour, deltaTime is in seconds, so divide by 3600
    double speedInNmPerSec = speed / 3600.0;
    velocity.x = cos(DEG2RAD * (heading - 90.0)) * speedInNmPerSec;
    velocity.y = sin(DEG2RAD * (heading - 90.0)) * speedInNmPerSec;

    position.x += velocity.x * deltaTime;
    position.y += velocity.y * deltaTime;

}

void Aircraft::setHeading(double newHeading) {
    targetHeading = normalizeAngle(newHeading);
}

std::string Aircraft::stateToString(AircraftState state) {
    switch (state) {
        case AircraftState::APPROACH: return "APPROACH";
        case AircraftState::VECTORING: return "VECTORING";
        case AircraftState::ON_FINAL: return "ON_FINAL";
        case AircraftState::LANDING: return "LANDING";
        case AircraftState::CONFLICT: return "CONFLICT";
        default: return "UNKNOWN";
    }
}