#include "Aircraft.h"
#include "raymath.h"
#include "../../constants/constants.h"
#include "../../util/utils.h"
#include <cmath> //this one is a bastard to link
#include <algorithm>

#ifndef DEG2RAD
#define DEG2RAD (PI / 180.0)
#endif

Aircraft::Aircraft(Vector2 startPos,
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

    velocity.x = cos(DEG2RAD * (heading - 90.0)) * speed;
    velocity.y = sin(DEG2RAD * (heading - 90.0)) * speed;

}

double Aircraft::distanceTo(const Aircraft& other) const {
    return Vector2Distance(position, other.position);
}

bool Aircraft::collidesWith(const Aircraft& other) const {
    return distanceTo(other) < 40.0;  //40px collision radius (random magic number) TODO: Add to config
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

    //poolar cooordinates
    velocity.x = cos(DEG2RAD * (heading - 90.0)) * speed;
    velocity.y = sin(DEG2RAD * (heading - 90.0)) * speed;

    position.x += velocity.x * deltaTime;
    position.y += velocity.y * deltaTime;

}

void Aircraft::setHeading(double newHeading) {
    targetHeading = normalizeAngle(newHeading);
}

void Aircraft::render() {
    auto aircraftColor = WHITE;
    if (selected) aircraftColor = YELLOW;
    if (state == AircraftState::CONFLICT) aircraftColor = RED;

    const int size = CONSTANTS.game.AIRCRAFT_SIZE;
    const int halfSize = size / 2;


    DrawRectangleLinesEx(
        Rectangle{
            position.x - static_cast<float>(halfSize),
            position.y - static_cast<float>(halfSize),
            static_cast<float>(size),
            static_cast<float>(size)
        },
        1.5f,  // Line thickness
        aircraftColor
    );

    const float vectorLength = 40.0f;
    Vector2 vectorEnd = {
        position.x + static_cast<float>(std::cos(DEG2RAD * (heading - 90.0)) * vectorLength),
        position.y + static_cast<float>(std::sin(DEG2RAD * (heading - 90.0)) * vectorLength)
    };

    DrawLineEx(position, vectorEnd, 2.0f, aircraftColor);

    DrawText(callsign.c_str(), (int)(position.x + 20), (int)(position.y - 12), 14, WHITE); //callsign

    if (selected) {
        DrawCircleLinesV(position, 25.0f, YELLOW);
    }
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