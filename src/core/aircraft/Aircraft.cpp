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
                    float initialHeading,
                    float initialSpeed,
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

    velocity.x = cos(DEG2RAD * heading) * speed;
    velocity.y = sin(DEG2RAD * heading) * speed;

}

float Aircraft::distanceTo(const Aircraft& other) const {
    return Vector2Distance(position, other.position);
}

bool Aircraft::collidesWith(const Aircraft& other) const {
    return distanceTo(other) < 40.0f;  //40px collision radius (random magic number) TODO: Add to config
}


void Aircraft::update(float deltaTime) {

    float headingDiff = getShortestAngleDiff(targetHeading, heading);

    //smooth heading change
    if (fabs(headingDiff) > 0.0f) {
        float turnAmount = turnRate * deltaTime;
        float actualTurn = std::min(turnAmount, fabsf(headingDiff));
        
        if (headingDiff > 0) {
            heading += actualTurn;
        } else {
            heading -= actualTurn;
        }

        heading = normalizeAngle(heading);
    }

    //smooth speed change
    if (fabs(speed - targetSpeed) > 0.0f) {
        if (speed < targetSpeed) {
            speed += acceleration * deltaTime;
            if (speed > targetSpeed) speed = targetSpeed; //pos.overshoot prot (i bet harry will complain)
        } else {
            speed -= acceleration * deltaTime;
            if (speed < targetSpeed) speed = targetSpeed; //neg.overshoot prot
        }
    }

    //poolar cooordinates
    velocity.x = cos(DEG2RAD * heading) * speed;
    velocity.y = sin(DEG2RAD * heading) * speed;

    position.x += velocity.x * deltaTime;
    position.y += velocity.y * deltaTime;

}

void Aircraft::setHeading(float newHeading) {
    targetHeading = normalizeAngle(newHeading);
}

void Aircraft::render() {
    auto aircraftColor = WHITE;
    if (selected) aircraftColor = YELLOW;
    if (state == AircraftState::CONFLICT) aircraftColor = RED;

    const int size = CONSTANTS.game.AIRCRAFT_SIZE;
    const int halfSize = size / 2;
    
    // Center the square on the aircraft position
    DrawRectangleLinesEx(
        Rectangle{position.x - halfSize, position.y - halfSize, (float)size, (float)size},
        1.5f,  // Line thickness for better visibility
        aircraftColor
    );


    const float vectorLength = 40.0f;
    Vector2 vectorEnd = {
        position.x + cos(DEG2RAD * heading) * vectorLength,
        position.y + sin(DEG2RAD * heading) * vectorLength
    };

    DrawLineEx(position, vectorEnd, 2.0f, aircraftColor);

    DrawText(callsign.c_str(), (int)(position.x + 20), (int)(position.y - 12), 14, WHITE); //callsign
    
    if (selected) {
        DrawCircleLinesV(position, 25.0f, YELLOW);
    }

}