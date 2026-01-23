#include "Aircraft.h"
#include "raymath.h"
#include "../../constants/constants.h"
#include <cmath> //this one is a bastard to link

#ifndef DEG2RAD
#define DEG2RAD (PI / 180.0f)
#endif

Aircraft::Aircraft(Vector2 startPos,
                    float initialHeading,
                    float initialSpeed,
                    const std::string &id):
                        position(startPos),
                        velocity{0, 0},
                        heading(initialHeading),
                        targetHeading(initialHeading),
                        speed(initialSpeed),
                        targetSpeed(initialSpeed),
                        callsign(id),
                        state(AircraftState::APPROACH),
                        selected(false) {
                            // keep initialising target values or they recieve bs values at first update()
                            //TODO: im stupid just add def val to header

    //i love polar coordinates
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

    float headingDiff = targetHeading - heading; // 5 - 20 = -15

    //wraparound
    if (headingDiff > 180) headingDiff -= 360;
    if (headingDiff < -180) headingDiff += 360;

    //smooth heading change
    if (fabs(headingDiff) > 0.5f) { // only turns if diff >0.5 DEG (stops jittering)
        float turnAmount = turnRate * deltaTime;
        if (headingDiff > 0) {
            heading += turnAmount;
        } else {
            heading -= turnAmount;
        }

        while (heading < 0) heading += 360;
        while (heading >=360) heading -= 360;
    }

    //smooth speed change
    if (fabs(speed - targetSpeed) > 1.0f) {
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