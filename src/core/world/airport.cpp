#include "airport.h"
#include <cmath>

#ifndef DEG2RAD
#define DEG2RAD (PI / 180.0)
#endif

void Airport::render() const {
    //draw rwy
    Rectangle rec = { position.x, position.y, (float)runwayLength, 10.0f };
    Vector2 origin = { (float)runwayLength / 2.0f, 5.0f };
    DrawRectanglePro(rec, origin, (float)(runwayHeading - 90.0), GRAY);

    //draw loc
    //heading to rad & flip 180
    float approachAngle = (float)(runwayHeading + 90.0) * (float)DEG2RAD;
    Vector2 localizerEnd = {
        position.x + cosf(approachAngle) * (float)localiserLength,
        position.y + sinf(approachAngle) * (float)localiserLength
    };
    DrawLineEx(position, localizerEnd, 1.0f, GREEN);
}