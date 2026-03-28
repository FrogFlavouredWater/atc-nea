#include "backend/navigation/airport.h"
#include <cmath>

#ifndef PI
#define PI 3.14159265358979323846
#endif

bool Localizer::isWithinSignal(Vec2 airportPos, double runwayHeading, Vec2 targetPos) const {
    double dx = targetPos.x - airportPos.x;
    double dy = targetPos.y - airportPos.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // The localizer signal extends from the airport in the direction of the approach.
    // If runwayHeading is the landing direction, the signal is at runwayHeading + 180.
    double signalCenterHeading = normalizeAngle(runwayHeading + 180.0);
    
    // Calculate heading from airport to target position
    // heading 0 is North (-Y), 90 is East (+X)
    double angleToTarget = normalizeAngle(std::atan2(dx, -dy) * (180.0 / PI));
    
    double angleDiff = std::abs(getShortestAngleDiff(signalCenterHeading, angleToTarget));

    for (const auto& sector : sectors) {
        if (distance <= sector.range && angleDiff <= (sector.width / 2.0)) {
            return true;
        }
    }
    return false;
}
