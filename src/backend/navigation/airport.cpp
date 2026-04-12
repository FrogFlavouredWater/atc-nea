#include "backend/navigation/airport.h"

#include "core/Config.h"
#include "core/MathUtils.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
double lerp(double x, double x0, double x1, double y0, double y1) {
    if (std::abs(x1 - x0) < 1e-6) {
        return y1;
    }

    const double t = std::clamp((x - x0) / (x1 - x0), 0.0, 1.0);
    return y0 + (y1 - y0) * t;
}
}

bool Localizer::isWithinSignal(Vec2 airportPos, double runwayHeading, Vec2 targetPos) const {
    const double dx = targetPos.x - airportPos.x;
    const double dy = targetPos.y - airportPos.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    const double signalCenterHeading = normalizeAngle(runwayHeading + 180.0);
    const double angleToTarget = normalizeAngle(std::atan2(dx, -dy) * (180.0 / std::numbers::pi_v<double>));
    const double angleDiff = std::abs(getShortestAngleDiff(signalCenterHeading, angleToTarget));

    for (const auto& sector : sectors) {
        if (distance <= sector.range && angleDiff <= (sector.width / 2.0)) {
            return true;
        }
    }
    return false;
}

double Airport::alongTrackToRunway(Vec2 pos) const {
    const Vec2 runwayDirection = directionVectorForHeading(runwayHeading);
    const Vec2 toRunway{
        position.x - pos.x,
        position.y - pos.y
    };
    return dot(toRunway, runwayDirection);
}

double Airport::crossTrackError(Vec2 pos) const {
    const Vec2 relativeToRunway{
        pos.x - position.x,
        pos.y - position.y
    };
    return dot(relativeToRunway, rightNormalForHeading(runwayHeading));
}

double Airport::minLocalizerRange() const {
    if (localizer.sectors.empty()) {
        return 0.0;
    }

    double minRange = localizer.sectors.front().range;
    for (const auto& sector : localizer.sectors) {
        minRange = std::min(minRange, sector.range);
    }
    return minRange;
}

std::pair<double, double> Airport::ilsCaptureAltitudeBandFt(double alongTrackNm) const {
    if (alongTrackNm > minLocalizerRange()) {
        return {SimTuning::ILS_OUTER_CAPTURE_MIN_ALTITUDE_FT, SimTuning::ILS_OUTER_CAPTURE_MAX_ALTITUDE_FT};
    }

    return {SimTuning::ILS_INNER_CAPTURE_MIN_ALTITUDE_FT, SimTuning::ILS_INNER_CAPTURE_MAX_ALTITUDE_FT};
}

double Airport::ilsProfileAltitudeFt(Vec2 aircraftPosition, double currentAltitudeFt) const {
    // Use a piecewise linear profile so aircraft descend smoothly from outer
    // capture through final, while never being told to climb on the glidepath.
    const double alongTrackNm = std::clamp(alongTrackToRunway(aircraftPosition), 0.0, localizer.length);
    const double innerRegionBoundaryNm = minLocalizerRange();
    double desiredAltitudeFt = 0.0;

    if (alongTrackNm > innerRegionBoundaryNm) {
        desiredAltitudeFt = lerp(alongTrackNm,
                                 innerRegionBoundaryNm,
                                 localizer.length,
                                 SimTuning::ILS_OUTER_CAPTURE_MIN_ALTITUDE_FT,
                                 SimTuning::ILS_OUTER_CAPTURE_MAX_ALTITUDE_FT);
    } else if (alongTrackNm > SimTuning::ILS_FINAL_DESCENT_START_NM) {
        desiredAltitudeFt = lerp(alongTrackNm,
                                 SimTuning::ILS_FINAL_DESCENT_START_NM,
                                 innerRegionBoundaryNm,
                                 SimTuning::ILS_INNER_CAPTURE_MIN_ALTITUDE_FT,
                                 SimTuning::ILS_INNER_CAPTURE_MAX_ALTITUDE_FT);
    } else {
        desiredAltitudeFt = lerp(alongTrackNm,
                                 0.0,
                                 SimTuning::ILS_FINAL_DESCENT_START_NM,
                                 0.0,
                                 SimTuning::ILS_INNER_CAPTURE_MIN_ALTITUDE_FT);
    }

    return std::min(desiredAltitudeFt, currentAltitudeFt);
}
