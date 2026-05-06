#pragma once

#include "core/Types.h"
#include <utility>
#include <string>
#include <vector>
#include <cmath>

struct LocalizerSector {
    // Each sector widens or narrows the usable signal cone as range increases.
    double range;      // NM
    double width;      // Degrees (total arc width)
};

struct Localizer {
    double length = 18.0;
    // Evaluate sectors from nearest matching range outward.
    std::vector<LocalizerSector> sectors = {
        {10.0, 35.0}, // 35-degree arc at 10nm
        {18.0, 10.0}  // 10-degree arc at 18nm
    };

    bool isWithinSignal(Vec2 airportPos, double runwayHeading, Vec2 targetPos) const;
};

// Airport bundles the runway/localizer geometry the backend needs for capture
// checks and the UI needs for rendering.
struct Airport
{
    std::string name;
    Vec2 position;
    double runwayHeading;
    double runwayLength = 2.0;
    Localizer localizer;

    [[nodiscard]] bool inLocalizerSignal(Vec2 pos) const {
        return localizer.isWithinSignal(position, runwayHeading, pos);
    }
    // Positive in front of the runway on final.
    [[nodiscard]] double alongTrackToRunway(Vec2 pos) const;
    // Signed cross-track. callers can tell left/right of centreline.
    [[nodiscard]] double crossTrackError(Vec2 pos) const;
    [[nodiscard]] double minLocalizerRange() const;
    [[nodiscard]] std::pair<double, double> ilsCaptureAltitudeBandFt(double alongTrackNm) const;
    [[nodiscard]] double ilsProfileAltitudeFt(Vec2 aircraftPosition, double currentAltitudeFt) const;
};
