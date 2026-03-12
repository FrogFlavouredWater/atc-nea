#pragma once

#include "common/utils.h"
#include <string>
#include <vector>
#include <cmath>

struct LocalizerSector {
    double range;      // NM
    double width;      // Degrees (total arc width)
};

struct Localizer {
    double length = 18.0;
    std::vector<LocalizerSector> sectors = {
        {10.0, 35.0}, // 35-degree arc at 10nm
        {18.0, 10.0}  // 10-degree arc at 18nm
    };

    bool isWithinSignal(Vec2 airportPos, double runwayHeading, Vec2 targetPos) const;
};

struct Airport
{
    std::string name;
    Vec2 position;
    double runwayHeading;
    double runwayLength = 2.0;
    Localizer localizer;

    bool inLocalizerSignal(Vec2 pos) const {
        return localizer.isWithinSignal(position, runwayHeading, pos);
    }
};