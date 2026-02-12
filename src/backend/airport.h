#pragma once

#include "common/utils.h"
#include <string>
#include <vector>

struct Airport
{
    std::string name;
    Vec2 position;
    double runwayHeading;
    double runwayLength = 6.0;    // 6.0 NM
    double localiserLength = 20.0; // 20.0 NM
};