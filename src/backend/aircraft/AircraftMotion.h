#pragma once

#include "backend/aircraft/AircraftControl.h"
#include "backend/aircraft/AircraftPerformance.h"
#include "common/utils.h"

struct AircraftMotionState {
    Vec2 position{};
    Vec2 velocity{};
    double heading = 0.0;
    double speed = 0.0;
    double altitude = 0.0;
    double verticalSpeedFpm = 0.0;
    double turnRateDegPerSec = 0.0;
    double turnRadiusNm = 0.0;
};

void initializeAircraftMotion(AircraftMotionState& motion, const AircraftPerformance& performance);
void stepAircraftMotion(AircraftMotionState& motion,
                        const AircraftCommand& command,
                        const AircraftPerformance& performance,
                        double deltaTime);
