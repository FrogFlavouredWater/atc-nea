#pragma once

#include "backend/Aircraft.h"
#include "backend/AircraftMotion.h"
#include <vector>

struct PredictedAircraftState {
    double timeSeconds = 0.0;
    AircraftMotionState motion{};
};

class TrajectoryPredictor {
public:
    static std::vector<PredictedAircraftState> predict(const Aircraft& aircraft,
                                                       double horizonSeconds,
                                                       double stepSeconds);
};
