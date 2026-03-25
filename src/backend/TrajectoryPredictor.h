#pragma once

#include "backend/Aircraft.h"
#include "backend/AircraftMotion.h"
#include <string>
#include <vector>

struct PredictedAircraftState {
    double timeSeconds = 0.0;
    AircraftMotionState motion{};
};

struct PredictedConflictAssessment {
    bool valid = false;
    bool breachesSeparation = false;
    std::string firstCallsign{};
    std::string secondCallsign{};
    double timeToClosestApproachSeconds = 0.0;
    double horizontalDistanceNm = 0.0;
    double verticalDistanceFt = 0.0;
    double severityScore = 0.0;
    AircraftMotionState firstMotion{};
    AircraftMotionState secondMotion{};
};

class TrajectoryPredictor {
public:
    static std::vector<PredictedAircraftState> predict(const Aircraft& aircraft,
                                                       double horizonSeconds,
                                                       double stepSeconds);
    static PredictedConflictAssessment assessConflict(const Aircraft& first,
                                                      const Aircraft& second,
                                                      double horizonSeconds,
                                                      double stepSeconds);
};
