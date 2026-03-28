#pragma once

#include "backend/aircraft/Aircraft.h"
#include "backend/aircraft/AircraftMotion.h"
#include <string>
#include <vector>

struct PredictedAircraftState {
    double timeSeconds = 0.0;
    AircraftMotionState motion{};
};

struct PredictedConflictAssessment {
    bool valid = false;
    bool breachesTacticalThreshold = false;
    bool breachesSeparation = false;
    std::string firstCallsign{};
    std::string secondCallsign{};
    double currentHorizontalDistanceNm = 0.0;
    double currentVerticalDistanceFt = 0.0;
    double timeToTacticalThresholdSeconds = -1.0;
    double tacticalHorizontalDistanceNm = 0.0;
    double tacticalVerticalDistanceFt = 0.0;
    double timeToSeparationLossSeconds = -1.0;
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
