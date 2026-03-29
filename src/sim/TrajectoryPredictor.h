#pragma once

#include "backend/aircraft/Aircraft.h"
#include "backend/aircraft/AircraftMotion.h"

#include <string>
#include <vector>

// Predictor samples are future snapshots of one aircraft under its current
// instruction state.
struct PredictedAircraftState {
    double timeSeconds = 0.0;
    AircraftMotionState motion{};
};

struct PredictedConflictAssessment {
    bool valid = false;
    bool breachesTacticalThreshold = false;
    std::string firstCallsign{};
    std::string secondCallsign{};
    // Separation at t=0, before any prediction steps are applied.
    double currentHorizontalDistanceNm = 0.0;
    double currentVerticalDistanceFt = 0.0;
    // First moment the pair enters the tactical threshold used for intervention.
    double timeToTacticalThresholdSeconds = -1.0;
    // Time of the closest or worst predicted sample across the whole lookahead.
    double timeToClosestApproachSeconds = 0.0;
    // "closest" values describe the worst predicted sample over the lookahead,
    // not necessarily the pair's current spacing.
    double closestHorizontalDistanceNm = 0.0;
    double closestVerticalDistanceFt = 0.0;
    // Relative urgency used by the resolver when it compares candidate maneuvers.
    double severityScore = 0.0;
};

class TrajectoryPredictor {
public:
    // Both helpers sample forward motion; assessConflict simply compares two
    // predicted paths after running the same predictor.
    [[nodiscard]] std::vector<PredictedAircraftState> predict(const Aircraft& aircraft,
                                                              double horizonSeconds,
                                                              double stepSeconds) const;
    [[nodiscard]] PredictedConflictAssessment assessConflict(const Aircraft& first,
                                                             const Aircraft& second,
                                                             double horizonSeconds,
                                                             double stepSeconds) const;
};
