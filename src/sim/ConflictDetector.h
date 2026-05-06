#pragma once

#include "backend/aircraft/Aircraft.h"
#include "sim/TrajectoryPredictor.h"

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

// ConflictDetectionReport is a snapshot of what the detector found this tick.
struct ConflictDetectionReport {
    // Normalized callsign pairs currently breaching legal separation.
    std::set<std::pair<std::string, std::string>> currentConflictPairs;
    // Normalized callsign pairs predicted to enter the tactical intervention zone.
    std::set<std::pair<std::string, std::string>> predictedConflictPairs;
    // Callsigns used to light the per-aircraft conflict alert in the UI.
    std::set<std::string> aircraftWithConflictAlert;
    // Full predicted assessments, sorted by the earliest tactical trigger time.
    std::vector<PredictedConflictAssessment> predictedConflicts;
};

class ConflictDetector {
public:
    // Detect scans all aircraft pairs and returns current plus predicted issues
    // without mutating the simulation directly.
    [[nodiscard]] ConflictDetectionReport detect(const std::vector<std::unique_ptr<Aircraft>>& aircraft) const;
    // Re-assess one pair using the standard conflict lookahead settings.
    [[nodiscard]] PredictedConflictAssessment assessConflict(const Aircraft& first, const Aircraft& second) const;
    // Used by spawning to reject entries that would quickly break separation
    // even if the spawn is safe at the exact current moment.
    [[nodiscard]] bool predictionBreachesSeparation(const std::vector<PredictedAircraftState>& firstPrediction,
                                                    const std::vector<PredictedAircraftState>& secondPrediction) const;

    // Normalize callsign ordering. every subsystem gets the same pair key.
    [[nodiscard]] static std::pair<std::string, std::string> makeConflictPair(const Aircraft& first,
                                                                               const Aircraft& second);
    [[nodiscard]] static std::pair<std::string, std::string> makeConflictPair(const std::string& first,
                                                                               const std::string& second);

private:
    TrajectoryPredictor predictor{};
};
