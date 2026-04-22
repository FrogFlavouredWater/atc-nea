#include "sim/ConflictDetector.h"

#include "core/Config.h"
#include "core/MathUtils.h"

#include <algorithm>
#include <cmath>

ConflictDetectionReport ConflictDetector::detect(const std::vector<std::unique_ptr<Aircraft>>& aircraft) const {
    ConflictDetectionReport report;

    for (size_t i = 0; i < aircraft.size(); ++i) {
        if (aircraft[i]->isDestroyed()) {
            continue;
        }
        for (size_t j = i + 1; j < aircraft.size(); ++j) {
            if (aircraft[j]->isDestroyed()) {
                continue;
            }

            const bool hasCurrentConflict = aircraft[i]->breachesSeparationWith(*aircraft[j]);
            if (hasCurrentConflict) {
                report.aircraftWithConflictAlert.insert(aircraft[i]->getCallsign());
                report.aircraftWithConflictAlert.insert(aircraft[j]->getCallsign());
                report.currentConflictPairs.insert(makeConflictPair(*aircraft[i], *aircraft[j]));
            }

            const PredictedConflictAssessment assessment = predictor.assessConflict(*aircraft[i],
                                                                                    *aircraft[j],
                                                                                    SimTuning::CONFLICT_LOOKAHEAD_SECONDS,
                                                                                    SimTuning::CONFLICT_PREDICTION_STEP_SECONDS);
            if (!hasCurrentConflict
                && assessment.valid
                && assessment.breachesTacticalThreshold
                && assessment.currentHorizontalDistanceNm <= SimTuning::TACTICAL_INTERVENTION_RANGE_NM
                && assessment.timeToTacticalThresholdSeconds >= 0.0) {
                report.predictedConflicts.push_back(assessment);
                report.predictedConflictPairs.insert(makeConflictPair(*aircraft[i], *aircraft[j]));
            }
        }
    }

    std::sort(report.predictedConflicts.begin(), report.predictedConflicts.end(),
              [](const PredictedConflictAssessment& first, const PredictedConflictAssessment& second) {
                  return first.timeToTacticalThresholdSeconds < second.timeToTacticalThresholdSeconds;
              });

    return report;
}



PredictedConflictAssessment ConflictDetector::assessConflict(const Aircraft& first, const Aircraft& second) const {
    return predictor.assessConflict(first,
                                    second,
                                    SimTuning::CONFLICT_LOOKAHEAD_SECONDS,
                                    SimTuning::CONFLICT_PREDICTION_STEP_SECONDS);
}

bool ConflictDetector::predictionBreachesSeparation(const std::vector<PredictedAircraftState>& firstPrediction,
                                                    const std::vector<PredictedAircraftState>& secondPrediction) const {
    const size_t sampleCount = std::min(firstPrediction.size(), secondPrediction.size());
    for (size_t i = 0; i < sampleCount; ++i) {
        const double horizontalDistanceNm = distanceNm(firstPrediction[i].motion.position,
                                                       secondPrediction[i].motion.position);
        const double verticalDistanceFt = std::abs(firstPrediction[i].motion.altitude - secondPrediction[i].motion.altitude);
        if (horizontalDistanceNm < SeparationRules::HORIZONTAL_NM
            && verticalDistanceFt < SeparationRules::VERTICAL_FT) {
            return true;
        }
    }

    return false;
}

std::pair<std::string, std::string> ConflictDetector::makeConflictPair(const Aircraft& first, const Aircraft& second) {
    return makeConflictPair(first.getCallsign(), second.getCallsign());
}

std::pair<std::string, std::string> ConflictDetector::makeConflictPair(const std::string& first,
                                                                       const std::string& second) {
    if (first < second) {
        return {first, second};
    }
    return {second, first};
}
