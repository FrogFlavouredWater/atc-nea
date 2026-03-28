#include "backend/navigation/TrajectoryPredictor.h"
#include "common/constants.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kTacticalHorizontalThresholdNm = 5.0;

double horizontalDistanceNm(const AircraftMotionState& first, const AircraftMotionState& second) {
    const double dx = first.position.x - second.position.x;
    const double dy = first.position.y - second.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double verticalDistanceFt(const AircraftMotionState& first, const AircraftMotionState& second) {
    return std::abs(first.altitude - second.altitude);
}

double severityScore(double horizontalDistanceNm,
                     double verticalDistanceFt,
                     double timeToClosestApproachSeconds,
                     double horizonSeconds) {
    const double horizontalPenalty = std::max(0.0,
        (kTacticalHorizontalThresholdNm - horizontalDistanceNm) / kTacticalHorizontalThresholdNm);
    const double verticalPenalty = std::max(0.0,
        (SeparationRules::VERTICAL_FT - verticalDistanceFt) / SeparationRules::VERTICAL_FT);
    const double timeWeight = horizonSeconds > 0.0
        ? std::clamp((horizonSeconds - timeToClosestApproachSeconds) / horizonSeconds, 0.0, 1.0)
        : 0.0;
    return horizontalPenalty + verticalPenalty + timeWeight * 0.25;
}
}

std::vector<PredictedAircraftState> TrajectoryPredictor::predict(const Aircraft& aircraft,
                                                                 double horizonSeconds,
                                                                 double stepSeconds) {
    if (horizonSeconds < 0.0 || stepSeconds <= 0.0) {
        return {};
    }

    std::vector<PredictedAircraftState> prediction;
    prediction.reserve(static_cast<size_t>(horizonSeconds / stepSeconds) + 2);

    Aircraft simulatedAircraft = aircraft;

    prediction.push_back({0.0, simulatedAircraft.getMotionState()});

    double elapsedSeconds = 0.0;
    while (elapsedSeconds < horizonSeconds) {
        const double delta = std::min(stepSeconds, horizonSeconds - elapsedSeconds);
        simulatedAircraft.update(delta);
        elapsedSeconds += delta;
        prediction.push_back({elapsedSeconds, simulatedAircraft.getMotionState()});
    }

    return prediction;
}

PredictedConflictAssessment TrajectoryPredictor::assessConflict(const Aircraft& first,
                                                                const Aircraft& second,
                                                                double horizonSeconds,
                                                                double stepSeconds) {
    if (horizonSeconds < 0.0 || stepSeconds <= 0.0) {
        return {};
    }

    const auto firstPrediction = predict(first, horizonSeconds, stepSeconds);
    const auto secondPrediction = predict(second, horizonSeconds, stepSeconds);
    if (firstPrediction.empty() || secondPrediction.empty()) {
        return {};
    }

    PredictedConflictAssessment assessment;
    assessment.valid = true;
    assessment.firstCallsign = first.getCallsign();
    assessment.secondCallsign = second.getCallsign();
    assessment.currentHorizontalDistanceNm = horizontalDistanceNm(firstPrediction.front().motion,
                                                                  secondPrediction.front().motion);
    assessment.currentVerticalDistanceFt = verticalDistanceFt(firstPrediction.front().motion,
                                                              secondPrediction.front().motion);

    double fallbackHorizontalDistanceNm = std::numeric_limits<double>::max();
    double fallbackVerticalDistanceFt = std::numeric_limits<double>::max();
    double bestBreachSeverity = -1.0;

    const size_t sampleCount = std::min(firstPrediction.size(), secondPrediction.size());
    for (size_t i = 0; i < sampleCount; ++i) {
        const double horizontal = horizontalDistanceNm(firstPrediction[i].motion, secondPrediction[i].motion);
        const double vertical = verticalDistanceFt(firstPrediction[i].motion, secondPrediction[i].motion);
        const bool breachesTacticalThreshold = horizontal < kTacticalHorizontalThresholdNm
            && vertical < SeparationRules::VERTICAL_FT;
        const bool breachesSeparation = horizontal < SeparationRules::HORIZONTAL_NM
            && vertical < SeparationRules::VERTICAL_FT;
        const double sampleTime = firstPrediction[i].timeSeconds;
        const double sampleSeverity = severityScore(horizontal, vertical, sampleTime, horizonSeconds);

        if (!assessment.breachesTacticalThreshold && breachesTacticalThreshold) {
            assessment.breachesTacticalThreshold = true;
            assessment.timeToTacticalThresholdSeconds = sampleTime;
            assessment.tacticalHorizontalDistanceNm = horizontal;
            assessment.tacticalVerticalDistanceFt = vertical;
        }

        if (breachesSeparation) {
            if (assessment.timeToSeparationLossSeconds < 0.0) {
                assessment.timeToSeparationLossSeconds = sampleTime;
            }

            const bool isBetterBreach = !assessment.breachesSeparation
                || sampleSeverity > bestBreachSeverity
                || (std::abs(sampleSeverity - bestBreachSeverity) < 1e-6
                    && sampleTime < assessment.timeToClosestApproachSeconds);

            if (!isBetterBreach) {
                continue;
            }

            bestBreachSeverity = sampleSeverity;
            assessment.breachesSeparation = true;
            assessment.timeToClosestApproachSeconds = sampleTime;
            assessment.horizontalDistanceNm = horizontal;
            assessment.verticalDistanceFt = vertical;
            assessment.firstMotion = firstPrediction[i].motion;
            assessment.secondMotion = secondPrediction[i].motion;
            continue;
        }

        if (assessment.breachesSeparation) {
            continue;
        }

        const bool isCloser = horizontal < fallbackHorizontalDistanceNm
            || (std::abs(horizontal - fallbackHorizontalDistanceNm) < 1e-6 && vertical < fallbackVerticalDistanceFt);
        if (!isCloser) {
            continue;
        }

        fallbackHorizontalDistanceNm = horizontal;
        fallbackVerticalDistanceFt = vertical;
        assessment.timeToClosestApproachSeconds = sampleTime;
        assessment.horizontalDistanceNm = horizontal;
        assessment.verticalDistanceFt = vertical;
        assessment.firstMotion = firstPrediction[i].motion;
        assessment.secondMotion = secondPrediction[i].motion;
    }

    assessment.severityScore = severityScore(assessment.horizontalDistanceNm,
                                             assessment.verticalDistanceFt,
                                             assessment.timeToClosestApproachSeconds,
                                             horizonSeconds);
    return assessment;
}
