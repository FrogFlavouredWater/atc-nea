#include "sim/TrajectoryPredictor.h"

#include "core/Config.h"
#include "core/MathUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr double kTacticalHorizontalThresholdNm = 5.0;

double horizontalDistanceNm(const AircraftMotionState& first, const AircraftMotionState& second) {
    return distanceNm(first.position, second.position);
}

double verticalDistanceFt(const AircraftMotionState& first, const AircraftMotionState& second) {
    return std::abs(first.altitude - second.altitude);
}

double severityScore(double horizontalDistanceNm,
                     double verticalDistanceFt,
                     double timeToClosestApproachSeconds,
                     double horizonSeconds) {
    // Severity is a simple heuristic: closer and sooner conflicts score higher.
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
                                                                 double horizon,
                                                                 double step) const {
    // Reject invalid sampling parameters. no invented fallback timing.
    if (horizon < 0.0 || step <= 0.0) {
        return {};
    }

    std::vector<PredictedAircraftState> prediction;
    prediction.reserve(static_cast<size_t>(horizon / step) + 2);

    Aircraft simulatedAircraft = aircraft;

    // Include the current state at t=0 so callers can compare "now" against the
    // later sampled points with one consistent array.
    prediction.push_back({0.0, simulatedAircraft.getMotionState()});

    double elapsedSeconds = 0.0;
    while (elapsedSeconds < horizon) {
        const double dt = std::min(step, horizon - elapsedSeconds);
        simulatedAircraft.update(dt);
        elapsedSeconds += dt;
        prediction.push_back({elapsedSeconds, simulatedAircraft.getMotionState()});
    }

    return prediction;
}

PredictedConflictAssessment TrajectoryPredictor::assessConflict(const Aircraft& first,
                                                                const Aircraft& second,
                                                                double horizon,
                                                                double step) const {
    // Same guard here too. assessConflict can be called directly.
    if (horizon < 0.0 || step <= 0.0) {
        return {};
    }

    const auto firstPrediction = predict(first, horizon, step);
    const auto secondPrediction = predict(second, horizon, step);
    if (firstPrediction.empty() || secondPrediction.empty()) {
        return {};
    }

    // Start with the current pair geometry so callers can compare "already bad"
    // against "will become bad soon" from the same assessment object.
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
    bool foundSeparationLoss = false;

    // Walk both predicted paths in lockstep and keep the earliest tactical
    // breach plus the closest/worst sample over the lookahead horizon.
    const size_t sampleCount = std::min(firstPrediction.size(), secondPrediction.size());
    for (size_t i = 0; i < sampleCount; ++i) {
        const double horizontal = horizontalDistanceNm(firstPrediction[i].motion, secondPrediction[i].motion);
        const double vertical = verticalDistanceFt(firstPrediction[i].motion, secondPrediction[i].motion);
        const bool breachesTacticalThreshold = horizontal < kTacticalHorizontalThresholdNm
            && vertical < SeparationRules::VERTICAL_FT;
        const bool breachesSeparation = horizontal < SeparationRules::HORIZONTAL_NM
            && vertical < SeparationRules::VERTICAL_FT;
        const double sampleTime = firstPrediction[i].timeSeconds;
        const double sampleSeverity = severityScore(horizontal, vertical, sampleTime, horizon);

        if (!assessment.breachesTacticalThreshold && breachesTacticalThreshold) {
            // Record only the first tactical breach time; later samples are more
            // useful for "closest approach" than for intervention timing.
            assessment.breachesTacticalThreshold = true;
            assessment.timeToTacticalThresholdSeconds = sampleTime;
        }

        if (breachesSeparation) {
            // Once separation is actually lost, keep the worst breach sample so
            // the resolver scores the most dangerous point in the encounter.
            const bool isBetterBreach = !foundSeparationLoss
                || sampleSeverity > bestBreachSeverity
                || (std::abs(sampleSeverity - bestBreachSeverity) < 1e-6
                    && sampleTime < assessment.timeToClosestApproachSeconds);

            if (!isBetterBreach) {
                continue;
            }

            bestBreachSeverity = sampleSeverity;
            foundSeparationLoss = true;
            assessment.timeToClosestApproachSeconds = sampleTime;
            assessment.closestHorizontalDistanceNm = horizontal;
            assessment.closestVerticalDistanceFt = vertical;
            continue;
        }

        if (foundSeparationLoss) {
            continue;
        }

        // If the pair never loses separation, still keep the closest sample so
        // the detector/resolver can reason about "near miss" situations.
        const bool isCloser = horizontal < fallbackHorizontalDistanceNm
            || (std::abs(horizontal - fallbackHorizontalDistanceNm) < 1e-6 && vertical < fallbackVerticalDistanceFt);
        if (!isCloser) {
            continue;
        }

        fallbackHorizontalDistanceNm = horizontal;
        fallbackVerticalDistanceFt = vertical;
        assessment.timeToClosestApproachSeconds = sampleTime;
        assessment.closestHorizontalDistanceNm = horizontal;
        assessment.closestVerticalDistanceFt = vertical;
    }

    assessment.severityScore = severityScore(assessment.closestHorizontalDistanceNm,
                                             assessment.closestVerticalDistanceFt,
                                             assessment.timeToClosestApproachSeconds,
                                             horizon);
    return assessment;
}
