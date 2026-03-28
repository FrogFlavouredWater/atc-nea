#include "backend/simulation/Simulation.h"

#include "backend/simulation/SimUtil.h"
#include "common/logger.h"
#include "common/utils.h"
#include <algorithm>
#include <cmath>

using namespace SimulationDetail;

std::vector<PredictedConflictAssessment> Simulation::collectResolvableConflicts() const {
    std::vector<PredictedConflictAssessment> conflicts = predictedConflicts;
    conflicts.reserve(conflicts.size() + activeConflictPairs.size());

    for (const auto& pair : activeConflictPairs) {
        const Aircraft* first = findAircraftByCallsign(pair.first);
        const Aircraft* second = findAircraftByCallsign(pair.second);
        if (!first || !second) {
            continue;
        }

        conflicts.push_back(TrajectoryPredictor::assessConflict(*first,
                                                                *second,
                                                                kConflictLookaheadSeconds,
                                                                kConflictPredictionStepSeconds));
    }

    std::sort(conflicts.begin(), conflicts.end(),
              [](const PredictedConflictAssessment& first, const PredictedConflictAssessment& second) {
                  const double firstTriggerTime = first.breachesTacticalThreshold
                      ? first.timeToTacticalThresholdSeconds
                      : first.timeToClosestApproachSeconds;
                  const double secondTriggerTime = second.breachesTacticalThreshold
                      ? second.timeToTacticalThresholdSeconds
                      : second.timeToClosestApproachSeconds;
                  if (std::abs(firstTriggerTime - secondTriggerTime) > 1e-6) {
                      return firstTriggerTime < secondTriggerTime;
                  }
                  return first.severityScore > second.severityScore;
              });
    return conflicts;
}

std::vector<AircraftInstruction> Simulation::buildResolutionCandidates(const Aircraft& plane,
                                                                       const Aircraft& other,
                                                                       const PredictedConflictAssessment& conflict) const {
    const AircraftCommand baseline = plane.getCommand();
    std::vector<AircraftInstruction> candidates;
    candidates.reserve(9);

    const double dx = other.getPosition().x - plane.getPosition().x;
    const double dy = other.getPosition().y - plane.getPosition().y;
    const double intruderBearingDeg = normalizeAngle(std::atan2(dy, dx) * 180.0 / kPi + 90.0);
    const double relativeBearingDeg = getShortestAngleDiff(intruderBearingDeg, plane.getHeading());
    const double preferredTurnSign = relativeBearingDeg >= 0.0 ? -1.0 : 1.0;
    const bool vectoringPriority = conflict.verticalDistanceFt <= kVectoringPriorityVerticalFt
        || conflict.currentVerticalDistanceFt <= kVectoringPriorityVerticalFt;
    const bool inTerminalArea = !airports.empty()
        && distanceNm(plane.getPosition(), airports.front().position) <= kArrivalSpeedDistanceNm;
    const bool allowSpeedCandidates = inTerminalArea
        && !plane.hasApproachClearance()
        && plane.getControlMode() != AircraftControlMode::ILS;

    auto addHeadingCandidate = [&](double offsetDeg) {
        candidates.push_back(AircraftInstruction{
            AircraftInstructionType::CONFLICT_RESOLUTION,
            normalizeAngle(plane.getHeading() + offsetDeg),
            baseline.targetSpeed,
            baseline.targetAltitude,
            AircraftControlMode::AUTONOMOUS
        });
    };

    auto addAltitudeCandidate = [&](int targetAltitude) {
        if (targetAltitude == baseline.targetAltitude) {
            return;
        }

        candidates.push_back(AircraftInstruction{
            AircraftInstructionType::CONFLICT_RESOLUTION,
            baseline.targetHeading,
            baseline.targetSpeed,
            std::max(0, targetAltitude),
            AircraftControlMode::AUTONOMOUS
        });
    };

    auto addSpeedCandidate = [&](double targetSpeedKts) {
        if (!allowSpeedCandidates || std::abs(targetSpeedKts - baseline.targetSpeed) < 1e-6) {
            return;
        }

        candidates.push_back(AircraftInstruction{
            AircraftInstructionType::CONFLICT_RESOLUTION,
            baseline.targetHeading,
            targetSpeedKts,
            baseline.targetAltitude,
            AircraftControlMode::AUTONOMOUS
        });
    };

    auto addAltitudeFamily = [&]() {
        if (plane.getAltitudeExact() > other.getAltitudeExact()) {
            addAltitudeCandidate(baseline.targetAltitude + kResolutionAltitudeStepFt);
            addAltitudeCandidate(baseline.targetAltitude - kResolutionAltitudeStepFt);
        } else if (plane.getAltitudeExact() < other.getAltitudeExact()) {
            addAltitudeCandidate(baseline.targetAltitude - kResolutionAltitudeStepFt);
            addAltitudeCandidate(baseline.targetAltitude + kResolutionAltitudeStepFt);
        } else {
            addAltitudeCandidate(baseline.targetAltitude + kResolutionAltitudeStepFt);
            addAltitudeCandidate(baseline.targetAltitude - kResolutionAltitudeStepFt);
        }
    };

    auto addHeadingFamily = [&]() {
        addHeadingCandidate(preferredTurnSign * kResolutionTurnSmallDeg);
        addHeadingCandidate(-preferredTurnSign * kResolutionTurnSmallDeg);
        addHeadingCandidate(preferredTurnSign * kResolutionTurnLargeDeg);
        addHeadingCandidate(-preferredTurnSign * kResolutionTurnLargeDeg);
    };

    if (vectoringPriority) {
        addHeadingFamily();
        addAltitudeFamily();
    } else {
        addAltitudeFamily();
        addHeadingFamily();
    }

    addSpeedCandidate(baseline.targetSpeed - kResolutionSpeedStepKts);
    addSpeedCandidate(baseline.targetSpeed - 2.0 * kResolutionSpeedStepKts);
    addSpeedCandidate(baseline.targetSpeed + kResolutionSpeedStepKts);

    return candidates;
}

PredictedConflictAssessment Simulation::assessConflictWithInstruction(const Aircraft& plane,
                                                                     const AircraftInstruction& instruction,
                                                                     const Aircraft& other) const {
    Aircraft trialAircraft(plane.getPosition(),
                           plane.getHeading(),
                           plane.getSpeed(),
                           plane.getAltitude(),
                           plane.getCallsign());
    trialAircraft.applyInstruction(instruction);

    return TrajectoryPredictor::assessConflict(trialAircraft,
                                               other,
                                               kConflictLookaheadSeconds,
                                               kConflictPredictionStepSeconds);
}

void Simulation::releaseResolvedAircraft() {
    for (auto it = activeResolutionStates.begin(); it != activeResolutionStates.end(); ) {
        Aircraft* plane = findAircraftByCallsign(it->first);
        if (!plane) {
            it = activeResolutionStates.erase(it);
            continue;
        }

        const bool pairStillActive = activeConflictPairs.contains(it->second.conflictPair)
            || activePredictedConflictPairs.contains(it->second.conflictPair);
        const bool minimumHoldElapsed = elapsedSimSeconds - it->second.assignedAtSeconds >= kResolutionHoldSeconds;
        if (pairStillActive || !minimumHoldElapsed) {
            ++it;
            continue;
        }

        issueInstruction(plane, it->second.resumeInstruction);
        Logger::info("Released " + plane->getCallsign() + " from conflict resolution");
        it = activeResolutionStates.erase(it);
    }
}

void Simulation::updateConflictResolutions() {
    const auto conflicts = collectResolvableConflicts();
    if (conflicts.empty()) {
        return;
    }

    for (const auto& conflict : conflicts) {
        Aircraft* first = findAircraftByCallsign(conflict.firstCallsign);
        Aircraft* second = findAircraftByCallsign(conflict.secondCallsign);
        if (!first || !second) {
            continue;
        }

        const auto pair = makeConflictPair(conflict.firstCallsign, conflict.secondCallsign);
        auto firstStateIt = activeResolutionStates.find(first->getCallsign());
        auto secondStateIt = activeResolutionStates.find(second->getCallsign());
        const bool samePairRecentlyAssigned =
            (firstStateIt != activeResolutionStates.end()
             && firstStateIt->second.conflictPair == pair
             && elapsedSimSeconds - firstStateIt->second.assignedAtSeconds < kResolutionReevaluationSeconds)
            || (secondStateIt != activeResolutionStates.end()
                && secondStateIt->second.conflictPair == pair
                && elapsedSimSeconds - secondStateIt->second.assignedAtSeconds < kResolutionReevaluationSeconds);
        if (samePairRecentlyAssigned) {
            continue;
        }

        auto shouldPreserve = [this](const Aircraft& plane) {
            if (plane.getControlMode() == AircraftControlMode::ILS) {
                return true;
            }
            if (plane.hasApproachClearance()) {
                return true;
            }
            if (airports.empty()) {
                return false;
            }
            return distanceNm(plane.getPosition(), airports.front().position) < 12.0;
        };

        Aircraft* preferredAircraft = first;
        Aircraft* alternateAircraft = second;
        const bool preserveFirst = shouldPreserve(*first);
        const bool preserveSecond = shouldPreserve(*second);
        if (preserveFirst != preserveSecond) {
            preferredAircraft = preserveFirst ? second : first;
            alternateAircraft = preserveFirst ? first : second;
        } else if (!airports.empty()) {
            const double firstDistanceToAirportNm = distanceNm(first->getPosition(), airports.front().position);
            const double secondDistanceToAirportNm = distanceNm(second->getPosition(), airports.front().position);
            preferredAircraft = firstDistanceToAirportNm >= secondDistanceToAirportNm ? first : second;
            alternateAircraft = preferredAircraft == first ? second : first;
        } else if (second->getCallsign() > first->getCallsign()) {
            preferredAircraft = second;
            alternateAircraft = first;
        }

        std::vector<Aircraft*> maneuverOrder;
        maneuverOrder.push_back(preferredAircraft);
        if (alternateAircraft != preferredAircraft) {
            maneuverOrder.push_back(alternateAircraft);
        }

        for (Aircraft* candidatePlane : maneuverOrder) {
            auto existingStateIt = activeResolutionStates.find(candidatePlane->getCallsign());
            if (existingStateIt != activeResolutionStates.end()
                && existingStateIt->second.conflictPair != pair) {
                continue;
            }

            Aircraft* otherPlane = candidatePlane == first ? second : first;
            const auto candidates = buildResolutionCandidates(*candidatePlane, *otherPlane, conflict);
            AircraftInstruction bestInstruction{};
            double bestSeverityScore = conflict.severityScore;
            bool foundImprovement = false;
            bool foundClearResolution = false;

            for (const auto& candidateInstruction : candidates) {
                const PredictedConflictAssessment assessment =
                    assessConflictWithInstruction(*candidatePlane, candidateInstruction, *otherPlane);
                if (!assessment.valid) {
                    continue;
                }

                if (!assessment.breachesTacticalThreshold) {
                    bestInstruction = candidateInstruction;
                    foundImprovement = true;
                    foundClearResolution = true;
                    break;
                }

                if (assessment.severityScore < bestSeverityScore - 1e-6) {
                    bestSeverityScore = assessment.severityScore;
                    bestInstruction = candidateInstruction;
                    foundImprovement = true;
                }
            }

            if (!foundImprovement) {
                continue;
            }

            const AircraftInstruction resumeInstruction =
                existingStateIt != activeResolutionStates.end()
                    ? existingStateIt->second.resumeInstruction
                    : candidatePlane->getActiveInstruction();
            issueInstruction(candidatePlane, bestInstruction);
            activeResolutionStates[candidatePlane->getCallsign()] = ConflictResolutionState{
                pair,
                resumeInstruction,
                elapsedSimSeconds
            };

            Logger::warn(std::string("Conflict resolution assigned to ")
                         + candidatePlane->getCallsign()
                         + " for pair "
                         + pair.first
                         + "/"
                         + pair.second
                         + (foundClearResolution ? " (clearing maneuver)" : " (mitigation maneuver)"));
            break;
        }
    }
}

void Simulation::detectConflicts() {
    for (auto& plane : aircraft) {
        plane->setConflictAlert(false);
    }

    std::set<std::pair<std::string, std::string>> currentConflictPairs;
    std::set<std::pair<std::string, std::string>> currentPredictedConflictPairs;
    predictedConflicts.clear();

    for (size_t i = 0; i < aircraft.size(); ++i) {
        for (size_t j = i + 1; j < aircraft.size(); ++j) {
            const bool hasCurrentConflict = aircraft[i]->breachesSeparationWith(*aircraft[j]);
            if (hasCurrentConflict) {
                aircraft[i]->setConflictAlert(true);
                aircraft[j]->setConflictAlert(true);
                currentConflictPairs.insert(makeConflictPair(*aircraft[i], *aircraft[j]));
            }

            const PredictedConflictAssessment assessment = TrajectoryPredictor::assessConflict(*aircraft[i],
                                                                                               *aircraft[j],
                                                                                               kConflictLookaheadSeconds,
                                                                                               kConflictPredictionStepSeconds);
            if (!hasCurrentConflict
                && assessment.valid
                && assessment.breachesTacticalThreshold
                && assessment.currentHorizontalDistanceNm <= kTacticalInterventionRangeNm
                && assessment.timeToTacticalThresholdSeconds >= 0.0) {
                predictedConflicts.push_back(assessment);
                currentPredictedConflictPairs.insert(makeConflictPair(*aircraft[i], *aircraft[j]));
            }
        }
    }

    std::sort(predictedConflicts.begin(), predictedConflicts.end(),
              [](const PredictedConflictAssessment& first, const PredictedConflictAssessment& second) {
                  return first.timeToTacticalThresholdSeconds < second.timeToTacticalThresholdSeconds;
              });

    for (const auto& pair : currentConflictPairs) {
        if (!activeConflictPairs.contains(pair)) {
            Logger::warn("Conflict detected between " + pair.first + " and " + pair.second);
        }
    }

    for (const auto& pair : activeConflictPairs) {
        if (!currentConflictPairs.contains(pair)) {
            Logger::info("Conflict resolved between " + pair.first + " and " + pair.second);
        }
    }

    for (const auto& assessment : predictedConflicts) {
        const auto pair = makeConflictPair(assessment.firstCallsign, assessment.secondCallsign);
        if (!activePredictedConflictPairs.contains(pair)) {
            Logger::warn("Predicted tactical conflict between "
                         + assessment.firstCallsign
                         + " and "
                         + assessment.secondCallsign
                         + " in "
                         + std::to_string(static_cast<int>(std::lround(assessment.timeToTacticalThresholdSeconds)))
                         + "s");
        }
    }

    for (const auto& pair : activePredictedConflictPairs) {
        if (!currentPredictedConflictPairs.contains(pair)) {
            Logger::info("Predicted conflict cleared between " + pair.first + " and " + pair.second);
        }
    }

    activeConflictPairs = std::move(currentConflictPairs);
    activePredictedConflictPairs = std::move(currentPredictedConflictPairs);
}
