#include "sim/ConflictResolver.h"

#include "core/Config.h"
#include "core/MathUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace {
using ConflictPair = std::pair<std::string, std::string>;
constexpr double kPreserveDistanceNm = 12.0;

Aircraft* findAircraftByCallsign(const std::vector<std::unique_ptr<Aircraft>>& aircraft, const std::string& callsign) {
    for (const auto& candidate : aircraft) {
        if (candidate->getCallsign() == callsign) {
            return candidate.get();
        }
    }
    return nullptr;
}

bool shouldPreserveAircraft(const Aircraft& plane, const std::vector<Airport>& airports) {
    // Preserve aircraft already committed to the runway environment when
    // possible, and maneuver the easier aircraft first.
    if (plane.getControlMode() == AircraftControlMode::ILS) {
        return true;
    }
    if (plane.hasApproachClearance()) {
        return true;
    }
    if (airports.empty()) {
        return false;
    }
    return distanceNm(plane.getPosition(), airports.front().position) < kPreserveDistanceNm;
}

AircraftInstruction makeResolutionInstruction(double heading, double speed, int altitude) {
    // All resolver-generated maneuvers are tagged as autonomous conflict
    // resolutions so the rest of the sim can treat them specially.
    return AircraftInstruction{
        AircraftInstructionType::CONFLICT_RESOLUTION,
        heading,
        speed,
        altitude,
        AircraftControlMode::AUTONOMOUS
    };
}

std::vector<const Aircraft*> buildManeuverOrder(const Aircraft& first,
                                                const Aircraft& second,
                                                const std::vector<Airport>& airports) {
    // Prefer to maneuver the aircraft that is less constrained by approach or
    // terminal-area considerations.
    const Aircraft* preferredAircraft = &first;
    const Aircraft* alternateAircraft = &second;
    const bool preserveFirst = shouldPreserveAircraft(first, airports);
    const bool preserveSecond = shouldPreserveAircraft(second, airports);

    if (preserveFirst != preserveSecond) {
        preferredAircraft = preserveFirst ? &second : &first;
        alternateAircraft = preserveFirst ? &first : &second;
    } else if (!airports.empty()) {
        const double firstDistanceToAirportNm = distanceNm(first.getPosition(), airports.front().position);
        const double secondDistanceToAirportNm = distanceNm(second.getPosition(), airports.front().position);
        preferredAircraft = firstDistanceToAirportNm >= secondDistanceToAirportNm ? &first : &second;
        alternateAircraft = preferredAircraft == &first ? &second : &first;
    } else if (second.getCallsign() > first.getCallsign()) {
        preferredAircraft = &second;
        alternateAircraft = &first;
    }

    std::vector<const Aircraft*> maneuverOrder{preferredAircraft};
    if (alternateAircraft != preferredAircraft) {
        maneuverOrder.push_back(alternateAircraft);
    }
    return maneuverOrder;
}

void appendHeadingCandidates(std::vector<AircraftInstruction>& candidates,
                             const Aircraft& plane,
                             const AircraftCommand& baseline,
                             double preferredTurnSign) {
    // Try both left and right vectors, but bias the order away from the
    // intruder bearing and start with the smaller heading changes.
    const std::array<double, 4> headingOffsetsDeg{
        preferredTurnSign * SimTuning::RESOLUTION_TURN_SMALL_DEG,
        -preferredTurnSign * SimTuning::RESOLUTION_TURN_SMALL_DEG,
        preferredTurnSign * SimTuning::RESOLUTION_TURN_LARGE_DEG,
        -preferredTurnSign * SimTuning::RESOLUTION_TURN_LARGE_DEG
    };

    for (double offsetDeg : headingOffsetsDeg) {
        candidates.push_back(makeResolutionInstruction(normalizeAngle(plane.getHeading() + offsetDeg),
                                                       baseline.targetSpeed,
                                                       baseline.targetAltitude));
    }
}

void appendAltitudeCandidates(std::vector<AircraftInstruction>& candidates,
                              const Aircraft& plane,
                              const Aircraft& other,
                              const AircraftCommand& baseline) {
    // Try moving away from the other aircraft's altitude first, then keep the
    // opposite direction as a fallback if needed.
    const auto appendAltitudeCandidate = [&](int targetAltitude) {
        if (targetAltitude == baseline.targetAltitude) {
            return;
        }

        candidates.push_back(makeResolutionInstruction(baseline.targetHeading,
                                                       baseline.targetSpeed,
                                                       std::max(0, targetAltitude)));
    };

    if (plane.getAltitudeExact() > other.getAltitudeExact()) {
        appendAltitudeCandidate(baseline.targetAltitude + SimTuning::RESOLUTION_ALTITUDE_STEP_FT);
        appendAltitudeCandidate(baseline.targetAltitude - SimTuning::RESOLUTION_ALTITUDE_STEP_FT);
        return;
    }

    if (plane.getAltitudeExact() < other.getAltitudeExact()) {
        appendAltitudeCandidate(baseline.targetAltitude - SimTuning::RESOLUTION_ALTITUDE_STEP_FT);
        appendAltitudeCandidate(baseline.targetAltitude + SimTuning::RESOLUTION_ALTITUDE_STEP_FT);
        return;
    }

    appendAltitudeCandidate(baseline.targetAltitude + SimTuning::RESOLUTION_ALTITUDE_STEP_FT);
    appendAltitudeCandidate(baseline.targetAltitude - SimTuning::RESOLUTION_ALTITUDE_STEP_FT);
}

void appendSpeedCandidates(std::vector<AircraftInstruction>& candidates,
                           const Aircraft& plane,
                           const std::vector<Airport>& airports,
                           const AircraftCommand& baseline) {
    // Speed control is intentionally limited to the terminal area and is not
    // used once an aircraft is approach-cleared or already on ILS.
    const bool inTerminalArea = !airports.empty()
        && distanceNm(plane.getPosition(), airports.front().position) <= SimTuning::ARRIVAL_SPEED_DISTANCE_NM;
    const bool allowSpeedCandidates = inTerminalArea
        && !plane.hasApproachClearance()
        && plane.getControlMode() != AircraftControlMode::ILS;
    if (!allowSpeedCandidates) {
        return;
    }

    const std::array<double, 3> speedTargetsKts{
        baseline.targetSpeed - SimTuning::RESOLUTION_SPEED_STEP_KTS,
        baseline.targetSpeed - 2.0 * SimTuning::RESOLUTION_SPEED_STEP_KTS,
        baseline.targetSpeed + SimTuning::RESOLUTION_SPEED_STEP_KTS
    };

    for (double targetSpeedKts : speedTargetsKts) {
        if (std::abs(targetSpeedKts - baseline.targetSpeed) < 1e-6) {
            continue;
        }

        candidates.push_back(makeResolutionInstruction(baseline.targetHeading,
                                                       targetSpeedKts,
                                                       baseline.targetAltitude));
    }
}
}

std::vector<ConflictResolutionRelease> ConflictResolver::collectReleases(
    const std::vector<std::unique_ptr<Aircraft>>& aircraft,
    const std::set<std::pair<std::string, std::string>>& activeConflictPairs,
    const std::set<std::pair<std::string, std::string>>& activePredictedConflictPairs,
    double elapsedSimSeconds,
    double elapsedUiSeconds) {
    std::vector<ConflictResolutionRelease> releases;

    // Release only after the pair is no longer active and the minimum hold
    // window has elapsed, to avoid rapid flip-flopping.
    for (auto it = activeStates.begin(); it != activeStates.end(); ) {
        Aircraft* plane = findAircraftByCallsign(aircraft, it->first);
        if (!plane || plane->isDestroyed()) {
            it = activeStates.erase(it);
            continue;
        }

        const bool pairStillActive = activeConflictPairs.contains(it->second.conflictPair)
            || activePredictedConflictPairs.contains(it->second.conflictPair);
        const bool minimumHoldElapsed =
            elapsedSimSeconds - it->second.assignedAtSeconds >= SimTuning::RESOLUTION_HOLD_SECONDS
            && elapsedUiSeconds - it->second.assignedAtUiSeconds >= SimTuning::RESOLUTION_HOLD_REAL_SECONDS;
        if (pairStillActive || !minimumHoldElapsed) {
            ++it;
            continue;
        }

        releases.push_back(ConflictResolutionRelease{
            plane->getCallsign(),
            it->second.resumeInstruction
        });
        it = activeStates.erase(it);
    }

    return releases;
}

std::vector<ConflictResolutionAssignment> ConflictResolver::resolve(
    const std::vector<std::unique_ptr<Aircraft>>& aircraft,
    const std::vector<Airport>& airports,
    const std::set<std::pair<std::string, std::string>>& activeConflictPairs,
    const std::vector<PredictedConflictAssessment>& predictedConflicts,
    const ConflictDetector& conflictDetector,
    double elapsedSimSeconds,
    double elapsedUiSeconds) {
    std::vector<ConflictResolutionAssignment> assignments;
    const auto conflicts = collectConflicts(aircraft, activeConflictPairs, predictedConflicts, conflictDetector);
    if (conflicts.empty()) {
        return assignments;
    }

    // Conflicts are already sorted by urgency, so earlier entries get first
    // access to maneuver candidates and resolution slots.
    for (const auto& conflict : conflicts) {
        Aircraft* first = findAircraftByCallsign(aircraft, conflict.firstCallsign);
        Aircraft* second = findAircraftByCallsign(aircraft, conflict.secondCallsign);
        if (!first || !second || first->isDestroyed() || second->isDestroyed()) {
            continue;
        }

        const auto pair = ConflictDetector::makeConflictPair(conflict.firstCallsign, conflict.secondCallsign);
        if (recentlyAssigned(*first, *second, pair, elapsedSimSeconds, elapsedUiSeconds)) {
            continue;
        }

        if (auto assignment = chooseAssignment(*first,
                                               *second,
                                               conflict,
                                               pair,
                                               airports,
                                               conflictDetector,
                                               elapsedSimSeconds,
                                               elapsedUiSeconds)) {
            assignments.push_back(*assignment);
        }
    }

    return assignments;
}

std::vector<PredictedConflictAssessment> ConflictResolver::collectConflicts(
    const std::vector<std::unique_ptr<Aircraft>>& aircraft,
    const std::set<std::pair<std::string, std::string>>& activeConflictPairs,
    const std::vector<PredictedConflictAssessment>& predictedConflicts,
    const ConflictDetector& conflictDetector) const {
    std::vector<PredictedConflictAssessment> conflicts = predictedConflicts;
    conflicts.reserve(conflicts.size() + activeConflictPairs.size());

    // Current conflicts are re-assessed through the same predictor path so the
    // resolver can sort and score all conflicts with one data shape.
    for (const auto& pair : activeConflictPairs) {
        const Aircraft* first = findAircraftByCallsign(aircraft, pair.first);
        const Aircraft* second = findAircraftByCallsign(aircraft, pair.second);
        if (!first || !second || first->isDestroyed() || second->isDestroyed()) {
            continue;
        }

        conflicts.push_back(conflictDetector.assessConflict(*first, *second));
    }

    std::sort(conflicts.begin(), conflicts.end(),
              [](const PredictedConflictAssessment& first, const PredictedConflictAssessment& second) {
                  // Use the tactical threshold time when available; otherwise
                  // fall back to the closest-approach time for current conflicts.
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

std::vector<AircraftInstruction> ConflictResolver::buildCandidates(
    const Aircraft& plane,
    const Aircraft& other,
    const std::vector<Airport>& airports) const {
    const AircraftCommand baseline = plane.getCommand();
    std::vector<AircraftInstruction> candidates;
    candidates.reserve(9);

    // Bias first turn away from the intruder. then add speed/altitude fallbacks around current command.
    const double dx = other.getPosition().x - plane.getPosition().x;
    const double dy = other.getPosition().y - plane.getPosition().y;
    const double intruderBearingDeg = normalizeAngle(std::atan2(dy, dx) * 180.0 / std::numbers::pi_v<double> + 90.0);
    const double relativeBearingDeg = getShortestAngleDiff(intruderBearingDeg, plane.getHeading());
    const double preferredTurnSign = relativeBearingDeg >= 0.0 ? -1.0 : 1.0;

    appendHeadingCandidates(candidates, plane, baseline, preferredTurnSign);
    appendSpeedCandidates(candidates, plane, airports, baseline);
    appendAltitudeCandidates(candidates, plane, other, baseline);

    return candidates;
}

PredictedConflictAssessment ConflictResolver::assessWithInstruction(
    const Aircraft& plane,
    const AircraftInstruction& instruction,
    const Aircraft& other,
    const ConflictDetector& conflictDetector) const {
    // Test candidate maneuvers on a temporary copy so the real aircraft is only
    // touched once the resolver has chosen an instruction to apply.
    Aircraft trialAircraft = plane;
    trialAircraft.applyInstruction(instruction);
    return conflictDetector.assessConflict(trialAircraft, other);
}

bool ConflictResolver::recentlyAssigned(
    const Aircraft& first,
    const Aircraft& second,
    const ConflictPair& pair,
    double simTime,
    double uiTime) const {
    // Check both clocks. avoids instant pair recycling in fast or paused sim states.
    const auto wasRecentlyAssigned = [&](const Aircraft& plane) {
        const auto stateIt = activeStates.find(plane.getCallsign());
        return stateIt != activeStates.end()
            && stateIt->second.conflictPair == pair
            && (simTime - stateIt->second.assignedAtSeconds
                    < SimTuning::RESOLUTION_REEVALUATION_SECONDS
                || uiTime - stateIt->second.assignedAtUiSeconds
                    < SimTuning::RESOLUTION_REEVALUATION_REAL_SECONDS);
    };

    return wasRecentlyAssigned(first) || wasRecentlyAssigned(second);
}

std::optional<ConflictResolutionAssignment> ConflictResolver::chooseAssignment(
    const Aircraft& first,
    const Aircraft& second,
    const PredictedConflictAssessment& conflict,
    const ConflictPair& pair,
    const std::vector<Airport>& airports,
    const ConflictDetector& conflictDetector,
    double simTime,
    double uiTime) {
    for (const Aircraft* candidatePlane : buildManeuverOrder(first, second, airports)) {
        // Do not stack two unrelated resolution states onto the same aircraft.
        const auto stateIt = activeStates.find(candidatePlane->getCallsign());
        if (stateIt != activeStates.end()
            && stateIt->second.conflictPair != pair) {
            continue;
        }

        const Aircraft& otherPlane = candidatePlane == &first ? second : first;
        const auto candidates = buildCandidates(*candidatePlane, otherPlane, airports);
        AircraftInstruction bestInstruction{};
        double bestSeverityScore = conflict.severityScore;
        bool foundImprovement = false;
        bool foundClearResolution = false;

        // Prefer the first candidate that clears the tactical conflict entirely.
        // If none do, keep the one that reduces severity the most.
        for (const auto& candidateInstruction : candidates) {
            const PredictedConflictAssessment assessment =
                assessWithInstruction(*candidatePlane, candidateInstruction, otherPlane, conflictDetector);
            if (!assessment.valid) {
                continue;
            }

            if (!assessment.breachesTacticalThreshold) {
                // A clear result wins immediately; there is no need to score
                // weaker candidates once the pair can be separated cleanly.
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

        // Keep the pre-resolution instruction. release can put the aircraft back on plan.
        const AircraftInstruction resumeInstruction =
            stateIt != activeStates.end()
                ? stateIt->second.resumeInstruction
                : candidatePlane->getActiveInstruction();
        storeState(candidatePlane->getCallsign(), pair, resumeInstruction, simTime, uiTime);

        return ConflictResolutionAssignment{
            candidatePlane->getCallsign(),
            pair,
            bestInstruction,
            foundClearResolution
        };
    }

    return std::nullopt;
}

void ConflictResolver::storeState(const std::string& callsign,
                                  const ConflictPair& pair,
                                  const AircraftInstruction& resumeInstruction,
                                  double simTime,
                                  double uiTime) {
    activeStates[callsign] = ConflictResolutionState{
        pair,
        resumeInstruction,
        simTime,
        uiTime
    };
}
