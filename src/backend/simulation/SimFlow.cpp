#include "backend/simulation/Simulation.h"

#include "backend/simulation/SimUtil.h"
#include "common/utils.h"
#include <algorithm>
#include <cmath>

using namespace SimulationDetail;

void Simulation::applyArrivalSpacingControls() {
    if (airports.empty()) {
        return;
    }

    struct ArrivalCandidate {
        Aircraft* plane = nullptr;
        size_t airportIndex = 0;
        double alongTrackNm = 0.0;
    };

    std::vector<ArrivalCandidate> arrivals;
    arrivals.reserve(aircraft.size());
    for (auto& plane : aircraft) {
        if (plane->getInstructionType() == AircraftInstructionType::CONFLICT_RESOLUTION) {
            continue;
        }
        if (!plane->hasApproachClearance()) {
            continue;
        }
        if (plane->getControlMode() == AircraftControlMode::MANUAL) {
            continue;
        }
        if (plane->getInstructionType() == AircraftInstructionType::HOLD) {
            continue;
        }

        for (size_t airportIndex = 0; airportIndex < airports.size(); ++airportIndex) {
            const Airport& airport = airports[airportIndex];
            const double alongTrackNm = alongTrackToRunwayNm(airport, plane->getPosition());
            if (alongTrackNm <= 0.0 || alongTrackNm > kArrivalSpeedDistanceNm) {
                continue;
            }

            const double crossTrackNm = std::abs(crossTrackErrorNm(airport, plane->getPosition()));
            if (crossTrackNm > kArrivalSpacingCrossTrackNm) {
                continue;
            }

            arrivals.push_back(ArrivalCandidate{
                plane.get(),
                airportIndex,
                alongTrackNm
            });
            break;
        }
    }

    std::sort(arrivals.begin(), arrivals.end(),
              [](const ArrivalCandidate& first, const ArrivalCandidate& second) {
                  if (first.airportIndex != second.airportIndex) {
                      return first.airportIndex < second.airportIndex;
                  }
                  return first.alongTrackNm < second.alongTrackNm;
              });

    for (size_t i = 1; i < arrivals.size(); ++i) {
        ArrivalCandidate& follower = arrivals[i];
        const ArrivalCandidate& leader = arrivals[i - 1];
        if (follower.airportIndex != leader.airportIndex) {
            continue;
        }

        const double spacingNm = follower.alongTrackNm - leader.alongTrackNm;
        if (spacingNm <= 0.0 || spacingNm > kArrivalSpacingDistanceNm) {
            continue;
        }

        if (std::abs(getShortestAngleDiff(follower.plane->getHeading(), leader.plane->getHeading()))
            > kArrivalSpacingHeadingToleranceDeg) {
            continue;
        }

        const double verticalDifferenceFt = follower.plane->altitudeDifferenceTo(*leader.plane);
        if (verticalDifferenceFt >= SeparationRules::VERTICAL_FT) {
            continue;
        }

        if (follower.plane->getControlMode() == AircraftControlMode::ILS) {
            continue;
        }

        const double targetSpeed = std::clamp(std::min(leader.plane->getTargetSpeed(),
                                                       leader.plane->getSpeed()) - kResolutionSpeedStepKts,
                                              kArrivalSpeedMinKts,
                                              kArrivalSpeedMaxKts);
        if (follower.plane->getTargetSpeed() <= targetSpeed + 1e-6) {
            continue;
        }

        issueInstruction(follower.plane, AircraftInstruction{
            AircraftInstructionType::VECTOR,
            follower.plane->getTargetHeading(),
            targetSpeed,
            follower.plane->getTargetAltitude(),
            AircraftControlMode::AUTONOMOUS
        });
    }
}

void Simulation::updateArrivalSequencing() {
    if (airports.empty()) {
        return;
    }

    struct ScheduledArrivalCandidate {
        Aircraft* plane = nullptr;
        double earliestArrivalTimeSeconds = 0.0;
        double distanceToAirportNm = 0.0;
    };

    std::vector<ScheduledArrivalCandidate> candidates;
    candidates.reserve(aircraft.size());
    for (auto& plane : aircraft) {
        if (!plane->hasApproachClearance()) {
            continue;
        }
        if (plane->getInstructionType() == AircraftInstructionType::CONFLICT_RESOLUTION) {
            continue;
        }
        if (plane->getControlMode() == AircraftControlMode::MANUAL
            && plane->getInstructionType() != AircraftInstructionType::HOLD) {
            continue;
        }

        const Airport& airport = airports.front();
        candidates.push_back(ScheduledArrivalCandidate{
            plane.get(),
            elapsedSimSeconds + estimateArrivalTimeSeconds(*plane, airport),
            distanceNm(plane->getPosition(), airport.position)
        });
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const ScheduledArrivalCandidate& first, const ScheduledArrivalCandidate& second) {
                  if (std::abs(first.earliestArrivalTimeSeconds - second.earliestArrivalTimeSeconds) > 1e-6) {
                      return first.earliestArrivalTimeSeconds < second.earliestArrivalTimeSeconds;
                  }
                  return first.plane->getCallsign() < second.plane->getCallsign();
              });

    double nextAvailableSlotTimeSeconds = elapsedSimSeconds;
    for (const auto& candidate : candidates) {
        const double slotTimeSeconds = std::max(candidate.earliestArrivalTimeSeconds,
                                                nextAvailableSlotTimeSeconds);
        nextAvailableSlotTimeSeconds = slotTimeSeconds + kArrivalScheduleSpacingSeconds;

        const double earlyBySeconds = slotTimeSeconds - candidate.earliestArrivalTimeSeconds;
        if (candidate.plane->getControlMode() == AircraftControlMode::ILS) {
            continue;
        }

        if (candidate.plane->getInstructionType() == AircraftInstructionType::HOLD) {
            const double postReleaseArrivalTimeSeconds =
                estimateArrivalTimeSeconds(*candidate.plane, airports.front());
            const double releaseWindowTimeSeconds =
                elapsedSimSeconds + postReleaseArrivalTimeSeconds + kArrivalReleaseLeadSeconds;
            if (releaseWindowTimeSeconds >= slotTimeSeconds) {
                releaseHold(candidate.plane);
            }
            continue;
        }

        if (earlyBySeconds < kArrivalHoldThresholdSeconds) {
            continue;
        }
        if (candidate.distanceToAirportNm > kArrivalHoldMaxRangeNm) {
            continue;
        }
        if (candidate.plane->getInstructionType() == AircraftInstructionType::ILS_INTERCEPT) {
            continue;
        }

        issueHoldAtCurrentPosition(candidate.plane);
    }
}
