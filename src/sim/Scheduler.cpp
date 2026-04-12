#include "sim/Scheduler.h"

#include "core/Config.h"
#include "core/MathUtils.h"

#include <algorithm>
#include <cmath>

std::vector<SchedulerAction> Scheduler::buildSequencingActions(
    const std::vector<std::unique_ptr<Aircraft>>& aircraft,
    const std::vector<Airport>& airports,
    double simTime) const {
    std::vector<SchedulerAction> actions;
    if (airports.empty()) {
        return actions;
    }

    // Sequence cleared arrivals into simple runway slots based on their
    // earliest likely arrival time at the active airport.
    struct ScheduledArrivalCandidate {
        Aircraft* plane = nullptr;
        double earliestArrivalTimeSeconds = 0.0;
        double distanceToAirportNm = 0.0;
    };

    std::vector<ScheduledArrivalCandidate> candidates;
    candidates.reserve(aircraft.size());
    for (const auto& plane : aircraft) {
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
            simTime + estimateArrivalTime(*plane, airport),
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

    double nextSlotTime = simTime;
    for (const auto& candidate : candidates) {
        const double slotTime = std::max(candidate.earliestArrivalTimeSeconds, nextSlotTime);
        nextSlotTime = slotTime + SimTuning::ARRIVAL_SCHEDULE_SPACING_SECONDS;

        const double earlyBy = slotTime - candidate.earliestArrivalTimeSeconds;
        if (candidate.plane->getControlMode() == AircraftControlMode::ILS) {
            continue;
        }

        if (candidate.plane->getInstructionType() == AircraftInstructionType::HOLD) {
            const double releaseArrival = estimateArrivalTime(*candidate.plane, airports.front());
            const double releaseTime = simTime + releaseArrival + SimTuning::ARRIVAL_RELEASE_LEAD_SECONDS;
            if (releaseTime >= slotTime) {
                actions.push_back(SchedulerAction{
                    SchedulerActionType::RELEASE_HOLD,
                    candidate.plane->getCallsign()
                });
            }
            continue;
        }

        if (earlyBy < SimTuning::ARRIVAL_HOLD_THRESHOLD_SECONDS) {
            continue;
        }
        if (candidate.distanceToAirportNm > SimTuning::ARRIVAL_HOLD_MAX_RANGE_NM) {
            continue;
        }
        if (candidate.plane->getInstructionType() == AircraftInstructionType::ILS_INTERCEPT) {
            continue;
        }

        actions.push_back(SchedulerAction{
            SchedulerActionType::ISSUE_HOLD,
            candidate.plane->getCallsign()
        });
    }

    return actions;
}

std::vector<SchedulerAction> Scheduler::buildSpacingActions(
    const std::vector<std::unique_ptr<Aircraft>>& aircraft,
    const std::vector<Airport>& airports) const {
    std::vector<SchedulerAction> actions;
    if (airports.empty()) {
        return actions;
    }

    // Spacing only considers aircraft that are already roughly lined up with
    // the same runway environment.
    struct ArrivalCandidate {
        Aircraft* plane = nullptr;
        size_t airportIndex = 0;
        double alongTrackNm = 0.0;
    };

    std::vector<ArrivalCandidate> arrivals;
    arrivals.reserve(aircraft.size());
    for (const auto& plane : aircraft) {
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
            const double alongTrackNm = airport.alongTrackToRunway(plane->getPosition());
            if (alongTrackNm <= 0.0 || alongTrackNm > SimTuning::ARRIVAL_SPEED_DISTANCE_NM) {
                continue;
            }

            const double crossTrackNm = std::abs(airport.crossTrackError(plane->getPosition()));
            if (crossTrackNm > SimTuning::ARRIVAL_SPACING_CROSS_TRACK_NM) {
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
        if (spacingNm <= 0.0 || spacingNm > SimTuning::ARRIVAL_SPACING_DISTANCE_NM) {
            continue;
        }

        if (std::abs(getShortestAngleDiff(follower.plane->getHeading(), leader.plane->getHeading()))
            > SimTuning::ARRIVAL_SPACING_HEADING_TOLERANCE_DEG) {
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
                                                       leader.plane->getSpeed()) - SimTuning::RESOLUTION_SPEED_STEP_KTS,
                                              SimTuning::ARRIVAL_SPEED_MIN_KTS,
                                              SimTuning::ARRIVAL_SPEED_MAX_KTS);
        if (follower.plane->getTargetSpeed() <= targetSpeed + 1e-6) {
            continue;
        }

        actions.push_back(SchedulerAction{
            SchedulerActionType::ISSUE_INSTRUCTION,
            follower.plane->getCallsign(),
            AircraftInstruction{
                AircraftInstructionType::VECTOR,
                follower.plane->getTargetHeading(),
                targetSpeed,
                follower.plane->getTargetAltitude(),
                AircraftControlMode::AUTONOMOUS
            }
        });
    }

    return actions;
}

double Scheduler::estimateArrivalTime(const Aircraft& plane, const Airport& airport) const {
    // Arrival time is a rough max of lateral travel time and descent time.
    const double distanceToAirportNm = distanceNm(plane.getPosition(), airport.position);
    const double speedKts = std::max(plane.getSpeed(), 120.0);
    const double lateralTravelSeconds = distanceToAirportNm / speedKts * 3600.0;
    const double targetAltitudeFt = airport.ilsProfileAltitudeFt(plane.getPosition(), plane.getAltitudeExact());
    const double altitudeDifferenceFt = std::max(0.0, plane.getAltitudeExact() - targetAltitudeFt);
    const double verticalRateFpm = std::max(plane.getPerformance().descentRateFpm, 1.0);
    const double verticalDelaySeconds = altitudeDifferenceFt / verticalRateFpm * 60.0;
    return std::max(lateralTravelSeconds, verticalDelaySeconds);
}
