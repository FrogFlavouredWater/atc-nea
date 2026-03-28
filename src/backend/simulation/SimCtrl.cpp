#include "backend/simulation/Simulation.h"

#include "backend/simulation/SimUtil.h"
#include "common/logger.h"
#include "common/utils.h"
#include <algorithm>
#include <cmath>

using namespace SimulationDetail;

bool Simulation::issueInstruction(const Aircraft* plane, const AircraftInstruction& instruction) {
    Aircraft* selected = findAircraft(plane);
    if (!selected) {
        Logger::warn("Ignored instruction for aircraft that is no longer in the simulation");
        return false;
    }

    selected->applyInstruction(instruction);
    if (instruction.controlMode == AircraftControlMode::MANUAL) {
        Logger::command("Issued manual instruction to "
                        + selected->getCallsign()
                        + ": "
                        + formatAircraftInstruction(instruction));
    }
    return true;
}

bool Simulation::issueCommand(const Aircraft* plane, const AircraftCommand& command) {
    return issueInstruction(plane, AircraftInstruction{
        command.source == AircraftControlMode::ILS
            ? AircraftInstructionType::ILS_INTERCEPT
            : AircraftInstructionType::VECTOR,
        command.targetHeading,
        command.targetSpeed,
        command.targetAltitude,
        command.source,
        {},
        0.0,
        0.0,
        1
    });
}

bool Simulation::issueHoldAtCurrentPosition(const Aircraft* plane) {
    Aircraft* selected = findAircraft(plane);
    if (!selected) {
        Logger::warn("Ignored hold request for aircraft that is no longer in the simulation");
        return false;
    }

    const AircraftInstruction holdInstruction{
        AircraftInstructionType::HOLD,
        selected->getHeading(),
        selected->getSpeed(),
        selected->getAltitude(),
        AircraftControlMode::MANUAL,
        selected->getPosition(),
        kHoldLegLengthNm,
        std::max(selected->getTurnRadiusNm(), kHoldMinTurnRadiusNm),
        1
    };

    selected->clearAssignedIlsAirportIndex();
    issueInstruction(selected, holdInstruction);
    Logger::command("Issued hold-at-position to " + selected->getCallsign());
    return true;
}

bool Simulation::releaseHold(const Aircraft* plane) {
    Aircraft* selected = findAircraft(plane);
    if (!selected) {
        Logger::warn("Ignored hold release for aircraft that is no longer in the simulation");
        return false;
    }
    if (airports.empty()) {
        Logger::warn("Ignored hold release because no airport is available");
        return false;
    }
    if (selected->getInstructionType() != AircraftInstructionType::HOLD) {
        return false;
    }

    const Airport& airport = airports.front();
    const AircraftInstruction releaseInstruction{
        AircraftInstructionType::VECTOR,
        headingToward(selected->getPosition(), airport.position),
        selected->getSpeed(),
        selected->getAltitude(),
        AircraftControlMode::AUTONOMOUS,
        {},
        0.0,
        0.0,
        1
    };

    issueInstruction(selected, releaseInstruction);
    Logger::info("Released " + selected->getCallsign() + " from hold");
    return true;
}

bool Simulation::toggleApproachClearance(const Aircraft* plane) {
    Aircraft* selected = findAircraft(plane);
    if (!selected) {
        Logger::warn("Ignored approach clearance toggle for aircraft that is no longer in the simulation");
        return false;
    }

    const bool newClearanceState = !selected->hasApproachClearance();
    selected->setApproachClearance(newClearanceState);
    Logger::info(std::string("Approach clearance ")
                 + (newClearanceState ? "granted to " : "revoked for ")
                 + selected->getCallsign());

    if (!newClearanceState) {
        selected->clearAssignedIlsAirportIndex();

        if (selected->getControlMode() == AircraftControlMode::ILS) {
            AircraftCommand releaseCommand{
                selected->getHeading(),
                selected->getSpeed(),
                selected->getAltitude(),
                AircraftControlMode::AUTONOMOUS
            };
            selected->applyCommand(releaseCommand);
            selected->setPhase(FlightPhase::ARRIVAL);
            Logger::info("Released " + selected->getCallsign() + " from ILS control");
        }
    }

    return true;
}

double Simulation::estimateArrivalTimeSeconds(const Aircraft& plane, const Airport& airport) const {
    const double distanceToAirportNm = distanceNm(plane.getPosition(), airport.position);
    const double speedKts = std::max(plane.getSpeed(), 120.0);
    const double lateralTravelSeconds = distanceToAirportNm / speedKts * 3600.0;
    const double targetAltitudeFt = ilsProfileAltitudeFt(plane, airport);
    const double altitudeDifferenceFt = std::max(0.0, plane.getAltitudeExact() - targetAltitudeFt);
    const double verticalRateFpm = std::max(plane.getPerformance().descentRateFpm, 1.0);
    const double verticalDelaySeconds = altitudeDifferenceFt / verticalRateFpm * 60.0;
    return std::max(lateralTravelSeconds, verticalDelaySeconds);
}

bool Simulation::canCaptureIls(const Aircraft& plane, const Airport& airport) const {
    if (!plane.hasApproachClearance()) {
        return false;
    }
    if (!airport.inLocalizerSignal(plane.getPosition())) {
        return false;
    }

    const double alongTrackNm = alongTrackToRunwayNm(airport, plane.getPosition());
    if (alongTrackNm <= 0.5 || alongTrackNm > airport.localizer.length + 0.5) {
        return false;
    }

    const double headingDiffDeg = std::abs(getShortestAngleDiff(airport.runwayHeading, plane.getHeading()));
    if (headingDiffDeg > kIlsCaptureHeadingToleranceDeg) {
        return false;
    }

    if (plane.getSpeed() < kIlsCaptureMinSpeedKts || plane.getSpeed() > kIlsCaptureMaxSpeedKts) {
        return false;
    }

    const auto [minAltitudeFt, maxAltitudeFt] = ilsCaptureAltitudeBandFt(airport, alongTrackNm);
    return plane.getAltitudeExact() >= minAltitudeFt && plane.getAltitudeExact() <= maxAltitudeFt;
}

AircraftCommand Simulation::buildIlsCommand(const Aircraft& plane, const Airport& airport) const {
    const double crossTrackNm = crossTrackErrorNm(airport, plane.getPosition());
    const double headingCorrectionDeg = std::clamp(crossTrackNm * kIlsHeadingCorrectionPerNm,
                                                   -kIlsHeadingCorrectionMaxDeg,
                                                   kIlsHeadingCorrectionMaxDeg);
    const double alongTrackNm = std::max(0.0, alongTrackToRunwayNm(airport, plane.getPosition()));
    const double targetSpeed = std::clamp(kIlsTargetSpeedMinKts + alongTrackNm * 4.0,
                                          kIlsTargetSpeedMinKts,
                                          kIlsTargetSpeedMaxKts);

    return AircraftCommand{
        normalizeAngle(airport.runwayHeading - headingCorrectionDeg),
        targetSpeed,
        static_cast<int>(std::lround(ilsProfileAltitudeFt(plane, airport))),
        AircraftControlMode::ILS
    };
}

void Simulation::updateAutonomousCommands(double deltaTime) {
    (void)deltaTime;

    updateArrivalSequencing();
    applyArrivalSpacingControls();

    for (auto& plane : aircraft) {
        if (plane->getInstructionType() == AircraftInstructionType::CONFLICT_RESOLUTION
            || plane->getInstructionType() == AircraftInstructionType::HOLD) {
            continue;
        }

        if (!plane->hasApproachClearance()) {
            continue;
        }

        if (plane->getControlMode() == AircraftControlMode::ILS) {
            const int airportIndex = plane->getAssignedIlsAirportIndex();
            if (!isValidAirportIndex(airportIndex, airports.size())) {
                Logger::warn("Aircraft " + plane->getCallsign() + " has an invalid assigned ILS airport index");
                continue;
            }

            const Airport& airport = airports[static_cast<size_t>(airportIndex)];
            issueCommand(plane.get(), buildIlsCommand(*plane, airport));

            if (alongTrackToRunwayNm(airport, plane->getPosition()) <= kIlsLandingPhaseDistanceNm) {
                plane->setPhase(FlightPhase::LANDING);
            }
            continue;
        }

        for (size_t airportIndex = 0; airportIndex < airports.size(); ++airportIndex) {
            if (!canCaptureIls(*plane, airports[airportIndex])) {
                continue;
            }

            plane->setAssignedIlsAirportIndex(static_cast<int>(airportIndex));
            issueCommand(plane.get(), buildIlsCommand(*plane, airports[airportIndex]));
            plane->setPhase(FlightPhase::ON_FINAL);
            Logger::info("Aircraft " + plane->getCallsign() + " captured ILS for " + airports[airportIndex].name);
            break;
        }
    }
}
