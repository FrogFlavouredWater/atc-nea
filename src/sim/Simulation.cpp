#include "sim/Simulation.h"

#include "core/Logger.h"
#include "core/MathUtils.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace {
bool isValidAirportIndex(int airportIndex, size_t airportCount) {
    return airportIndex >= 0 && static_cast<size_t>(airportIndex) < airportCount;
}

double collisionSquareSideNm(const SimSettings& settings) {
    return settings.pixelsPerNm > 0.0
        ? static_cast<double>(settings.aircraftSize) / settings.pixelsPerNm
        : 0.0;
}

std::string formatAircraftInstruction(const AircraftInstruction& instruction) {
    std::ostringstream stream;
    stream << toString(instruction.type)
           << " hdg " << static_cast<int>(std::lround(instruction.targetHeading))
           << " spd " << static_cast<int>(std::lround(instruction.targetSpeed))
           << " alt " << instruction.targetAltitude;
    return stream.str();
}
}

Simulation::Simulation() = default;

void Simulation::update(double deltaTime) {
    elapsedSimSeconds += deltaTime;
    // Automation feeds commands first, then aircraft move, then cleanup and
    // conflict handling operate on the updated positions for this tick.
    updateAutonomousCommands();

    for (auto& plane : aircraft) {
        plane->update(deltaTime);
    }

    removeCollidedAircraft();
    removeLandedAircraft();
    removeOutOfBoundsAircraft();
    runConflictCycle();
}

void Simulation::applySettings(const SimSettings& newSettings) {
    settings = newSettings;
}

SpawnRequestResult Simulation::requestRandomSpawn() {
    lastSpawnResult = spawnService.requestRandomSpawn(settings,
                                                      airports,
                                                      aircraft,
                                                      trajectoryPredictor);

    if (lastSpawnResult.success) {
        const SpawnPlan& spawn = lastSpawnResult.spawn;
        aircraft.push_back(std::make_unique<Aircraft>(spawn.position,
                                                      spawn.headingDeg,
                                                      spawn.speedKts,
                                                      spawn.altitudeFt,
                                                      spawn.callsign));
        lastSpawnResult.aircraft = aircraft.back().get();
        Logger::info(lastSpawnResult.message);
    } else if (!lastSpawnResult.message.empty()) {
        Logger::warn(lastSpawnResult.message);
    }
    return lastSpawnResult;
}

void Simulation::clearLastSpawnResult() {
    lastSpawnResult = {};
}

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
        SimTuning::HOLD_LEG_LENGTH_NM,
        std::max(selected->getTurnRadiusNm(), SimTuning::HOLD_MIN_TURN_RADIUS_NM),
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

void Simulation::addAirport(const Airport& airport) {
    airports.push_back(airport);
    Logger::info("Added airport " + airport.name);
}

void Simulation::detectConflicts() {
    const ConflictDetectionReport report = conflictDetector.detect(aircraft);

    // The detector only reports results; Simulation applies alert flags and
    // manages lifecycle logging around changes in conflict state.
    for (auto& plane : aircraft) {
        plane->setConflictAlert(report.aircraftWithConflictAlert.contains(plane->getCallsign()));
    }

    auto currentConflictPairs = report.currentConflictPairs;
    auto currentPredictedConflictPairs = report.predictedConflictPairs;
    predictedConflicts = report.predictedConflicts;

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
        const auto pair = ConflictDetector::makeConflictPair(assessment.firstCallsign, assessment.secondCallsign);
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

GuidancePreview Simulation::getGuidancePreview(const Aircraft* plane) const {
    const Aircraft* selected = findAircraft(plane);
    if (!selected) {
        return {};
    }
    return GuidancePreviewService::build(*selected);
}

bool Simulation::canSpawnMore() const {
    return aircraft.size() < static_cast<size_t>(settings.maxAircraft);
}

bool Simulation::containsAircraft(const Aircraft* plane) const {
    return findAircraft(plane) != nullptr;
}

Aircraft* Simulation::getAircraftAt(Vec2 pos, double radius) {
    for (auto& plane : aircraft) {
        const Vec2 planePosition = plane->getPosition();
        const double dx = planePosition.x - pos.x;
        const double dy = planePosition.y - pos.y;
        if (std::sqrt(dx * dx + dy * dy) < radius) {
            return plane.get();
        }
    }

    return nullptr;
}

bool Simulation::isOutOfBounds(const Aircraft& plane) const {
    const Vec2 position = plane.getPosition();

    return position.x < settings.minXNm
        || position.x > settings.maxXNm
        || position.y < settings.minYNm
        || position.y > settings.maxYNm;
}

Aircraft* Simulation::findAircraftByCallsign(const std::string& callsign) {
    for (auto& candidate : aircraft) {
        if (candidate->getCallsign() == callsign) {
            return candidate.get();
        }
    }
    return nullptr;
}

const Aircraft* Simulation::findAircraftByCallsign(const std::string& callsign) const {
    for (const auto& candidate : aircraft) {
        if (candidate->getCallsign() == callsign) {
            return candidate.get();
        }
    }
    return nullptr;
}

Aircraft* Simulation::findAircraft(const Aircraft* plane) {
    for (auto& candidate : aircraft) {
        if (candidate.get() == plane) {
            return candidate.get();
        }
    }
    return nullptr;
}

const Aircraft* Simulation::findAircraft(const Aircraft* plane) const {
    for (const auto& candidate : aircraft) {
        if (candidate.get() == plane) {
            return candidate.get();
        }
    }
    return nullptr;
}

bool Simulation::canCaptureIls(const Aircraft& plane, const Airport& airport) const {
    if (!plane.hasApproachClearance()) {
        return false;
    }
    if (!airport.inLocalizerSignal(plane.getPosition())) {
        return false;
    }

    const double alongTrackNm = airport.alongTrackToRunway(plane.getPosition());
    if (alongTrackNm <= 0.5 || alongTrackNm > airport.localizer.length + 0.5) {
        return false;
    }

    const double headingDiffDeg = std::abs(getShortestAngleDiff(airport.runwayHeading, plane.getHeading()));
    if (headingDiffDeg > SimTuning::ILS_CAPTURE_HEADING_TOLERANCE_DEG) {
        return false;
    }

    if (plane.getSpeed() < SimTuning::ILS_CAPTURE_MIN_SPEED_KTS
        || plane.getSpeed() > SimTuning::ILS_CAPTURE_MAX_SPEED_KTS) {
        return false;
    }

    const auto [minAltitudeFt, maxAltitudeFt] = airport.ilsCaptureAltitudeBandFt(alongTrackNm);
    return plane.getAltitudeExact() >= minAltitudeFt && plane.getAltitudeExact() <= maxAltitudeFt;
}

AircraftCommand Simulation::buildIlsCommand(const Aircraft& plane, const Airport& airport) const {
    const double crossTrackNm = airport.crossTrackError(plane.getPosition());
    const double headingCorrectionDeg = std::clamp(crossTrackNm * SimTuning::ILS_HEADING_CORRECTION_PER_NM,
                                                   -SimTuning::ILS_HEADING_CORRECTION_MAX_DEG,
                                                   SimTuning::ILS_HEADING_CORRECTION_MAX_DEG);
    const double alongTrackNm = std::max(0.0, airport.alongTrackToRunway(plane.getPosition()));
    const double targetSpeed = std::clamp(SimTuning::ILS_TARGET_SPEED_MIN_KTS + alongTrackNm * 4.0,
                                          SimTuning::ILS_TARGET_SPEED_MIN_KTS,
                                          SimTuning::ILS_TARGET_SPEED_MAX_KTS);

    return AircraftCommand{
        normalizeAngle(airport.runwayHeading - headingCorrectionDeg),
        targetSpeed,
        static_cast<int>(std::lround(airport.ilsProfileAltitudeFt(plane.getPosition(), plane.getAltitudeExact()))),
        AircraftControlMode::ILS
    };
}

void Simulation::applyArrivalSpacingControls() {
    const auto actions = scheduler.buildSpacingActions(aircraft, airports);
    for (const auto& action : actions) {
        Aircraft* plane = findAircraftByCallsign(action.callsign);
        if (!plane) {
            continue;
        }
        issueInstruction(plane, action.instruction);
    }
}

void Simulation::updateArrivalSequencing() {
    const auto actions = scheduler.buildSequencingActions(aircraft, airports, elapsedSimSeconds);
    for (const auto& action : actions) {
        Aircraft* plane = findAircraftByCallsign(action.callsign);
        if (!plane) {
            continue;
        }

        if (action.type == SchedulerActionType::RELEASE_HOLD) {
            releaseHold(plane);
        } else if (action.type == SchedulerActionType::ISSUE_HOLD) {
            issueHoldAtCurrentPosition(plane);
        } else {
            issueInstruction(plane, action.instruction);
        }
    }
}

void Simulation::runConflictCycle() {
    // Keep the conflict pass explicit so the update order is easy to follow.
    detectConflicts();
    releaseResolvedAircraft();
    updateConflictResolutions();
}

void Simulation::releaseResolvedAircraft() {
    const auto releases = conflictResolver.collectReleases(aircraft,
                                                           activeConflictPairs,
                                                           activePredictedConflictPairs,
                                                           elapsedSimSeconds);
    for (const auto& release : releases) {
        Aircraft* plane = findAircraftByCallsign(release.callsign);
        if (!plane) {
            continue;
        }

        issueInstruction(plane, release.resumeInstruction);
        Logger::info("Released " + plane->getCallsign() + " from conflict resolution");
    }
}

void Simulation::updateConflictResolutions() {
    const auto assignments = conflictResolver.resolve(aircraft,
                                                      airports,
                                                      activeConflictPairs,
                                                      predictedConflicts,
                                                      conflictDetector,
                                                      elapsedSimSeconds);
    for (const auto& assignment : assignments) {
        Aircraft* plane = findAircraftByCallsign(assignment.callsign);
        if (!plane) {
            continue;
        }

        issueInstruction(plane, assignment.instruction);
        Logger::warn(std::string("Conflict resolution assigned to ")
                     + plane->getCallsign()
                     + " for pair "
                     + assignment.conflictPair.first
                     + "/"
                     + assignment.conflictPair.second
                     + (assignment.clearsConflict ? " (clearing maneuver)" : " (mitigation maneuver)"));
    }
}

bool Simulation::hasReachedRunway(const Aircraft& plane, const Airport& airport) const {
    const double touchdownRadiusNm = std::max(SimTuning::ILS_TOUCHDOWN_DISTANCE_NM, airport.runwayLength * 0.35);
    if (distanceNm(plane.getPosition(), airport.position) > touchdownRadiusNm) {
        return false;
    }

    if (plane.getAltitudeExact() > SimTuning::ILS_TOUCHDOWN_ALTITUDE_FT) {
        return false;
    }

    if (plane.getSpeed() < SimTuning::ILS_TOUCHDOWN_SPEED_MIN_KTS
        || plane.getSpeed() > SimTuning::ILS_TOUCHDOWN_SPEED_MAX_KTS) {
        return false;
    }

    return std::abs(getShortestAngleDiff(airport.runwayHeading, plane.getHeading()))
        <= SimTuning::ILS_CAPTURE_HEADING_TOLERANCE_DEG;
}

void Simulation::removeCollidedAircraft() {
    if (aircraft.size() < 2) {
        return;
    }

    const double squareSideNm = collisionSquareSideNm(settings);
    std::set<size_t> collidedIndices;
    for (size_t i = 0; i < aircraft.size(); ++i) {
        for (size_t j = i + 1; j < aircraft.size(); ++j) {
            if (!aircraft[i]->collidesWith(*aircraft[j], squareSideNm)) {
                continue;
            }

            collidedIndices.insert(i);
            collidedIndices.insert(j);
            Logger::warn("Collision detected between "
                         + aircraft[i]->getCallsign()
                         + " and "
                         + aircraft[j]->getCallsign()
                         + "; both aircraft destroyed");
        }
    }

    if (collidedIndices.empty()) {
        return;
    }

    hullLossCount += collidedIndices.size();
    size_t index = 0;
    aircraft.erase(
        std::remove_if(aircraft.begin(), aircraft.end(),
            [&collidedIndices, &index](const std::unique_ptr<Aircraft>&) mutable {
                const bool shouldRemove = collidedIndices.contains(index);
                ++index;
                return shouldRemove;
            }),
        aircraft.end());
}

void Simulation::removeLandedAircraft() {
    aircraft.erase(
        std::remove_if(aircraft.begin(), aircraft.end(),
            [this](const std::unique_ptr<Aircraft>& plane) {
                const int airportIndex = plane->getAssignedIlsAirportIndex();
                if (!isValidAirportIndex(airportIndex, airports.size())) {
                    return false;
                }

                const Airport& airport = airports[static_cast<size_t>(airportIndex)];
                if (!hasReachedRunway(*plane, airport)) {
                    return false;
                }

                ++landedCount;
                Logger::success("Aircraft " + plane->getCallsign() + " landed on " + airport.name);
                return true;
            }),
        aircraft.end());
}

void Simulation::removeOutOfBoundsAircraft() {
    aircraft.erase(
        std::remove_if(aircraft.begin(), aircraft.end(),
            [this](const std::unique_ptr<Aircraft>& plane) {
                if (!isOutOfBounds(*plane)) {
                    return false;
                }

                ++outOfBoundsCount;
                Logger::warn("Aircraft " + plane->getCallsign() + " left simulation bounds and was removed");
                return true;
            }),
        aircraft.end());
}

void Simulation::updateAutonomousCommands() {
    updateArrivalSequencing();
    applyArrivalSpacingControls();

    // ILS capture and tracking run after sequencing/spacing so approach logic
    // can override autonomous arrival behaviour once capture is possible.
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

            if (airport.alongTrackToRunway(plane->getPosition()) <= SimTuning::ILS_LANDING_PHASE_DISTANCE_NM) {
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
