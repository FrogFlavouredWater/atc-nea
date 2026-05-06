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

double computeCollisionBoxSizeNm(const SimSettings& settings) {
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

void Simulation::update(double dt, double realDt) {
    // Advance clocks first. every subsystem sees the same sim/UI elapsed time this tick.
    simTime += dt;
    uiTime += realDt;
    updateAutomation();

    for (auto& plane : aircraft) {
        plane->update(dt);
    }

    removeCollisions();
    removeDestroyedAircraft();
    removeLanded();
    removeOutOfBoundsAircraft();
    updateConflicts(uiTime);
}

void Simulation::applySettings(const SimSettings& sim) {
    settings = sim;
}

SpawnRequestResult Simulation::requestRandomSpawn() {
    lastSpawnResult = spawnService.requestRandomSpawn(settings,
                                                      airports,
                                                      aircraft,
                                                      trajectoryPredictor);

    if (lastSpawnResult.success) {
        // Simulation owns insertion. spawn service can stay pure and just return a plan.
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
    Aircraft* target = findAircraft(plane);
    if (!target) {
        Logger::warn("Ignored instruction for aircraft that is no longer in the simulation");
        return false;
    }
    if (target->isDestroyed()) {
        return false;
    }

    target->applyInstruction(instruction);
    if (instruction.controlMode == AircraftControlMode::MANUAL) {
        Logger::command("Issued manual instruction to "
                        + target->getCallsign()
                        + ": "
                        + formatAircraftInstruction(instruction));
    }
    return true;
}

bool Simulation::issueCommand(const Aircraft* plane, const AircraftCommand& command) {
    // Wrap commands back into instructions. Aircraft keeps one public apply path.
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
    Aircraft* target = findAircraft(plane);
    if (!target) {
        Logger::warn("Ignored hold request for aircraft that is no longer in the simulation");
        return false;
    }
    if (target->isDestroyed()) {
        return false;
    }

    const AircraftInstruction holdInstruction{
        AircraftInstructionType::HOLD,
        target->getHeading(),
        target->getSpeed(),
        target->getAltitude(),
        AircraftControlMode::MANUAL,
        target->getPosition(),
        SimTuning::HOLD_LEG_LENGTH_NM,
        std::max(target->getTurnRadiusNm(), SimTuning::HOLD_MIN_TURN_RADIUS_NM),
        1
    };

    target->clearAssignedIlsAirportIndex();
    issueInstruction(target, holdInstruction);
    Logger::command("Issued hold-at-position to " + target->getCallsign());
    return true;
}

bool Simulation::releaseHold(const Aircraft* plane) {
    Aircraft* target = findAircraft(plane);
    if (!target) {
        Logger::warn("Ignored hold release for aircraft that is no longer in the simulation");
        return false;
    }
    if (target->isDestroyed()) {
        return false;
    }
    if (airports.empty()) {
        Logger::warn("Ignored hold release because no airport is available");
        return false;
    }
    if (target->getInstructionType() != AircraftInstructionType::HOLD) {
        return false;
    }

    const Airport& airport = airports.front();
    const AircraftInstruction releaseInstruction{
        AircraftInstructionType::VECTOR,
        headingToward(target->getPosition(), airport.position),
        target->getSpeed(),
        target->getAltitude(),
        AircraftControlMode::AUTONOMOUS,
        {},
        0.0,
        0.0,
        1
    };

    issueInstruction(target, releaseInstruction);
    Logger::info("Released " + target->getCallsign() + " from hold");
    return true;
}

bool Simulation::toggleApproachClearance(const Aircraft* plane) {
    Aircraft* target = findAircraft(plane);
    if (!target) {
        Logger::warn("Ignored approach clearance toggle for aircraft that is no longer in the simulation");
        return false;
    }
    if (target->isDestroyed()) {
        return false;
    }

    const bool cleared = !target->hasApproachClearance();
    target->setApproachClearance(cleared);
    Logger::info(std::string("Approach clearance ")
                 + (cleared ? "granted to " : "revoked for ")
                 + target->getCallsign());

    if (!cleared) {
        // Revoke approach clearance, drop runway assignment. no new state needed if already stable.
        target->clearAssignedIlsAirportIndex();

        if (target->getControlMode() == AircraftControlMode::ILS) {
            AircraftCommand releaseCommand{
                target->getHeading(),
                target->getSpeed(),
                target->getAltitude(),
                AircraftControlMode::AUTONOMOUS
            };
            target->applyCommand(releaseCommand);
            target->setPhase(FlightPhase::ARRIVAL);
            Logger::info("Released " + target->getCallsign() + " from ILS control");
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
        if (plane->isDestroyed()) {
            plane->setConflictAlert(true);
            continue;
        }
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
    updateVisualPredictedConflicts(activePredictedConflictPairs);
}

GuidancePreview Simulation::getGuidancePreview(const Aircraft* plane) const {
    const Aircraft* target = findAircraft(plane);
    if (!target) {
        return {};
    }
    return GuidancePreviewService::build(*target);
}

bool Simulation::canSpawnMore() const {
    return aircraft.size() < static_cast<size_t>(settings.maxAircraft);
}

bool Simulation::containsAircraft(const Aircraft* plane) const {
    return findAircraft(plane) != nullptr;
}

Aircraft* Simulation::getAircraftAt(Vec2 pos, double radius) {
    for (auto& plane : aircraft) {
        if (plane->isDestroyed()) {
            continue;
        }

        const Vec2 planePosition = plane->getPosition();
        const double dx = planePosition.x - pos.x;
        const double dy = planePosition.y - pos.y;
        if (std::sqrt(dx * dx + dy * dy) < radius) {
            return plane.get();
        }
    }

    return nullptr;
}

bool Simulation::outOfBounds(const Aircraft& plane) const {
    const Vec2 position = plane.getPosition();

    return position.x < settings.minXNm
        || position.x > settings.maxXNm
        || position.y < settings.minYNm
        || position.y > settings.maxYNm;
}

Aircraft* Simulation::findByCallsign(const std::string& callsign) {
    for (auto& candidate : aircraft) {
        if (candidate->getCallsign() == callsign) {
            return candidate.get();
        }
    }
    return nullptr;
}

const Aircraft* Simulation::findByCallsign(const std::string& callsign) const {
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
    // Conservative ILS capture. clearance, geometry, heading, speed, altitude all need to line up.
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

AircraftCommand Simulation::makeIlsCommand(const Aircraft& plane, const Airport& airport) const {
    // Pull heading toward localizer. trim speed as the aircraft closes on runway.
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

void Simulation::applySpacing() {
    const auto actions = scheduler.buildSpacingActions(aircraft, airports);
    for (const auto& action : actions) {
        Aircraft* plane = findByCallsign(action.callsign);
        if (!plane) {
            continue;
        }
        issueInstruction(plane, action.instruction);
    }
}

void Simulation::updateSequencing() {
    const auto actions = scheduler.buildSequencingActions(aircraft, airports, simTime);
    for (const auto& action : actions) {
        Aircraft* plane = findByCallsign(action.callsign);
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

void Simulation::updateConflicts(double elapsedUiSeconds) {
    // Keep the conflict pass explicit so the update order is easy to follow.
    detectConflicts();
    releaseResolved();
    updateResolutions(elapsedUiSeconds);
}

void Simulation::releaseResolved() {
    const auto releases = conflictResolver.collectReleases(aircraft,
                                                           activeConflictPairs,
                                                           activePredictedConflictPairs,
                                                           simTime,
                                                           uiTime);
    for (const auto& release : releases) {
        Aircraft* plane = findByCallsign(release.callsign);
        if (!plane) {
            continue;
        }

        issueInstruction(plane, release.resumeInstruction);
        Logger::info("Released " + plane->getCallsign() + " from conflict resolution");
    }
}

void Simulation::updateResolutions(double elapsedUiSeconds) {
    const auto assignments = conflictResolver.resolve(aircraft,
                                                      airports,
                                                      activeConflictPairs,
                                                      predictedConflicts,
                                                      conflictDetector,
                                                      simTime,
                                                      elapsedUiSeconds);
    for (const auto& assignment : assignments) {
        Aircraft* plane = findByCallsign(assignment.callsign);
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

bool Simulation::reachedRunway(const Aircraft& plane, const Airport& airport) const {
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

void Simulation::updateVisualPredictedConflicts(const std::set<std::pair<std::string, std::string>>& predictedPairs) {
    // Latch predicted pairs for a short window. avoids overlay flicker near threshold.
    for (const auto& pair : predictedPairs) {
        visualPredictedConflictExpiries[pair] = uiTime + SimTuning::CONFLICT_VISUAL_LATCH_SECONDS;
    }

    visualPredictedConflictPairs.clear();
    for (auto it = visualPredictedConflictExpiries.begin(); it != visualPredictedConflictExpiries.end(); ) {
        if (it->second < uiTime) {
            it = visualPredictedConflictExpiries.erase(it);
            continue;
        }

        if (findByCallsign(it->first.first) && findByCallsign(it->first.second)) {
            visualPredictedConflictPairs.insert(it->first);
        }
        ++it;
    }
}

void Simulation::removeCollisions() {
    if (aircraft.size() < 2) {
        return;
    }

    const double collisionBoxSizeNm = computeCollisionBoxSizeNm(settings);
    std::set<std::string> collidedCallsigns;
    for (size_t i = 0; i < aircraft.size(); ++i) {
        if (aircraft[i]->isDestroyed()) {
            continue;
        }

        for (size_t j = i + 1; j < aircraft.size(); ++j) {
            if (aircraft[j]->isDestroyed()) {
                continue;
            }

            if (!aircraft[i]->collidesWith(*aircraft[j], collisionBoxSizeNm)) {
                continue;
            }

            collidedCallsigns.insert(aircraft[i]->getCallsign());
            collidedCallsigns.insert(aircraft[j]->getCallsign());
            activeCollisionPairs.insert(ConflictDetector::makeConflictPair(*aircraft[i], *aircraft[j]));
            Logger::warn("Collision detected between "
                         + aircraft[i]->getCallsign()
                         + " and "
                         + aircraft[j]->getCallsign()
                         + "; both aircraft destroyed");
        }
    }

    if (collidedCallsigns.empty()) {
        return;
    }

    // Mark collisions after the pair scan. one crash won't hide a second overlap this frame.
    for (auto& plane : aircraft) {
        if (!collidedCallsigns.contains(plane->getCallsign()) || plane->isDestroyed()) {
            continue;
        }

        plane->markDestroyed();
        destroyedAircraftRemovalTimes[plane->getCallsign()] = uiTime + SimTuning::CRASH_DISPLAY_SECONDS;
        ++hullLossCount;
    }
}

void Simulation::removeDestroyedAircraft() {
    aircraft.erase(
        std::remove_if(aircraft.begin(), aircraft.end(),
            [this](const std::unique_ptr<Aircraft>& plane) {
                if (!plane->isDestroyed()) {
                    return false;
                }

                const auto removalIt = destroyedAircraftRemovalTimes.find(plane->getCallsign());
                if (removalIt == destroyedAircraftRemovalTimes.end() || removalIt->second > uiTime) {
                    return false;
                }

                destroyedAircraftRemovalTimes.erase(removalIt);
                return true;
            }),
        aircraft.end());

    // Trim stale collision pairs after aircraft removal.
    for (auto it = activeCollisionPairs.begin(); it != activeCollisionPairs.end(); ) {
        if (!findByCallsign(it->first) || !findByCallsign(it->second)) {
            it = activeCollisionPairs.erase(it);
            continue;
        }
        ++it;
    }
}

void Simulation::removeLanded() {
    aircraft.erase(
        std::remove_if(aircraft.begin(), aircraft.end(),
            [this](const std::unique_ptr<Aircraft>& plane) {
                if (plane->isDestroyed()) {
                    return false;
                }

                const int airportIndex = plane->getAssignedIlsAirportIndex();
                if (!isValidAirportIndex(airportIndex, airports.size())) {
                    return false;
                }

                const Airport& airport = airports[static_cast<size_t>(airportIndex)];
                if (!reachedRunway(*plane, airport)) {
                    return false;
                }

                // Landing is a clean removal path. drop live traffic once touchdown criteria are met.
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
                if (plane->isDestroyed()) {
                    return false;
                }

                if (!outOfBounds(*plane)) {
                    return false;
                }

                // Out-of-bounds counts as sector exit, not failure.
                ++outOfBoundsCount;
                Logger::warn("Aircraft " + plane->getCallsign() + " left simulation bounds and was removed");
                return true;
            }),
        aircraft.end());
}

void Simulation::updateAutomation() {
    updateSequencing();
    applySpacing();

    // ILS capture and tracking run after sequencing/spacing so approach logic
    // can override autonomous arrival behaviour once capture is possible.
    for (auto& plane : aircraft) {
        if (plane->isDestroyed()) {
            continue;
        }

        // Leave holds and conflict maneuvers alone. owning system releases them.
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
            issueCommand(plane.get(), makeIlsCommand(*plane, airport));

            if (airport.alongTrackToRunway(plane->getPosition()) <= SimTuning::ILS_LANDING_PHASE_DISTANCE_NM) {
                plane->setPhase(FlightPhase::LANDING);
            }
            continue;
        }

        // First legally capturable runway wins this update.
        for (size_t airportIndex = 0; airportIndex < airports.size(); ++airportIndex) {
            if (!canCaptureIls(*plane, airports[airportIndex])) {
                continue;
            }

            plane->setAssignedIlsAirportIndex(static_cast<int>(airportIndex));
            issueCommand(plane.get(), makeIlsCommand(*plane, airports[airportIndex]));
            plane->setPhase(FlightPhase::ON_FINAL);
            Logger::info("Aircraft " + plane->getCallsign() + " captured ILS for " + airports[airportIndex].name);
            break;
        }
    }
}
