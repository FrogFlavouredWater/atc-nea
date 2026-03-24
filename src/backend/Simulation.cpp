#include "backend/Simulation.h"
#include "common/logger.h"
#include "common/utils.h"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <utility>

#include "common/constants.h"

namespace {
constexpr double kSpawnLookaheadSeconds = 75.0;
constexpr double kSpawnPredictionStepSeconds = 5.0;
constexpr double kHorizontalSeparationNm = 3.0;
constexpr double kVerticalSeparationFt = 1000.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kIlsCaptureHeadingToleranceDeg = 35.0;
constexpr double kIlsCaptureMinSpeedKts = 120.0;
constexpr double kIlsCaptureMaxSpeedKts = 185.0;
constexpr double kIlsOuterCaptureMinAltitudeFt = 2500.0;
constexpr double kIlsOuterCaptureMaxAltitudeFt = 4000.0;
constexpr double kIlsInnerCaptureMinAltitudeFt = 1000.0;
constexpr double kIlsInnerCaptureMaxAltitudeFt = 2500.0;
constexpr double kIlsFinalDescentStartNm = 3.0;
constexpr double kIlsHeadingCorrectionPerNm = 12.0;
constexpr double kIlsHeadingCorrectionMaxDeg = 18.0;
constexpr double kIlsTargetSpeedMinKts = 120.0;
constexpr double kIlsTargetSpeedMaxKts = 150.0;
constexpr double kIlsLandingPhaseDistanceNm = 1.2;
constexpr double kIlsTouchdownDistanceNm = 0.45;
constexpr double kIlsTouchdownAltitudeFt = 150.0;
constexpr double kIlsTouchdownSpeedMinKts = 110.0;
constexpr double kIlsTouchdownSpeedMaxKts = 155.0;

double horizontalDistanceNm(const AircraftMotionState& first, const AircraftMotionState& second) {
    const double dx = first.position.x - second.position.x;
    const double dy = first.position.y - second.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double verticalDistanceFt(const AircraftMotionState& first, const AircraftMotionState& second) {
    return std::abs(first.altitude - second.altitude);
}

Vec2 directionVectorForHeading(double headingDeg) {
    return Vec2{
        std::cos(kDegToRad * (headingDeg - 90.0)),
        std::sin(kDegToRad * (headingDeg - 90.0))
    };
}

Vec2 rightNormalForHeading(double headingDeg) {
    return Vec2{
        std::cos(kDegToRad * headingDeg),
        std::sin(kDegToRad * headingDeg)
    };
}

double dot(Vec2 first, Vec2 second) {
    return first.x * second.x + first.y * second.y;
}

double distanceNm(Vec2 first, Vec2 second) {
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return std::sqrt(dx * dx + dy * dy);
}

double alongTrackToRunwayNm(const Airport& airport, Vec2 position) {
    const Vec2 runwayDirection = directionVectorForHeading(airport.runwayHeading);
    const Vec2 toRunway{
        airport.position.x - position.x,
        airport.position.y - position.y
    };
    return dot(toRunway, runwayDirection);
}

double crossTrackErrorNm(const Airport& airport, Vec2 position) {
    const Vec2 relativeToRunway{
        position.x - airport.position.x,
        position.y - airport.position.y
    };
    return dot(relativeToRunway, rightNormalForHeading(airport.runwayHeading));
}

double minLocalizerRangeNm(const Localizer& localizer) {
    if (localizer.sectors.empty()) {
        return 0.0;
    }

    double minRangeNm = localizer.sectors.front().range;
    for (const auto& sector : localizer.sectors) {
        minRangeNm = std::min(minRangeNm, sector.range);
    }
    return minRangeNm;
}

bool isInOuterIlsRegion(const Airport& airport, double alongTrackNm) {
    return alongTrackNm > minLocalizerRangeNm(airport.localizer);
}

std::pair<double, double> ilsCaptureAltitudeBandFt(const Airport& airport, double alongTrackNm) {
    if (isInOuterIlsRegion(airport, alongTrackNm)) {
        return {kIlsOuterCaptureMinAltitudeFt, kIlsOuterCaptureMaxAltitudeFt};
    }

    return {kIlsInnerCaptureMinAltitudeFt, kIlsInnerCaptureMaxAltitudeFt};
}

double interpolateLinear(double input,
                         double inputStart,
                         double inputEnd,
                         double outputStart,
                         double outputEnd) {
    if (std::abs(inputEnd - inputStart) < 1e-6) {
        return outputEnd;
    }

    const double t = std::clamp((input - inputStart) / (inputEnd - inputStart), 0.0, 1.0);
    return outputStart + (outputEnd - outputStart) * t;
}

double ilsProfileAltitudeFt(const Aircraft& plane, const Airport& airport) {
    const double alongTrackNm = std::clamp(alongTrackToRunwayNm(airport, plane.getPosition()),
                                           0.0,
                                           airport.localizer.length);
    const double innerRegionBoundaryNm = minLocalizerRangeNm(airport.localizer);
    double desiredAltitudeFt = 0.0;

    if (alongTrackNm > innerRegionBoundaryNm) {
        desiredAltitudeFt = interpolateLinear(alongTrackNm,
                                              innerRegionBoundaryNm,
                                              airport.localizer.length,
                                              kIlsOuterCaptureMinAltitudeFt,
                                              kIlsOuterCaptureMaxAltitudeFt);
    } else if (alongTrackNm > kIlsFinalDescentStartNm) {
        desiredAltitudeFt = interpolateLinear(alongTrackNm,
                                              kIlsFinalDescentStartNm,
                                              innerRegionBoundaryNm,
                                              kIlsInnerCaptureMinAltitudeFt,
                                              kIlsInnerCaptureMaxAltitudeFt);
    } else {
        desiredAltitudeFt = interpolateLinear(alongTrackNm,
                                              0.0,
                                              kIlsFinalDescentStartNm,
                                              0.0,
                                              kIlsInnerCaptureMinAltitudeFt);
    }

    return std::min(desiredAltitudeFt, plane.getAltitudeExact());
}

bool isValidAirportIndex(int airportIndex, size_t airportCount) {
    return airportIndex >= 0 && static_cast<size_t>(airportIndex) < airportCount;
}

std::string formatSpawnCandidate(const SpawnCandidate& candidate) {
    std::ostringstream stream;
    stream << candidate.callsign
           << " from " << candidate.entryLabel
           << " hdg " << static_cast<int>(std::lround(candidate.headingDeg))
           << " spd " << static_cast<int>(std::lround(candidate.speedKts))
           << " alt " << candidate.altitudeFt;
    return stream.str();
}

std::string formatAircraftCommand(const AircraftCommand& command) {
    std::ostringstream stream;
    stream << "hdg " << static_cast<int>(std::lround(command.targetHeading))
           << " spd " << static_cast<int>(std::lround(command.targetSpeed))
           << " alt " << command.targetAltitude;
    return stream.str();
}

std::pair<std::string, std::string> makeConflictPair(const Aircraft& first, const Aircraft& second) {
    if (first.getCallsign() < second.getCallsign()) {
        return {first.getCallsign(), second.getCallsign()};
    }
    return {second.getCallsign(), first.getCallsign()};
}
}

Simulation::Simulation() {}

void Simulation::update(double deltaTime) {
    updateAutonomousCommands(deltaTime);

    for (auto& plane : aircraft) {
        plane->update(deltaTime);
    }
    removeLandedAircraft();
    removeOutOfBoundsAircraft();
    detectConflicts();
}

void Simulation::applySettings(const SimSettings& newSettings) {
    settings = newSettings;
}

bool Simulation::spawnAircraft(Vec2 pos, double heading, double speed, int altitude, const std::string& callsign) {
    if (!canSpawnMore()) {
        return false;
    }
    aircraft.push_back(std::make_unique<Aircraft>(pos, heading, speed, altitude, callsign));
    return true;
}

SpawnRequestResult Simulation::requestRandomSpawn() {
    if (!canSpawnMore()) {
        lastSpawnResult = SpawnRequestResult{
            false,
            SpawnRejectionReason::CAPACITY_REACHED,
            "Spawn blocked: max aircraft reached"
        };
        Logger::warn(lastSpawnResult.message);
        return lastSpawnResult;
    }

    const std::vector<SpawnCandidate> candidates = spawnService.createCandidates(settings, airports);
    if (candidates.empty()) {
        lastSpawnResult = SpawnRequestResult{
            false,
            SpawnRejectionReason::NO_VALID_ENTRY_POINT,
            "Spawn blocked: no valid entry points"
        };
        Logger::warn(lastSpawnResult.message);
        return lastSpawnResult;
    }

    for (const auto& candidate : candidates) {
        if (!isSpawnSafe(candidate)) {
            continue;
        }

        if (!spawnAircraft(candidate.position,
                           candidate.headingDeg,
                           candidate.speedKts,
                           candidate.altitudeFt,
                           candidate.callsign)) {
            lastSpawnResult = SpawnRequestResult{
                false,
                SpawnRejectionReason::CAPACITY_REACHED,
                "Spawn blocked: max aircraft reached"
            };
            Logger::warn(lastSpawnResult.message);
            return lastSpawnResult;
        }

        lastSpawnResult = SpawnRequestResult{
            true,
            SpawnRejectionReason::NONE,
            "Spawned " + candidate.callsign + " from " + candidate.entryLabel,
            candidate.callsign,
            candidate.entryLabel,
            aircraft.back().get()
        };
        Logger::info("Spawned aircraft " + formatSpawnCandidate(candidate));
        return lastSpawnResult;
    }

    lastSpawnResult = SpawnRequestResult{
        false,
        SpawnRejectionReason::UNSAFE_SPAWN,
        "Spawn blocked: unsafe entry"
    };
    Logger::warn(lastSpawnResult.message);
    return lastSpawnResult;
}

void Simulation::clearLastSpawnResult() {
    lastSpawnResult = {};
}

bool Simulation::issueCommand(const Aircraft* plane, const AircraftCommand& command) {
    if (!containsAircraft(plane)) {
        Logger::warn("Ignored command for aircraft that is no longer in the simulation");
        return false;
    }

    for (auto& candidate : aircraft) {
        if (candidate.get() == plane) {
            candidate->applyCommand(command);
            if (command.source == AircraftControlMode::MANUAL) {
                Logger::command("Issued manual command to "
                                + candidate->getCallsign()
                                + ": "
                                + formatAircraftCommand(command));
            }
            return true;
        }
    }

    return false;
}

bool Simulation::toggleApproachClearance(const Aircraft* plane) {
    if (!containsAircraft(plane)) {
        Logger::warn("Ignored approach clearance toggle for aircraft that is no longer in the simulation");
        return false;
    }

    for (auto& candidate : aircraft) {
        if (candidate.get() != plane) {
            continue;
        }

        const bool newClearanceState = !candidate->hasApproachClearance();
        candidate->setApproachClearance(newClearanceState);
        Logger::info(std::string("Approach clearance ")
                     + (newClearanceState ? "granted to " : "revoked for ")
                     + candidate->getCallsign());

        if (!newClearanceState) {
            candidate->clearAssignedIlsAirportIndex();

            if (candidate->getControlMode() == AircraftControlMode::ILS) {
                AircraftCommand releaseCommand{
                    candidate->getHeading(),
                    candidate->getSpeed(),
                    candidate->getAltitude(),
                    AircraftControlMode::AUTONOMOUS
                };
                candidate->applyCommand(releaseCommand);
                candidate->setPhase(FlightPhase::ARRIVAL);
                Logger::info("Released " + candidate->getCallsign() + " from ILS control");
            }
        }

        return true;
    }

    return false;
}

void Simulation::addAirport(const Airport& airport) {
    airports.push_back(airport);
    Logger::info("Added airport " + airport.name);
}

bool Simulation::canSpawnMore() const {
    return aircraft.size() < static_cast<size_t>(settings.maxAircraft);
}

bool Simulation::containsAircraft(const Aircraft* plane) const {
    return std::any_of(aircraft.begin(), aircraft.end(),
        [plane](const auto& candidate) {
            return candidate.get() == plane;
        });
}

void Simulation::detectConflicts() {
    for (auto& plane : aircraft) {
        plane->setConflictAlert(false);
    }

    std::set<std::pair<std::string, std::string>> currentConflictPairs;
    for (size_t i = 0; i < aircraft.size(); i++) {
        for (size_t j = i + 1; j < aircraft.size(); j++) {
            if (aircraft[i]->breachesSeparationWith(*aircraft[j])) {
                aircraft[i]->setConflictAlert(true);
                aircraft[j]->setConflictAlert(true);
                currentConflictPairs.insert(makeConflictPair(*aircraft[i], *aircraft[j]));
            }
        }
    }

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

    activeConflictPairs = std::move(currentConflictPairs);
}

GuidancePreview Simulation::getGuidancePreview(const Aircraft* plane) const {
    if (!containsAircraft(plane)) {
        return {};
    }

    for (const auto& candidate : aircraft) {
        if (candidate.get() == plane) {
            return GuidancePreviewService::build(*candidate);
        }
    }

    return {};
}

std::vector<PredictedAircraftState> Simulation::predictTrajectory(const Aircraft* plane,
                                                                  double horizonSeconds,
                                                                  double stepSeconds) const {
    if (!containsAircraft(plane)) {
        return {};
    }

    for (const auto& candidate : aircraft) {
        if (candidate.get() == plane) {
            return TrajectoryPredictor::predict(*candidate, horizonSeconds, stepSeconds);
        }
    }

    return {};
}

void Simulation::updateAutonomousCommands(double deltaTime) {
    (void)deltaTime;

    for (auto& plane : aircraft) {
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

bool Simulation::isOutOfBounds(const Aircraft& plane) const {
    const Vec2 position = plane.getPosition();

    return position.x < settings.minXNm
        || position.x > settings.maxXNm
        || position.y < settings.minYNm
        || position.y > settings.maxYNm;
}

bool Simulation::isSpawnSafe(const SpawnCandidate& candidate) const {
    Aircraft spawnedAircraft(candidate.position,
                             candidate.headingDeg,
                             candidate.speedKts,
                             candidate.altitudeFt,
                             candidate.callsign);

    for (const auto& plane : aircraft) {
        if (spawnedAircraft.breachesSeparationWith(*plane)) {
            return false;
        }
    }

    const auto spawnedPrediction = TrajectoryPredictor::predict(spawnedAircraft,
                                                                kSpawnLookaheadSeconds,
                                                                kSpawnPredictionStepSeconds);
    for (const auto& plane : aircraft) {
        const auto existingPrediction = TrajectoryPredictor::predict(*plane,
                                                                     kSpawnLookaheadSeconds,
                                                                     kSpawnPredictionStepSeconds);
        if (predictionBreachesSeparation(spawnedPrediction, existingPrediction)) {
            return false;
        }
    }

    return true;
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

bool Simulation::hasReachedRunway(const Aircraft& plane, const Airport& airport) const {
    const double touchdownRadiusNm = std::max(kIlsTouchdownDistanceNm, airport.runwayLength * 0.35);
    if (distanceNm(plane.getPosition(), airport.position) > touchdownRadiusNm) {
        return false;
    }

    if (plane.getAltitudeExact() > kIlsTouchdownAltitudeFt) {
        return false;
    }

    if (plane.getSpeed() < kIlsTouchdownSpeedMinKts || plane.getSpeed() > kIlsTouchdownSpeedMaxKts) {
        return false;
    }

    return std::abs(getShortestAngleDiff(airport.runwayHeading, plane.getHeading())) <= kIlsCaptureHeadingToleranceDeg;
}

bool Simulation::predictionBreachesSeparation(const std::vector<PredictedAircraftState>& firstPrediction,
                                              const std::vector<PredictedAircraftState>& secondPrediction) const {
    const size_t sampleCount = std::min(firstPrediction.size(), secondPrediction.size());
    for (size_t i = 0; i < sampleCount; ++i) {
        if (horizontalDistanceNm(firstPrediction[i].motion, secondPrediction[i].motion) < kHorizontalSeparationNm
            && verticalDistanceFt(firstPrediction[i].motion, secondPrediction[i].motion) < kVerticalSeparationFt) {
            return true;
        }
    }

    return false;
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

Aircraft* Simulation::getAircraftAt(Vec2 pos, double radius) {
    for (auto& plane : aircraft) {
        Vec2 pPos = plane->getPosition();
        double dx = pPos.x - pos.x;
        double dy = pPos.y - pos.y;
        if (std::sqrt(dx*dx + dy*dy) < radius) {
            return plane.get();
        }
    }
    return nullptr;
}
