#include "backend/simulation/Simulation.h"

#include "backend/simulation/SimUtil.h"
#include "common/logger.h"
#include <algorithm>
#include <cmath>

using namespace SimulationDetail;

Simulation::Simulation() {}

void Simulation::update(double deltaTime) {
    elapsedSimSeconds += deltaTime;
    updateAutonomousCommands(deltaTime);

    for (auto& plane : aircraft) {
        plane->update(deltaTime);
    }

    removeCollidedAircraft();
    removeLandedAircraft();
    removeOutOfBoundsAircraft();
    detectConflicts();
    releaseResolvedAircraft();
    updateConflictResolutions();
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

void Simulation::addAirport(const Airport& airport) {
    airports.push_back(airport);
    Logger::info("Added airport " + airport.name);
}

bool Simulation::canSpawnMore() const {
    return aircraft.size() < static_cast<size_t>(settings.maxAircraft);
}

bool Simulation::containsAircraft(const Aircraft* plane) const {
    return findAircraft(plane) != nullptr;
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

GuidancePreview Simulation::getGuidancePreview(const Aircraft* plane) const {
    const Aircraft* selected = findAircraft(plane);
    if (!selected) {
        return {};
    }
    return GuidancePreviewService::build(*selected);
}

std::vector<PredictedAircraftState> Simulation::predictTrajectory(const Aircraft* plane,
                                                                  double horizonSeconds,
                                                                  double stepSeconds) const {
    const Aircraft* selected = findAircraft(plane);
    if (!selected) {
        return {};
    }
    return TrajectoryPredictor::predict(*selected, horizonSeconds, stepSeconds);
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
