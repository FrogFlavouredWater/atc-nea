#include "backend/Simulation.h"
#include "common/logger.h"
#include "common/utils.h"
#include <cmath>
#include <algorithm>

#include "common/constants.h"

Simulation::Simulation() {}

void Simulation::update(double deltaTime) {
    updateAutonomousCommands(deltaTime);

    for (auto& plane : aircraft) {
        plane->update(deltaTime);
    }
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

bool Simulation::issueCommand(const Aircraft* plane, const AircraftCommand& command) {
    if (!containsAircraft(plane)) {
        return false;
    }

    for (auto& candidate : aircraft) {
        if (candidate.get() == plane) {
            candidate->applyCommand(command);
            return true;
        }
    }

    return false;
}

void Simulation::addAirport(const Airport& airport) {
    airports.push_back(airport);
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

    for (size_t i = 0; i < aircraft.size(); i++) {
        for (size_t j = i + 1; j < aircraft.size(); j++) {
            if (aircraft[i]->collidesWith(*aircraft[j])) {
                aircraft[i]->setConflictAlert(true);
                aircraft[j]->setConflictAlert(true);
            }
        }
    }
}

void Simulation::updateAutonomousCommands(double deltaTime) {
    (void)deltaTime;

    // Placeholder for the future autonomous planner.
    // Manual input now goes through issueCommand(), and autonomous control will
    // use the same entrypoint in a later push.
}

bool Simulation::isOutOfBounds(const Aircraft& plane) const {
    const Vec2 position = plane.getPosition();

    return position.x < settings.minXNm
        || position.x > settings.maxXNm
        || position.y < settings.minYNm
        || position.y > settings.maxYNm;
}

void Simulation::removeOutOfBoundsAircraft() {
    aircraft.erase(
        std::remove_if(aircraft.begin(), aircraft.end(),
            [this](const std::unique_ptr<Aircraft>& plane) {
                if (!isOutOfBounds(*plane)) {
                    return false;
                }

                ++outOfBoundsCount;
                Logger::debug("Aircraft " + plane->getCallsign() + " left simulation bounds and was removed");
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
