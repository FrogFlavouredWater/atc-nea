#include "backend/Simulation.h"
#include "common/utils.h"
#include <cmath>
#include <algorithm>

Simulation::Simulation() {}

void Simulation::update(double deltaTime) {
    for (auto& plane : aircraft) {
        plane->update(deltaTime);
    }
    detectConflicts();
}

void Simulation::spawnAircraft(Vec2 pos, double heading, double speed, int altitude, const std::string& callsign) {
    aircraft.push_back(std::make_unique<Aircraft>(pos, heading, speed, altitude, callsign));
}

void Simulation::addAirport(const Airport& airport) {
    airports.push_back(airport);
}

void Simulation::detectConflicts() {
    for (auto& plane : aircraft) {
        if (plane->getState() == AircraftState::CONFLICT) {
            plane->setState(AircraftState::APPROACH);
        }
    }

    for (size_t i = 0; i < aircraft.size(); i++) {
        for (size_t j = i + 1; j < aircraft.size(); j++) {
            if (aircraft[i]->collidesWith(*aircraft[j])) {
                aircraft[i]->setState(AircraftState::CONFLICT);
                aircraft[j]->setState(AircraftState::CONFLICT);
            }
        }
    }
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
