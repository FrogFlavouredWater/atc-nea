#include "backend/simulation/Simulation.h"

#include "backend/simulation/SimUtil.h"
#include "common/logger.h"
#include "common/utils.h"
#include <algorithm>
#include <cmath>
#include <set>

using namespace SimulationDetail;

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
        if (horizontalDistanceNm(firstPrediction[i].motion, secondPrediction[i].motion) < SeparationRules::HORIZONTAL_NM
            && verticalDistanceFt(firstPrediction[i].motion, secondPrediction[i].motion) < SeparationRules::VERTICAL_FT) {
            return true;
        }
    }

    return false;
}

void Simulation::removeCollidedAircraft() {
    if (aircraft.size() < 2) {
        return;
    }

    std::set<size_t> collidedIndices;
    for (size_t i = 0; i < aircraft.size(); ++i) {
        for (size_t j = i + 1; j < aircraft.size(); ++j) {
            if (!aircraftSquaresTouch(*aircraft[i], *aircraft[j], settings)) {
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
