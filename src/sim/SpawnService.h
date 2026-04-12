#pragma once

#include "backend/aircraft/Aircraft.h"
#include "backend/navigation/airport.h"
#include "core/Types.h"
#include "sim/TrajectoryPredictor.h"

#include <array>
#include <memory>
#include <random>
#include <string>
#include <vector>

// SpawnService is the only place allowed to admit new aircraft into the sim.
enum class SpawnRejectionReason {
    NONE,
    CAPACITY_REACHED,
    NO_VALID_ENTRY_POINT,
    UNSAFE_SPAWN
};

struct SpawnPlan {
    Vec2 position{};
    double headingDeg = 0.0;
    double speedKts = 0.0;
    int altitudeFt = 0;
    std::string callsign{};
    std::string entryLabel{};
};

struct SpawnRequestResult {
    bool success = false;
    SpawnRejectionReason reason = SpawnRejectionReason::NONE;
    std::string message{};
    // The chosen spawn setup. Keeping it here lets the service report one clear
    // result object instead of a separate "attempt" wrapper plus a public result.
    SpawnPlan spawn{};
    // Filled in by Simulation once the approved candidate is actually inserted.
    Aircraft* aircraft = nullptr;
};

class SpawnService {
public:
    SpawnService();

    // requestRandomSpawn runs the full spawn pipeline:
    // 1. reject obvious failures like capacity,
    // 2. generate plausible edge-entry plans,
    // 3. find the first plan that is safe now and over the lookahead,
    // 4. return one result object describing the outcome.
    [[nodiscard]] SpawnRequestResult requestRandomSpawn(const SimSettings& settings,
                                                        const std::vector<Airport>& airports,
                                                        const std::vector<std::unique_ptr<Aircraft>>& aircraft,
                                                        const TrajectoryPredictor& predictor);

private:
    std::mt19937 rng;

    [[nodiscard]] bool isSafe(const SpawnPlan& plan,
                              const SimSettings& sim,
                              const std::vector<std::unique_ptr<Aircraft>>& aircraft,
                              const TrajectoryPredictor& predictor) const;
    [[nodiscard]] std::array<SpawnPlan, 8> buildPlans(const SimSettings& sim,
                                                      const std::vector<Airport>& airports);
};
