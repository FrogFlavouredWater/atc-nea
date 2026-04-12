#include "sim/SpawnService.h"

#include "core/Config.h"
#include "core/MathUtils.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace {
// Spawn safety only needs a short horizon: the goal is to stop aircraft from
// appearing directly into an imminent collision, not to solve all future
// traffic management at spawn time.
constexpr double kSpawnLookaheadSeconds = 75.0;
constexpr double kSpawnPredictionStepSeconds = 5.0;

// Each edge spawn points roughly toward the airport, but with a small heading
// spread so all arrivals do not fly the exact same line.
constexpr double kHeadingSpreadDeg = 25.0;

// Spawn points are pulled slightly inward from the map edge so aircraft do not
// appear clipped against the simulation boundary.
constexpr double kEdgeInsetNm = 4.0;

// New arrivals use one generic speed band.
constexpr double kMinSpeedKts = 190.0;
constexpr double kMaxSpeedKts = 260.0;

// Altitudes are chosen in simple 1000 ft steps.
constexpr int kAltitudeFloorFt = 3000;
constexpr int kAltitudeCeilingFt = 9000;
constexpr int kAltitudeStepFt = 1000;

// Small helper so all spawn failures return the same result shape.
SpawnRequestResult rejectSpawn(SpawnRejectionReason reason, std::string message) {
    return SpawnRequestResult{
        false,
        reason,
        std::move(message)
    };
}

// Pick a random altitude on a 1000 ft flight level between the fixed floor and
// ceiling used for spawned traffic.
int pickAltitudeFt(std::mt19937& rng) {
    std::uniform_int_distribution<int> altitudeDist(kAltitudeFloorFt / kAltitudeStepFt,
                                                    kAltitudeCeilingFt / kAltitudeStepFt);
    return altitudeDist(rng) * kAltitudeStepFt;
}

// Callsigns are intentionally lightweight: airline prefix plus a random number.
std::string makeCallsign(std::mt19937& rng) {
    static constexpr std::array<const char*, 8> prefixes{
        "AAL", "BAW", "DAL", "UAL", "KLM", "AFR", "EZY", "RYR"
    };

    std::uniform_int_distribution<size_t> prefixDist(0, prefixes.size() - 1);
    std::uniform_int_distribution<int> numberDist(100, 9999);
    return std::string(prefixes[prefixDist(rng)]) + std::to_string(numberDist(rng));
}

// Handle both normal and wrap-around heading ranges, e.g. 350 deg to 10 deg.
double pickHeading(std::mt19937& rng, double minHeadingDeg, double maxHeadingDeg) {
    if (minHeadingDeg <= maxHeadingDeg) {
        std::uniform_real_distribution<double> headingDist(minHeadingDeg, maxHeadingDeg);
        return headingDist(rng);
    }

    const double wrappedSpan = (360.0 - minHeadingDeg) + maxHeadingDeg;
    std::uniform_real_distribution<double> headingOffsetDist(0.0, wrappedSpan);
    return normalizeAngle(minHeadingDeg + headingOffsetDist(rng));
}
}

// service owns its RNG so repeated spawn requests naturally vary results.
SpawnService::SpawnService() : rng(std::random_device{}()) {}

SpawnRequestResult SpawnService::requestRandomSpawn(const SimSettings& settings,
                                                    const std::vector<Airport>& airports,
                                                    const std::vector<std::unique_ptr<Aircraft>>& aircraft,
                                                    const TrajectoryPredictor& predictor) {
    // check for airspace max capacity
    if (aircraft.size() >= static_cast<size_t>(settings.maxAircraft)) {
        return rejectSpawn(SpawnRejectionReason::CAPACITY_REACHED,
                           "Spawn blocked: max aircraft reached");
    }

    // if spawnable area too small, fixed edge points would collapse into invalid or overlapping positions
    const double widthNm = settings.maxXNm - settings.minXNm;
    const double heightNm = settings.maxYNm - settings.minYNm;
    if (widthNm <= 2.0 * kEdgeInsetNm || heightNm <= 2.0 * kEdgeInsetNm) {
        return rejectSpawn(SpawnRejectionReason::NO_VALID_ENTRY_POINT,
                           "Spawn blocked: no valid entry points");
    }

    // build 8 spawn candidates
    // create first one that does not create an immediate or near immediate collision
    auto plans = buildPlans(settings, airports);
    std::shuffle(plans.begin(), plans.end(), rng);
    for (const auto& plan : plans) {
        if (!isSafe(plan, settings, aircraft, predictor)) {
            continue;
        }
        return SpawnRequestResult{
            true,
            SpawnRejectionReason::NONE,
            "Spawned " + plan.callsign + " from " + plan.entryLabel,
            plan
        };
    }

    // if every edge candidate failed safety check
    return rejectSpawn(SpawnRejectionReason::UNSAFE_SPAWN,
                       "Spawn blocked: unsafe entry");
}

bool SpawnService::isSafe(const SpawnPlan& plan,
                          const SimSettings& sim,
                          const std::vector<std::unique_ptr<Aircraft>>& aircraft,
                          const TrajectoryPredictor& predictor) const {
    // Turn the plan into a temporary Aircraft so the same motion/prediction
    // code used elsewhere in the sim can evaluate it.
    Aircraft spawnedAircraft(plan.position,
                             plan.headingDeg,
                             plan.speedKts,
                             plan.altitudeFt,
                             plan.callsign);

    // Convert the sprite size from pixels into nautical miles so collision
    // checks operate in simulation coordinates.
    const double collisionBoxSizeNm = sim.pixelsPerNm > 0.0
        ? static_cast<double>(sim.aircraftSize) / sim.pixelsPerNm
        : 0.0;

    // Predict the new aircraft once up front, then compare that path against
    // each existing aircraft.
    const auto spawnPath = predictor.predict(spawnedAircraft,
                                             kSpawnLookaheadSeconds,
                                             kSpawnPredictionStepSeconds);

    for (const auto& plane : aircraft) {
        // Reject a spawn that already falls inside another aircraft's
        // collision box right now.
        if (spawnedAircraft.collidesWith(*plane, collisionBoxSizeNm)) {
            return false;
        }

        // Then reject a spawn that would overlap shortly after appearing.
        const auto planePath = predictor.predict(*plane,
                                                 kSpawnLookaheadSeconds,
                                                 kSpawnPredictionStepSeconds);
        const size_t sampleCount = std::min(spawnPath.size(), planePath.size());
        for (size_t i = 0; i < sampleCount; ++i) {
            // Compare the sampled future positions directly. A predicted
            // collision uses the same horizontal collision box plus vertical
            // collision band as the live cleanup rule.
            const double dx = std::abs(spawnPath[i].motion.position.x - planePath[i].motion.position.x);
            const double dy = std::abs(spawnPath[i].motion.position.y - planePath[i].motion.position.y);
            const double altitudeDiffFt = std::abs(spawnPath[i].motion.altitude - planePath[i].motion.altitude);
            if (dx <= collisionBoxSizeNm
                && dy <= collisionBoxSizeNm
                && altitudeDiffFt < CollisionRules::VERTICAL_FT) {
                return false;
            }
        }
    }

    return true;
}

std::array<SpawnPlan, 8> SpawnService::buildPlans(const SimSettings& sim,
                                                  const std::vector<Airport>& airports) {
    // If there is an airport, arrivals roughly aim for it. Otherwise they aim
    // toward the centre of the map.
    const Vec2 target = !airports.empty()
        ? airports.front().position
        : Vec2{
            (sim.minXNm + sim.maxXNm) / 2.0,
            (sim.minYNm + sim.maxYNm) / 2.0
        };

    // Compute the inset rectangle used for the fixed spawn points.
    const double minX = sim.minXNm + kEdgeInsetNm;
    const double maxX = sim.maxXNm - kEdgeInsetNm;
    const double minY = sim.minYNm + kEdgeInsetNm;
    const double maxY = sim.maxYNm - kEdgeInsetNm;
    const double usableWidthNm = maxX - minX;
    const double usableHeightNm = maxY - minY;

    // Eight spawn points: two on each side of the map.
    const std::array<Vec2, 8> positions{
        Vec2{minX, minY + usableHeightNm * 0.25},
        Vec2{minX, minY + usableHeightNm * 0.75},
        Vec2{maxX, minY + usableHeightNm * 0.25},
        Vec2{maxX, minY + usableHeightNm * 0.75},
        Vec2{minX + usableWidthNm * 0.25, minY},
        Vec2{minX + usableWidthNm * 0.75, minY},
        Vec2{minX + usableWidthNm * 0.25, maxY},
        Vec2{minX + usableWidthNm * 0.75, maxY}
    };

    const std::array<const char*, 8> labels{
        "WEST_NORTH",
        "WEST_SOUTH",
        "EAST_NORTH",
        "EAST_SOUTH",
        "NORTH_WEST",
        "NORTH_EAST",
        "SOUTH_WEST",
        "SOUTH_EAST"
    };

    // Fill each plan with one random heading, speed, altitude, and callsign.
    std::array<SpawnPlan, 8> plans{};
    for (size_t i = 0; i < positions.size(); ++i) {
        const Vec2 position = positions[i];

        // Base heading points straight at the target; the final heading is
        // sampled inside a small spread around that direction.
        const double headingDeg = headingToward(position, target);
        std::uniform_real_distribution<double> speedDist(kMinSpeedKts, kMaxSpeedKts);
        plans[i] = SpawnPlan{
            position,
            pickHeading(rng, normalizeAngle(headingDeg - kHeadingSpreadDeg), normalizeAngle(headingDeg + kHeadingSpreadDeg)),
            speedDist(rng),
            pickAltitudeFt(rng),
            makeCallsign(rng),
            labels[i]
        };
    }

    return plans;
}
