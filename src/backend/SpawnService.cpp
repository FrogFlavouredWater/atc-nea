#include "backend/SpawnService.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <random>

namespace {
constexpr double kHeadingSpreadDeg = 25.0;
constexpr double kEdgeInsetNm = 4.0;
constexpr double kMinSpeedKts = 190.0;
constexpr double kMaxSpeedKts = 260.0;
constexpr double kAltitudePerNmFt = 55.0;
constexpr int kAltitudeFloorFt = 3000;
constexpr int kAltitudeCeilingFt = 9000;
constexpr int kAltitudeBandFt = 2500;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

double headingToward(Vec2 from, Vec2 to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    return normalizeAngle(std::atan2(dy, dx) * kRadToDeg + 90.0);
}

double distanceNm(Vec2 from, Vec2 to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    return std::sqrt(dx * dx + dy * dy);
}

Vec2 resolveSpawnTarget(const SimSettings& settings, const std::vector<Airport>& airports) {
    if (!airports.empty()) {
        return airports.front().position;
    }

    return Vec2{
        (settings.minXNm + settings.maxXNm) / 2.0,
        (settings.minYNm + settings.maxYNm) / 2.0
    };
}
}

SpawnService::SpawnService() : rng(std::random_device{}()) {}

std::vector<SpawnCandidate> SpawnService::createCandidates(const SimSettings& settings,
                                                           const std::vector<Airport>& airports) {
    std::vector<SpawnEntryPoint> entryPoints = buildEntryPoints(settings, airports);
    std::shuffle(entryPoints.begin(), entryPoints.end(), rng);

    std::vector<SpawnCandidate> candidates;
    candidates.reserve(entryPoints.size());
    for (const auto& entryPoint : entryPoints) {
        candidates.push_back(buildCandidate(entryPoint));
    }

    return candidates;
}

std::vector<SpawnEntryPoint> SpawnService::buildEntryPoints(const SimSettings& settings,
                                                            const std::vector<Airport>& airports) const {
    const Vec2 target = resolveSpawnTarget(settings, airports);
    const double widthNm = settings.maxXNm - settings.minXNm;
    const double heightNm = settings.maxYNm - settings.minYNm;
    if (widthNm <= 2.0 * kEdgeInsetNm || heightNm <= 2.0 * kEdgeInsetNm) {
        return {};
    }

    const double minX = settings.minXNm + kEdgeInsetNm;
    const double maxX = settings.maxXNm - kEdgeInsetNm;
    const double minY = settings.minYNm + kEdgeInsetNm;
    const double maxY = settings.maxYNm - kEdgeInsetNm;
    const double usableWidthNm = maxX - minX;
    const double usableHeightNm = maxY - minY;

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

    std::vector<SpawnEntryPoint> entryPoints;
    entryPoints.reserve(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        const Vec2 position = positions[i];
        const double baseHeading = headingToward(position, target);
        const double distanceToTarget = distanceNm(position, target);
        const int altitudeMin = std::clamp(
            static_cast<int>(std::lround(distanceToTarget * kAltitudePerNmFt)),
            kAltitudeFloorFt,
            kAltitudeCeilingFt - kAltitudeBandFt
        );
        const int altitudeMax = std::clamp(altitudeMin + kAltitudeBandFt,
                                           altitudeMin + 500,
                                           kAltitudeCeilingFt);

        entryPoints.push_back(SpawnEntryPoint{
            labels[i],
            position,
            normalizeAngle(baseHeading - kHeadingSpreadDeg),
            normalizeAngle(baseHeading + kHeadingSpreadDeg),
            kMinSpeedKts,
            kMaxSpeedKts,
            altitudeMin,
            altitudeMax
        });
    }

    return entryPoints;
}

SpawnCandidate SpawnService::buildCandidate(const SpawnEntryPoint& entryPoint) {
    std::uniform_real_distribution<double> speedDist(entryPoint.speedMinKts, entryPoint.speedMaxKts);
    std::uniform_int_distribution<int> altitudeDist(entryPoint.altitudeMinFt, entryPoint.altitudeMaxFt);

    return SpawnCandidate{
        entryPoint.position,
        sampleHeading(entryPoint.headingMinDeg, entryPoint.headingMaxDeg),
        speedDist(rng),
        altitudeDist(rng),
        generateCallsign(),
        entryPoint.label
    };
}

std::string SpawnService::generateCallsign() {
    static constexpr std::array<const char*, 8> prefixes{
        "AAL", "BAW", "DAL", "UAL", "KLM", "AFR", "EZY", "RYR"
    };

    std::uniform_int_distribution<size_t> prefixDist(0, prefixes.size() - 1);
    std::uniform_int_distribution<int> numberDist(100, 9999);
    return std::string(prefixes[prefixDist(rng)]) + std::to_string(numberDist(rng));
}

double SpawnService::sampleHeading(double minHeadingDeg, double maxHeadingDeg) {
    if (minHeadingDeg <= maxHeadingDeg) {
        std::uniform_real_distribution<double> headingDist(minHeadingDeg, maxHeadingDeg);
        return headingDist(rng);
    }

    const double wrappedSpan = (360.0 - minHeadingDeg) + maxHeadingDeg;
    std::uniform_real_distribution<double> headingOffsetDist(0.0, wrappedSpan);
    return normalizeAngle(minHeadingDeg + headingOffsetDist(rng));
}
