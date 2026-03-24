#pragma once

#include "backend/airport.h"
#include "common/constants.h"
#include "common/utils.h"
#include <random>
#include <string>
#include <vector>

class Aircraft;

enum class SpawnRejectionReason {
    NONE,
    CAPACITY_REACHED,
    NO_VALID_ENTRY_POINT,
    UNSAFE_SPAWN
};

struct SpawnEntryPoint {
    std::string label{};
    Vec2 position{};
    double headingMinDeg = 0.0;
    double headingMaxDeg = 0.0;
    double speedMinKts = 0.0;
    double speedMaxKts = 0.0;
    int altitudeMinFt = 0;
    int altitudeMaxFt = 0;
};

struct SpawnCandidate {
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
    std::string callsign{};
    std::string entryLabel{};
    Aircraft* aircraft = nullptr;
};

class SpawnService {
public:
    SpawnService();

    std::vector<SpawnCandidate> createCandidates(const SimSettings& settings,
                                                 const std::vector<Airport>& airports);

private:
    std::mt19937 rng;

    std::vector<SpawnEntryPoint> buildEntryPoints(const SimSettings& settings,
                                                  const std::vector<Airport>& airports) const;
    SpawnCandidate buildCandidate(const SpawnEntryPoint& entryPoint);
    std::string generateCallsign();
    double sampleHeading(double minHeadingDeg, double maxHeadingDeg);
};
