#pragma once

#include "backend/aircraft/Aircraft.h"
#include "backend/navigation/GuidancePreview.h"
#include "backend/navigation/airport.h"
#include "core/Config.h"
#include "sim/ConflictDetector.h"
#include "sim/ConflictResolver.h"
#include "sim/Scheduler.h"
#include "sim/SpawnService.h"
#include "sim/TrajectoryPredictor.h"

#include <memory>
#include <set>
#include <string>
#include <vector>

// Simulation owns all live sim state and orchestrates the backend services.
class Simulation {
private:
    std::vector<std::unique_ptr<Aircraft>> aircraft;
    std::vector<Airport> airports;
    size_t outOfBoundsCount = 0;
    size_t landedCount = 0;
    size_t hullLossCount = 0;
    SimSettings settings = SimConfig::DEFAULTS;
    SpawnService spawnService;
    ConflictDetector conflictDetector;
    ConflictResolver conflictResolver;
    Scheduler scheduler;
    TrajectoryPredictor trajectoryPredictor;
    SpawnRequestResult lastSpawnResult{};
    std::set<std::pair<std::string, std::string>> activeConflictPairs;
    std::set<std::pair<std::string, std::string>> activePredictedConflictPairs;
    std::vector<PredictedConflictAssessment> predictedConflicts;
    double simTime = 0.0;

    bool outOfBounds(const Aircraft& plane) const;
    Aircraft* findByCallsign(const std::string& callsign);
    const Aircraft* findByCallsign(const std::string& callsign) const;
    Aircraft* findAircraft(const Aircraft* plane);
    const Aircraft* findAircraft(const Aircraft* plane) const;
    bool canCaptureIls(const Aircraft& plane, const Airport& airport) const;
    AircraftCommand makeIlsCommand(const Aircraft& plane, const Airport& airport) const;
    void applySpacing();
    void updateSequencing();
    void updateConflicts();
    void releaseResolved();
    void updateResolutions();
    bool reachedRunway(const Aircraft& plane, const Airport& airport) const;
    void removeCollisions();
    void removeLanded();
    void removeOutOfBoundsAircraft();
    void updateAutomation();

public:
    Simulation();
    void update(double dt);
    void applySettings(const SimSettings& sim);
    SpawnRequestResult requestRandomSpawn();
    void clearLastSpawnResult();
    bool issueInstruction(const Aircraft* plane, const AircraftInstruction& instruction);
    bool issueHoldAtCurrentPosition(const Aircraft* plane);
    bool releaseHold(const Aircraft* plane);
    bool issueCommand(const Aircraft* plane, const AircraftCommand& command);
    bool toggleApproachClearance(const Aircraft* plane);
    void addAirport(const Airport& airport);
    void detectConflicts();
    GuidancePreview getGuidancePreview(const Aircraft* plane) const;

    const std::vector<std::unique_ptr<Aircraft>>& getAircraft() const { return aircraft; }
    const std::vector<Airport>& getAirports() const { return airports; }

    size_t getAircraftCount() const { return aircraft.size(); }
    size_t getOutOfBoundsCount() const { return outOfBoundsCount; }
    size_t getLandedCount() const { return landedCount; }
    size_t getHullLossCount() const { return hullLossCount; }
    size_t getPredictedConflictCount() const { return predictedConflicts.size(); }
    const SpawnRequestResult& getLastSpawnResult() const { return lastSpawnResult; }
    bool canSpawnMore() const;
    bool containsAircraft(const Aircraft* plane) const;

    Aircraft* getAircraftAt(Vec2 pos, double radius);
};
