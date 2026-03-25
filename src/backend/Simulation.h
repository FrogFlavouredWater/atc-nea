#pragma once
#include "backend/Aircraft.h"
#include "backend/GuidancePreview.h"
#include "backend/SpawnService.h"
#include "backend/TrajectoryPredictor.h"
#include "backend/airport.h"
#include "common/constants.h"
#include "common/utils.h"
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

struct ConflictResolutionState {
    std::pair<std::string, std::string> pair{};
    AircraftInstruction resumeInstruction{};
    AircraftInstruction activeInstruction{};
    double assignedAtSeconds = 0.0;
};

class Simulation {
private:
    std::vector<std::unique_ptr<Aircraft>> aircraft;
    std::vector<Airport> airports;
    size_t outOfBoundsCount = 0;
    size_t landedCount = 0;
    SimSettings settings = SimConfig::DEFAULTS;
    SpawnService spawnService;
    SpawnRequestResult lastSpawnResult{};
    std::set<std::pair<std::string, std::string>> activeConflictPairs;
    std::set<std::pair<std::string, std::string>> activePredictedConflictPairs;
    std::vector<PredictedConflictAssessment> predictedConflicts;
    std::map<std::string, ConflictResolutionState> activeResolutionStates;
    double elapsedSimSeconds = 0.0;

    bool spawnAircraft(Vec2 pos, double heading, double speed, int altitude, const std::string& callsign);
    bool isOutOfBounds(const Aircraft& plane) const;
    bool isSpawnSafe(const SpawnCandidate& candidate) const;
    bool predictionBreachesSeparation(const std::vector<PredictedAircraftState>& firstPrediction,
                                      const std::vector<PredictedAircraftState>& secondPrediction) const;
    Aircraft* findAircraftByCallsign(const std::string& callsign);
    const Aircraft* findAircraftByCallsign(const std::string& callsign) const;
    bool canCaptureIls(const Aircraft& plane, const Airport& airport) const;
    AircraftCommand buildIlsCommand(const Aircraft& plane, const Airport& airport) const;
    std::vector<PredictedConflictAssessment> collectResolvableConflicts() const;
    std::vector<AircraftInstruction> buildResolutionCandidates(const Aircraft& plane, const Aircraft& other) const;
    PredictedConflictAssessment assessConflictWithInstruction(const Aircraft& plane,
                                                              const AircraftInstruction& instruction,
                                                              const Aircraft& other) const;
    void releaseResolvedAircraft();
    void updateConflictResolutions();
    bool hasReachedRunway(const Aircraft& plane, const Airport& airport) const;
    void removeLandedAircraft();
    void removeOutOfBoundsAircraft();
    void updateAutonomousCommands(double deltaTime);

public:
    Simulation();
    void update(double deltaTime);
    void applySettings(const SimSettings& newSettings);
    SpawnRequestResult requestRandomSpawn();
    void clearLastSpawnResult();
    bool issueInstruction(const Aircraft* plane, const AircraftInstruction& instruction);
    bool issueCommand(const Aircraft* plane, const AircraftCommand& command);
    bool toggleApproachClearance(const Aircraft* plane);
    void addAirport(const Airport& airport);
    void detectConflicts();
    GuidancePreview getGuidancePreview(const Aircraft* plane) const;
    std::vector<PredictedAircraftState> predictTrajectory(const Aircraft* plane,
                                                          double horizonSeconds,
                                                          double stepSeconds) const;

    const std::vector<std::unique_ptr<Aircraft>>& getAircraft() const { return aircraft; }
    const std::vector<Airport>& getAirports() const { return airports; }

    size_t getAircraftCount() const { return aircraft.size(); }
    size_t getOutOfBoundsCount() const { return outOfBoundsCount; }
    size_t getLandedCount() const { return landedCount; }
    size_t getPredictedConflictCount() const { return predictedConflicts.size(); }
    const SpawnRequestResult& getLastSpawnResult() const { return lastSpawnResult; }
    const std::vector<PredictedConflictAssessment>& getPredictedConflicts() const { return predictedConflicts; }
    bool canSpawnMore() const;
    bool containsAircraft(const Aircraft* plane) const;
    
    Aircraft* getAircraftAt(Vec2 pos, double radius);
};
