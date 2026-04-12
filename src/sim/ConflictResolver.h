#pragma once

#include "backend/aircraft/Aircraft.h"
#include "backend/navigation/airport.h"
#include "sim/ConflictDetector.h"

#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

// Active resolution state keeps enough context to restore an aircraft's prior
// instruction once a conflict has cleared.
struct ConflictResolutionState {
    std::pair<std::string, std::string> conflictPair{};
    AircraftInstruction resumeInstruction{};
    double assignedAtSeconds = 0.0;
};

struct ConflictResolutionRelease {
    std::string callsign{};
    // The pre-resolution instruction saved when the maneuver was first applied.
    AircraftInstruction resumeInstruction{};
};

struct ConflictResolutionAssignment {
    std::string callsign{};
    std::pair<std::string, std::string> conflictPair{};
    // The chosen conflict-resolution instruction to apply this tick.
    AircraftInstruction instruction{};
    bool clearsConflict = false;
};

class ConflictResolver {
public:
    // collectReleases decides which previously-resolved aircraft can safely
    // return to their saved instruction.
    [[nodiscard]] std::vector<ConflictResolutionRelease> collectReleases(
        const std::vector<std::unique_ptr<Aircraft>>& aircraft,
        const std::set<std::pair<std::string, std::string>>& activeConflictPairs,
        const std::set<std::pair<std::string, std::string>>& activePredictedConflictPairs,
        double elapsedSimSeconds);

    [[nodiscard]] std::vector<ConflictResolutionAssignment> resolve(
        const std::vector<std::unique_ptr<Aircraft>>& aircraft,
        const std::vector<Airport>& airports,
        const std::set<std::pair<std::string, std::string>>& activeConflictPairs,
        const std::vector<PredictedConflictAssessment>& predictedConflicts,
        const ConflictDetector& conflictDetector,
        double elapsedSimSeconds);

private:
    std::map<std::string, ConflictResolutionState> activeStates;

    [[nodiscard]] std::vector<PredictedConflictAssessment> collectConflicts(
        const std::vector<std::unique_ptr<Aircraft>>& aircraft,
        const std::set<std::pair<std::string, std::string>>& activeConflictPairs,
        const std::vector<PredictedConflictAssessment>& predictedConflicts,
        const ConflictDetector& conflictDetector) const;

    // Build a short ordered list of plausible maneuvers for one aircraft.
    [[nodiscard]] std::vector<AircraftInstruction> buildCandidates(
        const Aircraft& plane,
        const Aircraft& other,
        const PredictedConflictAssessment& conflict,
        const std::vector<Airport>& airports) const;

    [[nodiscard]] PredictedConflictAssessment assessWithInstruction(
        const Aircraft& plane,
        const AircraftInstruction& instruction,
        const Aircraft& other,
        const ConflictDetector& conflictDetector) const;

    [[nodiscard]] bool recentlyAssigned(
        const Aircraft& first,
        const Aircraft& second,
        const std::pair<std::string, std::string>& pair,
        double simTime) const;

    // Try each maneuverable aircraft in priority order and return the first
    // useful assignment found for this conflict pair.
    [[nodiscard]] std::optional<ConflictResolutionAssignment> chooseAssignment(
        const Aircraft& first,
        const Aircraft& second,
        const PredictedConflictAssessment& conflict,
        const std::pair<std::string, std::string>& pair,
        const std::vector<Airport>& airports,
        const ConflictDetector& conflictDetector,
        double simTime);

    // Save enough state to later release the aircraft back to its prior task.
    void storeState(const std::string& callsign,
                    const std::pair<std::string, std::string>& pair,
                    const AircraftInstruction& resumeInstruction,
                    double simTime);
};
