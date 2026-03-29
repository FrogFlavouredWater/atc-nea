#pragma once

#include "backend/aircraft/Aircraft.h"
#include "backend/navigation/airport.h"

#include <memory>
#include <string>
#include <vector>

// Scheduler builds sequencing and spacing actions only; Simulation decides how
// to apply them to live aircraft.
enum class SchedulerActionType {
    ISSUE_INSTRUCTION,
    ISSUE_HOLD,
    RELEASE_HOLD
};

struct SchedulerAction {
    SchedulerActionType type = SchedulerActionType::ISSUE_INSTRUCTION;
    std::string callsign{};
    AircraftInstruction instruction{};
};

class Scheduler {
public:
    [[nodiscard]] std::vector<SchedulerAction> buildSequencingActions(
        const std::vector<std::unique_ptr<Aircraft>>& aircraft,
        const std::vector<Airport>& airports,
        double elapsedSimSeconds) const;

    [[nodiscard]] std::vector<SchedulerAction> buildSpacingActions(
        const std::vector<std::unique_ptr<Aircraft>>& aircraft,
        const std::vector<Airport>& airports) const;

private:
    [[nodiscard]] double estimateArrivalTimeSeconds(const Aircraft& plane, const Airport& airport) const;
};
