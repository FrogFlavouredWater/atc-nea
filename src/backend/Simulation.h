#pragma once
#include "backend/Aircraft.h"
#include "backend/airport.h"
#include "common/constants.h"
#include "common/utils.h"
#include <vector>
#include <memory>
#include <string>

class Simulation {
private:
    std::vector<std::unique_ptr<Aircraft>> aircraft;
    std::vector<Airport> airports;
    size_t outOfBoundsCount = 0;
    SimSettings settings = SimConfig::DEFAULTS;

    bool isOutOfBounds(const Aircraft& plane) const;
    void removeOutOfBoundsAircraft();
    void updateAutonomousCommands(double deltaTime);

public:
    Simulation();
    void update(double deltaTime);
    void applySettings(const SimSettings& newSettings);
    bool spawnAircraft(Vec2 pos, double heading, double speed, int altitude, const std::string& callsign);
    bool issueCommand(const Aircraft* plane, const AircraftCommand& command);
    void addAirport(const Airport& airport);
    void detectConflicts();

    const std::vector<std::unique_ptr<Aircraft>>& getAircraft() const { return aircraft; }
    const std::vector<Airport>& getAirports() const { return airports; }

    size_t getAircraftCount() const { return aircraft.size(); }
    size_t getOutOfBoundsCount() const { return outOfBoundsCount; }
    bool canSpawnMore() const;
    bool containsAircraft(const Aircraft* plane) const;
    
    Aircraft* getAircraftAt(Vec2 pos, double radius);
};
