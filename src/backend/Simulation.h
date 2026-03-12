#pragma once
#include "backend/Aircraft.h"
#include "backend/airport.h"
#include "common/utils.h"
#include <vector>
#include <memory>
#include <string>

class Simulation {
private:
    std::vector<std::unique_ptr<Aircraft>> aircraft;
    std::vector<Airport> airports;

public:
    Simulation();
    void update(double deltaTime);
    bool spawnAircraft(Vec2 pos, double heading, double speed, int altitude, const std::string& callsign);
    void addAirport(const Airport& airport);
    void detectConflicts();

    const std::vector<std::unique_ptr<Aircraft>>& getAircraft() const { return aircraft; }
    const std::vector<Airport>& getAirports() const { return airports; }

    size_t getAircraftCount() const { return aircraft.size(); }
    bool canSpawnMore() const;
    
    Aircraft* getAircraftAt(Vec2 pos, double radius);
};
