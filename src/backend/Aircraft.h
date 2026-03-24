#pragma once
#include <cmath>
#include <deque>
#include <string>
#include "backend/AircraftControl.h"
#include "backend/AircraftMotion.h"
#include "backend/AircraftPerformance.h"
#include "common/utils.h"

struct AircraftTrailPoint {
    Vec2 position{};
    double recordedAtSeconds = 0.0;
};

class Aircraft {
private:
    AircraftPerformance performance;
    AircraftMotionState motion;
    std::deque<AircraftTrailPoint> trailPoints;
    double trailElapsedSeconds = 0.0;

    // Control and state
    AircraftCommand command;
    std::string callsign;
    FlightPhase phase;
    AircraftControlMode controlMode;
    bool conflictAlert;
    bool approachCleared = false;
    int assignedIlsAirportIndex = -1;

    void recordTrailPoint();
    void trimTrailPoints();

public:
    Aircraft(Vec2 startPos, double initialHeading, double initialSpeed, int initialAltitude, const std::string& id);

    [[nodiscard]] Vec2 getPosition() const { return motion.position; }
    [[nodiscard]] double getHeading() const { return motion.heading; }
    [[nodiscard]] double getSpeed() const { return motion.speed; }
    [[nodiscard]] int getAltitude() const { return static_cast<int>(std::lround(motion.altitude)); }
    [[nodiscard]] double getAltitudeExact() const { return motion.altitude; }
    [[nodiscard]] const AircraftMotionState& getMotionState() const { return motion; }
    [[nodiscard]] const AircraftPerformance& getPerformance() const { return performance; }
    [[nodiscard]] const AircraftCommand& getCommand() const { return command; }
    [[nodiscard]] double getTargetHeading() const { return command.targetHeading; }
    [[nodiscard]] double getTargetSpeed() const { return command.targetSpeed; }
    [[nodiscard]] int getTargetAltitude() const { return command.targetAltitude; }
    [[nodiscard]] const std::string& getCallsign() const { return callsign; }
    [[nodiscard]] FlightPhase getPhase() const { return phase; }
    [[nodiscard]] AircraftControlMode getControlMode() const { return controlMode; }
    [[nodiscard]] bool hasConflictAlert() const { return conflictAlert; }
    [[nodiscard]] bool hasApproachClearance() const { return approachCleared; }
    [[nodiscard]] int getAssignedIlsAirportIndex() const { return assignedIlsAirportIndex; }
    [[nodiscard]] double getVerticalSpeedFpm() const { return motion.verticalSpeedFpm; }
    [[nodiscard]] double getTurnRateDegPerSec() const { return motion.turnRateDegPerSec; }
    [[nodiscard]] double getTurnRadiusNm() const { return motion.turnRadiusNm; }
    [[nodiscard]] const std::deque<AircraftTrailPoint>& getTrailPoints() const { return trailPoints; }
    [[nodiscard]] double getTrailElapsedSeconds() const { return trailElapsedSeconds; }

    void applyCommand(const AircraftCommand& newCommand);
    void setPhase(FlightPhase newPhase) { phase = newPhase; }
    void setConflictAlert(bool inConflict) { conflictAlert = inConflict; }
    void setApproachClearance(bool cleared) { approachCleared = cleared; }
    void setAssignedIlsAirportIndex(int airportIndex) { assignedIlsAirportIndex = airportIndex; }
    void clearAssignedIlsAirportIndex() { assignedIlsAirportIndex = -1; }

    [[nodiscard]] double distanceTo(const Aircraft& other) const;
    [[nodiscard]] double altitudeDifferenceTo(const Aircraft& other) const;
    [[nodiscard]] bool breachesSeparationWith(const Aircraft& other) const;
    [[nodiscard]] bool collidesWith(const Aircraft& other) const;

    void update(double deltaTime);
};
