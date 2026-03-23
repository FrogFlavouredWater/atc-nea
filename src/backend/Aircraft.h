#pragma once
#include <string>
#include "backend/AircraftControl.h"
#include "common/utils.h"

class Aircraft {
private:
    Vec2 position;
    Vec2 velocity;

    // Live kinematics
    double heading;
    double speed;
    int altitude;

    // Control and state
    AircraftCommand command;
    std::string callsign;
    FlightPhase phase;
    AircraftControlMode controlMode;
    bool conflictAlert;

    // Constraints
    double turnRate = 3.0; //deg/s
    double acceleration = 2.0; //kts/s


public:
    Aircraft(Vec2 startPos, double initialHeading, double initialSpeed, int initialAltitude, const std::string& id);

    [[nodiscard]] Vec2 getPosition() const { return position; }
    [[nodiscard]] double getHeading() const { return heading; }
    [[nodiscard]] double getSpeed() const { return speed; }
    [[nodiscard]] int getAltitude() const { return altitude; }
    [[nodiscard]] AircraftCommand getCommand() const { return command; }
    [[nodiscard]] double getTargetHeading() const { return command.targetHeading; }
    [[nodiscard]] double getTargetSpeed() const { return command.targetSpeed; }
    [[nodiscard]] int getTargetAltitude() const { return command.targetAltitude; }
    [[nodiscard]] const std::string& getCallsign() const { return callsign; }
    [[nodiscard]] FlightPhase getPhase() const { return phase; }
    [[nodiscard]] AircraftControlMode getControlMode() const { return controlMode; }
    [[nodiscard]] bool hasConflictAlert() const { return conflictAlert; }

    void applyCommand(const AircraftCommand& newCommand);
    void setPhase(FlightPhase newPhase) { phase = newPhase; }
    void setConflictAlert(bool inConflict) { conflictAlert = inConflict; }

    [[nodiscard]] double distanceTo(const Aircraft& other) const;
    [[nodiscard]] bool collidesWith(const Aircraft& other) const;

    void update(double deltaTime);
};
