#pragma once

#include "backend/aircraft/AircraftMotion.h"
#include "backend/aircraft/AircraftPerformance.h"
#include "core/MathUtils.h"

#include <cmath>
#include <deque>
#include <string>

struct AircraftTrailPoint {
    Vec2 position{};
    // Aircraft-local elapsed time, not wall time.
    double recordedAtSeconds = 0.0;
};

enum class HoldPhase {
    // Two straight legs, two turn segments.
    OUTBOUND,
    TURN_INBOUND,
    INBOUND,
    TURN_OUTBOUND
};

// Aircraft owns the live state for a single target: motion, current command,
// higher-level instruction state, and the visible trail used by the UI.
class Aircraft {
private:
    AircraftPerformance performance;
    AircraftMotionState motion;
    std::deque<AircraftTrailPoint> trailPoints;
    double trailElapsedSeconds = 0.0;

    // Control and state
    AircraftInstruction activeInstruction;
    AircraftCommand command;
    std::string callsign;
    FlightPhase phase;
    bool conflictAlert;
    bool approachCleared = false;
    bool destroyed = false;
    int assignedIlsAirportIndex = -1;
    HoldPhase holdPhase = HoldPhase::OUTBOUND;

    // Internal helpers for command sync, hold logic, trail upkeep.
    void syncCommandToInstruction();
    void updateHoldCommand();
    void updatePhaseFromInstruction();
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
    [[nodiscard]] const AircraftInstruction& getActiveInstruction() const { return activeInstruction; }
    [[nodiscard]] const AircraftCommand& getCommand() const { return command; }
    [[nodiscard]] double getTargetHeading() const { return command.targetHeading; }
    [[nodiscard]] double getTargetSpeed() const { return command.targetSpeed; }
    [[nodiscard]] int getTargetAltitude() const { return command.targetAltitude; }
    [[nodiscard]] const std::string& getCallsign() const { return callsign; }
    [[nodiscard]] FlightPhase getPhase() const { return phase; }
    [[nodiscard]] AircraftControlMode getControlMode() const { return activeInstruction.controlMode; }
    [[nodiscard]] AircraftInstructionType getInstructionType() const { return activeInstruction.type; }
    [[nodiscard]] bool hasConflictAlert() const { return conflictAlert; }
    [[nodiscard]] bool hasApproachClearance() const { return approachCleared; }
    [[nodiscard]] bool isDestroyed() const { return destroyed; }
    [[nodiscard]] int getAssignedIlsAirportIndex() const { return assignedIlsAirportIndex; }
    [[nodiscard]] double getVerticalSpeedFpm() const { return motion.verticalSpeedFpm; }
    [[nodiscard]] double getTurnRateDegPerSec() const { return motion.turnRateDegPerSec; }
    [[nodiscard]] double getTurnRadiusNm() const { return motion.turnRadiusNm; }
    [[nodiscard]] const std::deque<AircraftTrailPoint>& getTrailPoints() const { return trailPoints; }
    [[nodiscard]] double getTrailElapsedSeconds() const { return trailElapsedSeconds; }

    void applyInstruction(const AircraftInstruction& newInstruction);
    void applyCommand(const AircraftCommand& newCommand);
    void setPhase(FlightPhase newPhase) { phase = newPhase; }
    void setConflictAlert(bool inConflict) { conflictAlert = inConflict; }
    void setApproachClearance(bool cleared) { approachCleared = cleared; }
    void markDestroyed();
    void setAssignedIlsAirportIndex(int airportIndex) { assignedIlsAirportIndex = airportIndex; }
    void clearAssignedIlsAirportIndex() { assignedIlsAirportIndex = -1; }

    // Separation checks use the tactical minima; collision checks use the
    // smaller visual collision box plus a narrow vertical band.
    [[nodiscard]] double distanceTo(const Aircraft& other) const;
    [[nodiscard]] double altitudeDifferenceTo(const Aircraft& other) const;
    [[nodiscard]] bool breachesSeparationWith(const Aircraft& other) const;
    [[nodiscard]] bool collidesWith(const Aircraft& other, double collisionBoxSizeNm) const;

    void update(double dt);
};
