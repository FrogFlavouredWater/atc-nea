#pragma once
#include <string>
#include "common/utils.h"

enum class AircraftState {
    APPROACH,
    VECTORING,
    ON_FINAL,
    LANDING,
    CONFLICT
};

class Aircraft {
private:
    Vec2 position;
    Vec2 velocity;

    //primary controls
    double heading;
    double targetHeading;

    double speed;
    double targetSpeed;

    int altitude;
    int targetAltitude;

    //properties
    std::string callsign;
    AircraftState state;
    bool selected;

    //constraints
    double turnRate = 3.0; //deg/s
    double acceleration = 2.0; //kts/s


public:
    Aircraft(Vec2 startPos, double initialHeading, double initialSpeed, int initialAltitude, const std::string& id);

    //TODO: move some of this to cpp when its no longer skeleton stuff
    //also clamp the values like heading and speed

    //gyatters
    [[nodiscard]] Vec2 getPosition() const { return position; }
    [[nodiscard]] double getHeading() const { return heading; }
    [[nodiscard]] double getTargetHeading() const { return targetHeading; }
    [[nodiscard]] double getSpeed() const { return speed; }
    [[nodiscard]] double getTargetSpeed() const { return targetSpeed; }
    [[nodiscard]] int getAltitude() const { return altitude; }
    [[nodiscard]] int getTargetAltitude() const { return targetAltitude; }
    [[nodiscard]] std::string getCallsign() const { return callsign; }
    [[nodiscard]] AircraftState getState() const { return state; }
    [[nodiscard]] bool isSelected() const { return selected; }

    //sixsetters
    void setHeading(double newHeading);
    void setSpeed(double newSpeed) { targetSpeed = newSpeed; }
    void setSelected(bool isSelected) { selected = isSelected; }
    void setState(AircraftState newState) { state = newState; }

    //funky shit
    [[nodiscard]] double distanceTo(const Aircraft& other) const;
    [[nodiscard]] bool collidesWith(const Aircraft& other) const;

    void update(double deltaTime);

    static std::string stateToString(AircraftState state);
};