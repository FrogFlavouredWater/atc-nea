#pragma once
#include <raylib.h>
#include <string>

enum class AircraftState {
    APPROACH,
    VECTORING,
    ON_FINAL,
    LANDING,
    CONFLICT
};

class Aircraft {
private:
    Vector2 position;
    Vector2 velocity;

    //primary controls
    float heading;
    float targetHeading;

    float speed;
    float targetSpeed;

    //properties
    std::string callsign;
    AircraftState state;
    bool selected;

    //constraints
    float turnRate = 30.0f;
    float acceleration = 10.0f;


public:
    Aircraft(Vector2 startPos, float initialHeading, float initialSpeed, const std::string& id);

    //TODO: move some of this to cpp when its no longer skeleton stuff
    //also clamp the values like heading and speed

    //gyatters
    [[nodiscard]] Vector2 getPosition() const { return position; }
    [[nodiscard]] float getHeading() const { return heading; }
    [[nodiscard]] float getSpeed() const { return speed; }
    [[nodiscard]] std::string getCallsign() const { return callsign; }
    [[nodiscard]] AircraftState getState() const { return state; }
    [[nodiscard]] bool isSelected() const { return selected; }

    //sixsetters
    void setHeading(float newHeading) { targetHeading = newHeading; }
    void setSpeed(float newSpeed) { targetSpeed = newSpeed; }
    void setSelected(bool isSelected) { selected = isSelected; }
    void setState(AircraftState newState) { state = newState; }

    //funky shit
    [[nodiscard]] float distanceTo(const Aircraft& other) const;
    [[nodiscard]] bool collidesWith(const Aircraft& other) const;

    void update(float deltaTime);
    void render();
};