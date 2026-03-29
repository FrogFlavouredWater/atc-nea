#pragma once

#include "core/Types.h"

#include <cmath>
#include <numbers>

// These helpers assume the sim's heading convention: 0 degrees points north
// and headings increase clockwise.
inline int centerHeight(int maxY, int objectHeight) {
    return (maxY - objectHeight) / 2;
}

inline int centerWidth(int maxX, int objectWidth) {
    return (maxX - objectWidth) / 2;
}

inline double normalizeAngle(double angle) {
    double result = std::fmod(angle, 360.0);
    if (result < 0.0) {
        result += 360.0;
    }
    return result;
}

inline double getShortestAngleDiff(double target, double current) {
    double diff = target - current;
    diff = std::fmod(diff + 180.0, 360.0);
    if (diff < 0.0) {
        diff += 360.0;
    }
    return diff - 180.0;
}

inline Vec2 directionVectorForHeading(double headingDeg) {
    constexpr double kDegToRad = std::numbers::pi_v<double> / 180.0;
    return Vec2{
        std::cos(kDegToRad * (headingDeg - 90.0)),
        std::sin(kDegToRad * (headingDeg - 90.0))
    };
}

inline Vec2 rightNormalForHeading(double headingDeg) {
    constexpr double kDegToRad = std::numbers::pi_v<double> / 180.0;
    return Vec2{
        std::cos(kDegToRad * headingDeg),
        std::sin(kDegToRad * headingDeg)
    };
}

inline double dot(Vec2 first, Vec2 second) {
    return first.x * second.x + first.y * second.y;
}

inline double distanceNm(Vec2 first, Vec2 second) {
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline double headingToward(Vec2 from, Vec2 to) {
    constexpr double kRadToDeg = 180.0 / std::numbers::pi_v<double>;
    return normalizeAngle(std::atan2(to.y - from.y, to.x - from.x) * kRadToDeg + 90.0);
}
