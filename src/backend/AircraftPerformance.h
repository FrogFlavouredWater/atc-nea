#pragma once

#include <algorithm>
#include <cmath>
#include <numbers>

struct AircraftPerformance {
    double minSpeedKts = 120.0;
    double maxSpeedKts = 320.0;
    double accelerationKtsPerSec = 2.0;
    double climbRateFpm = 1800.0;
    double descentRateFpm = 2200.0;
    double bankAngleDegrees = 25.0;
    double minTurnRateDegPerSec = 1.5;
    double maxTurnRateDegPerSec = 4.5;
    double altitudeCaptureToleranceFt = 25.0;
};

inline double calculateTurnRateDegPerSec(double speedKts, const AircraftPerformance& performance) {
    // if (speedKts <= 1.0) { // divide by zero prevention on ILS final
    //     return performance.maxTurnRateDegPerSec;
    // }

    const double bankAngleRad = performance.bankAngleDegrees * std::numbers::pi_v<double> / 180.0;
    const double turnRate = 1091.0 * std::tan(bankAngleRad) / speedKts; // approximation for turn rate (deg/s)
    return std::clamp(turnRate, performance.minTurnRateDegPerSec, performance.maxTurnRateDegPerSec);
}

inline double calculateTurnRadiusNm(double speedKts, double turnRateDegPerSec) {
    if (speedKts <= 0.0 || turnRateDegPerSec <= 0.0) {
        return 0.0;
    }

    const double speedNmPerSec = speedKts / 3600.0;
    const double turnRateRadPerSec = turnRateDegPerSec * std::numbers::pi_v<double> / 180.0;
    return speedNmPerSec / turnRateRadPerSec;
}
