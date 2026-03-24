#include "backend/TrajectoryPredictor.h"
#include <algorithm>

std::vector<PredictedAircraftState> TrajectoryPredictor::predict(const Aircraft& aircraft,
                                                                 double horizonSeconds,
                                                                 double stepSeconds) {
    if (horizonSeconds < 0.0 || stepSeconds <= 0.0) {
        return {};
    }

    std::vector<PredictedAircraftState> prediction;
    prediction.reserve(static_cast<size_t>(horizonSeconds / stepSeconds) + 2);

    AircraftMotionState motion = aircraft.getMotionState();
    const AircraftCommand& command = aircraft.getCommand();
    const AircraftPerformance& performance = aircraft.getPerformance();

    prediction.push_back({0.0, motion});

    double elapsedSeconds = 0.0;
    while (elapsedSeconds < horizonSeconds) {
        const double delta = std::min(stepSeconds, horizonSeconds - elapsedSeconds);
        stepAircraftMotion(motion, command, performance, delta);
        elapsedSeconds += delta;
        prediction.push_back({elapsedSeconds, motion});
    }

    return prediction;
}
