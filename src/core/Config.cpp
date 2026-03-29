#include "core/Config.h"

#include <algorithm>

namespace Config {

void clampDisplaySettings(DisplaySettings& settings) {
    // Clamp each independent field first, then leave mode selection alone.
    settings.screenWidth = std::clamp(settings.screenWidth,
                                      DisplayConfig::MIN_SCREEN_WIDTH,
                                      DisplayConfig::MAX_SCREEN_WIDTH);
    settings.screenHeight = std::clamp(settings.screenHeight,
                                       DisplayConfig::MIN_SCREEN_HEIGHT,
                                       DisplayConfig::MAX_SCREEN_HEIGHT);
    settings.targetFps = std::clamp(settings.targetFps,
                                    DisplayConfig::MIN_TARGET_FPS,
                                    DisplayConfig::MAX_TARGET_FPS);
}

void clampSimSettings(SimSettings& settings) {
    settings.maxAircraft = std::clamp(settings.maxAircraft,
                                      SimConfig::MIN_AIRCRAFT_COUNT,
                                      SimConfig::MAX_AIRCRAFT_COUNT);
    settings.aircraftSize = std::clamp(settings.aircraftSize,
                                       SimConfig::MIN_AIRCRAFT_SIZE,
                                       SimConfig::MAX_AIRCRAFT_SIZE);
    settings.pixelsPerNm = std::clamp(settings.pixelsPerNm,
                                      SimConfig::MIN_PIXELS_PER_NM,
                                      SimConfig::MAX_PIXELS_PER_NM);
    settings.simulationSpeed = std::clamp(settings.simulationSpeed,
                                          SimConfig::MIN_SIMULATION_SPEED,
                                          SimConfig::MAX_SIMULATION_SPEED);
    settings.minXNm = std::clamp(settings.minXNm,
                                 SimConfig::MIN_MIN_X_NM,
                                 SimConfig::MAX_MIN_X_NM);
    settings.maxXNm = std::clamp(settings.maxXNm,
                                 SimConfig::MIN_MAX_X_NM,
                                 SimConfig::MAX_MAX_X_NM);
    settings.minYNm = std::clamp(settings.minYNm,
                                 SimConfig::MIN_MIN_Y_NM,
                                 SimConfig::MAX_MIN_Y_NM);
    settings.maxYNm = std::clamp(settings.maxYNm,
                                 SimConfig::MIN_MAX_Y_NM,
                                 SimConfig::MAX_MAX_Y_NM);

    // Keep the world bounds valid even if the UI or startup values cross over.
    if (settings.minXNm >= settings.maxXNm) {
        settings.maxXNm = settings.minXNm + 1.0;
    }
    if (settings.minYNm >= settings.maxYNm) {
        settings.maxYNm = settings.minYNm + 1.0;
    }
}

void clampAppSettings(AppSettings& settings) {
    // App settings are just the display and sim settings bundled together.
    clampDisplaySettings(settings.display);
    clampSimSettings(settings.sim);
}

}
