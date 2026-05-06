#pragma once

#include "core/Types.h"

namespace DisplayConfig {
// DisplayConfig owns startup defaults and allowed limits for the window only.
inline constexpr int32_t DEFAULT_SCREEN_WIDTH = 1536;
inline constexpr int32_t DEFAULT_SCREEN_HEIGHT = 864;
inline constexpr ScreenMode DEFAULT_SCREEN_MODE = ScreenMode::WINDOWED;
inline constexpr int32_t DEFAULT_TARGET_FPS = 120;

inline constexpr int32_t MIN_SCREEN_WIDTH = 800;
inline constexpr int32_t MAX_SCREEN_WIDTH = 3840;
inline constexpr int32_t MIN_SCREEN_HEIGHT = 600;
inline constexpr int32_t MAX_SCREEN_HEIGHT = 2160;
inline constexpr int32_t MIN_TARGET_FPS = 30;
inline constexpr int32_t MAX_TARGET_FPS = 240;

inline constexpr DisplaySettings DEFAULTS{
    DEFAULT_SCREEN_WIDTH,
    DEFAULT_SCREEN_HEIGHT,
    DEFAULT_SCREEN_MODE,
    DEFAULT_TARGET_FPS
};
}

namespace SimConfig {
// SimConfig covers editable simulation settings that are exposed in the UI.
inline constexpr int32_t DEFAULT_MAX_AIRCRAFT = 5;
inline constexpr int32_t DEFAULT_AIRCRAFT_SIZE = 12;
inline constexpr double DEFAULT_PIXELS_PER_NM = 10.5;
inline constexpr double DEFAULT_SIMULATION_SPEED = 5.0;
inline constexpr double DEFAULT_MIN_X_NM = -70.0;
inline constexpr double DEFAULT_MAX_X_NM = 70.0;
inline constexpr double DEFAULT_MIN_Y_NM = -40.0;
inline constexpr double DEFAULT_MAX_Y_NM = 40.0;

inline constexpr int32_t MIN_AIRCRAFT_COUNT = 1;
inline constexpr int32_t MAX_AIRCRAFT_COUNT = 25;
inline constexpr int32_t MIN_AIRCRAFT_SIZE = 4;
inline constexpr int32_t MAX_AIRCRAFT_SIZE = 32;
inline constexpr double MIN_PIXELS_PER_NM = 2.0;
inline constexpr double MAX_PIXELS_PER_NM = 20.0;
inline constexpr double MIN_SIMULATION_SPEED = 1.0;
inline constexpr double MAX_SIMULATION_SPEED = 120.0;
inline constexpr double MIN_MIN_X_NM = -200.0;
inline constexpr double MAX_MIN_X_NM = -10.0;
inline constexpr double MIN_MAX_X_NM = 10.0;
inline constexpr double MAX_MAX_X_NM = 200.0;
inline constexpr double MIN_MIN_Y_NM = -150.0;
inline constexpr double MAX_MIN_Y_NM = -10.0;
inline constexpr double MIN_MAX_Y_NM = 10.0;
inline constexpr double MAX_MAX_Y_NM = 150.0;

inline constexpr SimSettings DEFAULTS{
    DEFAULT_MAX_AIRCRAFT,
    DEFAULT_AIRCRAFT_SIZE,
    DEFAULT_PIXELS_PER_NM,
    DEFAULT_SIMULATION_SPEED,
    DEFAULT_MIN_X_NM,
    DEFAULT_MAX_X_NM,
    DEFAULT_MIN_Y_NM,
    DEFAULT_MAX_Y_NM
};
}

namespace AppConfig {
// Bundle the two editable settings groups into one default.
inline constexpr AppSettings DEFAULTS{
    DisplayConfig::DEFAULTS,
    SimConfig::DEFAULTS
};
}

namespace SeparationRules {
// These are the tactical/legal separation minima used by conflict logic.
inline constexpr double HORIZONTAL_NM = 3.0;
inline constexpr double VERTICAL_FT = 1000.0;
}

namespace CollisionRules {
// Cleanup uses a much smaller horizontal overlap test plus a narrow vertical
// band so stacked aircraft do not "crash" just because their sprites overlap.
inline constexpr double VERTICAL_FT = 200.0;
}

namespace SimTuning {
// SimTuning contains behavioural constants used by services and automation.
inline constexpr double SPAWN_LOOKAHEAD_SECONDS = 75.0;
inline constexpr double SPAWN_PREDICTION_STEP_SECONDS = 5.0;
inline constexpr double CONFLICT_LOOKAHEAD_SECONDS = 180.0;
inline constexpr double CONFLICT_PREDICTION_STEP_SECONDS = 5.0;
inline constexpr double TACTICAL_INTERVENTION_RANGE_NM = 15.0;
inline constexpr double VECTORING_PRIORITY_VERTICAL_FT = 500.0;
inline constexpr double ARRIVAL_SPEED_DISTANCE_NM = 20.0;
inline constexpr double ARRIVAL_SPACING_DISTANCE_NM = 8.0;
inline constexpr double ARRIVAL_SPACING_CROSS_TRACK_NM = 4.0;
inline constexpr double ARRIVAL_SPACING_HEADING_TOLERANCE_DEG = 35.0;
inline constexpr double ARRIVAL_SPEED_MIN_KTS = 150.0;
inline constexpr double ARRIVAL_SPEED_MAX_KTS = 220.0;
inline constexpr double ARRIVAL_SCHEDULE_SPACING_SECONDS = 90.0;
inline constexpr double ARRIVAL_HOLD_THRESHOLD_SECONDS = 120.0;
inline constexpr double ARRIVAL_HOLD_MAX_RANGE_NM = 22.0;
inline constexpr double ARRIVAL_RELEASE_LEAD_SECONDS = 45.0;
inline constexpr double HOLD_LEG_LENGTH_NM = 4.0;
inline constexpr double HOLD_MIN_TURN_RADIUS_NM = 0.8;
inline constexpr double CONFLICT_VISUAL_LATCH_SECONDS = 4.0;
inline constexpr double CRASH_DISPLAY_SECONDS = 2.0;
inline constexpr double RESOLUTION_HOLD_SECONDS = 8.0;
inline constexpr double RESOLUTION_HOLD_REAL_SECONDS = 2.0;
inline constexpr double RESOLUTION_REEVALUATION_SECONDS = 12.0;
inline constexpr double RESOLUTION_REEVALUATION_REAL_SECONDS = 1.5;
inline constexpr double RESOLUTION_TURN_SMALL_DEG = 20.0;
inline constexpr double RESOLUTION_TURN_LARGE_DEG = 35.0;
inline constexpr int32_t RESOLUTION_ALTITUDE_STEP_FT = 1000;
inline constexpr double RESOLUTION_SPEED_STEP_KTS = 20.0;
inline constexpr double ILS_CAPTURE_HEADING_TOLERANCE_DEG = 35.0;
inline constexpr double ILS_CAPTURE_MIN_SPEED_KTS = 120.0;
inline constexpr double ILS_CAPTURE_MAX_SPEED_KTS = 185.0;
// Outer/inner capture bands. stops unrealistic ILS snaps from bad altitudes.
inline constexpr double ILS_OUTER_CAPTURE_MIN_ALTITUDE_FT = 2500.0;
inline constexpr double ILS_OUTER_CAPTURE_MAX_ALTITUDE_FT = 4000.0;
inline constexpr double ILS_INNER_CAPTURE_MIN_ALTITUDE_FT = 1000.0;
inline constexpr double ILS_INNER_CAPTURE_MAX_ALTITUDE_FT = 2500.0;
inline constexpr double ILS_FINAL_DESCENT_START_NM = 3.0;
inline constexpr double ILS_HEADING_CORRECTION_PER_NM = 12.0;
inline constexpr double ILS_HEADING_CORRECTION_MAX_DEG = 18.0;
inline constexpr double ILS_TARGET_SPEED_MIN_KTS = 120.0;
inline constexpr double ILS_TARGET_SPEED_MAX_KTS = 150.0;
inline constexpr double ILS_LANDING_PHASE_DISTANCE_NM = 1.2;
inline constexpr double ILS_TOUCHDOWN_DISTANCE_NM = 0.45;
inline constexpr double ILS_TOUCHDOWN_ALTITUDE_FT = 150.0;
inline constexpr double ILS_TOUCHDOWN_SPEED_MIN_KTS = 110.0;
inline constexpr double ILS_TOUCHDOWN_SPEED_MAX_KTS = 155.0;
}

namespace Config {
// Clamp helpers make sure startup values and UI-edited settings stay within
// supported ranges before they reach the app or simulation.
void clampDisplaySettings(DisplaySettings& settings);
void clampSimSettings(SimSettings& settings);
void clampAppSettings(AppSettings& settings);
}
