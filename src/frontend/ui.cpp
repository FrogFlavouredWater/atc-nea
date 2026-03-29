#include "frontend/ui.h"

#include "backend/aircraft/Aircraft.h"
#include "backend/navigation/GuidancePreview.h"
#include "backend/navigation/airport.h"
#include "core/Config.h"
#include "core/MathUtils.h"
#include "sim/Simulation.h"

#define RAYGUI_IMPLEMENTATION
#include <raygui.h>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
constexpr double kDegToRad = std::numbers::pi_v<double> / 180.0;

struct SettingsEditState {
    bool screenWidth = false;
    bool screenHeight = false;
    bool targetFps = false;
    bool maxAircraft = false;
    bool aircraftSize = false;
};

SettingsEditState& settingsEditState() {
    // Keep the raygui edit toggles in one static bucket instead of threading
    // them through every settings-drawing helper.
    static SettingsEditState state;
    return state;
}

void clampPreviewSettings(AppSettings& settings, const SettingsEditState& state) {
    if (!state.screenWidth) {
        settings.display.screenWidth = std::clamp(settings.display.screenWidth,
                                                  DisplayConfig::MIN_SCREEN_WIDTH,
                                                  DisplayConfig::MAX_SCREEN_WIDTH);
    }
    if (!state.screenHeight) {
        settings.display.screenHeight = std::clamp(settings.display.screenHeight,
                                                   DisplayConfig::MIN_SCREEN_HEIGHT,
                                                   DisplayConfig::MAX_SCREEN_HEIGHT);
    }
    if (!state.targetFps) {
        settings.display.targetFps = std::clamp(settings.display.targetFps,
                                                DisplayConfig::MIN_TARGET_FPS,
                                                DisplayConfig::MAX_TARGET_FPS);
    }
    if (!state.maxAircraft) {
        settings.sim.maxAircraft = std::clamp(settings.sim.maxAircraft,
                                              SimConfig::MIN_AIRCRAFT_COUNT,
                                              SimConfig::MAX_AIRCRAFT_COUNT);
    }
    if (!state.aircraftSize) {
        settings.sim.aircraftSize = std::clamp(settings.sim.aircraftSize,
                                               SimConfig::MIN_AIRCRAFT_SIZE,
                                               SimConfig::MAX_AIRCRAFT_SIZE);
    }

    Config::clampSimSettings(settings.sim);
}

void resetSettingsEditState(SettingsEditState& state) {
    state = {};
}
}

namespace ui_detail {
// ui_detail groups rendering-only helpers and theme constants used internally
// by UI.cpp so the public UI interface can stay small.
inline constexpr Color kBackdropTop{3, 10, 20, 255};
inline constexpr Color kBackdropBottom{9, 33, 67, 255};
inline constexpr Color kOverlay{0, 0, 0, 130};
inline constexpr Color kCard{11, 18, 31, 255};
inline constexpr Color kPanel{16, 27, 44, 255};
inline constexpr Color kBorder{38, 58, 84, 255};
inline constexpr Color kHeader{7, 13, 24, 255};
inline constexpr Color kFooter{9, 16, 28, 255};
inline constexpr Color kAccent{63, 190, 255, 255};
inline constexpr Color kAction{30, 98, 163, 255};
inline constexpr Color kActionFocused{45, 126, 200, 255};
inline constexpr Color kActionPressed{23, 77, 128, 255};
inline constexpr Color kText{235, 241, 247, 255};
inline constexpr Color kMuted{142, 158, 179, 255};
inline constexpr Color kValueFill{24, 41, 63, 255};
inline constexpr Color kGuidanceYellow{245, 220, 66, 255};
inline constexpr Color kApproachGreen{102, 255, 140, 255};
inline constexpr Color kIlsBlue{82, 174, 255, 255};
inline constexpr Color kTrackTrail{42, 210, 95, 255};

inline constexpr float kIlsCenterlineAlpha = 0.52f;
inline constexpr float kIlsMajorTickAlpha = 0.48f;
inline constexpr float kIlsMinorTickAlpha = 0.24f;
inline constexpr float kIlsBoundaryAlpha = 0.24f;
inline constexpr float kIlsSectorEdgeAlpha = 0.12f;
inline constexpr float kIlsLabelAlpha = 0.72f;

inline constexpr double kVisibleTrailAgeSeconds = 90.0;
inline constexpr double kSelectedTrailMinAlpha = 0.18;
inline constexpr double kSelectedTrailMaxAlpha = 0.72;
inline constexpr double kUnselectedTrailMaxAlpha = 0.55;

const char* screenModeLabel(ScreenMode mode);
ScreenMode nextScreenMode(ScreenMode mode);
void drawShadowedText(const char* text,
                      int x,
                      int y,
                      int fontSize,
                      Color textColor,
                      Color shadowColor,
                      int shadowOffset = 4);

Vec2 headingDirectionNm(double headingDeg) {
    return Vec2{
        std::cos((headingDeg - 90.0) * kDegToRad),
        std::sin((headingDeg - 90.0) * kDegToRad)
    };
}

Vec2 headingNormalNm(double headingDeg) {
    const Vec2 direction = headingDirectionNm(headingDeg);
    return Vec2{-direction.y, direction.x};
}

void drawArcSegmentNm(Vec2 centerNm,
                      double radiusNm,
                      double startAngleDeg,
                      double sweepAngleDeg,
                      int directionSign,
                      Color color,
                      float thickness,
                      const SimSettings& simSettings) {
    // Approximate arcs with short line segments so all preview drawing stays in
    // the same sim-space to screen-space pipeline as the rest of the radar.
    if (radiusNm <= 0.0 || sweepAngleDeg <= 0.0 || directionSign == 0) {
        return;
    }

    const int segments = std::max(12, static_cast<int>(std::ceil(sweepAngleDeg / 6.0)));
    const double angleStepDeg = sweepAngleDeg / static_cast<double>(segments);

    auto pointAtAngle = [&](double angleDeg) {
        return UI::NMToPixels(
            Vec2{
                centerNm.x + std::cos(angleDeg * kDegToRad) * radiusNm,
                centerNm.y + std::sin(angleDeg * kDegToRad) * radiusNm
            },
            simSettings
        );
    };

    Vector2 previous = pointAtAngle(startAngleDeg);
    for (int i = 1; i <= segments; ++i) {
        const double angleDeg = startAngleDeg + angleStepDeg * static_cast<double>(i) * static_cast<double>(directionSign);
        const Vector2 current = pointAtAngle(angleDeg);
        DrawLineEx(previous, current, thickness, color);
        previous = current;
    }
}

void applySettingsTheme() {
    GuiLoadStyleDefault();
    GuiSetStyle(DEFAULT, TEXT_SIZE, 18);
    GuiSetStyle(DEFAULT, BORDER_WIDTH, 1);

    GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(kAction));
    GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(kActionFocused));
    GuiSetStyle(BUTTON, BASE_COLOR_PRESSED, ColorToInt(kActionPressed));
    GuiSetStyle(BUTTON, BORDER_COLOR_NORMAL, ColorToInt(kAction));
    GuiSetStyle(BUTTON, BORDER_COLOR_FOCUSED, ColorToInt(kActionFocused));
    GuiSetStyle(BUTTON, BORDER_COLOR_PRESSED, ColorToInt(kActionPressed));
    GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(RAYWHITE));
    GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(RAYWHITE));
    GuiSetStyle(BUTTON, TEXT_COLOR_PRESSED, ColorToInt(RAYWHITE));

    GuiSetStyle(VALUEBOX, BASE_COLOR_NORMAL, ColorToInt(Color{14, 24, 38, 255}));
    GuiSetStyle(VALUEBOX, BORDER_COLOR_NORMAL, ColorToInt(kBorder));
    GuiSetStyle(VALUEBOX, TEXT_COLOR_NORMAL, ColorToInt(kText));

    GuiSetStyle(SLIDER, BASE_COLOR_NORMAL, ColorToInt(Color{34, 50, 71, 255}));
    GuiSetStyle(SLIDER, BASE_COLOR_FOCUSED, ColorToInt(kAccent));
    GuiSetStyle(SLIDER, BASE_COLOR_PRESSED, ColorToInt(kAccent));
    GuiSetStyle(SLIDER, BORDER_COLOR_NORMAL, ColorToInt(kBorder));
}

void drawPanel(Rectangle bounds, Color fill, Color border) {
    DrawRectangleRec(bounds, fill);
    DrawRectangleLinesEx(bounds, 1.0f, border);
}

void drawSection(Rectangle bounds, const char* title, const char* subtitle) {
    drawPanel(bounds, kPanel, kBorder);
    DrawRectangle(static_cast<int>(bounds.x), static_cast<int>(bounds.y), static_cast<int>(bounds.width), 4, kAccent);
    DrawText(title, static_cast<int>(bounds.x) + 18, static_cast<int>(bounds.y) + 16, 22, kText);
    DrawText(subtitle, static_cast<int>(bounds.x) + 18, static_cast<int>(bounds.y) + 42, 14, kMuted);
}

void drawValueChip(Rectangle bounds, const char* text) {
    drawPanel(bounds, kValueFill, kBorder);
    const int textWidth = MeasureText(text, 16);
    DrawText(text,
             static_cast<int>(bounds.x + (bounds.width - static_cast<float>(textWidth)) / 2.0f),
             static_cast<int>(bounds.y + 6.0f),
             16,
             kText);
}

void drawFieldText(int x, int y, const char* label, const char* subtitle) {
    DrawText(label, x, y, 18, kText);
    if (subtitle && subtitle[0] != '\0') {
        DrawText(subtitle, x, y + 20, 14, kMuted);
    }
}

void drawSpinnerRow(Rectangle row,
                    const char* label,
                    const char* subtitle,
                    int& value,
                    int minValue,
                    int maxValue,
                    bool& editMode) {
    const bool hasSubtitle = subtitle && subtitle[0] != '\0';
    const float controlWidth = 170.0f;
    const Rectangle control{
        row.x + row.width - controlWidth,
        row.y + (hasSubtitle ? 4.0f : 2.0f),
        controlWidth,
        34.0f
    };

    drawFieldText(static_cast<int>(row.x),
                  static_cast<int>(row.y + (hasSubtitle ? 0.0f : 7.0f)),
                  label,
                  subtitle);

    if (GuiSpinner(control, nullptr, &value, minValue, maxValue, editMode)) {
        editMode = !editMode;
    }
}

void drawModeRow(Rectangle row, const char* label, const char* subtitle, ScreenMode& mode) {
    const bool hasSubtitle = subtitle && subtitle[0] != '\0';
    const float buttonWidth = 190.0f;
    const Rectangle control{
        row.x + row.width - buttonWidth,
        row.y + (hasSubtitle ? 4.0f : 2.0f),
        buttonWidth,
        34.0f
    };

    drawFieldText(static_cast<int>(row.x),
                  static_cast<int>(row.y + (hasSubtitle ? 0.0f : 7.0f)),
                  label,
                  subtitle);

    if (GuiButton(control, screenModeLabel(mode))) {
        mode = nextScreenMode(mode);
    }
}

void drawSliderRow(Rectangle row,
                   const char* label,
                   const char* subtitle,
                   double& value,
                   double minValue,
                   double maxValue,
                   const char* valueFormat) {
    const bool hasSubtitle = subtitle && subtitle[0] != '\0';
    drawFieldText(static_cast<int>(row.x),
                  static_cast<int>(row.y + (hasSubtitle ? 0.0f : 4.0f)),
                  label,
                  subtitle);

    const Rectangle valueChip{
        row.x + row.width - 92.0f,
        row.y + 2.0f,
        92.0f,
        30.0f
    };
    drawValueChip(valueChip, TextFormat(valueFormat, value));

    float sliderValue = static_cast<float>(value);
    GuiSliderBar(
        Rectangle{
            row.x,
            row.y + (hasSubtitle ? 30.0f : 26.0f),
            row.width,
            18.0f
        },
        nullptr,
        nullptr,
        &sliderValue,
        static_cast<float>(minValue),
        static_cast<float>(maxValue)
    );
    value = sliderValue;
}

const char* screenModeLabel(ScreenMode mode) {
    switch (mode) {
    case ScreenMode::WINDOWED: return "Windowed";
    case ScreenMode::BORDERLESS_WINDOWED: return "Borderless";
    default: return "UNKNOWN";
    }
}

ScreenMode nextScreenMode(ScreenMode mode) {
    switch (mode) {
    case ScreenMode::WINDOWED: return ScreenMode::BORDERLESS_WINDOWED;
    case ScreenMode::BORDERLESS_WINDOWED: return ScreenMode::WINDOWED;
    default: return ScreenMode::WINDOWED;
    }
}

void drawShadowedText(const char* text,
                      int x,
                      int y,
                      int fontSize,
                      Color textColor,
                      Color shadowColor,
                      int shadowOffset) {
    DrawText(text, x + shadowOffset, y + shadowOffset, fontSize, shadowColor);
    DrawText(text, x, y, fontSize, textColor);
}

Color aircraftColor(const Aircraft& aircraft, bool selected) {
    Color color = WHITE;
    if (aircraft.getControlMode() == AircraftControlMode::ILS) {
        color = kIlsBlue;
    } else if (aircraft.hasApproachClearance()) {
        color = kApproachGreen;
    } else if (selected) {
        color = YELLOW;
    }

    if (aircraft.hasConflictAlert()) {
        color = RED;
    }

    return color;
}
} // namespace ui_detail

namespace {
void drawSelectedAircraftDetails(const Aircraft& aircraft, bool debugEnabled) {
    Color selectedColor = YELLOW;
    if (aircraft.getControlMode() == AircraftControlMode::ILS) {
        selectedColor = ui_detail::kIlsBlue;
    } else if (aircraft.hasApproachClearance()) {
        selectedColor = ui_detail::kApproachGreen;
    }

    DrawText("Selected: ", 10, 70, 20, WHITE);
    DrawText(aircraft.getCallsign().c_str(), 110, 70, 20, selectedColor);
    DrawText("Controls: A/D heading, W/S speed, Q/E altitude, H hold/release, I ILS clr, ,/. sim speed",
             10,
             95,
             16,
             GRAY);

    if (!debugEnabled) {
        return;
    }

    const int startY = 130;
    DrawText("--- DEBUG DATA ---", 10, startY, 16, GREEN);
    DrawText(TextFormat("Pos: %.2f, %.2f", aircraft.getPosition().x, aircraft.getPosition().y), 10, startY + 20, 16, GREEN);
    DrawText(TextFormat("Heading: %.2f (Target: %.2f)", aircraft.getHeading(), aircraft.getTargetHeading()), 10, startY + 40, 16, GREEN);
    DrawText(TextFormat("Speed: %.0f kts (Target: %.0f kts)", aircraft.getSpeed(), aircraft.getTargetSpeed()), 10, startY + 60, 16, GREEN);
    DrawText(TextFormat("Altitude: %i (Target: %i)", aircraft.getAltitude(), aircraft.getTargetAltitude()), 10, startY + 80, 16, GREEN);
    DrawText(TextFormat("Vertical Speed: %+.0f fpm", aircraft.getVerticalSpeedFpm()), 10, startY + 100, 16, GREEN);
    DrawText(TextFormat("Turn Rate: %.2f deg/s", aircraft.getTurnRateDegPerSec()), 10, startY + 120, 16, GREEN);
    DrawText(TextFormat("Turn Radius: %.2f NM", aircraft.getTurnRadiusNm()), 10, startY + 140, 16, GREEN);
    DrawText(TextFormat("Phase: %s", toString(aircraft.getPhase())), 10, startY + 160, 16, GREEN);
    DrawText(TextFormat("Control: %s", toString(aircraft.getControlMode())), 10, startY + 180, 16, GREEN);
    DrawText(TextFormat("Instruction: %s", toString(aircraft.getInstructionType())), 10, startY + 200, 16, GREEN);
    DrawText(TextFormat("Conflict: %s", aircraft.hasConflictAlert() ? "YES" : "NO"), 10, startY + 220, 16, GREEN);
    DrawText(TextFormat("Approach Clearance: %s", aircraft.hasApproachClearance() ? "CLEARED" : "NO"), 10, startY + 240, 16, GREEN);
    DrawText(TextFormat("ILS Airport Index: %d", aircraft.getAssignedIlsAirportIndex()), 10, startY + 260, 16, GREEN);
}
}

MainMenuAction UI::DrawMainMenu() {
    // The main menu is intentionally self-contained: render it and return the
    // user's chosen action for Engine to interpret.
    GuiLoadStyleDefault();
    DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK, DARKBLUE);

    DrawCircleLines(GetScreenWidth() / 2, GetScreenHeight() / 2, 200, Fade(DARKGRAY, 0.3f));
    DrawCircleLines(GetScreenWidth() / 2, GetScreenHeight() / 2, 400, Fade(DARKGRAY, 0.2f));
    DrawCircleLines(GetScreenWidth() / 2, GetScreenHeight() / 2, 600, Fade(DARKGRAY, 0.1f));

    constexpr const char* kTitle = "ATC SIMULATOR";
    constexpr int kTitleFontSize = 60;
    const int titleX = centerWidth(GetScreenWidth(), MeasureText(kTitle, kTitleFontSize));
    const int titleY = 150;
    ui_detail::drawShadowedText(kTitle, titleX, titleY, kTitleFontSize, RAYWHITE, BLACK);

    constexpr const char* kSubtitle = "Air Traffic Control Simulation";
    constexpr int kSubtitleFontSize = 20;
    DrawText(kSubtitle,
             centerWidth(GetScreenWidth(), MeasureText(kSubtitle, kSubtitleFontSize)),
             titleY + 70,
             kSubtitleFontSize,
             LIGHTGRAY);

    const double buttonWidth = 220.0;
    const double buttonHeight = 50.0;
    const double buttonX = static_cast<double>(centerWidth(GetScreenWidth(), static_cast<int>(buttonWidth)));
    const double buttonY = static_cast<double>(GetScreenHeight()) * 0.56;

    GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(DARKGRAY));
    GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(RAYWHITE));
    GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(BLUE));
    GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(WHITE));
    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    const Rectangle startButton{
        static_cast<float>(buttonX),
        static_cast<float>(buttonY),
        static_cast<float>(buttonWidth),
        static_cast<float>(buttonHeight)
    };
    const Rectangle settingsButton{
        static_cast<float>(buttonX),
        static_cast<float>(buttonY + buttonHeight + 20.0),
        static_cast<float>(buttonWidth),
        static_cast<float>(buttonHeight)
    };
    const Rectangle exitButton{
        static_cast<float>(buttonX),
        static_cast<float>(buttonY + (buttonHeight + 20.0) * 2.0),
        static_cast<float>(buttonWidth),
        static_cast<float>(buttonHeight)
    };

    if (GuiButton(startButton, "START MISSION")) {
        return MainMenuAction::START;
    }
    if (GuiButton(settingsButton, "SETTINGS")) {
        return MainMenuAction::OPEN_SETTINGS;
    }
    if (GuiButton(exitButton, "EXIT")) {
        return MainMenuAction::EXIT;
    }

    DrawText("v0.0.1a", 10, GetScreenHeight() - 25, 15, DARKGRAY);
    return MainMenuAction::NONE;
}

SettingsMenuResult UI::DrawSettingsMenu(AppSettings& settings) {
    SettingsMenuResult result;
    SettingsEditState& editState = settingsEditState();

    ui_detail::applySettingsTheme();

    DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), ui_detail::kBackdropTop, ui_detail::kBackdropBottom);
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), ui_detail::kOverlay);

    const float screenWidth = static_cast<float>(GetScreenWidth());
    const float screenHeight = static_cast<float>(GetScreenHeight());
    const float cardWidth = std::min(std::clamp(screenWidth * 0.74f, 980.0f, 1180.0f), screenWidth - 56.0f);
    const float cardHeight = std::min(std::clamp(screenHeight * 0.82f, 660.0f, 740.0f), screenHeight - 36.0f);

    const Rectangle card{
        (screenWidth - cardWidth) / 2.0f,
        (screenHeight - cardHeight) / 2.0f,
        cardWidth,
        cardHeight
    };
    const Rectangle header{card.x, card.y, card.width, 94.0f};
    const Rectangle footer{card.x, card.y + card.height - 76.0f, card.width, 76.0f};

    ui_detail::drawPanel(card, ui_detail::kCard, ui_detail::kBorder);
    DrawRectangleRec(header, ui_detail::kHeader);
    DrawRectangleRec(footer, ui_detail::kFooter);
    DrawRectangle(static_cast<int>(card.x), static_cast<int>(header.y + header.height), static_cast<int>(card.width), 1, ui_detail::kBorder);
    DrawRectangle(static_cast<int>(card.x), static_cast<int>(footer.y), static_cast<int>(card.width), 1, ui_detail::kBorder);

    DrawText("Simulator Settings", static_cast<int>(card.x) + 28, static_cast<int>(card.y) + 20, 34, RAYWHITE);
    DrawText("Review changes here, then press Apply when you are ready.",
             static_cast<int>(card.x) + 28,
             static_cast<int>(card.y) + 58,
             18,
             Color{208, 220, 235, 255});

    if (GuiButton(Rectangle{card.x + card.width - 58.0f, card.y + 20.0f, 32.0f, 32.0f}, "X")) {
        result.closeRequested = true;
    }

    const float sidePadding = 24.0f;
    const float sectionGap = 24.0f;
    const float sectionWidth = (card.width - sidePadding * 2.0f - sectionGap) / 2.0f;

    const Rectangle displaySection{
        card.x + sidePadding,
        card.y + 116.0f,
        sectionWidth,
        270.0f
    };
    const Rectangle simSection{
        displaySection.x + displaySection.width + sectionGap,
        displaySection.y,
        sectionWidth,
        270.0f
    };
    const Rectangle boundsSection{
        card.x + sidePadding,
        displaySection.y + displaySection.height + 22.0f,
        card.width - sidePadding * 2.0f,
        footer.y - (displaySection.y + displaySection.height + 40.0f)
    };

    ui_detail::drawSection(displaySection, "Display", "Window shape, frame pacing, and launch mode.");
    ui_detail::drawSection(simSection, "Simulation", "Core scale, limits, and playback speed.");
    ui_detail::drawSection(boundsSection, "Simulation Bounds", "These limits determine when aircraft leave the radar area.");

    ui_detail::drawSpinnerRow(Rectangle{displaySection.x + 20.0f, displaySection.y + 76.0f, displaySection.width - 40.0f, 40.0f},
                              "Screen Width",
                              "",
                              settings.display.screenWidth,
                              DisplayConfig::MIN_SCREEN_WIDTH,
                              DisplayConfig::MAX_SCREEN_WIDTH,
                              editState.screenWidth);
    ui_detail::drawSpinnerRow(Rectangle{displaySection.x + 20.0f, displaySection.y + 124.0f, displaySection.width - 40.0f, 40.0f},
                              "Screen Height",
                              "",
                              settings.display.screenHeight,
                              DisplayConfig::MIN_SCREEN_HEIGHT,
                              DisplayConfig::MAX_SCREEN_HEIGHT,
                              editState.screenHeight);
    ui_detail::drawSpinnerRow(Rectangle{displaySection.x + 20.0f, displaySection.y + 172.0f, displaySection.width - 40.0f, 40.0f},
                              "Target FPS",
                              "",
                              settings.display.targetFps,
                              DisplayConfig::MIN_TARGET_FPS,
                              DisplayConfig::MAX_TARGET_FPS,
                              editState.targetFps);
    ui_detail::drawModeRow(Rectangle{displaySection.x + 20.0f, displaySection.y + 220.0f, displaySection.width - 40.0f, 40.0f},
                           "Screen Mode",
                           "",
                           settings.display.screenMode);

    ui_detail::drawSpinnerRow(Rectangle{simSection.x + 20.0f, simSection.y + 76.0f, simSection.width - 40.0f, 40.0f},
                              "Max Aircraft",
                              "",
                              settings.sim.maxAircraft,
                              SimConfig::MIN_AIRCRAFT_COUNT,
                              SimConfig::MAX_AIRCRAFT_COUNT,
                              editState.maxAircraft);
    ui_detail::drawSpinnerRow(Rectangle{simSection.x + 20.0f, simSection.y + 124.0f, simSection.width - 40.0f, 40.0f},
                              "Aircraft Size",
                              "",
                              settings.sim.aircraftSize,
                              SimConfig::MIN_AIRCRAFT_SIZE,
                              SimConfig::MAX_AIRCRAFT_SIZE,
                              editState.aircraftSize);
    ui_detail::drawSliderRow(Rectangle{simSection.x + 20.0f, simSection.y + 172.0f, simSection.width - 40.0f, 46.0f},
                             "Pixels per NM",
                             "",
                             settings.sim.pixelsPerNm,
                             SimConfig::MIN_PIXELS_PER_NM,
                             SimConfig::MAX_PIXELS_PER_NM,
                             "%.1f");
    ui_detail::drawSliderRow(Rectangle{simSection.x + 20.0f, simSection.y + 220.0f, simSection.width - 40.0f, 46.0f},
                             "Simulation Speed",
                             "",
                             settings.sim.simulationSpeed,
                             SimConfig::MIN_SIMULATION_SPEED,
                             SimConfig::MAX_SIMULATION_SPEED,
                             "%.1fx");

    const float boundsInnerWidth = (boundsSection.width - 60.0f) / 2.0f;
    const Rectangle horizontalBounds{
        boundsSection.x + 20.0f,
        boundsSection.y + 72.0f,
        boundsInnerWidth,
        108.0f
    };
    const Rectangle verticalBounds{
        horizontalBounds.x + horizontalBounds.width + 20.0f,
        horizontalBounds.y,
        boundsInnerWidth,
        108.0f
    };

    DrawText("Horizontal", static_cast<int>(horizontalBounds.x), static_cast<int>(boundsSection.y) + 58, 18, ui_detail::kText);
    DrawText("Vertical", static_cast<int>(verticalBounds.x), static_cast<int>(boundsSection.y) + 58, 18, ui_detail::kText);

    ui_detail::drawSliderRow(Rectangle{horizontalBounds.x, horizontalBounds.y + 16.0f, horizontalBounds.width, 46.0f},
                             "Minimum X",
                             "",
                             settings.sim.minXNm,
                             SimConfig::MIN_MIN_X_NM,
                             SimConfig::MAX_MIN_X_NM,
                             "%.0f");
    ui_detail::drawSliderRow(Rectangle{horizontalBounds.x, horizontalBounds.y + 82.0f, horizontalBounds.width, 46.0f},
                             "Maximum X",
                             "",
                             settings.sim.maxXNm,
                             SimConfig::MIN_MAX_X_NM,
                             SimConfig::MAX_MAX_X_NM,
                             "%.0f");
    ui_detail::drawSliderRow(Rectangle{verticalBounds.x, verticalBounds.y + 16.0f, verticalBounds.width, 46.0f},
                             "Minimum Y",
                             "",
                             settings.sim.minYNm,
                             SimConfig::MIN_MIN_Y_NM,
                             SimConfig::MAX_MIN_Y_NM,
                             "%.0f");
    ui_detail::drawSliderRow(Rectangle{verticalBounds.x, verticalBounds.y + 82.0f, verticalBounds.width, 46.0f},
                             "Maximum Y",
                             "",
                             settings.sim.maxYNm,
                             SimConfig::MIN_MAX_Y_NM,
                             SimConfig::MAX_MAX_Y_NM,
                             "%.0f");

    clampPreviewSettings(settings, editState);
    // Leave changes in the preview copy until Apply is pressed in Engine.

    DrawText("Changes stay pending until you press Apply.", static_cast<int>(card.x) + 28, static_cast<int>(footer.y) + 30, 16, ui_detail::kMuted);

    if (GuiButton(Rectangle{footer.x + footer.width - 216.0f, footer.y + 20.0f, 96.0f, 34.0f}, "APPLY")) {
        Config::clampAppSettings(settings);
        resetSettingsEditState(editState);
        result.applyRequested = true;
    }
    if (GuiButton(Rectangle{footer.x + footer.width - 108.0f, footer.y + 20.0f, 84.0f, 34.0f}, "BACK")) {
        result.closeRequested = true;
    }

    return result;
}

PauseMenuAction UI::DrawPauseMenu() {
    GuiLoadStyleDefault();
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(BLACK, 0.6f));

    constexpr const char* kPausedText = "PAUSED";
    constexpr int kPausedFontSize = 60;
    const int textX = centerWidth(GetScreenWidth(), MeasureText(kPausedText, kPausedFontSize));
    const int textY = GetScreenHeight() / 2 - 150;
    ui_detail::drawShadowedText(kPausedText, textX, textY, kPausedFontSize, RAYWHITE, BLACK);

    const double buttonWidth = 200.0;
    const double buttonHeight = 50.0;
    const double buttonX = static_cast<double>(centerWidth(GetScreenWidth(), static_cast<int>(buttonWidth)));
    const double buttonY = static_cast<double>(centerHeight(GetScreenHeight(), static_cast<int>(buttonHeight)));

    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    if (GuiButton(Rectangle{static_cast<float>(buttonX), static_cast<float>(buttonY), static_cast<float>(buttonWidth), static_cast<float>(buttonHeight)}, "RESUME")) {
        return PauseMenuAction::RESUME;
    }
    if (GuiButton(Rectangle{static_cast<float>(buttonX), static_cast<float>(buttonY + buttonHeight + 20.0), static_cast<float>(buttonWidth), static_cast<float>(buttonHeight)}, "MAIN MENU")) {
        return PauseMenuAction::RETURN_TO_MENU;
    }

    return PauseMenuAction::NONE;
}

Vector2 UI::NMToPixels(Vec2 nmPos, const SimSettings& simSettings) {
    // The radar origin is the screen centre, with sim-space measured in NM.
    return Vector2{
        static_cast<float>(GetScreenWidth() / 2 + nmPos.x * simSettings.pixelsPerNm),
        static_cast<float>(GetScreenHeight() / 2 + nmPos.y * simSettings.pixelsPerNm)
    };
}

double UI::NMToPixels(double nmDistance, const SimSettings& simSettings) {
    return nmDistance * simSettings.pixelsPerNm;
}

Vec2 UI::PixelsToNM(Vector2 pixelPos, const SimSettings& simSettings) {
    return Vec2{
        static_cast<double>(pixelPos.x - GetScreenWidth() / 2) / simSettings.pixelsPerNm,
        static_cast<double>(pixelPos.y - GetScreenHeight() / 2) / simSettings.pixelsPerNm
    };
}

namespace {
SimulationViewResult drawSimulationHud(int aircraftCount,
                                       int outOfBoundsCount,
                                       int landedCount,
                                       int hullLossCount,
                                       int predictedConflictCount,
                                       double simulationSpeed,
                                       bool debugEnabled,
                                       const SpawnRequestResult& spawnResult) {
    // HUD buttons only report intent; Engine decides what those clicks do.
    SimulationViewResult result;
    GuiLoadStyleDefault();
    DrawText(TextFormat("Aircraft: %i", aircraftCount), 10, 10, 20, DARKGRAY);
    DrawText(TextFormat("Out: %i  Landed: %i  Hull: %i  Predicted: %i",
                        outOfBoundsCount,
                        landedCount,
                        hullLossCount,
                        predictedConflictCount),
             10,
             38,
             20,
             DARKGRAY);
    DrawText("P = Pause", GetScreenWidth() - 100, 10, 16, DARKGRAY);
    DrawText(TextFormat("SimSpeed: %.1f", simulationSpeed), GetScreenWidth() - 145, 40, 20, WHITE);

    if (!spawnResult.message.empty()) {
        const Color messageColor = spawnResult.success ? Color{152, 223, 115, 255} : Color{255, 190, 102, 255};
        DrawText(spawnResult.message.c_str(), 10, GetScreenHeight() - 28, 18, messageColor);
    }

    const double buttonWidth = 92.0;
    const double buttonHeight = 30.0;
    const double buttonX = static_cast<double>(GetScreenWidth()) - buttonWidth - 10.0;
    const double buttonY = static_cast<double>(GetScreenHeight()) - buttonHeight - 10.0;
    const double spawnButtonX = buttonX - buttonWidth - 10.0;

    if (GuiButton(Rectangle{static_cast<float>(spawnButtonX), static_cast<float>(buttonY), static_cast<float>(buttonWidth), static_cast<float>(buttonHeight)}, "SPAWN")) {
        result.spawnRequested = true;
    }
    if (GuiButton(Rectangle{static_cast<float>(buttonX), static_cast<float>(buttonY), static_cast<float>(buttonWidth), static_cast<float>(buttonHeight)}, debugEnabled ? "DEBUG: ON" : "DEBUG: OFF")) {
        result.toggleDebugRequested = true;
    }

    return result;
}

void drawBackground() {
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK);
}

void drawRangeRings(Vec2 airportPos, const SimSettings& simSettings) {
    constexpr float kRings[] = {5.0f, 10.0f, 20.0f, 30.0f, 40.0f};
    const Vector2 center = UI::NMToPixels(airportPos, simSettings);

    for (const float radiusNm : kRings) {
        const float pixelRadius = static_cast<float>(UI::NMToPixels(radiusNm, simSettings));
        DrawCircleLinesV(center, pixelRadius, Fade(DARKGRAY, 0.9f));
        DrawText(TextFormat("%0.f NM", radiusNm),
                 static_cast<int>(center.x) + 5,
                 static_cast<int>(center.y - pixelRadius - 15),
                 12,
                 DARKGRAY);
    }
}

void drawGuidancePreviewVisual(const GuidancePreview& preview, const SimSettings& simSettings, bool debugEnabled) {
    // Holds get their own dedicated preview because they are already a complete
    // path; the turn/vector/altitude previews are drawn separately otherwise.
    if (preview.hold.visible) {
        DrawLineEx(UI::NMToPixels(preview.hold.firstStraightStart, simSettings),
                   UI::NMToPixels(preview.hold.firstStraightEnd, simSettings),
                   2.0f,
                   ui_detail::kGuidanceYellow);
        DrawLineEx(UI::NMToPixels(preview.hold.secondStraightStart, simSettings),
                   UI::NMToPixels(preview.hold.secondStraightEnd, simSettings),
                   2.0f,
                   ui_detail::kGuidanceYellow);
        ui_detail::drawArcSegmentNm(preview.hold.firstTurnCenter,
                                    preview.hold.radiusNm,
                                    preview.hold.firstTurnStartAngleDeg,
                                    preview.hold.turnSweepAngleDeg,
                                    preview.hold.turnDirectionSign,
                                    ui_detail::kGuidanceYellow,
                                    2.0f,
                                    simSettings);
        ui_detail::drawArcSegmentNm(preview.hold.secondTurnCenter,
                                    preview.hold.radiusNm,
                                    preview.hold.secondTurnStartAngleDeg,
                                    preview.hold.turnSweepAngleDeg,
                                    preview.hold.turnDirectionSign,
                                    ui_detail::kGuidanceYellow,
                                    2.0f,
                                    simSettings);
        return;
    }

    if (preview.headingVector.visible) {
        DrawLineEx(UI::NMToPixels(preview.headingVector.start, simSettings),
                   UI::NMToPixels(preview.headingVector.end, simSettings),
                   2.0f,
                   ui_detail::kGuidanceYellow);
    }

    if (preview.turnArc.visible) {
        ui_detail::drawArcSegmentNm(preview.turnArc.center,
                                    preview.turnArc.radiusNm,
                                    preview.turnArc.startAngleDeg,
                                    preview.turnArc.sweepAngleDeg,
                                    preview.turnArc.directionSign,
                                    Fade(ui_detail::kGuidanceYellow, 0.9f),
                                    2.0f,
                                    simSettings);
    }

    if (preview.altitudeCapture.visible) {
        ui_detail::drawArcSegmentNm(preview.altitudeCapture.arcCenter,
                                    preview.altitudeCapture.radiusNm,
                                    preview.altitudeCapture.startAngleDeg,
                                    preview.altitudeCapture.sweepAngleDeg,
                                    preview.altitudeCapture.directionSign,
                                    ui_detail::kGuidanceYellow,
                                    2.5f,
                                    simSettings);

        if (debugEnabled) {
            const Vector2 markerPos = UI::NMToPixels(preview.altitudeCapture.position, simSettings);
            DrawText(TextFormat("ALT %.0fs", preview.altitudeCapture.timeSeconds),
                     static_cast<int>(markerPos.x + 8.0f),
                     static_cast<int>(markerPos.y - 12.0f),
                     12,
                     ui_detail::kGuidanceYellow);
        }
    }
}

void drawAircraftTrailVisual(const Aircraft& aircraft, const SimSettings& simSettings, bool selected) {
    const auto& trailPoints = aircraft.getTrailPoints();
    if (trailPoints.size() < 2) {
        return;
    }

    const double trailElapsedSeconds = aircraft.getTrailElapsedSeconds();
    const double oldestVisibleAge = trailElapsedSeconds - trailPoints.front().recordedAtSeconds;
    const double selectedFadeWindow = std::max(ui_detail::kVisibleTrailAgeSeconds, oldestVisibleAge);
    const float dotRadius = selected ? 2.6f : 2.0f;

    for (const auto& trailPoint : trailPoints) {
        // Fade older samples so selected aircraft show more history without
        // filling the whole radar with equally bright trail dots.
        const double ageSeconds = trailElapsedSeconds - trailPoint.recordedAtSeconds;
        if (!selected && ageSeconds > ui_detail::kVisibleTrailAgeSeconds) {
            continue;
        }

        double alpha = 0.0;
        if (selected) {
            const double normalizedAge = selectedFadeWindow > 0.0
                ? std::clamp(ageSeconds / selectedFadeWindow, 0.0, 1.0)
                : 1.0;
            alpha = ui_detail::kSelectedTrailMaxAlpha
                - (ui_detail::kSelectedTrailMaxAlpha - ui_detail::kSelectedTrailMinAlpha) * normalizedAge;
        } else {
            const double normalizedAge = std::clamp(ageSeconds / ui_detail::kVisibleTrailAgeSeconds, 0.0, 1.0);
            alpha = ui_detail::kUnselectedTrailMaxAlpha * (1.0 - normalizedAge);
        }

        if (alpha <= 0.01) {
            continue;
        }

        DrawCircleV(UI::NMToPixels(trailPoint.position, simSettings),
                    dotRadius,
                    Fade(ui_detail::kTrackTrail, static_cast<float>(alpha)));
    }
}

void drawAircraftVisual(const Aircraft& aircraft, const SimSettings& simSettings, bool selected) {
    const Color color = ui_detail::aircraftColor(aircraft, selected);

    drawAircraftTrailVisual(aircraft, simSettings, selected);

    const int size = simSettings.aircraftSize;
    const int halfSize = size / 2;
    const Vector2 pixelPos = UI::NMToPixels(aircraft.getPosition(), simSettings);

    DrawRectangleLinesEx(
        Rectangle{
            static_cast<float>(pixelPos.x - static_cast<float>(halfSize)),
            static_cast<float>(pixelPos.y - static_cast<float>(halfSize)),
            static_cast<float>(size),
            static_cast<float>(size)
        },
        1.5f,
        color
    );

    const Vector2 vectorEnd{
        static_cast<float>(pixelPos.x + std::cos((aircraft.getHeading() - 90.0) * kDegToRad) * 40.0),
        static_cast<float>(pixelPos.y + std::sin((aircraft.getHeading() - 90.0) * kDegToRad) * 40.0)
    };

    DrawLineEx(pixelPos, vectorEnd, 2.0f, color);
    DrawText(aircraft.getCallsign().c_str(), static_cast<int>(pixelPos.x + 20), static_cast<int>(pixelPos.y - 12), 14, color);

    if (selected) {
        DrawCircleLinesV(pixelPos, 25.0f, color);
    }
}

void drawAirportVisual(const Airport& airport, const SimSettings& simSettings) {
    const Vector2 pixelPos = UI::NMToPixels(airport.position, simSettings);
    const float pixelRunwayLength = static_cast<float>(UI::NMToPixels(airport.runwayLength, simSettings));

    DrawRectanglePro(Rectangle{pixelPos.x, pixelPos.y, pixelRunwayLength, 10.0f},
                     Vector2{pixelRunwayLength / 2.0f, 5.0f},
                     static_cast<float>(airport.runwayHeading - 90.0),
                     GRAY);

    // Draw the localizer as a centreline with sector boundaries so the capture
    // volume on screen matches the checks used by the backend.
    const double localizerHeading = normalizeAngle(airport.runwayHeading + 180.0);
    const Vec2 localizerDirection = ui_detail::headingDirectionNm(localizerHeading);
    const Vec2 localizerNormal = ui_detail::headingNormalNm(localizerHeading);
    const Vec2 localizerEndNm{
        airport.position.x + localizerDirection.x * airport.localizer.length,
        airport.position.y + localizerDirection.y * airport.localizer.length
    };

    DrawLineEx(pixelPos,
               UI::NMToPixels(localizerEndNm, simSettings),
               2.0f,
               Fade(ui_detail::kIlsBlue, ui_detail::kIlsCenterlineAlpha));

    const int tickCount = static_cast<int>(std::floor(airport.localizer.length));
    for (int i = 1; i <= tickCount; ++i) {
        const double distanceNm = static_cast<double>(i);
        const double halfTickLengthNm = (i % 5 == 0) ? 0.72 : 0.34;
        const Vec2 tickCenter{
            airport.position.x + localizerDirection.x * distanceNm,
            airport.position.y + localizerDirection.y * distanceNm
        };
        const Vec2 tickStart{
            tickCenter.x - localizerNormal.x * halfTickLengthNm,
            tickCenter.y - localizerNormal.y * halfTickLengthNm
        };
        const Vec2 tickEnd{
            tickCenter.x + localizerNormal.x * halfTickLengthNm,
            tickCenter.y + localizerNormal.y * halfTickLengthNm
        };

        DrawLineEx(UI::NMToPixels(tickStart, simSettings),
                   UI::NMToPixels(tickEnd, simSettings),
                   i % 5 == 0 ? 1.8f : 1.2f,
                   Fade(ui_detail::kIlsBlue, i % 5 == 0 ? ui_detail::kIlsMajorTickAlpha : ui_detail::kIlsMinorTickAlpha));
    }

    for (const auto& sector : airport.localizer.sectors) {
        const double leftHeading = normalizeAngle(localizerHeading - sector.width / 2.0);
        const double rightHeading = normalizeAngle(localizerHeading + sector.width / 2.0);
        const Vec2 leftDirection = ui_detail::headingDirectionNm(leftHeading);
        const Vec2 rightDirection = ui_detail::headingDirectionNm(rightHeading);
        const Vec2 leftEndNm{
            airport.position.x + leftDirection.x * sector.range,
            airport.position.y + leftDirection.y * sector.range
        };
        const Vec2 rightEndNm{
            airport.position.x + rightDirection.x * sector.range,
            airport.position.y + rightDirection.y * sector.range
        };

        DrawLineEx(pixelPos, UI::NMToPixels(leftEndNm, simSettings), 1.4f, Fade(ui_detail::kIlsBlue, ui_detail::kIlsBoundaryAlpha));
        DrawLineEx(pixelPos, UI::NMToPixels(rightEndNm, simSettings), 1.4f, Fade(ui_detail::kIlsBlue, ui_detail::kIlsBoundaryAlpha));
        DrawLineEx(UI::NMToPixels(leftEndNm, simSettings),
                   UI::NMToPixels(rightEndNm, simSettings),
                   1.0f,
                   Fade(ui_detail::kIlsBlue, ui_detail::kIlsSectorEdgeAlpha));
    }

    DrawText(airport.name.c_str(),
             static_cast<int>(pixelPos.x + 10.0f),
             static_cast<int>(pixelPos.y + 12.0f),
             14,
             Fade(ui_detail::kIlsBlue, ui_detail::kIlsLabelAlpha));
}
}

SimulationViewResult UI::DrawSimulation(const Simulation& sim,
                                        const AppSettings& settings,
                                        bool debugEnabled,
                                        const Aircraft* selectedAircraft) {
    // Draw order matters: static radar features first, then previews, then live
    // aircraft, then HUD/selection overlays on top.
    drawBackground();

    for (const auto& airport : sim.getAirports()) {
        drawAirportVisual(airport, settings.sim);
        drawRangeRings(airport.position, settings.sim);
    }

    if (selectedAircraft) {
        drawGuidancePreviewVisual(sim.getGuidancePreview(selectedAircraft), settings.sim, debugEnabled);
    }

    for (const auto& plane : sim.getAircraft()) {
        drawAircraftVisual(*plane, settings.sim, plane.get() == selectedAircraft);
    }

    const SimulationViewResult result = drawSimulationHud(static_cast<int>(sim.getAircraft().size()),
                                                          static_cast<int>(sim.getOutOfBoundsCount()),
                                                          static_cast<int>(sim.getLandedCount()),
                                                          static_cast<int>(sim.getHullLossCount()),
                                                          static_cast<int>(sim.getPredictedConflictCount()),
                                                          settings.sim.simulationSpeed,
                                                          debugEnabled,
                                                          sim.getLastSpawnResult());

    if (selectedAircraft) {
        drawSelectedAircraftDetails(*selectedAircraft, debugEnabled);
    } else {
        DrawText("Click aircraft to vector", 10, 70, 20, WHITE);
    }

    return result;
}
