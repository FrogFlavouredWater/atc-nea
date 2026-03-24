#include "frontend/ui.h"

// IMPORTANT: dont be stupid and remove this macro like last time,
// must be defined in exactly one .cpp file
// to generate implementation code for RayGui
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>
#include "common/utils.h"
#include <algorithm>
#include <cmath>

#ifndef DEG2RAD
#define DEG2RAD (PI / 180.0)
#endif

namespace {
constexpr Color kBackdropTop{3, 10, 20, 255};
constexpr Color kBackdropBottom{9, 33, 67, 255};
constexpr Color kOverlay{0, 0, 0, 130};
constexpr Color kCard{11, 18, 31, 255};
constexpr Color kPanel{16, 27, 44, 255};
constexpr Color kBorder{38, 58, 84, 255};
constexpr Color kHeader{7, 13, 24, 255};
constexpr Color kFooter{9, 16, 28, 255};
constexpr Color kAccent{63, 190, 255, 255};
constexpr Color kAction{30, 98, 163, 255};
constexpr Color kActionFocused{45, 126, 200, 255};
constexpr Color kActionPressed{23, 77, 128, 255};
constexpr Color kText{235, 241, 247, 255};
constexpr Color kMuted{142, 158, 179, 255};
constexpr Color kValueFill{24, 41, 63, 255};
constexpr Color kGuidanceYellow{245, 220, 66, 255};
constexpr Color kApproachGreen{102, 255, 140, 255};
constexpr Color kIlsBlue{82, 174, 255, 255};
constexpr Color kTrackTrail{42, 210, 95, 255};
constexpr float kIlsCenterlineAlpha = 0.52f;
constexpr float kIlsMajorTickAlpha = 0.48f;
constexpr float kIlsMinorTickAlpha = 0.24f;
constexpr float kIlsBoundaryAlpha = 0.24f;
constexpr float kIlsSectorEdgeAlpha = 0.12f;
constexpr float kIlsLabelAlpha = 0.72f;
constexpr double kVisibleTrailAgeSeconds = 90.0;
constexpr double kSelectedTrailMinAlpha = 0.18;
constexpr double kSelectedTrailMaxAlpha = 0.72;
constexpr double kUnselectedTrailMaxAlpha = 0.55;

const char* screenModeLabel(ScreenMode mode);
ScreenMode nextScreenMode(ScreenMode mode);

Vec2 headingDirectionNm(double headingDeg) {
    return Vec2{
        std::cos((headingDeg - 90.0) * DEG2RAD),
        std::sin((headingDeg - 90.0) * DEG2RAD)
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
    if (radiusNm <= 0.0 || sweepAngleDeg <= 0.0 || directionSign == 0) {
        return;
    }

    const int segments = std::max(12, static_cast<int>(std::ceil(sweepAngleDeg / 6.0)));
    const double angleStepDeg = sweepAngleDeg / static_cast<double>(segments);

    auto pointAtAngle = [&](double angleDeg) {
        const double angleRad = angleDeg * DEG2RAD;
        return UI::NMToPixels(
            Vec2{
                centerNm.x + std::cos(angleRad) * radiusNm,
                centerNm.y + std::sin(angleRad) * radiusNm
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
    int textWidth = MeasureText(text, 16);
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
    const int left = static_cast<int>(row.x);
    const int top = static_cast<int>(row.y + (hasSubtitle ? 0.0f : 7.0f));
    const float controlWidth = 170.0f;
    Rectangle control{
        row.x + row.width - controlWidth,
        row.y + (hasSubtitle ? 4.0f : 2.0f),
        controlWidth,
        34.0f
    };

    drawFieldText(left, top, label, subtitle);

    if (GuiSpinner(control, nullptr, &value, minValue, maxValue, editMode)) {
        editMode = !editMode;
    }
}

void drawModeRow(Rectangle row, const char* label, const char* subtitle, ScreenMode& mode) {
    const bool hasSubtitle = subtitle && subtitle[0] != '\0';
    const float buttonWidth = 190.0f;
    Rectangle control{
        row.x + row.width - buttonWidth,
        row.y + (hasSubtitle ? 4.0f : 2.0f),
        buttonWidth,
        34.0f
    };

    drawFieldText(static_cast<int>(row.x), static_cast<int>(row.y + (hasSubtitle ? 0.0f : 7.0f)), label, subtitle);

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
    drawFieldText(static_cast<int>(row.x), static_cast<int>(row.y + (hasSubtitle ? 0.0f : 4.0f)), label, subtitle);

    Rectangle valueChip{
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

void clampPendingSettings(AppSettings& settings) {
    settings.display.screenWidth = std::clamp(settings.display.screenWidth, 800, 3840);
    settings.display.screenHeight = std::clamp(settings.display.screenHeight, 600, 2160);
    settings.display.targetFps = std::clamp(settings.display.targetFps, 30, 240);
    settings.sim.maxAircraft = std::clamp(settings.sim.maxAircraft, 1, 25);
    settings.sim.aircraftSize = std::clamp(settings.sim.aircraftSize, 4, 32);

    if (settings.sim.minXNm >= settings.sim.maxXNm) {
        settings.sim.maxXNm = settings.sim.minXNm + 1.0;
    }
    if (settings.sim.minYNm >= settings.sim.maxYNm) {
        settings.sim.maxYNm = settings.sim.minYNm + 1.0;
    }
}

const char* screenModeLabel(ScreenMode mode) {
    switch (mode) {
    case ScreenMode::WINDOWED: return "Windowed";
    case ScreenMode::BORDERLESS_WINDOWED: return "Borderless";
    case ScreenMode::FULLSCREEN: return "Fullscreen";
    default: return "UNKNOWN";
    }
}

ScreenMode nextScreenMode(ScreenMode mode) {
    switch (mode) {
    case ScreenMode::WINDOWED: return ScreenMode::BORDERLESS_WINDOWED;
    case ScreenMode::BORDERLESS_WINDOWED: return ScreenMode::FULLSCREEN;
    case ScreenMode::FULLSCREEN: return ScreenMode::WINDOWED;
    default: return ScreenMode::WINDOWED;
    }
}
}

MainMenuAction UI::DrawMainMenu() {
    GuiLoadStyleDefault();
    DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK, DARKBLUE);

    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 200, Fade(DARKGRAY, 0.3f));
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 400, Fade(DARKGRAY, 0.2f));
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 600, Fade(DARKGRAY, 0.1f));

    const char* title = "ATC SIMULATOR";
    int fontSize = 60;
    int titleWidth = MeasureText(title, fontSize);
    int titleX = centerWidth(GetScreenWidth(), titleWidth);
    int titleY = 150;

    DrawText(title, titleX + 4, titleY + 4, fontSize, BLACK);
    DrawText(title, titleX, titleY, fontSize, RAYWHITE);

    const char* subtitle = "Air Traffic Control Simulation";
    int subFontSize = 20;
    int subWidth = MeasureText(subtitle, subFontSize);
    DrawText(subtitle, centerWidth(GetScreenWidth(), subWidth), titleY + 70, subFontSize, LIGHTGRAY);

    double btnWidth = 220.0;
    double btnHeight = 50.0;
    double btnX = static_cast<double>(centerWidth(GetScreenWidth(), static_cast<int>(btnWidth)));
    double btnY = static_cast<double>(GetScreenHeight()) * 0.56;

    GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(DARKGRAY));
    GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(RAYWHITE));
    GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(BLUE));
    GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(WHITE));
    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    if (GuiButton(Rectangle{ static_cast<float>(btnX), static_cast<float>(btnY), static_cast<float>(btnWidth), static_cast<float>(btnHeight) }, "START MISSION")) {
        return MainMenuAction::START;
    }

    if (GuiButton(Rectangle{ static_cast<float>(btnX), static_cast<float>(btnY + btnHeight + 20.0), static_cast<float>(btnWidth), static_cast<float>(btnHeight) }, "SETTINGS")) {
        return MainMenuAction::OPEN_SETTINGS;
    }

    if (GuiButton(Rectangle{ static_cast<float>(btnX), static_cast<float>(btnY + (btnHeight + 20.0) * 2.0), static_cast<float>(btnWidth), static_cast<float>(btnHeight) }, "EXIT")) {
        return MainMenuAction::EXIT;
    }

    DrawText("v0.0.1a", 10, GetScreenHeight() - 25, 15, DARKGRAY);
    return MainMenuAction::NONE;
}

SettingsMenuResult UI::DrawSettingsMenu(AppSettings& settings) {
    static bool editScreenWidth = false;
    static bool editScreenHeight = false;
    static bool editTargetFps = false;
    static bool editMaxAircraft = false;
    static bool editAircraftSize = false;

    SettingsMenuResult result;

    applySettingsTheme();

    DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), kBackdropTop, kBackdropBottom);
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), kOverlay);

    const float screenWidth = static_cast<float>(GetScreenWidth());
    const float screenHeight = static_cast<float>(GetScreenHeight());
    const float cardWidth = std::min(std::clamp(screenWidth * 0.74f, 980.0f, 1180.0f), screenWidth - 56.0f);
    const float cardHeight = std::min(std::clamp(screenHeight * 0.82f, 660.0f, 740.0f), screenHeight - 36.0f);

    Rectangle card{
        (screenWidth - cardWidth) / 2.0f,
        (screenHeight - cardHeight) / 2.0f,
        cardWidth,
        cardHeight
    };
    Rectangle header{ card.x, card.y, card.width, 94.0f };
    Rectangle footer{ card.x, card.y + card.height - 76.0f, card.width, 76.0f };

    drawPanel(card, kCard, kBorder);
    DrawRectangleRec(header, kHeader);
    DrawRectangleRec(footer, kFooter);
    DrawRectangle(static_cast<int>(card.x), static_cast<int>(header.y + header.height), static_cast<int>(card.width), 1, kBorder);
    DrawRectangle(static_cast<int>(card.x), static_cast<int>(footer.y), static_cast<int>(card.width), 1, kBorder);

    DrawText("Simulator Settings", static_cast<int>(card.x) + 28, static_cast<int>(card.y) + 20, 34, RAYWHITE);
    DrawText("Review changes here, then press Apply when you are ready.", static_cast<int>(card.x) + 28, static_cast<int>(card.y) + 58, 18, Color{208, 220, 235, 255});

    if (GuiButton(Rectangle{ card.x + card.width - 58.0f, card.y + 20.0f, 32.0f, 32.0f }, "X")) {
        result.closeRequested = true;
    }

    const float sidePadding = 24.0f;
    const float sectionGap = 24.0f;
    const float sectionWidth = (card.width - sidePadding * 2.0f - sectionGap) / 2.0f;
    Rectangle displaySection{
        card.x + sidePadding,
        card.y + 116.0f,
        sectionWidth,
        270.0f
    };
    Rectangle simSection{
        displaySection.x + displaySection.width + sectionGap,
        displaySection.y,
        sectionWidth,
        270.0f
    };
    Rectangle boundsSection{
        card.x + sidePadding,
        displaySection.y + displaySection.height + 22.0f,
        card.width - sidePadding * 2.0f,
        footer.y - (displaySection.y + displaySection.height + 40.0f)
    };

    drawSection(displaySection, "Display", "Window shape, frame pacing, and launch mode.");
    drawSection(simSection, "Simulation", "Core scale, limits, and playback speed.");
    drawSection(boundsSection, "Simulation Bounds", "These limits determine when aircraft leave the radar area.");

    drawSpinnerRow(Rectangle{ displaySection.x + 20.0f, displaySection.y + 76.0f, displaySection.width - 40.0f, 40.0f },
                   "Screen Width",
                   "",
                   settings.display.screenWidth,
                   800,
                   3840,
                   editScreenWidth);
    drawSpinnerRow(Rectangle{ displaySection.x + 20.0f, displaySection.y + 124.0f, displaySection.width - 40.0f, 40.0f },
                   "Screen Height",
                   "",
                   settings.display.screenHeight,
                   600,
                   2160,
                   editScreenHeight);
    drawSpinnerRow(Rectangle{ displaySection.x + 20.0f, displaySection.y + 172.0f, displaySection.width - 40.0f, 40.0f },
                   "Target FPS",
                   "",
                   settings.display.targetFps,
                   30,
                   240,
                   editTargetFps);
    drawModeRow(Rectangle{ displaySection.x + 20.0f, displaySection.y + 220.0f, displaySection.width - 40.0f, 40.0f },
                "Screen Mode",
                "",
                settings.display.screenMode);

    drawSpinnerRow(Rectangle{ simSection.x + 20.0f, simSection.y + 76.0f, simSection.width - 40.0f, 40.0f },
                   "Max Aircraft",
                   "",
                   settings.sim.maxAircraft,
                   1,
                   25,
                   editMaxAircraft);
    drawSpinnerRow(Rectangle{ simSection.x + 20.0f, simSection.y + 124.0f, simSection.width - 40.0f, 40.0f },
                   "Aircraft Size",
                   "",
                   settings.sim.aircraftSize,
                   4,
                   32,
                   editAircraftSize);
    drawSliderRow(Rectangle{ simSection.x + 20.0f, simSection.y + 172.0f, simSection.width - 40.0f, 46.0f },
                  "Pixels per NM",
                  "",
                  settings.sim.pixelsPerNm,
                  2.0,
                  20.0,
                  "%.1f");
    drawSliderRow(Rectangle{ simSection.x + 20.0f, simSection.y + 220.0f, simSection.width - 40.0f, 46.0f },
                  "Simulation Speed",
                  "",
                  settings.sim.simulationSpeed,
                  1.0,
                  120.0,
                  "%.1fx");

    const float boundsInnerWidth = (boundsSection.width - 60.0f) / 2.0f;
    Rectangle horizontalBounds{
        boundsSection.x + 20.0f,
        boundsSection.y + 72.0f,
        boundsInnerWidth,
        108.0f
    };
    Rectangle verticalBounds{
        horizontalBounds.x + horizontalBounds.width + 20.0f,
        horizontalBounds.y,
        boundsInnerWidth,
        108.0f
    };

    DrawText("Horizontal", static_cast<int>(horizontalBounds.x), static_cast<int>(boundsSection.y) + 58, 18, kText);
    DrawText("Vertical", static_cast<int>(verticalBounds.x), static_cast<int>(boundsSection.y) + 58, 18, kText);

    drawSliderRow(Rectangle{ horizontalBounds.x, horizontalBounds.y + 16.0f, horizontalBounds.width, 46.0f },
                  "Minimum X",
                  "",
                  settings.sim.minXNm,
                  -200.0,
                  -10.0,
                  "%.0f");
    drawSliderRow(Rectangle{ horizontalBounds.x, horizontalBounds.y + 82.0f, horizontalBounds.width, 46.0f },
                  "Maximum X",
                  "",
                  settings.sim.maxXNm,
                  10.0,
                  200.0,
                  "%.0f");
    drawSliderRow(Rectangle{ verticalBounds.x, verticalBounds.y + 16.0f, verticalBounds.width, 46.0f },
                  "Minimum Y",
                  "",
                  settings.sim.minYNm,
                  -150.0,
                  -10.0,
                  "%.0f");
    drawSliderRow(Rectangle{ verticalBounds.x, verticalBounds.y + 82.0f, verticalBounds.width, 46.0f },
                  "Maximum Y",
                  "",
                  settings.sim.maxYNm,
                  10.0,
                  150.0,
                  "%.0f");

    if (!editScreenWidth) {
        settings.display.screenWidth = std::clamp(settings.display.screenWidth, 800, 3840);
    }
    if (!editScreenHeight) {
        settings.display.screenHeight = std::clamp(settings.display.screenHeight, 600, 2160);
    }
    if (!editTargetFps) {
        settings.display.targetFps = std::clamp(settings.display.targetFps, 30, 240);
    }
    if (!editMaxAircraft) {
        settings.sim.maxAircraft = std::clamp(settings.sim.maxAircraft, 1, 25);
    }
    if (!editAircraftSize) {
        settings.sim.aircraftSize = std::clamp(settings.sim.aircraftSize, 4, 32);
    }

    if (settings.sim.minXNm >= settings.sim.maxXNm) {
        settings.sim.maxXNm = settings.sim.minXNm + 1.0;
    }
    if (settings.sim.minYNm >= settings.sim.maxYNm) {
        settings.sim.maxYNm = settings.sim.minYNm + 1.0;
    }

    DrawText("Changes stay pending until you press Apply.", static_cast<int>(card.x) + 28, static_cast<int>(footer.y) + 30, 16, kMuted);

    if (GuiButton(Rectangle{ footer.x + footer.width - 216.0f, footer.y + 20.0f, 96.0f, 34.0f }, "APPLY")) {
        clampPendingSettings(settings);
        editScreenWidth = false;
        editScreenHeight = false;
        editTargetFps = false;
        editMaxAircraft = false;
        editAircraftSize = false;
        result.applyRequested = true;
    }
    if (GuiButton(Rectangle{ footer.x + footer.width - 108.0f, footer.y + 20.0f, 84.0f, 34.0f }, "BACK")) {
        result.closeRequested = true;
    }

    return result;
}

SimulationViewResult UI::DrawSimulationHUD(int aircraftCount,
                                           int outOfBoundsCount,
                                           int landedCount,
                                           double simulationSpeed,
                                           bool& debugEnabled,
                                           const SpawnRequestResult& spawnResult) {
    SimulationViewResult result;
    GuiLoadStyleDefault();
    DrawText(TextFormat("Aircraft: %i", aircraftCount), 10, 10, 20, DARKGRAY);
    DrawText(TextFormat("Out: %i  Landed: %i", outOfBoundsCount, landedCount), 10, 38, 20, DARKGRAY);
    DrawText("P = Pause", GetScreenWidth() - 100, 10, 16, DARKGRAY);
    DrawText(TextFormat("SimSpeed: %.1f", simulationSpeed), GetScreenWidth() - 145, 40, 20, WHITE);

    if (!spawnResult.message.empty()) {
        const Color messageColor = spawnResult.success ? Color{152, 223, 115, 255} : Color{255, 190, 102, 255};
        DrawText(spawnResult.message.c_str(), 10, GetScreenHeight() - 28, 18, messageColor);
    }

    double btnWidth = 92.0;
    double btnHeight = 30.0;
    double btnX = static_cast<double>(GetScreenWidth()) - btnWidth - 10.0;
    double btnY = static_cast<double>(GetScreenHeight()) - btnHeight - 10.0;
    double spawnBtnX = btnX - btnWidth - 10.0;

    if (GuiButton(Rectangle{ static_cast<float>(spawnBtnX), static_cast<float>(btnY), static_cast<float>(btnWidth), static_cast<float>(btnHeight) }, "SPAWN")) {
        result.spawnRequested = true;
    }

    if (GuiButton(Rectangle{ static_cast<float>(btnX), static_cast<float>(btnY), static_cast<float>(btnWidth), static_cast<float>(btnHeight) }, debugEnabled ? "DEBUG: ON" : "DEBUG: OFF")) {
        debugEnabled = !debugEnabled;
    }

    return result;
}

void UI::DrawBackground() {
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK);
}

void UI::DrawRangeRings(Vec2 airportPos, const SimSettings& simSettings) {
    float rings[] = { 5.0f, 10.0f, 20.0f, 30.0f, 40.0f };

    for (float radiusNm : rings) {
        Vector2 center = NMToPixels(airportPos, simSettings);
        float pixelRadius = static_cast<float>(NMToPixels(radiusNm, simSettings));

        DrawCircleLinesV(center, pixelRadius, Fade(DARKGRAY, 0.9f));
        DrawText(TextFormat("%0.f NM", radiusNm), static_cast<int>(center.x) + 5, static_cast<int>(center.y - pixelRadius - 15), 12, DARKGRAY);
    }
}

void UI::DrawPauseMenu(GameState& currentState) {
    GuiLoadStyleDefault();
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(BLACK, 0.6f));

    const char* text = "PAUSED";
    int fontSize = 60;
    int textWidth = MeasureText(text, fontSize);
    int textX = centerWidth(GetScreenWidth(), textWidth);
    int textY = GetScreenHeight() / 2 - 150;

    DrawText(text, textX + 4, textY + 4, fontSize, BLACK);
    DrawText(text, textX, textY, fontSize, RAYWHITE);

    double btnWidth = 200.0;
    double btnHeight = 50.0;
    double btnX = static_cast<double>(centerWidth(GetScreenWidth(), static_cast<int>(btnWidth)));
    double btnY = static_cast<double>(centerHeight(GetScreenHeight(), static_cast<int>(btnHeight)));

    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    if (GuiButton(Rectangle{ static_cast<float>(btnX), static_cast<float>(btnY), static_cast<float>(btnWidth), static_cast<float>(btnHeight) }, "RESUME")) {
        currentState = GameState::RUNNING;
    }

    if (GuiButton(Rectangle{ static_cast<float>(btnX), static_cast<float>(btnY + btnHeight + 20.0), static_cast<float>(btnWidth), static_cast<float>(btnHeight) }, "MAIN MENU")) {
        currentState = GameState::MENU;
    }
}

Vector2 UI::NMToPixels(Vec2 nmPos, const SimSettings& simSettings) {
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

SimulationViewResult UI::DrawSimulation(const Simulation& sim,
                                        const AppSettings& settings,
                                        bool& debugEnabled,
                                        Aircraft* selectedAircraft) {
    DrawBackground();

    for (const auto& airport : sim.getAirports()) {
        DrawAirport(airport, settings.sim);
        DrawRangeRings(airport.position, settings.sim);
    }

    if (selectedAircraft) {
        DrawGuidancePreview(sim.getGuidancePreview(selectedAircraft), settings.sim, debugEnabled);
    }

    for (const auto& plane : sim.getAircraft()) {
        DrawAircraft(*plane, settings.sim, plane.get() == selectedAircraft);
    }

    const SimulationViewResult result = DrawSimulationHUD(static_cast<int>(sim.getAircraft().size()),
                                                          static_cast<int>(sim.getOutOfBoundsCount()),
                                                          static_cast<int>(sim.getLandedCount()),
                                                          settings.sim.simulationSpeed,
                                                          debugEnabled,
                                                          sim.getLastSpawnResult());

    if (selectedAircraft) {
        DrawText("Selected: ", 10, 70, 20, WHITE);
        Color selectedColor = YELLOW;
        if (selectedAircraft->getControlMode() == AircraftControlMode::ILS) {
            selectedColor = kIlsBlue;
        } else if (selectedAircraft->hasApproachClearance()) {
            selectedColor = kApproachGreen;
        }
        DrawText(selectedAircraft->getCallsign().c_str(),
                 110,
                 70,
                 20,
                 selectedColor);
        DrawText("Controls: A/D heading, W/S speed, Q/E altitude, I ILS clr, ,/. sim speed", 10, 95, 16, GRAY);

        if (debugEnabled) {
            int startY = 130;
            DrawText("--- DEBUG DATA ---", 10, startY, 16, GREEN);
            DrawText(TextFormat("Pos: %.2f, %.2f", selectedAircraft->getPosition().x, selectedAircraft->getPosition().y), 10, startY + 20, 16, GREEN);
            DrawText(TextFormat("Heading: %.2f (Target: %.2f)", selectedAircraft->getHeading(), selectedAircraft->getTargetHeading()), 10, startY + 40, 16, GREEN);
            DrawText(TextFormat("Speed: %.0f kts (Target: %.0f kts)", selectedAircraft->getSpeed(), selectedAircraft->getTargetSpeed()), 10, startY + 60, 16, GREEN);
            DrawText(TextFormat("Altitude: %i (Target: %i)", selectedAircraft->getAltitude(), selectedAircraft->getTargetAltitude()), 10, startY + 80, 16, GREEN);
            DrawText(TextFormat("Vertical Speed: %+.0f fpm", selectedAircraft->getVerticalSpeedFpm()), 10, startY + 100, 16, GREEN);
            DrawText(TextFormat("Turn Rate: %.2f deg/s", selectedAircraft->getTurnRateDegPerSec()), 10, startY + 120, 16, GREEN);
            DrawText(TextFormat("Turn Radius: %.2f NM", selectedAircraft->getTurnRadiusNm()), 10, startY + 140, 16, GREEN);
            DrawText(TextFormat("Phase: %s", toString(selectedAircraft->getPhase())), 10, startY + 160, 16, GREEN);
            DrawText(TextFormat("Control: %s", toString(selectedAircraft->getControlMode())), 10, startY + 180, 16, GREEN);
            DrawText(TextFormat("Conflict: %s", selectedAircraft->hasConflictAlert() ? "YES" : "NO"), 10, startY + 200, 16, GREEN);
            DrawText(TextFormat("Approach Clearance: %s", selectedAircraft->hasApproachClearance() ? "CLEARED" : "NO"), 10, startY + 220, 16, GREEN);
            DrawText(TextFormat("ILS Airport Index: %d", selectedAircraft->getAssignedIlsAirportIndex()), 10, startY + 240, 16, GREEN);
        }
    } else {
        DrawText("Click aircraft to vector", 10, 70, 20, WHITE);
    }

    return result;
}

void UI::DrawGuidancePreview(const GuidancePreview& preview, const SimSettings& simSettings, bool debugEnabled) {
    if (preview.headingVector.visible) {
        DrawLineEx(
            NMToPixels(preview.headingVector.start, simSettings),
            NMToPixels(preview.headingVector.end, simSettings),
            2.0f,
            kGuidanceYellow
        );
    }

    if (preview.turnArc.visible) {
        drawArcSegmentNm(preview.turnArc.center,
                         preview.turnArc.radiusNm,
                         preview.turnArc.startAngleDeg,
                         preview.turnArc.sweepAngleDeg,
                         preview.turnArc.directionSign,
                         Fade(kGuidanceYellow, 0.9f),
                         2.0f,
                         simSettings);
    }

    if (preview.altitudeCapture.visible) {
        drawArcSegmentNm(preview.altitudeCapture.arcCenter,
                         preview.altitudeCapture.radiusNm,
                         preview.altitudeCapture.startAngleDeg,
                         preview.altitudeCapture.sweepAngleDeg,
                         preview.altitudeCapture.directionSign,
                         kGuidanceYellow,
                         2.5f,
                         simSettings);

        if (debugEnabled) {
            const Vector2 markerPos = NMToPixels(preview.altitudeCapture.position, simSettings);
            DrawText(TextFormat("ALT %.0fs", preview.altitudeCapture.timeSeconds),
                     static_cast<int>(markerPos.x + 8.0f),
                     static_cast<int>(markerPos.y - 12.0f),
                     12,
                     kGuidanceYellow);
        }
    }
}

void UI::DrawAircraft(const Aircraft& aircraft, const SimSettings& simSettings, bool selected) {
    auto aircraftColor = WHITE;
    if (aircraft.getControlMode() == AircraftControlMode::ILS) {
        aircraftColor = kIlsBlue;
    } else if (aircraft.hasApproachClearance()) {
        aircraftColor = kApproachGreen;
    } else if (selected) {
        aircraftColor = YELLOW;
    }
    if (aircraft.hasConflictAlert()) {
        aircraftColor = RED;
    }

    DrawAircraftTrail(aircraft, simSettings, selected);

    const int size = simSettings.aircraftSize;
    const int halfSize = size / 2;
    Vector2 pixelPos = NMToPixels(aircraft.getPosition(), simSettings);

    DrawRectangleLinesEx(
        Rectangle{
            static_cast<float>(pixelPos.x - static_cast<float>(halfSize)),
            static_cast<float>(pixelPos.y - static_cast<float>(halfSize)),
            static_cast<float>(size),
            static_cast<float>(size)
        },
        1.5f,
        aircraftColor
    );

    const double vectorLength = 40.0;
    Vector2 vectorEnd = {
        static_cast<float>(pixelPos.x + cos(DEG2RAD * (aircraft.getHeading() - 90.0)) * vectorLength),
        static_cast<float>(pixelPos.y + sin(DEG2RAD * (aircraft.getHeading() - 90.0)) * vectorLength)
    };

    DrawLineEx(pixelPos, vectorEnd, 2.0f, aircraftColor);
    DrawText(aircraft.getCallsign().c_str(), static_cast<int>(pixelPos.x + 20), static_cast<int>(pixelPos.y - 12), 14, aircraftColor);

    if (selected) {
        DrawCircleLinesV(pixelPos, 25.0f, aircraftColor);
    }
}

void UI::DrawAircraftTrail(const Aircraft& aircraft, const SimSettings& simSettings, bool selected) {
    const auto& trailPoints = aircraft.getTrailPoints();
    if (trailPoints.size() < 2) {
        return;
    }

    const double trailElapsedSeconds = aircraft.getTrailElapsedSeconds();
    const double oldestVisibleAge = trailElapsedSeconds - trailPoints.front().recordedAtSeconds;
    const double selectedFadeWindow = std::max(kVisibleTrailAgeSeconds, oldestVisibleAge);
    const float dotRadius = selected ? 2.6f : 2.0f;

    for (const auto& trailPoint : trailPoints) {
        const double ageSeconds = trailElapsedSeconds - trailPoint.recordedAtSeconds;
        if (!selected && ageSeconds > kVisibleTrailAgeSeconds) {
            continue;
        }

        double alpha = 0.0;
        if (selected) {
            const double normalizedAge = selectedFadeWindow > 0.0
                ? std::clamp(ageSeconds / selectedFadeWindow, 0.0, 1.0)
                : 1.0;
            alpha = kSelectedTrailMaxAlpha - (kSelectedTrailMaxAlpha - kSelectedTrailMinAlpha) * normalizedAge;
        } else {
            const double normalizedAge = std::clamp(ageSeconds / kVisibleTrailAgeSeconds, 0.0, 1.0);
            alpha = kUnselectedTrailMaxAlpha * (1.0 - normalizedAge);
        }

        if (alpha <= 0.01) {
            continue;
        }

        DrawCircleV(NMToPixels(trailPoint.position, simSettings),
                    dotRadius,
                    Fade(kTrackTrail, static_cast<float>(alpha)));
    }
}

void UI::DrawAirport(const Airport& airport, const SimSettings& simSettings) {
    Vector2 pixelPos = NMToPixels(airport.position, simSettings);
    float pixelRunwayLength = static_cast<float>(NMToPixels(airport.runwayLength, simSettings));

    Rectangle rec = { pixelPos.x, pixelPos.y, pixelRunwayLength, 10.0f };
    Vector2 origin = { pixelRunwayLength / 2.0f, 5.0f };
    DrawRectanglePro(rec, origin, static_cast<float>(airport.runwayHeading - 90.0), GRAY);

    const double localizerHeading = normalizeAngle(airport.runwayHeading + 180.0);
    const Vec2 localizerDirection = headingDirectionNm(localizerHeading);
    const Vec2 localizerNormal = headingNormalNm(localizerHeading);
    const Vec2 localizerEndNm{
        airport.position.x + localizerDirection.x * airport.localizer.length,
        airport.position.y + localizerDirection.y * airport.localizer.length
    };

    DrawLineEx(pixelPos, NMToPixels(localizerEndNm, simSettings), 2.0f, Fade(kIlsBlue, kIlsCenterlineAlpha));

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
        DrawLineEx(NMToPixels(tickStart, simSettings),
                   NMToPixels(tickEnd, simSettings),
                   i % 5 == 0 ? 1.8f : 1.2f,
                   Fade(kIlsBlue, i % 5 == 0 ? kIlsMajorTickAlpha : kIlsMinorTickAlpha));
    }

    for (const auto& sector : airport.localizer.sectors) {
        const double leftHeading = normalizeAngle(localizerHeading - sector.width / 2.0);
        const double rightHeading = normalizeAngle(localizerHeading + sector.width / 2.0);
        const Vec2 leftDirection = headingDirectionNm(leftHeading);
        const Vec2 rightDirection = headingDirectionNm(rightHeading);
        const Vec2 leftEndNm{
            airport.position.x + leftDirection.x * sector.range,
            airport.position.y + leftDirection.y * sector.range
        };
        const Vec2 rightEndNm{
            airport.position.x + rightDirection.x * sector.range,
            airport.position.y + rightDirection.y * sector.range
        };

        DrawLineEx(pixelPos, NMToPixels(leftEndNm, simSettings), 1.4f, Fade(kIlsBlue, kIlsBoundaryAlpha));
        DrawLineEx(pixelPos, NMToPixels(rightEndNm, simSettings), 1.4f, Fade(kIlsBlue, kIlsBoundaryAlpha));
        DrawLineEx(NMToPixels(leftEndNm, simSettings),
                   NMToPixels(rightEndNm, simSettings),
                   1.0f,
                   Fade(kIlsBlue, kIlsSectorEdgeAlpha));
    }

    DrawText(airport.name.c_str(),
             static_cast<int>(pixelPos.x + 10.0f),
             static_cast<int>(pixelPos.y + 12.0f),
             14,
             Fade(kIlsBlue, kIlsLabelAlpha));
}
