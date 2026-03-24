#include "frontend/ui_internal.h"

#define RAYGUI_IMPLEMENTATION
#include <raygui.h>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
constexpr double kDegToRad = std::numbers::pi_v<double> / 180.0;
}

namespace ui_detail {
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
