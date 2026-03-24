#include "frontend/ui_internal.h"

#include <raygui.h>
#include <algorithm>

namespace {
struct SettingsEditState {
    bool screenWidth = false;
    bool screenHeight = false;
    bool targetFps = false;
    bool maxAircraft = false;
    bool aircraftSize = false;
};

SettingsEditState& settingsEditState() {
    static SettingsEditState state;
    return state;
}

void clampPreviewSettings(AppSettings& settings, const SettingsEditState& state) {
    if (!state.screenWidth) {
        settings.display.screenWidth = std::clamp(settings.display.screenWidth, 800, 3840);
    }
    if (!state.screenHeight) {
        settings.display.screenHeight = std::clamp(settings.display.screenHeight, 600, 2160);
    }
    if (!state.targetFps) {
        settings.display.targetFps = std::clamp(settings.display.targetFps, 30, 240);
    }
    if (!state.maxAircraft) {
        settings.sim.maxAircraft = std::clamp(settings.sim.maxAircraft, 1, 25);
    }
    if (!state.aircraftSize) {
        settings.sim.aircraftSize = std::clamp(settings.sim.aircraftSize, 4, 32);
    }

    if (settings.sim.minXNm >= settings.sim.maxXNm) {
        settings.sim.maxXNm = settings.sim.minXNm + 1.0;
    }
    if (settings.sim.minYNm >= settings.sim.maxYNm) {
        settings.sim.maxYNm = settings.sim.minYNm + 1.0;
    }
}

void reset(SettingsEditState& state) {
    state = {};
}
} // namespace

MainMenuAction UI::DrawMainMenu() {
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
                              800,
                              3840,
                              editState.screenWidth);
    ui_detail::drawSpinnerRow(Rectangle{displaySection.x + 20.0f, displaySection.y + 124.0f, displaySection.width - 40.0f, 40.0f},
                              "Screen Height",
                              "",
                              settings.display.screenHeight,
                              600,
                              2160,
                              editState.screenHeight);
    ui_detail::drawSpinnerRow(Rectangle{displaySection.x + 20.0f, displaySection.y + 172.0f, displaySection.width - 40.0f, 40.0f},
                              "Target FPS",
                              "",
                              settings.display.targetFps,
                              30,
                              240,
                              editState.targetFps);
    ui_detail::drawModeRow(Rectangle{displaySection.x + 20.0f, displaySection.y + 220.0f, displaySection.width - 40.0f, 40.0f},
                           "Screen Mode",
                           "",
                           settings.display.screenMode);

    ui_detail::drawSpinnerRow(Rectangle{simSection.x + 20.0f, simSection.y + 76.0f, simSection.width - 40.0f, 40.0f},
                              "Max Aircraft",
                              "",
                              settings.sim.maxAircraft,
                              1,
                              25,
                              editState.maxAircraft);
    ui_detail::drawSpinnerRow(Rectangle{simSection.x + 20.0f, simSection.y + 124.0f, simSection.width - 40.0f, 40.0f},
                              "Aircraft Size",
                              "",
                              settings.sim.aircraftSize,
                              4,
                              32,
                              editState.aircraftSize);
    ui_detail::drawSliderRow(Rectangle{simSection.x + 20.0f, simSection.y + 172.0f, simSection.width - 40.0f, 46.0f},
                             "Pixels per NM",
                             "",
                             settings.sim.pixelsPerNm,
                             2.0,
                             20.0,
                             "%.1f");
    ui_detail::drawSliderRow(Rectangle{simSection.x + 20.0f, simSection.y + 220.0f, simSection.width - 40.0f, 46.0f},
                             "Simulation Speed",
                             "",
                             settings.sim.simulationSpeed,
                             1.0,
                             120.0,
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
                             -200.0,
                             -10.0,
                             "%.0f");
    ui_detail::drawSliderRow(Rectangle{horizontalBounds.x, horizontalBounds.y + 82.0f, horizontalBounds.width, 46.0f},
                             "Maximum X",
                             "",
                             settings.sim.maxXNm,
                             10.0,
                             200.0,
                             "%.0f");
    ui_detail::drawSliderRow(Rectangle{verticalBounds.x, verticalBounds.y + 16.0f, verticalBounds.width, 46.0f},
                             "Minimum Y",
                             "",
                             settings.sim.minYNm,
                             -150.0,
                             -10.0,
                             "%.0f");
    ui_detail::drawSliderRow(Rectangle{verticalBounds.x, verticalBounds.y + 82.0f, verticalBounds.width, 46.0f},
                             "Maximum Y",
                             "",
                             settings.sim.maxYNm,
                             10.0,
                             150.0,
                             "%.0f");

    clampPreviewSettings(settings, editState);

    DrawText("Changes stay pending until you press Apply.", static_cast<int>(card.x) + 28, static_cast<int>(footer.y) + 30, 16, ui_detail::kMuted);

    if (GuiButton(Rectangle{footer.x + footer.width - 216.0f, footer.y + 20.0f, 96.0f, 34.0f}, "APPLY")) {
        ui_detail::clampPendingSettings(settings);
        reset(editState);
        result.applyRequested = true;
    }
    if (GuiButton(Rectangle{footer.x + footer.width - 108.0f, footer.y + 20.0f, 84.0f, 34.0f}, "BACK")) {
        result.closeRequested = true;
    }

    return result;
}

void UI::DrawPauseMenu(GameState& currentState) {
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
        currentState = GameState::RUNNING;
    }
    if (GuiButton(Rectangle{static_cast<float>(buttonX), static_cast<float>(buttonY + buttonHeight + 20.0), static_cast<float>(buttonWidth), static_cast<float>(buttonHeight)}, "MAIN MENU")) {
        currentState = GameState::MENU;
    }
}
