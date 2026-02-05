#include "ui.h"

// IMPORTANT: dont be stupid and remove this macro like last time,
// must be defined in exactly one .cpp file
// to generate implementation code for RayGui
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>
#include "constants/constants.h"
#include "util/utils.h"

void UI::DrawMainMenu(GameState& currentState) {
    //background
    DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK, DARKBLUE);

    //radar range circles in background
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 200, Fade(DARKGRAY, 0.3));
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 400, Fade(DARKGRAY, 0.2));
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 600, Fade(DARKGRAY, 0.1));

    //title + shadow
    const char* title = "ATC SIMULATOR";
    int fontSize = 60;
    int titleWidth = MeasureText(title, fontSize);
    int titleX = centerWidth(GetScreenWidth(), titleWidth);
    int titleY = 150;

    DrawText(title, titleX + 4, titleY + 4, fontSize, BLACK);
    DrawText(title, titleX, titleY, fontSize, RAYWHITE);

    //subtitle
    const char* subtitle = "Air Traffic Control Simulation";
    int subFontSize = 20;
    int subWidth = MeasureText(subtitle, subFontSize);
    DrawText(subtitle, centerWidth(GetScreenWidth(), subWidth), titleY + 70, subFontSize, LIGHTGRAY);

    //button crap
    float btnWidth = 200.0f;
    float btnHeight = 50.0f;
    float btnX = static_cast<float>(centerWidth(GetScreenWidth(), static_cast<int>(btnWidth)));
    float btnY = static_cast<float>(GetScreenHeight()) * 0.6f;

    //GUI flags
    GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(DARKGRAY));
    GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(RAYWHITE));
    GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(BLUE));
    GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(WHITE));
    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    if (GuiButton(Rectangle{ btnX, btnY, btnWidth, btnHeight }, "START MISSION")) {
        currentState = GameState::RUNNING;
    }

    if (GuiButton(Rectangle{ btnX, btnY + btnHeight + 20.0f, btnWidth, btnHeight }, "EXIT")) {
        currentState = GameState::EXIT;
    }

    //version info
    DrawText("v0.0.1a", 10, GetScreenHeight() - 25, 15, DARKGRAY);
}

void UI::DrawSimulationHUD(int aircraftCount, bool& debugEnabled) {
    DrawText(TextFormat("Aircraft: %i", aircraftCount), 10, 10, 20, DARKGRAY);
    DrawText("P = Pause", GetScreenWidth() - 100, 10, 16, DARKGRAY);

    // Debug toggle button
    float btnWidth = 80.0f;
    float btnHeight = 30.0f;
    float btnX = static_cast<float>(GetScreenWidth()) - btnWidth - 10.0f;
    float btnY = static_cast<float>(GetScreenHeight()) - btnHeight - 10.0f;

    if (GuiButton(Rectangle{ btnX, btnY, btnWidth, btnHeight }, debugEnabled ? "DEBUG: ON" : "DEBUG: OFF")) {
        debugEnabled = !debugEnabled;
    }
}

void UI::DrawBackground()
{
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK);
}

void UI::DrawPauseMenu(GameState& currentState) {
    //semitransparent overlay (absolutely cooked with this one)
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(BLACK, 0.6));

    const char* text = "PAUSED";
    int fontSize = 60;
    int textWidth = MeasureText(text, fontSize);
    int textX = centerWidth(GetScreenWidth(), textWidth);
    int textY = GetScreenHeight() / 2 - 150;

    DrawText(text, textX + 4, textY + 4, fontSize, BLACK);
    DrawText(text, textX, textY, fontSize, RAYWHITE);

    float btnWidth = 200.0f;
    float btnHeight = 50.0f;
    float btnX = static_cast<float>(centerWidth(GetScreenWidth(), static_cast<int>(btnWidth)));
    float btnY = static_cast<float>(centerHeight(GetScreenHeight(), static_cast<int>(btnHeight)));

    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    if (GuiButton(Rectangle{ btnX, btnY, btnWidth, btnHeight }, "RESUME")) {
        currentState = GameState::RUNNING;
    }

    if (GuiButton(Rectangle{ btnX, btnY + btnHeight + 20.0f, btnWidth, btnHeight }, "MAIN MENU")) {
        currentState = GameState::MENU;
    }
}