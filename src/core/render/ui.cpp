#include "ui.h"

// IMPORTANT: dont be stupid and remove this macro like last time,
// must be defined in exactly one .cpp file
// to generate  implementation code for RayGui
#define RAYGUI_IMPLEMENTATION 
#include <raygui.h> 
#include "constants/constants.h"
#include "util/utils.h"

void UI::DrawMainMenu(GameState& currentState) {
    // Draw Title
    const char* title = "ATC SIMULATOR";
    int fontSize = 40;
    int titleWidth = MeasureText(title, fontSize);
    DrawText(title, centerWidth(CONSTANTS.display.SCREEN_WIDTH, titleWidth), 100, fontSize, DARKBLUE);

    // Draw Start Button
    // Use Rectangle{...} instead of (Rectangle){...} for C++ compliance
    int btnWidth = 120;
    int btnHeight = 40;
    // Using centerWidth and centerHeight for perfect centering
    float btnX = (float)centerWidth(CONSTANTS.display.SCREEN_WIDTH, btnWidth);
    float btnY = (float)centerHeight(CONSTANTS.display.SCREEN_HEIGHT, btnHeight);
    
    if (GuiButton(Rectangle{ btnX, btnY, (float)btnWidth, (float)btnHeight }, "START GAME")) {
        currentState = GameState::RUNNING;
    }

    // Draw Quit Button
    if (GuiButton(Rectangle{ btnX, 360, (float)btnWidth, (float)btnHeight }, "EXIT")) {
        currentState = GameState::EXIT;
    }
}

void UI::DrawGameHUD(int aircraftCount) {
    DrawText(TextFormat("Aircraft: %i", aircraftCount), 10, 10, 20, DARKGRAY);
}

void UI::DrawBackground()
{
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK);
}

void UI::DrawPauseMenu(GameState& currentState) {
    DrawText("PAUSED", 350, 200, 40, GRAY);

    if (GuiButton(Rectangle{ 350, 300, 120, 40 }, "RESUME")) {
        currentState = GameState::RUNNING;
    }
}