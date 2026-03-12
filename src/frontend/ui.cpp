#include "frontend/ui.h"

// IMPORTANT: dont be stupid and remove this macro like last time,
// must be defined in exactly one .cpp file
// to generate implementation code for RayGui
#define RAYGUI_IMPLEMENTATION
#include <raygui.h>
#include "common/constants.h"
#include "common/utils.h"
#include <cmath>

#ifndef DEG2RAD
#define DEG2RAD (PI / 180.0)
#endif

void UI::DrawMainMenu(GameState& currentState) {
    //background
    DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK, DARKBLUE);

    //radar range circles in background
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 200, Fade(DARKGRAY, 0.3f)); // Input must take float
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 400, Fade(DARKGRAY, 0.2f)); // Input must take float
    DrawCircleLines(GetScreenWidth()/2, GetScreenHeight()/2, 600, Fade(DARKGRAY, 0.1f)); // Input must take float

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
    double btnWidth = 200.0;
    double btnHeight = 50.0;
    double btnX = (double)centerWidth(GetScreenWidth(), (int)btnWidth);
    double btnY = (double)GetScreenHeight() * 0.6;

    //GUI flags
    GuiSetStyle(BUTTON, BASE_COLOR_NORMAL, ColorToInt(DARKGRAY));
    GuiSetStyle(BUTTON, TEXT_COLOR_NORMAL, ColorToInt(RAYWHITE));
    GuiSetStyle(BUTTON, BASE_COLOR_FOCUSED, ColorToInt(BLUE));
    GuiSetStyle(BUTTON, TEXT_COLOR_FOCUSED, ColorToInt(WHITE));
    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    if (GuiButton(Rectangle{ (float)btnX, (float)btnY, (float)btnWidth, (float)btnHeight }, "START MISSION")) { // Input must take float
        currentState = GameState::RUNNING;
    }

    if (GuiButton(Rectangle{ (float)btnX, (float)(btnY + btnHeight + 20.0), (float)btnWidth, (float)btnHeight }, "EXIT")) { // Input must take float
        currentState = GameState::EXIT;
    }

    //version info
    DrawText("v0.0.1a", 10, GetScreenHeight() - 25, 15, DARKGRAY);
}

void UI::DrawSimulationHUD(int aircraftCount, bool& debugEnabled) {
    DrawText(TextFormat("Aircraft: %i", aircraftCount), 10, 10, 20, DARKGRAY);
    DrawText("P = Pause", GetScreenWidth() - 100, 10, 16, DARKGRAY);
    DrawText(TextFormat("SimSpeed: %.0f", CONSTANTS.game.SIMULATION_SPEED), GetScreenWidth() - 130, 40, 20, WHITE);
    // Debug toggle button
    double btnWidth = 80.0;
    double btnHeight = 30.0;
    double btnX = (double)GetScreenWidth() - btnWidth - 10.0;
    double btnY = (double)GetScreenHeight() - btnHeight - 10.0;

    if (GuiButton(Rectangle{ (float)btnX, (float)btnY, (float)btnWidth, (float)btnHeight }, debugEnabled ? "DEBUG: ON" : "DEBUG: OFF")) { // Input must take float
        debugEnabled = !debugEnabled;
    }
}

void UI::DrawBackground()
{
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK);
}

void UI::DrawRangeRings(Vec2 airportPos)
{
    // Standard approach distances
    float rings[] = { 5.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f };

    for (float radiusNm : rings) {
        Vector2 center = NMToPixels(airportPos);
        float pixelRadius = (float)NMToPixels(radiusNm);

        // Draw a faint dashed or solid circle
        DrawCircleLinesV(center, pixelRadius, Fade(DARKGRAY, 0.9f));

        // Label the ring (optional)
        DrawText(TextFormat("%0.f NM", radiusNm), (int)center.x + 5, (int)(center.y - pixelRadius - 15), 12, DARKGRAY);
    }
}

void UI::DrawPauseMenu(GameState& currentState) {
    //semitransparent overlay (absolutely cooked with this one)
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), Fade(BLACK, 0.6f)); // Input must take float

    const char* text = "PAUSED";
    int fontSize = 60;
    int textWidth = MeasureText(text, fontSize);
    int textX = centerWidth(GetScreenWidth(), textWidth);
    int textY = GetScreenHeight() / 2 - 150;

    DrawText(text, textX + 4, textY + 4, fontSize, BLACK);
    DrawText(text, textX, textY, fontSize, RAYWHITE);

    double btnWidth = 200.0;
    double btnHeight = 50.0;
    double btnX = (double)centerWidth(GetScreenWidth(), (int)btnWidth);
    double btnY = (double)centerHeight(GetScreenHeight(), (int)btnHeight);

    GuiSetStyle(BUTTON, TEXT_SIZE, 20);

    if (GuiButton(Rectangle{ (float)btnX, (float)btnY, (float)btnWidth, (float)btnHeight }, "RESUME")) { // Input must take float
        currentState = GameState::RUNNING;
    }

    if (GuiButton(Rectangle{ (float)btnX, (float)(btnY + btnHeight + 20.0), (float)btnWidth, (float)btnHeight }, "MAIN MENU")) { // Input must take float
        currentState = GameState::MENU;
    }
}

Vector2 UI::NMToPixels(Vec2 nmPos) {
    return Vector2{
        (float)(GetScreenWidth() / 2 + nmPos.x * CONSTANTS.game.PIXELS_PER_NM),
        (float)(GetScreenHeight() / 2 + nmPos.y * CONSTANTS.game.PIXELS_PER_NM)
    };
}

double UI::NMToPixels(double nmDistance) {
    return nmDistance * CONSTANTS.game.PIXELS_PER_NM;
}

Vec2 UI::PixelsToNM(Vector2 pixelPos) {
    return Vec2{
        (double)(pixelPos.x - GetScreenWidth() / 2) / CONSTANTS.game.PIXELS_PER_NM,
        (double)(pixelPos.y - GetScreenHeight() / 2) / CONSTANTS.game.PIXELS_PER_NM
    };
}

void UI::DrawSimulation(const Simulation& sim, bool debugEnabled, Aircraft* selectedAircraft) {
    DrawBackground();

    for (const auto& airport : sim.getAirports()) {
        DrawAirport(airport);
        DrawRangeRings(airport.position);
    }

    for (const auto& plane : sim.getAircraft()) {
        DrawAircraft(*plane, debugEnabled, plane.get() == selectedAircraft);
    }

    DrawSimulationHUD((int)sim.getAircraft().size(), debugEnabled);


    if (selectedAircraft) {
        DrawText("Selected: ", 10, 40, 20, WHITE);
        DrawText(selectedAircraft->getCallsign().c_str(), 110, 40, 20, YELLOW);
        DrawText("Controls: WASD/Arrows = Vector/Speed", 10, 65, 16, GRAY);

        if (debugEnabled) {
            int startY = 100;
            DrawText("--- DEBUG DATA ---", 10, startY, 16, GREEN);
            DrawText(TextFormat("Pos: %.2f, %.2f", selectedAircraft->getPosition().x, selectedAircraft->getPosition().y), 10, startY + 20, 16, GREEN);
            DrawText(TextFormat("Heading: %.2f (Target: %.2f)", selectedAircraft->getHeading(), selectedAircraft->getTargetHeading()), 10, startY + 40, 16, GREEN);
            DrawText(TextFormat("Speed: %.0f kts (Target: %.0f kts)", selectedAircraft->getSpeed(), selectedAircraft->getTargetSpeed()), 10, startY + 60, 16, GREEN);
            DrawText(TextFormat("Altitude: %i (Target: %i)", selectedAircraft->getAltitude(), selectedAircraft->getTargetAltitude()), 10, startY + 80, 16, GREEN);
            DrawText(TextFormat("State: %s", Aircraft::stateToString(selectedAircraft->getState()).c_str()), 10, startY + 100, 16, GREEN);
        }
    } else {
        DrawText("Click aircraft to vector", 10, 40, 20, WHITE);
    }
}

void UI::DrawAircraft(const Aircraft& aircraft, bool debugEnabled, bool selected) {
    auto aircraftColor = WHITE;
    if (selected) aircraftColor = YELLOW;
    if (aircraft.getState() == AircraftState::CONFLICT) aircraftColor = RED;

    const int size = CONSTANTS.game.AIRCRAFT_SIZE;
    const int halfSize = size / 2;
    Vector2 pixelPos = NMToPixels(aircraft.getPosition());

    DrawRectangleLinesEx(
        Rectangle{
            (float)(pixelPos.x - (float)halfSize), // Input must take float
            (float)(pixelPos.y - (float)halfSize), // Input must take float
            (float)size, // Input must take float
            (float)size // Input must take float
        },
        1.5f, // Input must take float
        aircraftColor
    );

    const double vectorLength = 40.0; // Display pixels
    Vector2 vectorEnd = {
        (float)(pixelPos.x + cos(DEG2RAD * (aircraft.getHeading() - 90.0)) * vectorLength), // Input must take float
        (float)(pixelPos.y + sin(DEG2RAD * (aircraft.getHeading() - 90.0)) * vectorLength) // Input must take float
    };

    DrawLineEx(pixelPos, vectorEnd, 2.0f, aircraftColor); // Input must take float

    DrawText(aircraft.getCallsign().c_str(), (int)(pixelPos.x + 20), (int)(pixelPos.y - 12), 14, WHITE);

    if (selected) {
        DrawCircleLinesV(pixelPos, 25.0f, YELLOW); // Input must take float
    }
}

void UI::DrawAirport(const Airport& airport) {
    Vector2 pixelPos = NMToPixels(airport.position);
    float pixelRunwayLength = (float)NMToPixels(airport.runwayLength);

    Rectangle rec = { pixelPos.x, pixelPos.y, pixelRunwayLength, 10.0f }; // Input must take float
    Vector2 origin = { pixelRunwayLength / 2.0f, 5.0f }; // Input must take float
    DrawRectanglePro(rec, origin, (float)(airport.runwayHeading - 90.0), GRAY); // Input must take float

    double approachAngle = (airport.runwayHeading + 90.0) * DEG2RAD;
    float approachAngleDeg = (float)(airport.runwayHeading + 90.0);
    float pixelLocaliserLength = (float)NMToPixels(airport.localizer.length);

    // Draw localizer availability cones from airport data
    for (const auto& sector : airport.localizer.sectors) {
        float halfWidth = (float)(sector.width / 2.0);
        DrawCircleSector(pixelPos, (float)NMToPixels(sector.range), approachAngleDeg - halfWidth, approachAngleDeg + halfWidth, 60, Fade(GREEN, 0.1f));
    }

    Vector2 localizerEnd = {
        (float)(pixelPos.x + cos(approachAngle) * pixelLocaliserLength), // Input must take float
        (float)(pixelPos.y + sin(approachAngle) * pixelLocaliserLength) // Input must take float
    };
    DrawLineEx(pixelPos, localizerEnd, 1.0f, GREEN); // Input must take float
}