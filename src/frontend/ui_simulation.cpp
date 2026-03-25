#include "frontend/ui_internal.h"

#include <raygui.h>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace {
constexpr double kDegToRad = std::numbers::pi_v<double> / 180.0;

void drawSelectedAircraftDetails(const Aircraft& aircraft, bool debugEnabled) {
    Color selectedColor = YELLOW;
    if (aircraft.getControlMode() == AircraftControlMode::ILS) {
        selectedColor = ui_detail::kIlsBlue;
    } else if (aircraft.hasApproachClearance()) {
        selectedColor = ui_detail::kApproachGreen;
    }

    DrawText("Selected: ", 10, 70, 20, WHITE);
    DrawText(aircraft.getCallsign().c_str(),
             110,
             70,
             20,
             selectedColor);
    DrawText("Controls: A/D heading, W/S speed, Q/E altitude, I ILS clr, ,/. sim speed", 10, 95, 16, GRAY);

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
} // namespace

SimulationViewResult UI::DrawSimulationHUD(int aircraftCount,
                                           int outOfBoundsCount,
                                           int landedCount,
                                           int predictedConflictCount,
                                           double simulationSpeed,
                                           bool& debugEnabled,
                                           const SpawnRequestResult& spawnResult) {
    SimulationViewResult result;
    GuiLoadStyleDefault();
    DrawText(TextFormat("Aircraft: %i", aircraftCount), 10, 10, 20, DARKGRAY);
    DrawText(TextFormat("Out: %i  Landed: %i  Predicted: %i", outOfBoundsCount, landedCount, predictedConflictCount), 10, 38, 20, DARKGRAY);
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
        debugEnabled = !debugEnabled;
    }

    return result;
}

void UI::DrawBackground() {
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), BLACK);
}

void UI::DrawRangeRings(Vec2 airportPos, const SimSettings& simSettings) {
    constexpr float kRings[] = {5.0f, 10.0f, 20.0f, 30.0f, 40.0f};
    const Vector2 center = NMToPixels(airportPos, simSettings);

    for (const float radiusNm : kRings) {
        const float pixelRadius = static_cast<float>(NMToPixels(radiusNm, simSettings));
        DrawCircleLinesV(center, pixelRadius, Fade(DARKGRAY, 0.9f));
        DrawText(TextFormat("%0.f NM", radiusNm),
                 static_cast<int>(center.x) + 5,
                 static_cast<int>(center.y - pixelRadius - 15),
                 12,
                 DARKGRAY);
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

void UI::DrawGuidancePreview(const GuidancePreview& preview, const SimSettings& simSettings, bool debugEnabled) {
    if (preview.headingVector.visible) {
        DrawLineEx(NMToPixels(preview.headingVector.start, simSettings),
                   NMToPixels(preview.headingVector.end, simSettings),
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
            const Vector2 markerPos = NMToPixels(preview.altitudeCapture.position, simSettings);
            DrawText(TextFormat("ALT %.0fs", preview.altitudeCapture.timeSeconds),
                     static_cast<int>(markerPos.x + 8.0f),
                     static_cast<int>(markerPos.y - 12.0f),
                     12,
                     ui_detail::kGuidanceYellow);
        }
    }
}

void UI::DrawAircraft(const Aircraft& aircraft, const SimSettings& simSettings, bool selected) {
    const Color color = ui_detail::aircraftColor(aircraft, selected);

    DrawAircraftTrail(aircraft, simSettings, selected);

    const int size = simSettings.aircraftSize;
    const int halfSize = size / 2;
    const Vector2 pixelPos = NMToPixels(aircraft.getPosition(), simSettings);

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

void UI::DrawAircraftTrail(const Aircraft& aircraft, const SimSettings& simSettings, bool selected) {
    const auto& trailPoints = aircraft.getTrailPoints();
    if (trailPoints.size() < 2) {
        return;
    }

    const double trailElapsedSeconds = aircraft.getTrailElapsedSeconds();
    const double oldestVisibleAge = trailElapsedSeconds - trailPoints.front().recordedAtSeconds;
    const double selectedFadeWindow = std::max(ui_detail::kVisibleTrailAgeSeconds, oldestVisibleAge);
    const float dotRadius = selected ? 2.6f : 2.0f;

    for (const auto& trailPoint : trailPoints) {
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

        DrawCircleV(NMToPixels(trailPoint.position, simSettings),
                    dotRadius,
                    Fade(ui_detail::kTrackTrail, static_cast<float>(alpha)));
    }
}

void UI::DrawAirport(const Airport& airport, const SimSettings& simSettings) {
    const Vector2 pixelPos = NMToPixels(airport.position, simSettings);
    const float pixelRunwayLength = static_cast<float>(NMToPixels(airport.runwayLength, simSettings));

    DrawRectanglePro(Rectangle{pixelPos.x, pixelPos.y, pixelRunwayLength, 10.0f},
                     Vector2{pixelRunwayLength / 2.0f, 5.0f},
                     static_cast<float>(airport.runwayHeading - 90.0),
                     GRAY);

    const double localizerHeading = normalizeAngle(airport.runwayHeading + 180.0);
    const Vec2 localizerDirection = ui_detail::headingDirectionNm(localizerHeading);
    const Vec2 localizerNormal = ui_detail::headingNormalNm(localizerHeading);
    const Vec2 localizerEndNm{
        airport.position.x + localizerDirection.x * airport.localizer.length,
        airport.position.y + localizerDirection.y * airport.localizer.length
    };

    DrawLineEx(pixelPos,
               NMToPixels(localizerEndNm, simSettings),
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

        DrawLineEx(NMToPixels(tickStart, simSettings),
                   NMToPixels(tickEnd, simSettings),
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

        DrawLineEx(pixelPos, NMToPixels(leftEndNm, simSettings), 1.4f, Fade(ui_detail::kIlsBlue, ui_detail::kIlsBoundaryAlpha));
        DrawLineEx(pixelPos, NMToPixels(rightEndNm, simSettings), 1.4f, Fade(ui_detail::kIlsBlue, ui_detail::kIlsBoundaryAlpha));
        DrawLineEx(NMToPixels(leftEndNm, simSettings),
                   NMToPixels(rightEndNm, simSettings),
                   1.0f,
                   Fade(ui_detail::kIlsBlue, ui_detail::kIlsSectorEdgeAlpha));
    }

    DrawText(airport.name.c_str(),
             static_cast<int>(pixelPos.x + 10.0f),
             static_cast<int>(pixelPos.y + 12.0f),
             14,
             Fade(ui_detail::kIlsBlue, ui_detail::kIlsLabelAlpha));
}
