#pragma once

#include "frontend/ui.h"

namespace ui_detail {
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

Vec2 headingDirectionNm(double headingDeg);
Vec2 headingNormalNm(double headingDeg);
void drawArcSegmentNm(Vec2 centerNm,
                      double radiusNm,
                      double startAngleDeg,
                      double sweepAngleDeg,
                      int directionSign,
                      Color color,
                      float thickness,
                      const SimSettings& simSettings);

void applySettingsTheme();
void drawPanel(Rectangle bounds, Color fill, Color border);
void drawSection(Rectangle bounds, const char* title, const char* subtitle);
void drawValueChip(Rectangle bounds, const char* text);
void drawFieldText(int x, int y, const char* label, const char* subtitle);
void drawSpinnerRow(Rectangle row,
                    const char* label,
                    const char* subtitle,
                    int& value,
                    int minValue,
                    int maxValue,
                    bool& editMode);
void drawModeRow(Rectangle row, const char* label, const char* subtitle, ScreenMode& mode);
void drawSliderRow(Rectangle row,
                   const char* label,
                   const char* subtitle,
                   double& value,
                   double minValue,
                   double maxValue,
                   const char* valueFormat);
void clampPendingSettings(AppSettings& settings);
const char* screenModeLabel(ScreenMode mode);
ScreenMode nextScreenMode(ScreenMode mode);

void drawShadowedText(const char* text,
                      int x,
                      int y,
                      int fontSize,
                      Color textColor,
                      Color shadowColor,
                      int shadowOffset = 4);
Color aircraftColor(const Aircraft& aircraft, bool selected);
} // namespace ui_detail
