#pragma once

#include "common/utils.h"

struct HeadingVectorPreview {
    bool visible = false;
    Vec2 start{};
    Vec2 end{};
};

struct TurnArcPreview {
    bool visible = false;
    Vec2 center{};
    Vec2 startPoint{};
    Vec2 endPoint{};
    double radiusNm = 0.0;
    double startAngleDeg = 0.0;
    double sweepAngleDeg = 0.0;
    int directionSign = 0;
};

struct AltitudeCapturePreview {
    bool visible = false;
    Vec2 position{};
    Vec2 arcCenter{};
    double headingDeg = 0.0;
    double radiusNm = 0.0;
    double startAngleDeg = 0.0;
    double sweepAngleDeg = 0.0;
    int directionSign = 0;
    double timeSeconds = 0.0;
};

struct HoldPreview {
    bool visible = false;
    Vec2 firstStraightStart{};
    Vec2 firstStraightEnd{};
    Vec2 secondStraightStart{};
    Vec2 secondStraightEnd{};
    Vec2 firstTurnCenter{};
    Vec2 secondTurnCenter{};
    double radiusNm = 0.0;
    double firstTurnStartAngleDeg = 0.0;
    double secondTurnStartAngleDeg = 0.0;
    double turnSweepAngleDeg = 180.0;
    int turnDirectionSign = 1;
};

struct GuidancePreview {
    HeadingVectorPreview headingVector{};
    TurnArcPreview turnArc{};
    AltitudeCapturePreview altitudeCapture{};
    HoldPreview hold{};
};

class Aircraft;

class GuidancePreviewService {
public:
    static GuidancePreview build(const Aircraft& aircraft);
};
