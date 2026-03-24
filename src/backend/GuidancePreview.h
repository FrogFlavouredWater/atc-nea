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

struct GuidancePreview {
    HeadingVectorPreview headingVector{};
    TurnArcPreview turnArc{};
    AltitudeCapturePreview altitudeCapture{};
};

class Aircraft;

class GuidancePreviewService {
public:
    static GuidancePreview build(const Aircraft& aircraft);
};
