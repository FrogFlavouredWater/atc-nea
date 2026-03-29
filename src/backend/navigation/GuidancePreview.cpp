#include "backend/navigation/GuidancePreview.h"
#include "backend/aircraft/Aircraft.h"
#include "core/MathUtils.h"
#include "sim/TrajectoryPredictor.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr double kGuidanceHorizonMinSeconds = 120.0;
constexpr double kGuidanceHorizonMaxSeconds = 600.0;
constexpr double kGuidanceMarginSeconds = 60.0;
constexpr double kGuidanceStepSeconds = 2.0;
constexpr double kHeadingDiffThresholdDeg = 1.0;
constexpr double kHeadingVectorMinLengthNm = 20.0;
constexpr double kHeadingVectorMaxLengthNm = 42.0;
constexpr double kHeadingVectorPreviewSeconds = 420.0;
constexpr double kAltitudeMarkerSweepDeg = 120.0;
constexpr double kAltitudeMarkerMinRadiusNm = 3.0;
constexpr double kAltitudeMarkerMaxRadiusNm = 6.0;
constexpr double kAltitudeMarkerRadiusScale = 0.2;
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kDegToRad = kPi / 180.0;

Vec2 pointFromPolar(Vec2 center, double radiusNm, double angleDeg) {
    const double angleRad = angleDeg * kDegToRad;
    return Vec2{
        center.x + std::cos(angleRad) * radiusNm,
        center.y + std::sin(angleRad) * radiusNm
    };
}

Vec2 interpolatePosition(Vec2 a, Vec2 b, double ratio) {
    return Vec2{
        a.x + (b.x - a.x) * ratio,
        a.y + (b.y - a.y) * ratio
    };
}

double interpolateHeading(double fromHeading, double toHeading, double ratio) {
    return normalizeAngle(fromHeading + getShortestAngleDiff(toHeading, fromHeading) * ratio);
}

double computePredictionHorizonSeconds(const Aircraft& aircraft) {
    // Extend the preview long enough to show likely altitude capture, but keep
    // it bounded so guidance drawing stays responsive.
    const double altitudeDiffFt = std::abs(static_cast<double>(aircraft.getTargetAltitude()) - aircraft.getAltitudeExact());
    if (altitudeDiffFt <= aircraft.getPerformance().altitudeCaptureToleranceFt) {
        return kGuidanceHorizonMinSeconds;
    }

    const double verticalRateFpm = aircraft.getTargetAltitude() >= aircraft.getAltitudeExact()
        ? aircraft.getPerformance().climbRateFpm
        : aircraft.getPerformance().descentRateFpm;
    if (verticalRateFpm <= 0.0) {
        return kGuidanceHorizonMinSeconds;
    }

    const double secondsToCapture = altitudeDiffFt / verticalRateFpm * 60.0;
    return std::clamp(secondsToCapture + kGuidanceMarginSeconds,
                      kGuidanceHorizonMinSeconds,
                      kGuidanceHorizonMaxSeconds);
}

HeadingVectorPreview buildHeadingVector(const Aircraft& aircraft, const TurnArcPreview& turnArc) {
    HeadingVectorPreview preview;

    const double previewLengthNm = std::clamp(
        aircraft.getTargetSpeed() / 3600.0 * kHeadingVectorPreviewSeconds,
        kHeadingVectorMinLengthNm,
        kHeadingVectorMaxLengthNm
    );
    if (previewLengthNm <= 0.0) {
        return preview;
    }

    const Vec2 direction = directionVectorForHeading(aircraft.getTargetHeading());

    preview.visible = true;
    preview.start = turnArc.visible ? turnArc.endPoint : aircraft.getPosition();
    preview.end = Vec2{
        preview.start.x + direction.x * previewLengthNm,
        preview.start.y + direction.y * previewLengthNm
    };
    return preview;
}

TurnArcPreview buildTurnArc(const Aircraft& aircraft) {
    TurnArcPreview preview;

    const double headingDiff = getShortestAngleDiff(aircraft.getTargetHeading(), aircraft.getHeading());
    if (std::abs(headingDiff) < kHeadingDiffThresholdDeg || aircraft.getTurnRadiusNm() <= 0.0) {
        return preview;
    }

    const int directionSign = headingDiff > 0.0 ? 1 : -1;
    const Vec2 currentRightNormal = rightNormalForHeading(aircraft.getHeading());
    const Vec2 centerOffset = Vec2{
        currentRightNormal.x * aircraft.getTurnRadiusNm() * static_cast<double>(directionSign),
        currentRightNormal.y * aircraft.getTurnRadiusNm() * static_cast<double>(directionSign)
    };

    preview.visible = true;
    preview.center = Vec2{
        aircraft.getPosition().x + centerOffset.x,
        aircraft.getPosition().y + centerOffset.y
    };
    preview.radiusNm = aircraft.getTurnRadiusNm();
    preview.startPoint = aircraft.getPosition();
    preview.startAngleDeg = std::atan2(
        aircraft.getPosition().y - preview.center.y,
        aircraft.getPosition().x - preview.center.x
    ) * kRadToDeg;
    preview.sweepAngleDeg = std::abs(headingDiff);
    preview.directionSign = directionSign;
    preview.endPoint = pointFromPolar(preview.center,
                                      preview.radiusNm,
                                      preview.startAngleDeg + preview.sweepAngleDeg * static_cast<double>(preview.directionSign));
    return preview;
}

HoldPreview buildHoldPreview(const Aircraft& aircraft) {
    HoldPreview preview;
    const AircraftInstruction& instruction = aircraft.getActiveInstruction();
    if (instruction.type != AircraftInstructionType::HOLD) {
        return preview;
    }

    const double turnDirection = instruction.holdTurnDirection >= 0 ? 1.0 : -1.0;
    const Vec2 axis = directionVectorForHeading(instruction.targetHeading);
    const Vec2 lateralAxis = Vec2{
        rightNormalForHeading(instruction.targetHeading).x * turnDirection,
        rightNormalForHeading(instruction.targetHeading).y * turnDirection
    };
    const double radiusNm = std::max(instruction.holdTurnRadiusNm, aircraft.getTurnRadiusNm());
    const Vec2 lateralOffset{lateralAxis.x * radiusNm * 2.0, lateralAxis.y * radiusNm * 2.0};
    const Vec2 longitudinalOffset{axis.x * instruction.holdLegLengthNm, axis.y * instruction.holdLegLengthNm};

    preview.visible = true;
    preview.firstStraightStart = instruction.holdEntryPosition;
    preview.firstStraightEnd = Vec2{
        instruction.holdEntryPosition.x + longitudinalOffset.x,
        instruction.holdEntryPosition.y + longitudinalOffset.y
    };
    preview.secondStraightStart = Vec2{
        preview.firstStraightEnd.x + lateralOffset.x,
        preview.firstStraightEnd.y + lateralOffset.y
    };
    preview.secondStraightEnd = Vec2{
        instruction.holdEntryPosition.x + lateralOffset.x,
        instruction.holdEntryPosition.y + lateralOffset.y
    };
    preview.firstTurnCenter = Vec2{
        preview.firstStraightEnd.x + lateralAxis.x * radiusNm,
        preview.firstStraightEnd.y + lateralAxis.y * radiusNm
    };
    preview.secondTurnCenter = Vec2{
        instruction.holdEntryPosition.x + lateralAxis.x * radiusNm,
        instruction.holdEntryPosition.y + lateralAxis.y * radiusNm
    };
    preview.radiusNm = radiusNm;
    preview.firstTurnStartAngleDeg = std::atan2(
        preview.firstStraightEnd.y - preview.firstTurnCenter.y,
        preview.firstStraightEnd.x - preview.firstTurnCenter.x
    ) * kRadToDeg;
    preview.secondTurnStartAngleDeg = std::atan2(
        preview.secondStraightEnd.y - preview.secondTurnCenter.y,
        preview.secondStraightEnd.x - preview.secondTurnCenter.x
    ) * kRadToDeg;
    preview.turnDirectionSign = instruction.holdTurnDirection >= 0 ? 1 : -1;
    return preview;
}

AltitudeCapturePreview buildAltitudeCapture(const Aircraft& aircraft,
                                            const std::vector<PredictedAircraftState>& prediction) {
    AltitudeCapturePreview preview;

    const double targetAltitude = static_cast<double>(aircraft.getTargetAltitude());
    const double toleranceFt = aircraft.getPerformance().altitudeCaptureToleranceFt;
    if (std::abs(targetAltitude - aircraft.getAltitudeExact()) <= toleranceFt || prediction.size() < 2) {
        return preview;
    }

    for (size_t i = 1; i < prediction.size(); ++i) {
        // Interpolate between predictor samples so the marker lands near the
        // actual capture point instead of snapping to a coarse time step.
        const auto& previous = prediction[i - 1];
        const auto& current = prediction[i];
        const double previousAltitude = previous.motion.altitude;
        const double currentAltitude = current.motion.altitude;

        const bool targetReached =
            (targetAltitude >= previousAltitude && targetAltitude <= currentAltitude)
            || (targetAltitude <= previousAltitude && targetAltitude >= currentAltitude)
            || std::abs(targetAltitude - currentAltitude) <= toleranceFt;

        if (!targetReached) {
            continue;
        }

        double ratio = 1.0;
        const double altitudeDelta = currentAltitude - previousAltitude;
        if (std::abs(altitudeDelta) > 0.001) {
            ratio = std::clamp((targetAltitude - previousAltitude) / altitudeDelta, 0.0, 1.0);
        }

        preview.visible = true;
        preview.position = interpolatePosition(previous.motion.position, current.motion.position, ratio);
        preview.headingDeg = interpolateHeading(previous.motion.heading, current.motion.heading, ratio);
        preview.radiusNm = std::clamp(
            distanceNm(aircraft.getPosition(), preview.position) * kAltitudeMarkerRadiusScale,
            kAltitudeMarkerMinRadiusNm,
            kAltitudeMarkerMaxRadiusNm
        );

        const Vec2 arcOffset = directionVectorForHeading(normalizeAngle(preview.headingDeg + 180.0));
        preview.arcCenter = Vec2{
            preview.position.x + arcOffset.x * preview.radiusNm,
            preview.position.y + arcOffset.y * preview.radiusNm
        };

        const Vec2 captureOffset = Vec2{
            preview.position.x - preview.arcCenter.x,
            preview.position.y - preview.arcCenter.y
        };
        const double captureAngleDeg = std::atan2(captureOffset.y, captureOffset.x) * kRadToDeg;
        preview.startAngleDeg = normalizeAngle(captureAngleDeg - kAltitudeMarkerSweepDeg / 2.0);
        preview.sweepAngleDeg = kAltitudeMarkerSweepDeg;
        preview.directionSign = 1;
        preview.timeSeconds = previous.timeSeconds + (current.timeSeconds - previous.timeSeconds) * ratio;
        return preview;
    }

    return preview;
}
}

GuidancePreview GuidancePreviewService::build(const Aircraft& aircraft) {
    // Guidance previews are built from the aircraft's current command state, so
    // they reflect what the sim would do next without mutating live aircraft.
    const TrajectoryPredictor predictor;
    GuidancePreview preview;
    preview.hold = buildHoldPreview(aircraft);
    if (preview.hold.visible) {
        return preview;
    }

    const double horizonSeconds = computePredictionHorizonSeconds(aircraft);
    const auto predictedTrajectory = predictor.predict(aircraft, horizonSeconds, kGuidanceStepSeconds);

    preview.turnArc = buildTurnArc(aircraft);
    preview.headingVector = buildHeadingVector(aircraft, preview.turnArc);
    preview.altitudeCapture = buildAltitudeCapture(aircraft, predictedTrajectory);
    return preview;
}
