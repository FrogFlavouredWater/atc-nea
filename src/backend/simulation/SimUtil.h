#pragma once

#include "backend/aircraft/Aircraft.h"
#include "backend/navigation/airport.h"
#include "backend/traffic/SpawnService.h"
#include "common/constants.h"
#include <string>
#include <utility>

namespace SimulationDetail {

inline constexpr double kSpawnLookaheadSeconds = 75.0;
inline constexpr double kSpawnPredictionStepSeconds = 5.0;
inline constexpr double kConflictLookaheadSeconds = 180.0;
inline constexpr double kConflictPredictionStepSeconds = 5.0;
inline constexpr double kTacticalInterventionRangeNm = 15.0;
inline constexpr double kVectoringPriorityVerticalFt = 500.0;
inline constexpr double kArrivalSpeedDistanceNm = 20.0;
inline constexpr double kArrivalSpacingDistanceNm = 8.0;
inline constexpr double kArrivalSpacingCrossTrackNm = 4.0;
inline constexpr double kArrivalSpacingHeadingToleranceDeg = 35.0;
inline constexpr double kArrivalSpeedMinKts = 150.0;
inline constexpr double kArrivalSpeedMaxKts = 220.0;
inline constexpr double kArrivalScheduleSpacingSeconds = 90.0;
inline constexpr double kArrivalHoldThresholdSeconds = 120.0;
inline constexpr double kArrivalHoldMaxRangeNm = 22.0;
inline constexpr double kArrivalReleaseLeadSeconds = 45.0;
inline constexpr double kHoldLegLengthNm = 4.0;
inline constexpr double kHoldMinTurnRadiusNm = 0.8;
inline constexpr double kResolutionHoldSeconds = 8.0;
inline constexpr double kResolutionReevaluationSeconds = 12.0;
inline constexpr double kResolutionTurnSmallDeg = 20.0;
inline constexpr double kResolutionTurnLargeDeg = 35.0;
inline constexpr int kResolutionAltitudeStepFt = 1000;
inline constexpr double kResolutionSpeedStepKts = 20.0;
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDegToRad = kPi / 180.0;
inline constexpr double kIlsCaptureHeadingToleranceDeg = 35.0;
inline constexpr double kIlsCaptureMinSpeedKts = 120.0;
inline constexpr double kIlsCaptureMaxSpeedKts = 185.0;
inline constexpr double kIlsOuterCaptureMinAltitudeFt = 2500.0;
inline constexpr double kIlsOuterCaptureMaxAltitudeFt = 4000.0;
inline constexpr double kIlsInnerCaptureMinAltitudeFt = 1000.0;
inline constexpr double kIlsInnerCaptureMaxAltitudeFt = 2500.0;
inline constexpr double kIlsFinalDescentStartNm = 3.0;
inline constexpr double kIlsHeadingCorrectionPerNm = 12.0;
inline constexpr double kIlsHeadingCorrectionMaxDeg = 18.0;
inline constexpr double kIlsTargetSpeedMinKts = 120.0;
inline constexpr double kIlsTargetSpeedMaxKts = 150.0;
inline constexpr double kIlsLandingPhaseDistanceNm = 1.2;
inline constexpr double kIlsTouchdownDistanceNm = 0.45;
inline constexpr double kIlsTouchdownAltitudeFt = 150.0;
inline constexpr double kIlsTouchdownSpeedMinKts = 110.0;
inline constexpr double kIlsTouchdownSpeedMaxKts = 155.0;

double horizontalDistanceNm(const AircraftMotionState& first, const AircraftMotionState& second);
double verticalDistanceFt(const AircraftMotionState& first, const AircraftMotionState& second);
Vec2 directionVectorForHeading(double headingDeg);
Vec2 rightNormalForHeading(double headingDeg);
double dot(Vec2 first, Vec2 second);
double distanceNm(Vec2 first, Vec2 second);
double headingToward(Vec2 from, Vec2 to);
bool aircraftSquaresTouch(const Aircraft& first, const Aircraft& second, const SimSettings& settings);
double alongTrackToRunwayNm(const Airport& airport, Vec2 position);
double crossTrackErrorNm(const Airport& airport, Vec2 position);
double minLocalizerRangeNm(const Localizer& localizer);
bool isInOuterIlsRegion(const Airport& airport, double alongTrackNm);
std::pair<double, double> ilsCaptureAltitudeBandFt(const Airport& airport, double alongTrackNm);
double interpolateLinear(double input,
                         double inputStart,
                         double inputEnd,
                         double outputStart,
                         double outputEnd);
double ilsProfileAltitudeFt(const Aircraft& plane, const Airport& airport);
bool isValidAirportIndex(int airportIndex, size_t airportCount);
std::string formatSpawnCandidate(const SpawnCandidate& candidate);
std::string formatAircraftInstruction(const AircraftInstruction& instruction);
std::pair<std::string, std::string> makeConflictPair(const Aircraft& first, const Aircraft& second);
std::pair<std::string, std::string> makeConflictPair(const std::string& first, const std::string& second);

}  // namespace SimulationDetail
