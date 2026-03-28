#include "backend/simulation/SimUtil.h"

#include "common/utils.h"
#include <algorithm>
#include <cmath>
#include <sstream>

namespace SimulationDetail {

double horizontalDistanceNm(const AircraftMotionState& first, const AircraftMotionState& second) {
    const double dx = first.position.x - second.position.x;
    const double dy = first.position.y - second.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

double verticalDistanceFt(const AircraftMotionState& first, const AircraftMotionState& second) {
    return std::abs(first.altitude - second.altitude);
}

Vec2 directionVectorForHeading(double headingDeg) {
    return Vec2{
        std::cos(kDegToRad * (headingDeg - 90.0)),
        std::sin(kDegToRad * (headingDeg - 90.0))
    };
}

Vec2 rightNormalForHeading(double headingDeg) {
    return Vec2{
        std::cos(kDegToRad * headingDeg),
        std::sin(kDegToRad * headingDeg)
    };
}

double dot(Vec2 first, Vec2 second) {
    return first.x * second.x + first.y * second.y;
}

double distanceNm(Vec2 first, Vec2 second) {
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    return std::sqrt(dx * dx + dy * dy);
}

double headingToward(Vec2 from, Vec2 to) {
    return normalizeAngle(std::atan2(to.y - from.y, to.x - from.x) * 180.0 / kPi + 90.0);
}

bool aircraftSquaresTouch(const Aircraft& first, const Aircraft& second, const SimSettings& settings) {
    const double squareSideNm = settings.pixelsPerNm > 0.0
        ? static_cast<double>(settings.aircraftSize) / settings.pixelsPerNm
        : 0.0;
    const double dx = std::abs(first.getPosition().x - second.getPosition().x);
    const double dy = std::abs(first.getPosition().y - second.getPosition().y);
    return dx <= squareSideNm && dy <= squareSideNm;
}

double alongTrackToRunwayNm(const Airport& airport, Vec2 position) {
    const Vec2 runwayDirection = directionVectorForHeading(airport.runwayHeading);
    const Vec2 toRunway{
        airport.position.x - position.x,
        airport.position.y - position.y
    };
    return dot(toRunway, runwayDirection);
}

double crossTrackErrorNm(const Airport& airport, Vec2 position) {
    const Vec2 relativeToRunway{
        position.x - airport.position.x,
        position.y - airport.position.y
    };
    return dot(relativeToRunway, rightNormalForHeading(airport.runwayHeading));
}

double minLocalizerRangeNm(const Localizer& localizer) {
    if (localizer.sectors.empty()) {
        return 0.0;
    }

    double minRangeNm = localizer.sectors.front().range;
    for (const auto& sector : localizer.sectors) {
        minRangeNm = std::min(minRangeNm, sector.range);
    }
    return minRangeNm;
}

bool isInOuterIlsRegion(const Airport& airport, double alongTrackNm) {
    return alongTrackNm > minLocalizerRangeNm(airport.localizer);
}

std::pair<double, double> ilsCaptureAltitudeBandFt(const Airport& airport, double alongTrackNm) {
    if (isInOuterIlsRegion(airport, alongTrackNm)) {
        return {kIlsOuterCaptureMinAltitudeFt, kIlsOuterCaptureMaxAltitudeFt};
    }

    return {kIlsInnerCaptureMinAltitudeFt, kIlsInnerCaptureMaxAltitudeFt};
}

double interpolateLinear(double input,
                         double inputStart,
                         double inputEnd,
                         double outputStart,
                         double outputEnd) {
    if (std::abs(inputEnd - inputStart) < 1e-6) {
        return outputEnd;
    }

    const double t = std::clamp((input - inputStart) / (inputEnd - inputStart), 0.0, 1.0);
    return outputStart + (outputEnd - outputStart) * t;
}

double ilsProfileAltitudeFt(const Aircraft& plane, const Airport& airport) {
    const double alongTrackNm = std::clamp(alongTrackToRunwayNm(airport, plane.getPosition()),
                                           0.0,
                                           airport.localizer.length);
    const double innerRegionBoundaryNm = minLocalizerRangeNm(airport.localizer);
    double desiredAltitudeFt = 0.0;

    if (alongTrackNm > innerRegionBoundaryNm) {
        desiredAltitudeFt = interpolateLinear(alongTrackNm,
                                              innerRegionBoundaryNm,
                                              airport.localizer.length,
                                              kIlsOuterCaptureMinAltitudeFt,
                                              kIlsOuterCaptureMaxAltitudeFt);
    } else if (alongTrackNm > kIlsFinalDescentStartNm) {
        desiredAltitudeFt = interpolateLinear(alongTrackNm,
                                              kIlsFinalDescentStartNm,
                                              innerRegionBoundaryNm,
                                              kIlsInnerCaptureMinAltitudeFt,
                                              kIlsInnerCaptureMaxAltitudeFt);
    } else {
        desiredAltitudeFt = interpolateLinear(alongTrackNm,
                                              0.0,
                                              kIlsFinalDescentStartNm,
                                              0.0,
                                              kIlsInnerCaptureMinAltitudeFt);
    }

    return std::min(desiredAltitudeFt, plane.getAltitudeExact());
}

bool isValidAirportIndex(int airportIndex, size_t airportCount) {
    return airportIndex >= 0 && static_cast<size_t>(airportIndex) < airportCount;
}

std::string formatSpawnCandidate(const SpawnCandidate& candidate) {
    std::ostringstream stream;
    stream << candidate.callsign
           << " from " << candidate.entryLabel
           << " hdg " << static_cast<int>(std::lround(candidate.headingDeg))
           << " spd " << static_cast<int>(std::lround(candidate.speedKts))
           << " alt " << candidate.altitudeFt;
    return stream.str();
}

std::string formatAircraftInstruction(const AircraftInstruction& instruction) {
    std::ostringstream stream;
    stream << toString(instruction.type)
           << " hdg " << static_cast<int>(std::lround(instruction.targetHeading))
           << " spd " << static_cast<int>(std::lround(instruction.targetSpeed))
           << " alt " << instruction.targetAltitude;
    return stream.str();
}

std::pair<std::string, std::string> makeConflictPair(const Aircraft& first, const Aircraft& second) {
    if (first.getCallsign() < second.getCallsign()) {
        return {first.getCallsign(), second.getCallsign()};
    }
    return {second.getCallsign(), first.getCallsign()};
}

std::pair<std::string, std::string> makeConflictPair(const std::string& first, const std::string& second) {
    if (first < second) {
        return {first, second};
    }
    return {second, first};
}

}  // namespace SimulationDetail
