// Survey grid ("lawnmower") planner in the style of Mission Planner's Survey
// (Grid): parallel passes over a lat/lon polygon, spaced by a pass count or
// by the side overlap of the camera footprint. Plain C++ (no Unreal types)
// so it can be tested outside the engine.
#pragma once

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>

namespace SurveyGrid
{
constexpr double kEarthRadiusM = 6378137.0; // WGS84 semi-major axis
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxPasses = 2000;

// Own min / max: windows.h (pulled in by some engine headers) may define
// min and max macros, which break std::min / std::max.
inline double Smaller(double A, double B)
{
    return A < B ? A : B;
}

inline double Larger(double A, double B)
{
    return A > B ? A : B;
}

struct LatLon
{
    double Lat = 0.0;
    double Lon = 0.0;
};

struct Params
{
    double AltitudeM = 500.0;     // height above ground; sets the footprint
    double HFovDeg = 36.87;       // camera horizontal field of view
    double Aspect = 16.0 / 9.0;   // image width / height
    bool bAutoAngle = true;       // legs parallel to the polygon's longest edge
    double LegAngleDeg = 0.0;     // compass direction of the legs otherwise
    bool bUsePassCount = false;   // true: PassCount passes; false: spacing from SideOverlapPct
    int PassCount = 6;
    double SideOverlapPct = 30.0; // overlap of neighbouring passes' footprints
    double OvershootM = 0.0;      // extend every pass beyond the polygon edge
    bool bFixedHeading = false;   // camera heading fixed (else along the legs)
    double FixedHeadingDeg = 0.0;
};

struct Plan
{
    std::vector<LatLon> Waypoints; // start and end of every pass, in flight order
    int Passes = 0;
    double LegAngleDeg = 0.0;      // compass direction of the first pass, [0, 180)
    double SpacingM = 0.0;
    double FootprintWidthM = 0.0;  // image width on the ground
    double FootprintHeightM = 0.0;
    double FootprintCrossM = 0.0;  // footprint extent across the passes
    double LengthM = 0.0;          // path length through all waypoints
    double AreaM2 = 0.0;
    std::string Error;             // non-empty: no plan
};

inline double DegToRad(double Deg)
{
    return Deg * kPi / 180.0;
}

inline double RadToDeg(double Rad)
{
    return Rad * 180.0 / kPi;
}

// Reads "lat, lon; lat, lon; ..." (any of , ; space tab newline between the
// numbers): every two numbers make one vertex.
inline bool ParseLatLonList(const std::string& Source, std::vector<LatLon>& OutVertices, std::string& OutError)
{
    OutVertices.clear();
    std::vector<double> Numbers;
    const char* Cursor = Source.c_str();
    while (*Cursor != '\0')
    {
        const char C = *Cursor;
        const bool bNumberStart = (C >= '0' && C <= '9') || C == '-' || C == '+' || C == '.';
        if (!bNumberStart)
        {
            ++Cursor;
            continue;
        }
        char* End = nullptr;
        const double Value = std::strtod(Cursor, &End);
        if (End == Cursor)
        {
            ++Cursor;
            continue;
        }
        Numbers.push_back(Value);
        Cursor = End;
    }
    if (Numbers.size() % 2 != 0)
    {
        OutError = "odd count of numbers: every vertex needs a latitude and a longitude";
        return false;
    }
    for (size_t I = 0; I + 1 < Numbers.size(); I += 2)
    {
        LatLon P;
        P.Lat = Numbers[I];
        P.Lon = Numbers[I + 1];
        if (P.Lat < -90.0 || P.Lat > 90.0 || P.Lon < -180.0 || P.Lon > 180.0)
        {
            OutError = "vertex " + std::to_string(I / 2 + 1) + " is outside lat -90..90 / lon -180..180";
            return false;
        }
        OutVertices.push_back(P);
    }
    return true;
}

inline Plan PlanSurvey(const std::vector<LatLon>& Vertices, const Params& In)
{
    Plan Result;
    const size_t Count = Vertices.size();
    if (Count < 3)
    {
        Result.Error = "the area needs at least 3 vertices";
        return Result;
    }
    if (In.AltitudeM <= 0.0 || In.HFovDeg <= 0.0 || In.HFovDeg >= 179.0 || In.Aspect <= 0.0)
    {
        Result.Error = "altitude and camera field of view must be positive";
        return Result;
    }

    // Local east/north metres around the vertex mean (equirectangular: fine
    // for areas of a few tens of km).
    double Lat0 = 0.0, Lon0 = 0.0;
    for (const LatLon& P : Vertices)
    {
        Lat0 += P.Lat;
        Lon0 += P.Lon;
    }
    Lat0 /= static_cast<double>(Count);
    Lon0 /= static_cast<double>(Count);
    const double CosLat0 = std::cos(DegToRad(Lat0));
    std::vector<double> Xs(Count), Ys(Count); // east, north
    for (size_t I = 0; I < Count; ++I)
    {
        Xs[I] = DegToRad(Vertices[I].Lon - Lon0) * kEarthRadiusM * CosLat0;
        Ys[I] = DegToRad(Vertices[I].Lat - Lat0) * kEarthRadiusM;
    }

    double Shoelace = 0.0;
    for (size_t I = 0; I < Count; ++I)
    {
        const size_t J = (I + 1) % Count;
        Shoelace += Xs[I] * Ys[J] - Xs[J] * Ys[I];
    }
    Result.AreaM2 = std::fabs(Shoelace) * 0.5;
    if (Result.AreaM2 < 1.0)
    {
        Result.Error = "the area is empty (vertices on one line?)";
        return Result;
    }

    double AngleDeg = In.LegAngleDeg;
    if (In.bAutoAngle)
    {
        double LongestM = -1.0;
        for (size_t I = 0; I < Count; ++I)
        {
            const size_t J = (I + 1) % Count;
            const double DX = Xs[J] - Xs[I];
            const double DY = Ys[J] - Ys[I];
            const double EdgeM = std::hypot(DX, DY);
            if (EdgeM > LongestM)
            {
                LongestM = EdgeM;
                AngleDeg = RadToDeg(std::atan2(DX, DY));
            }
        }
    }
    AngleDeg = std::fmod(AngleDeg, 180.0);
    if (AngleDeg < 0.0)
    {
        AngleDeg += 180.0;
    }
    Result.LegAngleDeg = AngleDeg;

    // u = along the legs, w = across them (to the right of the leg direction).
    const double Theta = DegToRad(AngleDeg);
    const double AlongX = std::sin(Theta), AlongY = std::cos(Theta);
    const double CrossX = std::cos(Theta), CrossY = -std::sin(Theta);
    std::vector<double> Us(Count), Ws(Count);
    double WMin = 1e300, WMax = -1e300;
    for (size_t I = 0; I < Count; ++I)
    {
        Us[I] = Xs[I] * AlongX + Ys[I] * AlongY;
        Ws[I] = Xs[I] * CrossX + Ys[I] * CrossY;
        WMin = Smaller(WMin, Ws[I]);
        WMax = Larger(WMax, Ws[I]);
    }
    const double SpanM = WMax - WMin;

    Result.FootprintWidthM = 2.0 * In.AltitudeM * std::tan(DegToRad(In.HFovDeg) * 0.5);
    Result.FootprintHeightM = Result.FootprintWidthM / In.Aspect;
    const double HeadingMinusLeg = In.bFixedHeading ? DegToRad(In.FixedHeadingDeg - AngleDeg) : 0.0;
    Result.FootprintCrossM = Result.FootprintWidthM * std::fabs(std::cos(HeadingMinusLeg)) +
                             Result.FootprintHeightM * std::fabs(std::sin(HeadingMinusLeg));

    // Pass centre lines.
    std::vector<double> Lanes;
    if (In.bUsePassCount)
    {
        if (In.PassCount < 1 || In.PassCount > kMaxPasses)
        {
            Result.Error = "pass count must be 1.." + std::to_string(kMaxPasses);
            return Result;
        }
        Result.SpacingM = SpanM / In.PassCount;
        for (int K = 0; K < In.PassCount; ++K)
        {
            Lanes.push_back(WMin + Result.SpacingM * (K + 0.5));
        }
    }
    else
    {
        if (In.SideOverlapPct < 0.0 || In.SideOverlapPct >= 100.0)
        {
            Result.Error = "side overlap must be 0..99 %";
            return Result;
        }
        Result.SpacingM = Result.FootprintCrossM * (1.0 - In.SideOverlapPct / 100.0);
        if (Result.SpacingM < 1.0)
        {
            Result.Error = "pass spacing below 1 m";
            return Result;
        }
        const double Needed = std::ceil(SpanM / Result.SpacingM - 1e-9);
        if (Needed > kMaxPasses)
        {
            Result.Error = "more than " + std::to_string(kMaxPasses) + " passes: raise the altitude or lower the overlap";
            return Result;
        }
        const int Passes = Needed < 1.0 ? 1 : static_cast<int>(Needed);
        const double Middle = 0.5 * (WMin + WMax);
        for (int K = 0; K < Passes; ++K)
        {
            Lanes.push_back(Middle + (K - 0.5 * (Passes - 1)) * Result.SpacingM);
        }
    }

    // Every pass spans the polygon between its first and last edge crossing
    // (a concave notch is flown over), plus the overshoot at both ends.
    std::vector<LatLon>& Out = Result.Waypoints;
    bool bForward = true;
    for (const double W : Lanes)
    {
        double UMin = 1e300, UMax = -1e300;
        int Hits = 0;
        for (size_t I = 0; I < Count; ++I)
        {
            const size_t J = (I + 1) % Count;
            const double WA = Ws[I], WB = Ws[J];
            const bool bCrosses = (WA <= W && W < WB) || (WB <= W && W < WA);
            if (!bCrosses)
            {
                continue;
            }
            const double T = (W - WA) / (WB - WA);
            const double U = Us[I] + T * (Us[J] - Us[I]);
            UMin = Smaller(UMin, U);
            UMax = Larger(UMax, U);
            ++Hits;
        }
        if (Hits < 2)
        {
            continue;
        }
        UMin -= In.OvershootM;
        UMax += In.OvershootM;
        const double UFrom = bForward ? UMin : UMax;
        const double UTo = bForward ? UMax : UMin;
        for (const double U : {UFrom, UTo})
        {
            const double X = U * AlongX + W * CrossX;
            const double Y = U * AlongY + W * CrossY;
            LatLon P;
            P.Lat = Lat0 + RadToDeg(Y / kEarthRadiusM);
            P.Lon = Lon0 + RadToDeg(X / (kEarthRadiusM * CosLat0));
            Out.push_back(P);
        }
        bForward = !bForward;
        ++Result.Passes;
    }
    if (Result.Passes == 0)
    {
        Result.Error = "no pass crosses the area";
        return Result;
    }

    for (size_t I = 1; I < Out.size(); ++I)
    {
        const double DN = DegToRad(Out[I].Lat - Out[I - 1].Lat) * kEarthRadiusM;
        const double DE = DegToRad(Out[I].Lon - Out[I - 1].Lon) * kEarthRadiusM * CosLat0;
        Result.LengthM += std::hypot(DN, DE);
    }
    return Result;
}
} // namespace SurveyGrid
