// Glue between the panel settings and the survey grid planner.
#pragma once

#include "CoreMinimal.h"
#include "Sim/DroneSettingsPanel.h"
#include "Sim/SurveyGrid.h"

namespace DroneMission
{
// The vertex list is numbers and separators only; anything else becomes a space.
inline std::string ToAscii(const FString& Source)
{
    std::string Out;
    Out.reserve(static_cast<size_t>(Source.Len()));
    for (int32 Index = 0; Index < Source.Len(); ++Index)
    {
        const TCHAR Ch = Source[Index];
        Out.push_back(Ch < 128 ? static_cast<char>(Ch) : ' ');
    }
    return Out;
}

inline FString FromAscii(const std::string& Source)
{
    FString Out;
    for (const char Ch : Source)
    {
        Out.AppendChar(static_cast<TCHAR>(Ch));
    }
    return Out;
}

inline SurveyGrid::Plan PlanMission(const FDroneSimSettings& S)
{
    SurveyGrid::Plan Result;
    std::vector<SurveyGrid::LatLon> Vertices;
    std::string ParseError;
    if (!SurveyGrid::ParseLatLonList(ToAscii(S.MissionPolygon), Vertices, ParseError))
    {
        Result.Error = ParseError;
        return Result;
    }
    SurveyGrid::Params In;
    In.AltitudeM = S.MissionAltitudeM;
    In.HFovDeg = S.CameraHFovDeg;
    In.Aspect = S.CameraAspect;
    In.bAutoAngle = S.bMissionAutoAngle;
    In.LegAngleDeg = S.MissionAngleDeg;
    In.bUsePassCount = S.bMissionUsePassCount;
    In.PassCount = S.MissionPassCount;
    In.SideOverlapPct = S.MissionSideOverlapPct;
    In.OvershootM = S.MissionOvershootM;
    In.bFixedHeading = S.bMissionFixedHeading;
    In.FixedHeadingDeg = S.MissionHeadingDeg;
    return SurveyGrid::PlanSurvey(Vertices, In);
}

inline FString Summary(const SurveyGrid::Plan& MissionPlan, const FDroneSimSettings& S)
{
    if (!MissionPlan.Error.empty())
    {
        return FString(TEXT("Mission: ")) + FromAscii(MissionPlan.Error);
    }
    const double OverlapPct =
        MissionPlan.FootprintCrossM > 0.0 ? 100.0 * (1.0 - MissionPlan.SpacingM / MissionPlan.FootprintCrossM) : 0.0;
    const double Minutes = MissionPlan.LengthM / FMath::Max(S.MissionSpeedMps, 0.1) / 60.0;
    const double GsdCm = 100.0 * MissionPlan.FootprintWidthM / FMath::Max(S.ImageWidthPx, 1);
    return FString::Printf(
        TEXT("Area %.2f km2, legs %.0f deg, %d passes %.0f m apart (side overlap %.0f %%)\n")
        TEXT("Footprint %.0f x %.0f m, %.0f cm/px; path %.1f km, about %.0f min, %d waypoints"),
        MissionPlan.AreaM2 / 1.0e6, MissionPlan.LegAngleDeg, MissionPlan.Passes, MissionPlan.SpacingM, OverlapPct, MissionPlan.FootprintWidthM,
        MissionPlan.FootprintHeightM, GsdCm, MissionPlan.LengthM / 1000.0, Minutes, static_cast<int32>(MissionPlan.Waypoints.size()));
}
} // namespace DroneMission
