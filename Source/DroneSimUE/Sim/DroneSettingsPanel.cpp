#include "Sim/DroneSettingsPanel.h"

#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Sim/DroneMission.h"
#include "Styling/CoreStyle.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Input/SEditableTextBox.h"
#include "Widgets/Input/SMultiLineEditableTextBox.h"
#include "Widgets/Input/SSpinBox.h"
#include "Widgets/Layout/SBorder.h"
#include "Widgets/Layout/SBox.h"
#include "Widgets/Layout/SScrollBox.h"
#include "Widgets/Layout/SWidgetSwitcher.h"
#include "Widgets/SBoxPanel.h"
#include "Widgets/SOverlay.h"
#include "Widgets/Text/STextBlock.h"

namespace
{
const TCHAR* const EncoderNames[] = {TEXT("h264_nvenc"), TEXT("libx264")};
constexpr double MetresPerDegree = 6378137.0 * 3.14159265358979323846 / 180.0;

const TCHAR* const KeysHint = TEXT("Esc (packaged program) / F2 / Tab: menu and back.  All keys: Controls.");

// --- JSON ------------------------------------------------------------------------

void WriteMapFields(FJsonObject& O, const FDroneSimSettings& S)
{
    O.SetStringField(TEXT("map_name"), S.MapName);
    O.SetNumberField(TEXT("start_latitude"), S.StartLatitude);
    O.SetNumberField(TEXT("start_longitude"), S.StartLongitude);
    O.SetNumberField(TEXT("altitude_m"), S.AltitudeM);
    O.SetNumberField(TEXT("start_heading_deg"), S.StartHeadingDeg);
    O.SetStringField(TEXT("mission_polygon"), S.MissionPolygon);
    O.SetNumberField(TEXT("mission_altitude_m"), S.MissionAltitudeM);
    O.SetNumberField(TEXT("mission_speed_mps"), S.MissionSpeedMps);
    O.SetBoolField(TEXT("mission_auto_angle"), S.bMissionAutoAngle);
    O.SetNumberField(TEXT("mission_angle_deg"), S.MissionAngleDeg);
    O.SetBoolField(TEXT("mission_use_pass_count"), S.bMissionUsePassCount);
    O.SetNumberField(TEXT("mission_pass_count"), S.MissionPassCount);
    O.SetNumberField(TEXT("mission_side_overlap_pct"), S.MissionSideOverlapPct);
    O.SetNumberField(TEXT("mission_overshoot_m"), S.MissionOvershootM);
    O.SetBoolField(TEXT("mission_fixed_heading"), S.bMissionFixedHeading);
    O.SetNumberField(TEXT("mission_heading_deg"), S.MissionHeadingDeg);
    O.SetNumberField(TEXT("mission_after"), S.MissionAfter);
    O.SetBoolField(TEXT("mission_jump_to_start"), S.bMissionJumpToStart);
    O.SetNumberField(TEXT("mission_square_size_m"), S.MissionSquareSizeM);
}

void ReadMapFields(const FJsonObject& O, FDroneSimSettings& S)
{
    O.TryGetStringField(TEXT("map_name"), S.MapName);
    O.TryGetNumberField(TEXT("start_latitude"), S.StartLatitude);
    O.TryGetNumberField(TEXT("start_longitude"), S.StartLongitude);
    O.TryGetNumberField(TEXT("altitude_m"), S.AltitudeM);
    O.TryGetNumberField(TEXT("start_heading_deg"), S.StartHeadingDeg);
    O.TryGetStringField(TEXT("mission_polygon"), S.MissionPolygon);
    O.TryGetNumberField(TEXT("mission_altitude_m"), S.MissionAltitudeM);
    O.TryGetNumberField(TEXT("mission_speed_mps"), S.MissionSpeedMps);
    O.TryGetBoolField(TEXT("mission_auto_angle"), S.bMissionAutoAngle);
    O.TryGetNumberField(TEXT("mission_angle_deg"), S.MissionAngleDeg);
    O.TryGetBoolField(TEXT("mission_use_pass_count"), S.bMissionUsePassCount);
    O.TryGetNumberField(TEXT("mission_pass_count"), S.MissionPassCount);
    O.TryGetNumberField(TEXT("mission_side_overlap_pct"), S.MissionSideOverlapPct);
    O.TryGetNumberField(TEXT("mission_overshoot_m"), S.MissionOvershootM);
    O.TryGetBoolField(TEXT("mission_fixed_heading"), S.bMissionFixedHeading);
    O.TryGetNumberField(TEXT("mission_heading_deg"), S.MissionHeadingDeg);
    O.TryGetNumberField(TEXT("mission_after"), S.MissionAfter);
    O.TryGetBoolField(TEXT("mission_jump_to_start"), S.bMissionJumpToStart);
    O.TryGetNumberField(TEXT("mission_square_size_m"), S.MissionSquareSizeM);
    S.MissionAfter = FMath::Clamp(S.MissionAfter, 0, 2);
}

bool WriteJson(const TSharedRef<FJsonObject>& O, const FString& Path)
{
    FString Out;
    const TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Out);
    if (!FJsonSerializer::Serialize(O, Writer))
    {
        return false;
    }
    return FFileHelper::SaveStringToFile(Out, *Path);
}

TSharedPtr<FJsonObject> ReadJson(const FString& Path)
{
    FString FileText;
    if (!FFileHelper::LoadFileToString(FileText, *Path))
    {
        return nullptr;
    }
    TSharedPtr<FJsonObject> O;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(FileText);
    if (!FJsonSerializer::Deserialize(Reader, O) || !O.IsValid())
    {
        return nullptr;
    }
    return O;
}
} // namespace

bool FDroneSimSettings::SaveToFile(const FString& Path) const
{
    TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
    WriteMapFields(*O, *this);
    O->SetNumberField(TEXT("speed_mps"), SpeedMps);
    O->SetBoolField(TEXT("manual"), bManual);
    O->SetNumberField(TEXT("manual_max_speed_mps"), ManualMaxSpeedMps);
    O->SetNumberField(TEXT("manual_acceleration_mps2"), ManualAccelerationMps2);
    O->SetNumberField(TEXT("manual_braking_mps2"), ManualBrakingMps2);
    O->SetNumberField(TEXT("manual_fast_multiplier"), ManualFastMultiplier);
    O->SetNumberField(TEXT("manual_yaw_rate_dps"), ManualYawRateDegPerSec);
    O->SetNumberField(TEXT("manual_climb_rate_mps"), ManualClimbRateMps);
    O->SetNumberField(TEXT("manual_vertical_acceleration_mps2"), ManualVerticalAccelerationMps2);
    O->SetNumberField(TEXT("stick_dead_zone"), StickDeadZone);
    O->SetNumberField(TEXT("stick_expo"), StickExpo);
    O->SetNumberField(TEXT("camera_tilt_rate_dps"), CameraTiltRateDegPerSec);
    O->SetNumberField(TEXT("fpv_thrust_to_weight"), FpvThrustToWeight);
    O->SetNumberField(TEXT("fpv_roll_pitch_center_dps"), FpvRollPitchCenterRateDps);
    O->SetNumberField(TEXT("fpv_roll_pitch_max_dps"), FpvRollPitchMaxRateDps);
    O->SetNumberField(TEXT("fpv_roll_pitch_expo"), FpvRollPitchExpo);
    O->SetNumberField(TEXT("fpv_yaw_center_dps"), FpvYawCenterRateDps);
    O->SetNumberField(TEXT("fpv_yaw_max_dps"), FpvYawMaxRateDps);
    O->SetNumberField(TEXT("fpv_yaw_expo"), FpvYawExpo);
    O->SetNumberField(TEXT("fpv_camera_uptilt_deg"), FpvCameraUptiltDeg);
    O->SetNumberField(TEXT("fpv_camera_fov_deg"), FpvCameraFovDeg);
    O->SetNumberField(TEXT("fpv_keyboard_stick"), FpvKeyboardStick);
    O->SetNumberField(TEXT("fpv_throttle_key_rate"), FpvThrottleKeyRate);
    O->SetNumberField(TEXT("fpv_crash_speed_mps"), FpvCrashSpeedMps);
    O->SetBoolField(TEXT("camera_gimbal"), bCameraGimbal);
    O->SetNumberField(TEXT("exposure_compensation_ev"), ExposureCompensationEV);
    O->SetNumberField(TEXT("video_mode"), VideoMode);
    O->SetStringField(TEXT("ffmpeg_path"), FfmpegPath);
    O->SetStringField(TEXT("video_encoder"), VideoEncoder);
    O->SetStringField(TEXT("rtsp_url"), RtspUrl);
    O->SetBoolField(TEXT("start_rtsp_server"), bStartRtspServer);
    O->SetStringField(TEXT("rtsp_server_path"), RtspServerPath);
    O->SetStringField(TEXT("mavlink_host"), MavlinkHost);
    O->SetNumberField(TEXT("mavlink_port"), MavlinkPort);
    O->SetNumberField(TEXT("time_zone"), TimeZone);
    O->SetNumberField(TEXT("solar_time"), SolarTime);
    O->SetNumberField(TEXT("day"), Day);
    O->SetNumberField(TEXT("month"), Month);
    O->SetNumberField(TEXT("year"), Year);
    O->SetBoolField(TEXT("daylight_saving"), bDaylightSaving);
    O->SetBoolField(TEXT("show_area_poles"), bShowAreaPoles);
    O->SetBoolField(TEXT("show_area_walls"), bShowAreaWalls);
    O->SetNumberField(TEXT("area_wall_opacity_pct"), AreaWallOpacityPct);
    O->SetBoolField(TEXT("markers_in_video"), bMarkersInVideo);
    O->SetNumberField(TEXT("map_quality"), MapQuality);
    O->SetBoolField(TEXT("forbid_holes"), bForbidHoles);
    O->SetBoolField(TEXT("preload_area"), bPreloadArea);
    O->SetNumberField(TEXT("preload_margin_m"), PreloadMarginM);
    O->SetNumberField(TEXT("tile_memory_cache_mb"), TileMemoryCacheMB);
    O->SetNumberField(TEXT("max_simultaneous_tile_loads"), MaxSimultaneousTileLoads);
    O->SetNumberField(TEXT("around_drone_detail"), AroundDroneDetail);
    O->SetNumberField(TEXT("tile_detail_at_once"), TileLoadingStyle); // was tile_loading_style (default changed)
    O->SetNumberField(TEXT("imagery"), ImagerySource); // was imagery_source (default changed to Bing)
    O->SetNumberField(TEXT("esri_max_level"), EsriMaxLevel);
    O->SetStringField(TEXT("tile_store_dir"), TileStoreDir);
    O->SetNumberField(TEXT("tile_store_port"), TileStorePort);
    O->SetNumberField(TEXT("wayback_release"), WaybackRelease);
    O->SetStringField(TEXT("wayback_label"), WaybackLabel);
    return WriteJson(O, Path);
}

bool FDroneSimSettings::LoadFromFile(const FString& Path)
{
    const TSharedPtr<FJsonObject> O = ReadJson(Path);
    if (!O.IsValid())
    {
        return false;
    }
    ReadMapFields(*O, *this);
    O->TryGetNumberField(TEXT("speed_mps"), SpeedMps);
    O->TryGetBoolField(TEXT("manual"), bManual);
    O->TryGetNumberField(TEXT("manual_max_speed_mps"), ManualMaxSpeedMps);
    O->TryGetNumberField(TEXT("manual_acceleration_mps2"), ManualAccelerationMps2);
    O->TryGetNumberField(TEXT("manual_braking_mps2"), ManualBrakingMps2);
    O->TryGetNumberField(TEXT("manual_fast_multiplier"), ManualFastMultiplier);
    O->TryGetNumberField(TEXT("manual_yaw_rate_dps"), ManualYawRateDegPerSec);
    O->TryGetNumberField(TEXT("manual_climb_rate_mps"), ManualClimbRateMps);
    O->TryGetNumberField(TEXT("manual_vertical_acceleration_mps2"), ManualVerticalAccelerationMps2);
    O->TryGetNumberField(TEXT("stick_dead_zone"), StickDeadZone);
    O->TryGetNumberField(TEXT("stick_expo"), StickExpo);
    O->TryGetNumberField(TEXT("camera_tilt_rate_dps"), CameraTiltRateDegPerSec);
    O->TryGetNumberField(TEXT("fpv_thrust_to_weight"), FpvThrustToWeight);
    O->TryGetNumberField(TEXT("fpv_roll_pitch_center_dps"), FpvRollPitchCenterRateDps);
    O->TryGetNumberField(TEXT("fpv_roll_pitch_max_dps"), FpvRollPitchMaxRateDps);
    O->TryGetNumberField(TEXT("fpv_roll_pitch_expo"), FpvRollPitchExpo);
    O->TryGetNumberField(TEXT("fpv_yaw_center_dps"), FpvYawCenterRateDps);
    O->TryGetNumberField(TEXT("fpv_yaw_max_dps"), FpvYawMaxRateDps);
    O->TryGetNumberField(TEXT("fpv_yaw_expo"), FpvYawExpo);
    O->TryGetNumberField(TEXT("fpv_camera_uptilt_deg"), FpvCameraUptiltDeg);
    O->TryGetNumberField(TEXT("fpv_camera_fov_deg"), FpvCameraFovDeg);
    O->TryGetNumberField(TEXT("fpv_keyboard_stick"), FpvKeyboardStick);
    O->TryGetNumberField(TEXT("fpv_throttle_key_rate"), FpvThrottleKeyRate);
    O->TryGetNumberField(TEXT("fpv_crash_speed_mps"), FpvCrashSpeedMps);
    O->TryGetBoolField(TEXT("camera_gimbal"), bCameraGimbal);
    O->TryGetNumberField(TEXT("exposure_compensation_ev"), ExposureCompensationEV);
    O->TryGetNumberField(TEXT("video_mode"), VideoMode);
    O->TryGetStringField(TEXT("ffmpeg_path"), FfmpegPath);
    O->TryGetStringField(TEXT("video_encoder"), VideoEncoder);
    O->TryGetStringField(TEXT("rtsp_url"), RtspUrl);
    O->TryGetBoolField(TEXT("start_rtsp_server"), bStartRtspServer);
    O->TryGetStringField(TEXT("rtsp_server_path"), RtspServerPath);
    O->TryGetStringField(TEXT("mavlink_host"), MavlinkHost);
    O->TryGetNumberField(TEXT("mavlink_port"), MavlinkPort);
    O->TryGetNumberField(TEXT("time_zone"), TimeZone);
    O->TryGetNumberField(TEXT("solar_time"), SolarTime);
    O->TryGetNumberField(TEXT("day"), Day);
    O->TryGetNumberField(TEXT("month"), Month);
    O->TryGetNumberField(TEXT("year"), Year);
    O->TryGetBoolField(TEXT("daylight_saving"), bDaylightSaving);
    O->TryGetBoolField(TEXT("show_area_poles"), bShowAreaPoles);
    O->TryGetBoolField(TEXT("show_area_walls"), bShowAreaWalls);
    O->TryGetNumberField(TEXT("area_wall_opacity_pct"), AreaWallOpacityPct);
    O->TryGetBoolField(TEXT("markers_in_video"), bMarkersInVideo);
    O->TryGetNumberField(TEXT("map_quality"), MapQuality);
    O->TryGetBoolField(TEXT("forbid_holes"), bForbidHoles);
    O->TryGetBoolField(TEXT("preload_area"), bPreloadArea);
    O->TryGetNumberField(TEXT("preload_margin_m"), PreloadMarginM);
    O->TryGetNumberField(TEXT("tile_memory_cache_mb"), TileMemoryCacheMB);
    O->TryGetNumberField(TEXT("max_simultaneous_tile_loads"), MaxSimultaneousTileLoads);
    O->TryGetNumberField(TEXT("around_drone_detail"), AroundDroneDetail);
    O->TryGetNumberField(TEXT("tile_detail_at_once"), TileLoadingStyle);
    O->TryGetNumberField(TEXT("imagery"), ImagerySource);
    O->TryGetNumberField(TEXT("esri_max_level"), EsriMaxLevel);
    O->TryGetStringField(TEXT("tile_store_dir"), TileStoreDir);
    O->TryGetNumberField(TEXT("tile_store_port"), TileStorePort);
    O->TryGetNumberField(TEXT("wayback_release"), WaybackRelease);
    O->TryGetStringField(TEXT("wayback_label"), WaybackLabel);
    ImagerySource = FMath::Clamp(ImagerySource, 0, 2);
    WaybackRelease = FMath::Max(WaybackRelease, 0);
    EsriMaxLevel = FMath::Clamp(EsriMaxLevel, 10, 23);
    TileStorePort = FMath::Clamp(TileStorePort, 1024, 65535);
    MapQuality = FMath::Clamp(MapQuality, 0, 2);
    AroundDroneDetail = FMath::Clamp(AroundDroneDetail, 0, 3);
    TileLoadingStyle = FMath::Clamp(TileLoadingStyle, 0, 1);
    VideoMode = FMath::Clamp(VideoMode, 0, 2);
    MavlinkPort = FMath::Clamp(MavlinkPort, 1, 65535);
    return true;
}

// --- Map library ----------------------------------------------------------------

namespace DroneMaps
{
FString MapsDir()
{
    return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Maps"));
}

FString FileNameFor(const FString& Name)
{
    FString Clean = FPaths::MakeValidFileName(Name.TrimStartAndEnd(), TEXT('_'));
    return Clean.IsEmpty() ? FString(TEXT("Map")) : Clean;
}

TArray<FString> ListMaps()
{
    TArray<FString> Files;
    IFileManager::Get().FindFiles(Files, *(MapsDir() / TEXT("*.json")), true, false);
    TArray<FString> Names;
    for (const FString& FoundFile : Files)
    {
        Names.Add(FPaths::GetBaseFilename(FoundFile));
    }
    Names.Sort();
    return Names;
}

bool SaveMap(const FDroneSimSettings& S)
{
    IFileManager::Get().MakeDirectory(*MapsDir(), true);
    TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
    WriteMapFields(*O, S);
    return WriteJson(O, MapsDir() / (FileNameFor(S.MapName) + TEXT(".json")));
}

bool LoadMap(const FString& Name, FDroneSimSettings& InOut)
{
    const TSharedPtr<FJsonObject> O = ReadJson(MapsDir() / (FileNameFor(Name) + TEXT(".json")));
    if (!O.IsValid())
    {
        return false;
    }
    // Keys missing from a (hand-written) map file get the defaults, not the previous map's values.
    FDroneSimSettings Fresh;
    CopyMapFields(Fresh, InOut);
    ReadMapFields(*O, InOut);
    InOut.MapName = FileNameFor(Name);
    return true;
}

bool DeleteMap(const FString& Name)
{
    return IFileManager::Get().Delete(*(MapsDir() / (FileNameFor(Name) + TEXT(".json"))), false, false, true);
}

void CopyMissionFields(const FDroneSimSettings& From, FDroneSimSettings& To)
{
    To.MissionPolygon = From.MissionPolygon;
    To.MissionAltitudeM = From.MissionAltitudeM;
    To.MissionSpeedMps = From.MissionSpeedMps;
    To.bMissionAutoAngle = From.bMissionAutoAngle;
    To.MissionAngleDeg = From.MissionAngleDeg;
    To.bMissionUsePassCount = From.bMissionUsePassCount;
    To.MissionPassCount = From.MissionPassCount;
    To.MissionSideOverlapPct = From.MissionSideOverlapPct;
    To.MissionOvershootM = From.MissionOvershootM;
    To.bMissionFixedHeading = From.bMissionFixedHeading;
    To.MissionHeadingDeg = From.MissionHeadingDeg;
    To.MissionAfter = From.MissionAfter;
    To.bMissionJumpToStart = From.bMissionJumpToStart;
    To.MissionSquareSizeM = From.MissionSquareSizeM;
}

void CopyMapFields(const FDroneSimSettings& From, FDroneSimSettings& To)
{
    To.MapName = From.MapName;
    To.StartLatitude = From.StartLatitude;
    To.StartLongitude = From.StartLongitude;
    To.AltitudeM = From.AltitudeM;
    To.StartHeadingDeg = From.StartHeadingDeg;
    CopyMissionFields(From, To);
}

void CopyControlFields(const FDroneSimSettings& From, FDroneSimSettings& To)
{
    To.ManualMaxSpeedMps = From.ManualMaxSpeedMps;
    To.ManualAccelerationMps2 = From.ManualAccelerationMps2;
    To.ManualBrakingMps2 = From.ManualBrakingMps2;
    To.ManualFastMultiplier = From.ManualFastMultiplier;
    To.ManualYawRateDegPerSec = From.ManualYawRateDegPerSec;
    To.ManualClimbRateMps = From.ManualClimbRateMps;
    To.ManualVerticalAccelerationMps2 = From.ManualVerticalAccelerationMps2;
    To.StickDeadZone = From.StickDeadZone;
    To.StickExpo = From.StickExpo;
    To.CameraTiltRateDegPerSec = From.CameraTiltRateDegPerSec;
    To.FpvThrustToWeight = From.FpvThrustToWeight;
    To.FpvRollPitchCenterRateDps = From.FpvRollPitchCenterRateDps;
    To.FpvRollPitchMaxRateDps = From.FpvRollPitchMaxRateDps;
    To.FpvRollPitchExpo = From.FpvRollPitchExpo;
    To.FpvYawCenterRateDps = From.FpvYawCenterRateDps;
    To.FpvYawMaxRateDps = From.FpvYawMaxRateDps;
    To.FpvYawExpo = From.FpvYawExpo;
    To.FpvCameraUptiltDeg = From.FpvCameraUptiltDeg;
    To.FpvCameraFovDeg = From.FpvCameraFovDeg;
    To.FpvKeyboardStick = From.FpvKeyboardStick;
    To.FpvThrottleKeyRate = From.FpvThrottleKeyRate;
    To.FpvCrashSpeedMps = From.FpvCrashSpeedMps;
}

void AppendVertex(FString& VertexText, double Lat, double Lon)
{
    FString Trimmed = VertexText.TrimStartAndEnd();
    if (!Trimmed.IsEmpty() && !Trimmed.EndsWith(TEXT(";")))
    {
        Trimmed += TEXT(";");
    }
    if (!Trimmed.IsEmpty())
    {
        Trimmed += TEXT("\n");
    }
    VertexText = Trimmed + FString::Printf(TEXT("%.6f, %.6f"), Lat, Lon);
}

int32 CountVertices(const FString& VertexText)
{
    std::vector<SurveyGrid::LatLon> Vertices;
    std::string ParseError;
    if (!SurveyGrid::ParseLatLonList(DroneMission::ToAscii(VertexText), Vertices, ParseError))
    {
        return 0;
    }
    return static_cast<int32>(Vertices.size());
}

FString SquareAround(double Lat, double Lon, double SizeM)
{
    const double HalfM = 0.5 * FMath::Max(SizeM, 10.0);
    const double DLat = HalfM / MetresPerDegree;
    const double DLon = HalfM / (MetresPerDegree * FMath::Cos(FMath::DegreesToRadians(Lat)));
    return FString::Printf(TEXT("%.6f, %.6f;\n%.6f, %.6f;\n%.6f, %.6f;\n%.6f, %.6f"), Lat - DLat, Lon - DLon,
                           Lat + DLat, Lon - DLon, Lat + DLat, Lon + DLon, Lat - DLat, Lon + DLon);
}

void EnsureDefaultMaps(const FDroneSimSettings& Current)
{
    if (IFileManager::Get().DirectoryExists(*MapsDir()))
    {
        return;
    }
    IFileManager::Get().MakeDirectory(*MapsDir(), true);
    FDroneSimSettings Defaults;
    {
        // FlightSimulator's test area: the drone's default 2 x 2 km route.
        FDroneSimSettings TestArea = Defaults;
        TestArea.MapName = TEXT("Test area 2x2 km");
        TestArea.StartLatitude = 48.365;
        TestArea.StartLongitude = 26.065;
        TestArea.AltitudeM = 500.0;
        TestArea.MissionPolygon =
            TEXT("48.365000, 26.065000;\n48.365000, 26.092000;\n48.383000, 26.092000;\n48.383000, 26.065000");
        SaveMap(TestArea);
    }
    if (FMath::Abs(Current.StartLatitude - 48.365) > 1e-4 || FMath::Abs(Current.StartLongitude - 26.065) > 1e-4)
    {
        FDroneSimSettings AroundStart = Defaults;
        AroundStart.MapName = TEXT("Start point 2x2 km");
        AroundStart.StartLatitude = Current.StartLatitude;
        AroundStart.StartLongitude = Current.StartLongitude;
        AroundStart.AltitudeM = Current.AltitudeM;
        AroundStart.StartHeadingDeg = Current.StartHeadingDeg;
        AroundStart.MissionPolygon = SquareAround(Current.StartLatitude, Current.StartLongitude, 2000.0);
        SaveMap(AroundStart);
    }
    if (!Current.MissionPolygon.TrimStartAndEnd().IsEmpty())
    {
        FDroneSimSettings LastMission = Current;
        LastMission.MapName = TEXT("Last mission");
        SaveMap(LastMission);
    }
}
} // namespace DroneMaps

// --- Menu building blocks ---------------------------------------------------------

namespace
{
FSlateFontInfo PanelFont(int32 Size = 10)
{
    return FCoreStyle::GetDefaultFontStyle("Regular", Size);
}

FText Txt(const TCHAR* S)
{
    return FText::FromString(S);
}

// Buttons do not take keyboard focus, so Space never clicks one.
TSharedRef<SHorizontalBox> ChoiceButtons(int32* Value, const TArray<FText>& Options, TFunction<void()> OnChanged = nullptr)
{
    TSharedRef<SHorizontalBox> Buttons = SNew(SHorizontalBox);
    for (int32 Index = 0; Index < Options.Num(); ++Index)
    {
        Buttons->AddSlot()
            .FillWidth(1.f)
            .Padding(1.f)
            [
                SNew(SButton)
                .HAlign(HAlign_Center)
                .IsFocusable(false)
                .Text(Options[Index])
                .ButtonColorAndOpacity_Lambda([Value, Index]()
                {
                    return FSlateColor(*Value == Index ? FLinearColor(0.15f, 0.6f, 0.2f) : FLinearColor(0.3f, 0.3f, 0.3f));
                })
                .OnClicked_Lambda([Value, Index, OnChanged]()
                {
                    const bool bChanged = *Value != Index;
                    *Value = Index;
                    if (bChanged && OnChanged)
                    {
                        OnChanged();
                    }
                    return FReply::Handled();
                })
            ];
    }
    return Buttons;
}
} // namespace

TSharedRef<SWidget> SDroneSettingsPanel::Header(const FText& Title)
{
    return SNew(STextBlock)
        .Text(Title)
        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 11))
        .ColorAndOpacity(FLinearColor(0.85f, 0.95f, 1.f));
}

TSharedRef<SWidget> SDroneSettingsPanel::PageTitle(const FText& Title)
{
    return SNew(STextBlock)
        .Text(Title)
        .Font(FCoreStyle::GetDefaultFontStyle("Bold", 22))
        .ColorAndOpacity(FLinearColor(0.9f, 0.97f, 1.f));
}

TSharedRef<SWidget> SDroneSettingsPanel::Hint(const FText& Message)
{
    return SNew(STextBlock)
        .Text(Message)
        .Font(PanelFont(9))
        .AutoWrapText(true)
        .ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f));
}

TSharedRef<SWidget> SDroneSettingsPanel::ActionButton(const FText& Label, TFunction<void()> OnClick)
{
    return SNew(SButton)
        .HAlign(HAlign_Center)
        .IsFocusable(false)
        .Text(Label)
        .OnClicked_Lambda([Callback = MoveTemp(OnClick)]()
        {
            Callback();
            return FReply::Handled();
        });
}

TSharedRef<SWidget> SDroneSettingsPanel::BigButton(const FText& Label, TFunction<void()> OnClick,
                                                   TFunction<bool()> IsEnabledFn)
{
    return SNew(SBox)
        .HeightOverride(42.f)
        .IsEnabled_Lambda([EnabledFn = MoveTemp(IsEnabledFn)]() { return EnabledFn ? EnabledFn() : true; })
        [
            SNew(SButton)
            .HAlign(HAlign_Center)
            .VAlign(VAlign_Center)
            .IsFocusable(false)
            .OnClicked_Lambda([Callback = MoveTemp(OnClick)]()
            {
                Callback();
                return FReply::Handled();
            })
            [
                SNew(STextBlock)
                .Text(Label)
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 13))
            ]
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::Card(const TSharedRef<SWidget>& Inner)
{
    return SNew(SBorder)
        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor(0.02f, 0.025f, 0.03f, 0.93f))
        .Padding(FMargin(20.f, 16.f))
        [
            Inner
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::NumberRow(const FText& Label, double* Value, double Min, double Max,
                                                   double Delta, int32 Digits)
{
    return SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
        [
            SNew(STextBlock).Text(Label).Font(PanelFont()).ColorAndOpacity(FLinearColor::White)
        ]
        + SHorizontalBox::Slot().FillWidth(0.45f)
        [
            SNew(SSpinBox<double>)
            .Value_Lambda([Value]() { return *Value; })
            .OnValueChanged_Lambda([Value](double V) { *Value = V; })
            .OnValueCommitted_Lambda([Value](double V, ETextCommit::Type) { *Value = V; })
            .MinValue(Min)
            .MaxValue(Max)
            .Delta(Delta)
            .MinFractionalDigits(Digits)
            .MaxFractionalDigits(Digits)
            .EnableSlider(false)
            .Font(PanelFont())
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::IntRow(const FText& Label, int32* Value, int32 Min, int32 Max)
{
    return SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(0.55f).VAlign(VAlign_Center)
        [
            SNew(STextBlock).Text(Label).Font(PanelFont()).ColorAndOpacity(FLinearColor::White)
        ]
        + SHorizontalBox::Slot().FillWidth(0.45f)
        [
            SNew(SSpinBox<int32>)
            .Value_Lambda([Value]() { return *Value; })
            .OnValueChanged_Lambda([Value](int32 V) { *Value = V; })
            .OnValueCommitted_Lambda([Value](int32 V, ETextCommit::Type) { *Value = V; })
            .MinValue(Min)
            .MaxValue(Max)
            .Delta(1)
            .EnableSlider(false)
            .Font(PanelFont())
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::TextRow(const FText& Label, FString* Value)
{
    return SNew(SHorizontalBox)
        + SHorizontalBox::Slot().FillWidth(0.3f).VAlign(VAlign_Center)
        [
            SNew(STextBlock).Text(Label).Font(PanelFont()).ColorAndOpacity(FLinearColor::White)
        ]
        + SHorizontalBox::Slot().FillWidth(0.7f)
        [
            SNew(SEditableTextBox)
            .Text_Lambda([Value]() { return FText::FromString(*Value); })
            .OnTextChanged_Lambda([Value](const FText& NewText) { *Value = NewText.ToString(); })
            .OnTextCommitted_Lambda([Value](const FText& NewText, ETextCommit::Type) { *Value = NewText.ToString(); })
            .Font(PanelFont(10))
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::ChoiceRow(const FText& Label, int32* Value, const TArray<FText>& Options,
                                                   TFunction<void()> OnChanged)
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()
        [
            SNew(STextBlock).Text(Label).Font(PanelFont()).ColorAndOpacity(FLinearColor::White)
        ]
        + SVerticalBox::Slot().AutoHeight()
        [
            ChoiceButtons(Value, Options, MoveTemp(OnChanged))
        ];
}

// --- State ------------------------------------------------------------------------------

void SDroneSettingsPanel::SyncChoices()
{
    GimbalChoice = Settings.bCameraGimbal ? 0 : 1;
    EncoderChoice = Settings.VideoEncoder.Contains(TEXT("nvenc")) ? 0 : 1;
    RtspServerChoice = Settings.bStartRtspServer ? 1 : 0;
    DstChoice = Settings.bDaylightSaving ? 1 : 0;
    MissionAngleChoice = Settings.bMissionAutoAngle ? 0 : 1;
    MissionSpacingChoice = Settings.bMissionUsePassCount ? 1 : 0;
    MissionHeadingChoice = Settings.bMissionFixedHeading ? 1 : 0;
    MissionStartChoice = Settings.bMissionJumpToStart ? 1 : 0;
    PolesChoice = Settings.bShowAreaPoles ? 1 : 0;
    WallsChoice = Settings.bShowAreaWalls ? 1 : 0;
    MarkersVideoChoice = Settings.bMarkersInVideo ? 1 : 0;
    ForbidHolesChoice = Settings.bForbidHoles ? 1 : 0;
    PreloadChoice = Settings.bPreloadArea ? 1 : 0;
}

void SDroneSettingsPanel::CommitChoices()
{
    Settings.bManual = true;
    Settings.bCameraGimbal = GimbalChoice == 0;
    Settings.VideoEncoder = EncoderNames[FMath::Clamp(EncoderChoice, 0, 1)];
    Settings.bStartRtspServer = RtspServerChoice == 1;
    Settings.bDaylightSaving = DstChoice == 1;
    Settings.bMissionAutoAngle = MissionAngleChoice == 0;
    Settings.bMissionUsePassCount = MissionSpacingChoice == 1;
    Settings.bMissionFixedHeading = MissionHeadingChoice == 1;
    Settings.bMissionJumpToStart = MissionStartChoice == 1;
    Settings.bShowAreaPoles = PolesChoice == 1;
    Settings.bShowAreaWalls = WallsChoice == 1;
    Settings.bMarkersInVideo = MarkersVideoChoice == 1;
    Settings.bForbidHoles = ForbidHolesChoice == 1;
    Settings.bPreloadArea = PreloadChoice == 1;
}

FDroneSimSettings SDroneSettingsPanel::GetDraft()
{
    CommitChoices();
    return Settings;
}

void SDroneSettingsPanel::Fire(EDroneMenuAction Action)
{
    CommitChoices();
    OnAction.ExecuteIfBound(Action, Settings);
}

int32 SDroneSettingsPanel::RootPage() const
{
    return bInFlight ? PagePause : PageMain;
}

bool SDroneSettingsPanel::GoBack()
{
    if (ActivePage == PageSettings)
    {
        LeaveSettings(/*bSave*/ false);
        return true;
    }
    if (ActivePage == PageMaps)
    {
        LeaveMaps();
        return true;
    }
    if (ActivePage == PageHelp || ActivePage == PageDates)
    {
        ActivePage = RootPage();
        return true;
    }
    return false;
}

FText SDroneSettingsPanel::MissionSummary()
{
    CommitChoices();
    bMissionAreaEmpty = Settings.MissionPolygon.TrimStartAndEnd().IsEmpty();
    if (bMissionAreaEmpty)
    {
        bMissionPlanOk = false;
        return Txt(TEXT("No survey area on this map: free flight only (add the area in Maps, or fly to the corners and press V)."));
    }
    const SurveyGrid::Plan MissionPlan = DroneMission::PlanMission(Settings);
    bMissionPlanOk = MissionPlan.Error.empty();
    return FText::FromString(DroneMission::Summary(MissionPlan, Settings));
}

FText SDroneSettingsPanel::MapLine() const
{
    return FText::FromString(FString::Printf(TEXT("Map: %s     start %.6f, %.6f, %.0f m, heading %.0f"),
                                             Settings.MapName.IsEmpty() ? TEXT("(not saved)") : *Settings.MapName,
                                             Settings.StartLatitude, Settings.StartLongitude, Settings.AltitudeM,
                                             Settings.StartHeadingDeg));
}

void SDroneSettingsPanel::AddDronePositionVertex()
{
    if (DronePosition)
    {
        const FVector2D LatLon = DronePosition();
        DroneMaps::AppendVertex(Settings.MissionPolygon, LatLon.X, LatLon.Y);
    }
}

void SDroneSettingsPanel::StartAtDronePosition()
{
    if (DronePosition)
    {
        const FVector2D LatLon = DronePosition();
        Settings.StartLatitude = LatLon.X;
        Settings.StartLongitude = LatLon.Y;
    }
}

// --- Maps ------------------------------------------------------------------------------------

void SDroneSettingsPanel::OpenMaps()
{
    ActivePage = PageMaps;
    MapNote.Reset();
    RefreshMapList();
}

void SDroneSettingsPanel::LeaveMaps()
{
    ActivePage = RootPage();
    Fire(EDroneMenuAction::PreviewMap); // the drone moves if the start point changed
}

void SDroneSettingsPanel::RefreshMapList()
{
    if (!MapListBox.IsValid())
    {
        return;
    }
    MapListBox->ClearChildren();
    const TArray<FString> Names = DroneMaps::ListMaps();
    if (Names.Num() == 0)
    {
        MapListBox->AddSlot().AutoHeight()[ Hint(Txt(TEXT("No saved maps yet."))) ];
    }
    for (const FString& MapEntry : Names)
    {
        MapListBox->AddSlot()
            .AutoHeight()
            .Padding(FMargin(0.f, 1.f))
            [
                SNew(SButton)
                .IsFocusable(false)
                .Text(FText::FromString(MapEntry))
                .ButtonColorAndOpacity_Lambda([this, MapEntry]()
                {
                    return FSlateColor(DroneMaps::FileNameFor(Settings.MapName) == MapEntry ? FLinearColor(0.15f, 0.6f, 0.2f)
                                                                                            : FLinearColor(0.3f, 0.3f, 0.3f));
                })
                .OnClicked_Lambda([this, MapEntry]()
                {
                    SelectMap(MapEntry);
                    return FReply::Handled();
                })
            ];
    }
}

void SDroneSettingsPanel::SelectMap(const FString& MapEntry)
{
    FDroneSimSettings Loaded = Settings;
    if (!DroneMaps::LoadMap(MapEntry, Loaded))
    {
        MapNote = FString::Printf(TEXT("Could not read %s.json"), *MapEntry);
        return;
    }
    Settings = Loaded;
    SyncChoices();
    MapNote = FString::Printf(TEXT("Loaded \"%s\"."), *MapEntry);
    Fire(EDroneMenuAction::PreviewMap); // the drone moves to its start point
}

void SDroneSettingsPanel::NewMap()
{
    FDroneSimSettings Fresh;
    Fresh.StartLatitude = Settings.StartLatitude;
    Fresh.StartLongitude = Settings.StartLongitude;
    Fresh.AltitudeM = Settings.AltitudeM;
    Fresh.StartHeadingDeg = Settings.StartHeadingDeg;
    if (bInFlight && DronePosition)
    {
        const FVector2D LatLon = DronePosition();
        Fresh.StartLatitude = LatLon.X;
        Fresh.StartLongitude = LatLon.Y;
    }
    Fresh.MapName = TEXT("New map");
    DroneMaps::CopyMapFields(Fresh, Settings);
    SyncChoices();
    MapNote = TEXT("New map: give it a name, set the start point and the area, then Save.");
}

void SDroneSettingsPanel::SaveCurrentMap()
{
    CommitChoices();
    if (Settings.MapName.TrimStartAndEnd().IsEmpty())
    {
        Settings.MapName = TEXT("New map");
    }
    Settings.MapName = DroneMaps::FileNameFor(Settings.MapName);
    MapNote = DroneMaps::SaveMap(Settings)
        ? FString::Printf(TEXT("Saved %s"), *(DroneMaps::MapsDir() / (Settings.MapName + TEXT(".json"))))
        : FString(TEXT("Could not save the map (see the log)."));
    RefreshMapList();
    Fire(EDroneMenuAction::PreviewMap); // the drone moves if the start point changed
}

void SDroneSettingsPanel::DeleteCurrentMap()
{
    const FString Target = DroneMaps::FileNameFor(Settings.MapName);
    MapNote = DroneMaps::DeleteMap(Target)
        ? FString::Printf(TEXT("Deleted \"%s\" (still shown here: Save brings it back)."), *Target)
        : FString::Printf(TEXT("There is no saved map \"%s\"."), *Target);
    RefreshMapList();
}

// --- Settings page ---------------------------------------------------------------------------

void SDroneSettingsPanel::EnterSettings()
{
    CommitChoices();
    SettingsBackup = Settings;
    ActivePage = PageSettings;
}

void SDroneSettingsPanel::LeaveSettings(bool bSave)
{
    if (bSave)
    {
        Fire(EDroneMenuAction::ApplySettings);
    }
    else
    {
        // Cancel: back to the settings as they were; map edits made elsewhere stay.
        CommitChoices();
        const FDroneSimSettings Edited = Settings;
        Settings = SettingsBackup;
        DroneMaps::CopyMapFields(Edited, Settings);
        SyncChoices();
    }
    ActivePage = RootPage();
}

// --- Pages -------------------------------------------------------------------------------------

TSharedRef<SWidget> SDroneSettingsPanel::MapInfo()
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()
        [
            SNew(STextBlock)
            .Text_Lambda([this]() { return MapLine(); })
            .Font(PanelFont(10))
            .AutoWrapText(true)
            .ColorAndOpacity(FLinearColor::White)
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 3.f))
        [
            SNew(STextBlock)
            .Text_Lambda([this]() { return MissionSummary(); })
            .Font(PanelFont(9))
            .AutoWrapText(true)
            .ColorAndOpacity_Lambda([this]()
            {
                if (bMissionAreaEmpty)
                {
                    return FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f));
                }
                return FSlateColor(bMissionPlanOk ? FLinearColor(0.55f, 1.f, 0.6f) : FLinearColor(1.f, 0.45f, 0.4f));
            })
        ]
        + SVerticalBox::Slot().AutoHeight()
        [
            SNew(STextBlock)
            .Text_Lambda([this]() { return StatusFn ? StatusFn() : FText::GetEmpty(); })
            .Font(PanelFont(9))
            .AutoWrapText(true)
            .ColorAndOpacity(FLinearColor(0.75f, 0.85f, 1.f))
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::MainPage()
{
    const FMargin ButtonPad(0.f, 4.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("DRONE SIMULATOR"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 8.f))[ MapInfo() ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 0.f, 0.f, 10.f))[ ImageryModeRow() ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Free flight")), [this]() { Fire(EDroneMenuAction::FreeFlight); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [
            BigButton(Txt(TEXT("Fly the mission")), [this]() { Fire(EDroneMenuAction::FlyMission); },
                      [this]() { return bMissionPlanOk; })
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("FPV flight (acro, from the ground)")), [this]() { Fire(EDroneMenuAction::FpvFlight); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Maps and missions")), [this]() { OpenMaps(); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Settings")), [this]() { EnterSettings(); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Controls")), [this]() { ActivePage = PageHelp; }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Quit")), [this]() { Fire(EDroneMenuAction::Quit); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 12.f, 0.f, 0.f))
        [ Hint(Txt(TEXT("Video and MAVLink start with the flight (a new folder in Saved/Flights for each flight). ")
                   TEXT("The terrain behind this menu is the start point of the map."))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 0.f))[ Hint(Txt(KeysHint)) ];
}

// The map modes, switched at once (also in Settings > Map loading); "Esri by date"
// adds the date chooser below.
TSharedRef<SWidget> SDroneSettingsPanel::ImageryModeRow()
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()
        [
            ChoiceRow(Txt(TEXT("Map imagery")), &Settings.ImagerySource,
                      {Txt(TEXT("Standard: Bing, streamed")), Txt(TEXT("Esri, kept on disk")), Txt(TEXT("Esri by date"))},
                      [this]() { Fire(EDroneMenuAction::ApplySettings); })
        ]
        + SVerticalBox::Slot().AutoHeight()[ WaybackRow() ];
}

const TArray<FDroneWaybackVersion>& SDroneSettingsPanel::WaybackVersionList() const
{
    static const TArray<FDroneWaybackVersion> NoVersions;
    return WaybackVersionsFn ? WaybackVersionsFn() : NoVersions;
}

int32 SDroneSettingsPanel::WaybackIndex() const
{
    if (Settings.WaybackRelease <= 0)
    {
        return -1;
    }
    const int32 Release = Settings.WaybackRelease;
    return WaybackVersionList().IndexOfByPredicate([Release](const FDroneWaybackVersion& Version) { return Version.Release == Release; });
}

FText SDroneSettingsPanel::WaybackChoiceText() const
{
    if (Settings.WaybackRelease <= 0)
    {
        return Txt(TEXT("Current Esri World Imagery (newest)"));
    }
    const int32 Index = WaybackIndex();
    const TArray<FDroneWaybackVersion>& List = WaybackVersionList();
    if (List.IsValidIndex(Index))
    {
        return FText::FromString(FString::Printf(TEXT("%s   [%d of %d]"), *List[Index].Label(), Index + 1, List.Num()));
    }
    return FText::FromString(Settings.WaybackLabel.IsEmpty() ? FString::Printf(TEXT("release %d"), Settings.WaybackRelease)
                                                             : Settings.WaybackLabel);
}

void SDroneSettingsPanel::StepWayback(int32 Direction)
{
    const TArray<FDroneWaybackVersion>& List = WaybackVersionList();
    const int32 Next = FMath::Clamp(WaybackIndex() + Direction, -1, List.Num() - 1);
    if (List.IsValidIndex(Next))
    {
        PickWayback(List[Next].Release, List[Next].Label());
    }
    else
    {
        PickWayback(0, FString());
    }
}

void SDroneSettingsPanel::PickWayback(int32 Release, const FString& Label)
{
    Settings.ImagerySource = 2;
    Settings.WaybackRelease = Release;
    Settings.WaybackLabel = Label;
    Fire(EDroneMenuAction::ApplySettings);
}

// "< Older | taken 2015-10-25, Maxar WV02 0.5 m (release 2019-04-03) [3 of 12] | Newer > | All dates"
TSharedRef<SWidget> SDroneSettingsPanel::WaybackRow()
{
    auto SmallButton = [](const FText& Label, TFunction<void()> OnClick, TFunction<bool()> IsEnabledFn) -> TSharedRef<SWidget>
    {
        return SNew(SButton)
            .IsFocusable(false)
            .HAlign(HAlign_Center)
            .VAlign(VAlign_Center)
            .IsEnabled_Lambda([EnabledFn = MoveTemp(IsEnabledFn)]() { return EnabledFn ? EnabledFn() : true; })
            .OnClicked_Lambda([Callback = MoveTemp(OnClick)]()
            {
                Callback();
                return FReply::Handled();
            })
            [
                SNew(STextBlock).Text(Label).Font(PanelFont(10))
            ];
    };
    return SNew(SVerticalBox)
        .Visibility_Lambda([this]() { return Settings.ImagerySource == 2 ? EVisibility::Visible : EVisibility::Collapsed; })
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f, 0.f, 2.f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth().Padding(FMargin(0.f, 0.f, 4.f, 0.f))
            [
                SmallButton(Txt(TEXT("< Older")), [this]() { StepWayback(1); },
                            [this]() { return WaybackIndex() + 1 < WaybackVersionList().Num(); })
            ]
            + SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
            [
                SNew(STextBlock)
                .Text_Lambda([this]() { return WaybackChoiceText(); })
                .Font(PanelFont(10))
                .AutoWrapText(true)
                .Justification(ETextJustify::Center)
                .ColorAndOpacity(FLinearColor::White)
            ]
            + SHorizontalBox::Slot().AutoWidth().Padding(FMargin(4.f, 0.f))
            [ SmallButton(Txt(TEXT("Newer >")), [this]() { StepWayback(-1); }, [this]() { return WaybackIndex() >= 0; }) ]
            + SHorizontalBox::Slot().AutoWidth()
            [ SmallButton(Txt(TEXT("All dates")), [this]() { ActivePage = PageDates; }, nullptr) ]
        ]
        + SVerticalBox::Slot().AutoHeight()
        [
            SNew(STextBlock)
            .Text_Lambda([this]() { return FText::FromString(WaybackStatusFn ? WaybackStatusFn() : FString()); })
            .Font(PanelFont(9))
            .AutoWrapText(true)
            .ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f))
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::DatesPage()
{
    constexpr int32 MaxRows = 120;
    TSharedRef<SVerticalBox> Rows = SNew(SVerticalBox);
    // Row -1: the current imagery; rows 0.. follow the list (filled while the search runs).
    for (int32 Row = -1; Row < MaxRows; ++Row)
    {
        Rows->AddSlot()
            .AutoHeight()
            .Padding(FMargin(0.f, 2.f))
            [
                SNew(SBox)
                .Visibility_Lambda([this, Row]() { return Row < WaybackVersionList().Num() ? EVisibility::Visible : EVisibility::Collapsed; })
                .HeightOverride(32.f)
                [
                    SNew(SButton)
                    .IsFocusable(false)
                    .VAlign(VAlign_Center)
                    .ButtonColorAndOpacity_Lambda([this, Row]()
                    {
                        const TArray<FDroneWaybackVersion>& List = WaybackVersionList();
                        const bool bPicked = Row < 0 ? Settings.WaybackRelease <= 0
                                                     : (List.IsValidIndex(Row) && List[Row].Release == Settings.WaybackRelease);
                        return FSlateColor(bPicked ? FLinearColor(0.15f, 0.6f, 0.2f) : FLinearColor(0.3f, 0.3f, 0.3f));
                    })
                    .OnClicked_Lambda([this, Row]()
                    {
                        const TArray<FDroneWaybackVersion>& List = WaybackVersionList();
                        if (Row < 0)
                        {
                            PickWayback(0, FString());
                        }
                        else if (List.IsValidIndex(Row))
                        {
                            PickWayback(List[Row].Release, List[Row].Label());
                        }
                        return FReply::Handled();
                    })
                    [
                        SNew(STextBlock)
                        .Text_Lambda([this, Row]()
                        {
                            const TArray<FDroneWaybackVersion>& List = WaybackVersionList();
                            if (Row < 0)
                            {
                                return Txt(TEXT("Current Esri World Imagery (newest)"));
                            }
                            return List.IsValidIndex(Row) ? FText::FromString(List[Row].Label()) : FText();
                        })
                        .Font(PanelFont(10))
                    ]
                ]
            ];
    }
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("IMAGERY DATES"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
        [
            Hint(Txt(TEXT("Esri World Imagery Wayback: earlier versions of the Esri imagery. Listed are only the versions ")
                     TEXT("whose image differs at the map's area, newest first: \"taken\" is when that image was captured ")
                     TEXT("there, the release is when Esri published it. Click one to fly over it (the view changes at once); ")
                     TEXT("each version's tiles are kept on disk. Older versions often stop at level 17 or 18 (coarser up ")
                     TEXT("close). DroneLocalization's maps were built from the current Esri imagery, so an older version ")
                     TEXT("tests the localizer on imagery of another date.")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 2.f, 0.f, 6.f))
        [
            SNew(STextBlock)
            .Text_Lambda([this]() { return FText::FromString(WaybackStatusFn ? WaybackStatusFn() : FString()); })
            .Font(PanelFont(10))
            .AutoWrapText(true)
            .ColorAndOpacity(FLinearColor(0.8f, 0.8f, 0.8f))
        ]
        + SVerticalBox::Slot().FillHeight(1.f)
        [
            SNew(SScrollBox)
            + SScrollBox::Slot()[ Rows ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 10.f, 0.f, 0.f))
        [ BigButton(Txt(TEXT("Back")), [this]() { ActivePage = RootPage(); }) ];
}

TSharedRef<SWidget> SDroneSettingsPanel::PausePage()
{
    const FMargin ButtonPad(0.f, 4.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("MENU"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 2.f))
        [ Hint(Txt(TEXT("The flight goes on behind the menu: in manual mode the drone hovers, a mission keeps flying."))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 8.f))[ MapInfo() ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 0.f, 0.f, 10.f))[ ImageryModeRow() ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Resume")), [this]() { Fire(EDroneMenuAction::Resume); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [
            BigButton(Txt(TEXT("Start the mission")), [this]() { Fire(EDroneMenuAction::FlyMission); },
                      [this]() { return bMissionPlanOk; })
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Stop the mission (manual)")), [this]() { Fire(EDroneMenuAction::StopMission); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Maps and missions")), [this]() { OpenMaps(); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Settings")), [this]() { EnterSettings(); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Controls")), [this]() { ActivePage = PageHelp; }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Main menu (ends the flight)")), [this]() { Fire(EDroneMenuAction::MainMenu); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Quit")), [this]() { Fire(EDroneMenuAction::Quit); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 12.f, 0.f, 0.f))[ Hint(Txt(KeysHint)) ];
}

TSharedRef<SWidget> SDroneSettingsPanel::MapEditor()
{
    const FMargin RowPad(0.f, 2.f);
    const FMargin SectionPad(0.f, 12.f, 0.f, 2.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ Header(Txt(TEXT("Map"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ TextRow(Txt(TEXT("Name")), &Settings.MapName) ]

        + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Start point"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Latitude, deg")), &Settings.StartLatitude, -85.0, 85.0, 0.0001, 6) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Longitude, deg")), &Settings.StartLongitude, -180.0, 180.0, 0.0001, 6) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Altitude above ground, m")), &Settings.AltitudeM, 5.0, 5000.0, 10.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Heading, deg")), &Settings.StartHeadingDeg, 0.0, 359.9, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 2.f, 0.f))
            [
                ActionButton(Txt(TEXT("Go to the start point")), [this]()
                {
                    MapNote = TEXT("Moving the drone to the start point...");
                    Fire(EDroneMenuAction::PreviewMap);
                })
            ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(2.f, 0.f, 0.f, 0.f))
            [ ActionButton(Txt(TEXT("Start point = drone position")), [this]() { StartAtDronePosition(); }) ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("The drone moves to the start point when you pick a map, Save, press Go to the start point ")
                     TEXT("or Back (a running mission stops there).")))
        ]

        + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Survey area"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("Vertices in decimal degrees \"lat, lon\", separated by ; or new lines (at least 3), ")
                     TEXT("e.g. pasted from Google Maps. In flight: fly to each corner and press V.")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            SNew(SBox)
            .HeightOverride(110.f)
            [
                SNew(SMultiLineEditableTextBox)
                .Text_Lambda([this]() { return FText::FromString(Settings.MissionPolygon); })
                .OnTextChanged_Lambda([this](const FText& NewText) { Settings.MissionPolygon = NewText.ToString(); })
                .OnTextCommitted_Lambda([this](const FText& NewText, ETextCommit::Type)
                {
                    Settings.MissionPolygon = NewText.ToString();
                })
                .AutoWrapText(true)
                .Font(PanelFont(9))
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 2.f, 0.f))
            [ ActionButton(Txt(TEXT("Add drone position")), [this]() { AddDronePositionVertex(); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(2.f, 0.f))
            [
                ActionButton(Txt(TEXT("Square around the start")), [this]()
                {
                    Settings.MissionPolygon = DroneMaps::SquareAround(Settings.StartLatitude, Settings.StartLongitude,
                                                                      Settings.MissionSquareSizeM);
                })
            ]
            + SHorizontalBox::Slot().AutoWidth().Padding(FMargin(2.f, 0.f, 0.f, 0.f))
            [ ActionButton(Txt(TEXT("Clear")), [this]() { Settings.MissionPolygon.Reset(); }) ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Square size, m")), &Settings.MissionSquareSizeM, 50.0, 50000.0, 100.0, 0) ]

        + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Grid"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Altitude above ground, m")), &Settings.MissionAltitudeM, 10.0, 5000.0, 10.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Speed, m/s")), &Settings.MissionSpeedMps, 1.0, 60.0, 1.0, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Leg direction")), &MissionAngleChoice,
                    {Txt(TEXT("Along the longest edge")), Txt(TEXT("Set angle"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Leg angle (compass), deg")), &Settings.MissionAngleDeg, 0.0, 179.9, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Pass spacing")), &MissionSpacingChoice,
                    {Txt(TEXT("From side overlap")), Txt(TEXT("Number of passes"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Side overlap, %")), &Settings.MissionSideOverlapPct, 0.0, 95.0, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ IntRow(Txt(TEXT("Number of passes")), &Settings.MissionPassCount, 1, 2000) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Overshoot past the edge, m")), &Settings.MissionOvershootM, 0.0, 2000.0, 10.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Camera heading")), &MissionHeadingChoice,
                    {Txt(TEXT("Along the legs")), Txt(TEXT("Fixed"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Fixed heading, deg")), &Settings.MissionHeadingDeg, 0.0, 359.9, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("After the last pass")), &Settings.MissionAfter,
                    {Txt(TEXT("Hover")), Txt(TEXT("Return to launch")), Txt(TEXT("Repeat"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Mission start")), &MissionStartChoice,
                    {Txt(TEXT("Fly there from the drone")), Txt(TEXT("Jump to the first pass"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)
        [
            SNew(STextBlock)
            .Text_Lambda([this]() { return MissionSummary(); })
            .Font(PanelFont(9))
            .AutoWrapText(true)
            .ColorAndOpacity_Lambda([this]()
            {
                if (bMissionAreaEmpty)
                {
                    return FSlateColor(FLinearColor(0.7f, 0.7f, 0.7f));
                }
                return FSlateColor(bMissionPlanOk ? FLinearColor(0.55f, 1.f, 0.6f) : FLinearColor(1.f, 0.45f, 0.4f));
            })
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::MapsPage()
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("MAPS AND MISSIONS"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 2.f, 0.f, 8.f))
        [
            Hint(FText::FromString(FString::Printf(
                TEXT("A map = a start point + a survey area with its grid settings. Click a map to load it; ")
                TEXT("Save writes %s/<name>.json (copy these files to share maps)."), *DroneMaps::MapsDir())))
        ]
        + SVerticalBox::Slot().FillHeight(1.f)
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().AutoWidth()
            [
                SNew(SBox)
                .WidthOverride(260.f)
                [
                    SNew(SVerticalBox)
                    + SVerticalBox::Slot().AutoHeight()[ Header(Txt(TEXT("Saved maps"))) ]
                    + SVerticalBox::Slot().FillHeight(1.f).Padding(FMargin(0.f, 4.f))
                    [
                        SNew(SScrollBox)
                        + SScrollBox::Slot()
                        [
                            SAssignNew(MapListBox, SVerticalBox)
                        ]
                    ]
                    + SVerticalBox::Slot().AutoHeight()
                    [ ActionButton(Txt(TEXT("New map")), [this]() { NewMap(); }) ]
                ]
            ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(18.f, 0.f, 0.f, 0.f))
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()
                [
                    MapEditor()
                ]
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 8.f, 0.f, 4.f))
        [
            SNew(STextBlock)
            .Text_Lambda([this]() { return FText::FromString(MapNote); })
            .Font(PanelFont(9))
            .AutoWrapText(true)
            .ColorAndOpacity(FLinearColor(0.75f, 0.85f, 1.f))
        ]
        + SVerticalBox::Slot().AutoHeight()
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
            [ BigButton(Txt(TEXT("Save map")), [this]() { SaveCurrentMap(); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f))
            [ BigButton(Txt(TEXT("Delete map")), [this]() { DeleteCurrentMap(); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
            [ BigButton(Txt(TEXT("Back")), [this]() { LeaveMaps(); }) ]
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::ControlsTab()
{
    const FMargin RowPad(0.f, 2.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ Hint(Txt(TEXT("Manual flight (keyboard, gamepad, RC transmitter). The changes apply as soon as you Save."))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Max speed, m/s")), &Settings.ManualMaxSpeedMps, 1.0, 100.0, 1.0, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Acceleration, m/s2")), &Settings.ManualAccelerationMps2, 0.5, 50.0, 0.5, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Braking, m/s2")), &Settings.ManualBrakingMps2, 0.5, 50.0, 0.5, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Shift: speed x")), &Settings.ManualFastMultiplier, 1.0, 5.0, 0.1, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Turn rate (Left / Right arrows), deg/s")), &Settings.ManualYawRateDegPerSec, 5.0, 360.0, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Climb rate (Space/Ctrl), m/s")), &Settings.ManualClimbRateMps, 0.5, 50.0, 0.5, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Vertical acceleration, m/s2")), &Settings.ManualVerticalAccelerationMps2, 0.5, 50.0, 0.5, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Stick dead zone (0..0.5)")), &Settings.StickDeadZone, 0.0, 0.5, 0.01, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Stick expo (0 linear, 1 soft centre)")), &Settings.StickExpo, 0.0, 1.0, 0.05, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Camera tilt (Up / Down) / pan (Q / E), deg/s")), &Settings.CameraTiltRateDegPerSec, 1.0, 180.0, 5.0, 0) ]

        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 12.f, 0.f, 2.f))[ Header(Txt(TEXT("FPV (acro, no assistance)"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ Hint(Txt(TEXT("Rates like Betaflight \"Actual\": centre = deg/s near the stick centre, max = deg/s at full stick."))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Thrust / weight")), &Settings.FpvThrustToWeight, 1.5, 15.0, 0.5, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Roll / pitch: centre rate, deg/s")), &Settings.FpvRollPitchCenterRateDps, 10.0, 2000.0, 10.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Roll / pitch: max rate, deg/s")), &Settings.FpvRollPitchMaxRateDps, 10.0, 2000.0, 10.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Roll / pitch: expo")), &Settings.FpvRollPitchExpo, 0.0, 1.0, 0.05, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Yaw: centre rate, deg/s")), &Settings.FpvYawCenterRateDps, 10.0, 2000.0, 10.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Yaw: max rate, deg/s")), &Settings.FpvYawMaxRateDps, 10.0, 2000.0, 10.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Yaw: expo")), &Settings.FpvYawExpo, 0.0, 1.0, 0.05, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Camera uptilt, deg")), &Settings.FpvCameraUptiltDeg, -30.0, 80.0, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Camera field of view, deg")), &Settings.FpvCameraFovDeg, 40.0, 170.0, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Keyboard: stick per key (0..1)")), &Settings.FpvKeyboardStick, 0.05, 1.0, 0.05, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Keyboard: throttle per second")), &Settings.FpvThrottleKeyRate, 0.05, 5.0, 0.05, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Crash above, m/s")), &Settings.FpvCrashSpeedMps, 0.5, 50.0, 0.5, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 8.f, 0.f, 0.f))
        [
            ActionButton(Txt(TEXT("Reset the controls to the defaults")), [this]()
            {
                FDroneSimSettings Defaults;
                DroneMaps::CopyControlFields(Defaults, Settings);
            })
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::VideoTab()
{
    const FMargin RowPad(0.f, 2.f);
    const FMargin SectionPad(0.f, 10.f, 0.f, 2.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            SNew(SBox)
            .Visibility(bInFlight ? EVisibility::Visible : EVisibility::Collapsed)
            [
                Hint(Txt(TEXT("Video and MAVLink change in the main menu: they restart the stream.")))
            ]
        ]
        + SVerticalBox::Slot().AutoHeight()
        [
            SNew(SBox)
            .IsEnabled(!bInFlight)
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()[ Header(Txt(TEXT("Video"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ ChoiceRow(Txt(TEXT("Output")), &Settings.VideoMode,
                            {Txt(TEXT("Off")), Txt(TEXT("Record mp4")), Txt(TEXT("Live RTSP"))}) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ ChoiceRow(Txt(TEXT("Encoder")), &EncoderChoice, {Txt(TEXT("NVIDIA (nvenc)")), Txt(TEXT("CPU (x264)"))}) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ TextRow(Txt(TEXT("ffmpeg")), &Settings.FfmpegPath) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ TextRow(Txt(TEXT("RTSP URL")), &Settings.RtspUrl) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ ChoiceRow(Txt(TEXT("Start mediamtx when needed")), &RtspServerChoice, {Txt(TEXT("No")), Txt(TEXT("Yes"))}) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ TextRow(Txt(TEXT("mediamtx.exe")), &Settings.RtspServerPath) ]

                + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("MAVLink telemetry (UDP)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ TextRow(Txt(TEXT("Host")), &Settings.MavlinkHost) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ IntRow(Txt(TEXT("Port")), &Settings.MavlinkPort, 1, 65535) ]
            ]
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::SunTab()
{
    const FMargin RowPad(0.f, 2.f);
    const FMargin SectionPad(0.f, 10.f, 0.f, 2.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ Header(Txt(TEXT("Camera"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Mount")), &GimbalChoice,
                    {Txt(TEXT("Gimbal (stays level)")), Txt(TEXT("Fixed (tilts with body)"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Exposure compensation, EV")), &Settings.ExposureCompensationEV, -5.0, 5.0, 0.1, 1) ]

        + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Sun"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Local time, h")), &Settings.SolarTime, 0.0, 23.99, 0.25, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Time zone, h from UTC")), &Settings.TimeZone, -12.0, 14.0, 1.0, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ IntRow(Txt(TEXT("Day")), &Settings.Day, 1, 31) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ IntRow(Txt(TEXT("Month")), &Settings.Month, 1, 12) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ IntRow(Txt(TEXT("Year")), &Settings.Year, 1900, 2100) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Daylight saving time")), &DstChoice, {Txt(TEXT("No")), Txt(TEXT("Yes"))}) ];
}

TSharedRef<SWidget> SDroneSettingsPanel::MapLoadingTab()
{
    const FMargin RowPad(0.f, 2.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("Cesium streams the terrain and the imagery for what the cameras see. More detail and preloading ")
                     TEXT("cost loading time, network and memory (GPU and RAM).")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Imagery")), &Settings.ImagerySource,
                    {Txt(TEXT("Standard: Bing, streamed")), Txt(TEXT("Esri, kept on disk")), Txt(TEXT("Esri by date"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("Standard: Bing Maps Aerial from Cesium ion, as before, downloaded again every run. ")
                     TEXT("Esri World Imagery: each tile is downloaded once and kept in the tile store folder, so flying ")
                     TEXT("over an area again needs no internet for its imagery (the terrain is kept by Cesium's cache). ")
                     TEXT("Esri by date: earlier versions (Wayback), the date is chosen in the main menu under Map imagery. ")
                     TEXT("Note: DroneLocalization's maps were built from Esri World Imagery, so with Esri the localizer ")
                     TEXT("sees the imagery of its own map (easier than a real flight); use Bing to test across sources. ")
                     TEXT("Deepest level 19 is about 0.2 m per pixel; deeper levels exist only in some places. Empty folder: ")
                     TEXT("C:/Users/<you>/AppData/Local/DroneSimUE/TileStore (shared by the editor and the packaged game).")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ IntRow(Txt(TEXT("Esri deepest level")), &Settings.EsriMaxLevel, 10, 23) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ TextRow(Txt(TEXT("Tile store folder")), &Settings.TileStoreDir) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Detail")), &Settings.MapQuality,
                    {Txt(TEXT("Normal")), Txt(TEXT("High (x2)")), Txt(TEXT("Very high (x4)"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("While tiles load")), &ForbidHolesChoice,
                    {Txt(TEXT("Allow holes (faster)")), Txt(TEXT("No holes (coarser tile meanwhile)"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("What turning would show")), &Settings.AroundDroneDetail,
                    {Txt(TEXT("Not loaded")), Txt(TEXT("Same as in view")), Txt(TEXT("Half detail")),
                     Txt(TEXT("Quarter"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("Invisible copies of the camera turned to the other headings keep that map loaded, so turning ")
                     TEXT("the drone or the camera shows it at once. Cheap with the camera pointing down; with the camera ")
                     TEXT("towards the horizon up to 360 / FOV times the tiles of the view: lower it if the frame rate or GPU memory suffer.")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("New detail appears")), &Settings.TileLoadingStyle,
                    {Txt(TEXT("Step by step (coarse, finer, final)")), Txt(TEXT("At once (may stay coarse a long time)"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Preload the map's area")), &PreloadChoice, {Txt(TEXT("Off")), Txt(TEXT("On"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("Preload: the survey area (or 1 km around the start point) plus the margin is loaded at the detail ")
                     TEXT("of the mission altitude and kept loaded while the map is selected. Progress: the main menu and the status line.")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Margin around the area, m")), &Settings.PreloadMarginM, 0.0, 20000.0, 100.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ IntRow(Txt(TEXT("Tile memory cache, MB")), &Settings.TileMemoryCacheMB, 64, 65536) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ IntRow(Txt(TEXT("Tiles loading at once")), &Settings.MaxSimultaneousTileLoads, 1, 512) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("Cesium's disk cache: cesium-request-cache.sqlite (next to the packaged program's folders; the ")
                     TEXT("project folder in the editor), at most MaxCacheItems entries (Config/DefaultEngine.ini, 100000). ")
                     TEXT("Terrain tiles are reused from it in later runs (the server allows a year). The Bing imagery is ")
                     TEXT("not (its addresses carry a key that changes every run); the Esri imagery is kept by the tile store.")))
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::MarkersTab()
{
    const FMargin RowPad(0.f, 2.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("The current map's survey area in the world: a pole at every vertex and see-through walls ")
                     TEXT("along the edges, from under the ground to 50 m above the flight altitude. ")
                     TEXT("They have no collision: fly through them.")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Corner poles")), &PolesChoice, {Txt(TEXT("Off")), Txt(TEXT("On"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Walls")), &WallsChoice, {Txt(TEXT("Off")), Txt(TEXT("On"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Wall opacity, %")), &Settings.AreaWallOpacityPct, 2.0, 90.0, 5.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Also in the video")), &MarkersVideoChoice,
                    {Txt(TEXT("No (screen only)")), Txt(TEXT("Yes"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ Hint(Txt(TEXT("In the video the localizer would see them as well, so keep this off for localization tests."))) ];
}

TSharedRef<SWidget> SDroneSettingsPanel::KeyRow(const FText& Keys, const FText& Action)
{
    return SNew(SHorizontalBox)
        + SHorizontalBox::Slot().AutoWidth().VAlign(VAlign_Top)
        [
            SNew(SBox)
            .WidthOverride(230.f)
            [
                SNew(STextBlock).Text(Keys).Font(FCoreStyle::GetDefaultFontStyle("Bold", 10)).ColorAndOpacity(FLinearColor(1.f, 0.85f, 0.45f))
            ]
        ]
        + SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Top)
        [
            SNew(STextBlock).Text(Action).Font(PanelFont(10)).AutoWrapText(true).ColorAndOpacity(FLinearColor::White)
        ];
}

TSharedRef<SWidget> SDroneSettingsPanel::HelpPage()
{
    const FMargin RowPad(0.f, 2.f);
    const FMargin SectionPad(0.f, 12.f, 0.f, 4.f);
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("CONTROLS"))) ]
        + SVerticalBox::Slot().FillHeight(1.f).Padding(FMargin(0.f, 8.f, 0.f, 0.f))
        [
            SNew(SScrollBox)
            + SScrollBox::Slot()
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()[ Header(Txt(TEXT("Manual flight (keyboard)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("W / S")), Txt(TEXT("forward / back"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("A / D")), Txt(TEXT("sideways left / right"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Left / Right arrows")), Txt(TEXT("turn the drone"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Space / Ctrl")), Txt(TEXT("climb / descend"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("Shift (hold)")), Txt(TEXT("faster: x2 speed and acceleration by default (Settings > Controls)"))) ]

                + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Camera"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("Up / Down arrows")), Txt(TEXT("tilt: from straight down (-90) to the horizon (0)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("Q / E")), Txt(TEXT("pan left / right relative to the nose (straight down: turns the image)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("C")), Txt(TEXT("camera straight down and forward again"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [
                    Hint(Txt(TEXT("Every flight starts with the camera straight down. The compass at the top: white line = top of ")
                             TEXT("the image, orange triangle = nose, green chevron = where the drone moves, cyan diamond = next waypoint.")))
                ]

                + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("FPV (acro, no assistance)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("F")), Txt(TEXT("FPV on / off in flight (in the air it starts level, throttle at hover)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("Space / Ctrl")), Txt(TEXT("throttle up / down; it stays where you leave it"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("W / S")), Txt(TEXT("pitch: nose down / up (the drone keeps the angle when you let go)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("A / D")), Txt(TEXT("roll left / right"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Left / Right arrows")), Txt(TEXT("yaw"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("R")), Txt(TEXT("after a crash: put the drone back on its legs"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [
                    KeyRow(Txt(TEXT("Gamepad")),
                           Txt(TEXT("left stick: throttle (up / down, full range once moved) and yaw; right stick: pitch and roll")))
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [
                    Hint(Txt(TEXT("No self-levelling and no altitude hold: like a real acro quad. \"FPV flight\" in the main menu ")
                             TEXT("starts on the ground at the map's start point. The menu pauses an FPV flight. Rates, thrust, ")
                             TEXT("camera uptilt and field of view: Settings > Controls.")))
                ]

                + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Missions and maps"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("M")), Txt(TEXT("manual <-> route / mission: pauses and resumes a mission"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("V")), Txt(TEXT("add the drone position as the next vertex of the map's area (then Save in Maps)"))) ]

                + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Menu"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ KeyRow(Txt(TEXT("Esc / F2 / F1 / Tab")), Txt(TEXT("open / close the menu; on a sub page: back (Settings: cancel)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ Hint(Txt(TEXT("Esc only in the packaged program: in the editor it stops Play. While the cursor is in a text field only F2 / F1 work."))) ]

                + SVerticalBox::Slot().AutoHeight().Padding(SectionPad)[ Header(Txt(TEXT("Gamepad"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Left stick")), Txt(TEXT("up / down: climb / descend, left / right: turn"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Right stick")), Txt(TEXT("move forward / back / sideways"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Right trigger")), Txt(TEXT("faster (like Shift)"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("D-pad")), Txt(TEXT("up / down: camera tilt, left / right: camera pan"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Right stick click")), Txt(TEXT("camera straight down again"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("Start / Menu button")), Txt(TEXT("like M"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("View / Back button")), Txt(TEXT("menu"))) ]
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
                [ Hint(Txt(TEXT("Stick dead zone and expo, speeds and rates: Settings > Controls."))) ]
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 10.f, 0.f, 0.f))
        [ BigButton(Txt(TEXT("Back")), [this]() { ActivePage = RootPage(); }) ];
}

TSharedRef<SWidget> SDroneSettingsPanel::SettingsPage()
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("SETTINGS"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 8.f))
        [
            ChoiceButtons(&SettingsTab, {Txt(TEXT("Controls")), Txt(TEXT("Video / MAVLink")), Txt(TEXT("Camera / Sun")),
                                         Txt(TEXT("Area markers")), Txt(TEXT("Map loading"))})
        ]
        + SVerticalBox::Slot().FillHeight(1.f)
        [
            SNew(SScrollBox)
            + SScrollBox::Slot()
            [
                SNew(SWidgetSwitcher)
                .WidgetIndex_Lambda([this]() { return SettingsTab; })
                + SWidgetSwitcher::Slot()[ ControlsTab() ]
                + SWidgetSwitcher::Slot()[ VideoTab() ]
                + SWidgetSwitcher::Slot()[ SunTab() ]
                + SWidgetSwitcher::Slot()[ MarkersTab() ]
                + SWidgetSwitcher::Slot()[ MapLoadingTab() ]
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 10.f, 0.f, 0.f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
            [ BigButton(Txt(TEXT("Save")), [this]() { LeaveSettings(/*bSave*/ true); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
            [ BigButton(Txt(TEXT("Cancel")), [this]() { LeaveSettings(/*bSave*/ false); }) ]
        ];
}

void SDroneSettingsPanel::Construct(const FArguments& InArgs)
{
    Settings = InArgs._Initial;
    bInFlight = InArgs._InFlight;
    DronePosition = InArgs._DronePosition;
    StatusFn = InArgs._Status;
    WaybackVersionsFn = InArgs._WaybackVersions;
    WaybackStatusFn = InArgs._WaybackStatus;
    OnAction = InArgs._OnAction;
    SyncChoices();
    ActivePage = RootPage();

    ChildSlot
    [
        SNew(SBorder)
        .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
        .BorderBackgroundColor(FLinearColor(0.f, 0.f, 0.f, bInFlight ? 0.25f : 0.45f))
        .Padding(FMargin(40.f, 30.f))
        [
            SNew(SOverlay)
            // Main menu / in-flight menu: a narrow card in the middle.
            + SOverlay::Slot()
            .HAlign(HAlign_Center)
            .VAlign(VAlign_Center)
            [
                SNew(SBox)
                .Visibility_Lambda([this]()
                {
                    return (ActivePage == PageMain || ActivePage == PagePause) ? EVisibility::Visible : EVisibility::Collapsed;
                })
                .WidthOverride(560.f)
                [
                    Card(SNew(SWidgetSwitcher)
                         .WidgetIndex_Lambda([this]() { return ActivePage == PagePause ? 1 : 0; })
                         + SWidgetSwitcher::Slot()[ MainPage() ]
                         + SWidgetSwitcher::Slot()[ PausePage() ])
                ]
            ]
            // Maps / settings: a tall card.
            + SOverlay::Slot()
            .HAlign(HAlign_Center)
            .VAlign(VAlign_Fill)
            [
                SNew(SBox)
                .Visibility_Lambda([this]()
                {
                    return (ActivePage == PageMaps || ActivePage == PageSettings || ActivePage == PageHelp || ActivePage == PageDates)
                        ? EVisibility::Visible
                        : EVisibility::Collapsed;
                })
                .WidthOverride_Lambda([this]() { return FOptionalSize(ActivePage == PageMaps ? 1040.f : 660.f); })
                [
                    Card(SNew(SWidgetSwitcher)
                         .WidgetIndex_Lambda([this]()
                         {
                             return ActivePage == PageSettings ? 1 : (ActivePage == PageHelp ? 2 : (ActivePage == PageDates ? 3 : 0));
                         })
                         + SWidgetSwitcher::Slot()[ MapsPage() ]
                         + SWidgetSwitcher::Slot()[ SettingsPage() ]
                         + SWidgetSwitcher::Slot()[ HelpPage() ]
                         + SWidgetSwitcher::Slot()[ DatesPage() ])
                ]
            ]
        ]
    ];
    RefreshMapList();
}

TSharedRef<SWidget> MakeDroneStatusLine(TFunction<FText()> GetText, float TopPadding)
{
    return SNew(SBox)
        .Visibility(EVisibility::HitTestInvisible)
        .HAlign(HAlign_Left)
        .VAlign(VAlign_Top)
        .Padding(FMargin(12.f, TopPadding, 12.f, 8.f))
        [
            SNew(SBorder)
            .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
            .BorderBackgroundColor(FLinearColor(0.f, 0.f, 0.f, 0.6f))
            .Padding(FMargin(8.f, 4.f))
            [
                SNew(STextBlock)
                .Text_Lambda(MoveTemp(GetText))
                .Font(FCoreStyle::GetDefaultFontStyle("Bold", 13))
                .ColorAndOpacity(FLinearColor(0.35f, 1.f, 0.45f))
                .ShadowOffset(FVector2D(1.0, 1.0))
                .ShadowColorAndOpacity(FLinearColor(0.f, 0.f, 0.f, 0.9f))
            ]
        ];
}
