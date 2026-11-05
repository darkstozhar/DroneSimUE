#include "Sim/DroneSettingsPanel.h"

#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"
#include "Sim/DroneMission.h"
#include "Sim/FpvBuild.h"
#include "Sim/DronePreview.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Fonts/FontMeasure.h"
#include "Framework/Application/SlateApplication.h"
#include "Rendering/DrawElements.h"
#include "Rendering/SlateRenderer.h"
#include "Styling/SlateBrush.h"
#include "Widgets/Images/SImage.h"
#include "Widgets/SLeafWidget.h"
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
    O->SetNumberField(TEXT("fpv_use_build"), FpvUseBuild);
    O->SetStringField(TEXT("fpv_drone_name"), FpvDroneName);
    O->SetNumberField(TEXT("fpv_frame"), FpvFrame);
    O->SetNumberField(TEXT("fpv_motor"), FpvMotor);
    O->SetNumberField(TEXT("fpv_prop"), FpvProp);
    O->SetNumberField(TEXT("fpv_battery"), FpvBattery);
    O->SetNumberField(TEXT("fpv_camera_system"), FpvCameraSystem);
    O->SetNumberField(TEXT("fpv_pid_master"), FpvPidMaster);
    O->SetNumberField(TEXT("fpv_pid_damping"), FpvPidDamping);
    O->SetNumberField(TEXT("fpv_throttle_mid"), FpvThrottleMid);
    O->SetNumberField(TEXT("fpv_throttle_expo"), FpvThrottleExpo);
    O->SetNumberField(TEXT("fpv_throttle_limit_pct"), FpvThrottleLimitPct);
    O->SetStringField(TEXT("fpv_rates_name"), FpvRatesName);
    O->SetNumberField(TEXT("camera_width_px"), CamWidthPx);
    O->SetNumberField(TEXT("camera_height_px"), CamHeightPx);
    O->SetNumberField(TEXT("camera_fps"), CamFps);
    O->SetNumberField(TEXT("camera_survey_hfov_deg"), CamSurveyHFovDeg);
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
    O->SetNumberField(TEXT("world_mode"), WorldMode);
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
    O->TryGetNumberField(TEXT("fpv_use_build"), FpvUseBuild);
    O->TryGetStringField(TEXT("fpv_drone_name"), FpvDroneName);
    O->TryGetNumberField(TEXT("fpv_frame"), FpvFrame);
    O->TryGetNumberField(TEXT("fpv_motor"), FpvMotor);
    O->TryGetNumberField(TEXT("fpv_prop"), FpvProp);
    O->TryGetNumberField(TEXT("fpv_battery"), FpvBattery);
    O->TryGetNumberField(TEXT("fpv_camera_system"), FpvCameraSystem);
    O->TryGetNumberField(TEXT("fpv_pid_master"), FpvPidMaster);
    O->TryGetNumberField(TEXT("fpv_pid_damping"), FpvPidDamping);
    O->TryGetNumberField(TEXT("fpv_throttle_mid"), FpvThrottleMid);
    O->TryGetNumberField(TEXT("fpv_throttle_expo"), FpvThrottleExpo);
    O->TryGetNumberField(TEXT("fpv_throttle_limit_pct"), FpvThrottleLimitPct);
    O->TryGetStringField(TEXT("fpv_rates_name"), FpvRatesName);
    O->TryGetNumberField(TEXT("camera_width_px"), CamWidthPx);
    O->TryGetNumberField(TEXT("camera_height_px"), CamHeightPx);
    O->TryGetNumberField(TEXT("camera_fps"), CamFps);
    O->TryGetNumberField(TEXT("camera_survey_hfov_deg"), CamSurveyHFovDeg);
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
    O->TryGetNumberField(TEXT("world_mode"), WorldMode);
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
    To.FpvUseBuild = From.FpvUseBuild;
    To.FpvDroneName = From.FpvDroneName;
    To.FpvFrame = From.FpvFrame;
    To.FpvMotor = From.FpvMotor;
    To.FpvProp = From.FpvProp;
    To.FpvBattery = From.FpvBattery;
    To.FpvCameraSystem = From.FpvCameraSystem;
    To.FpvPidMaster = From.FpvPidMaster;
    To.FpvPidDamping = From.FpvPidDamping;
    To.FpvThrottleMid = From.FpvThrottleMid;
    To.FpvThrottleExpo = From.FpvThrottleExpo;
    To.FpvThrottleLimitPct = From.FpvThrottleLimitPct;
    To.FpvRatesName = From.FpvRatesName;
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
    if (ActivePage == PageBuilder)
    {
        LeaveBuilder(/*bSave*/ false);
        return true;
    }
    if (ActivePage == PageRates)
    {
        LeaveRates(/*bSave*/ false);
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
        [
            BigButton(Txt(TEXT("Drone builder")), [this]() { EnterBuilder(); })
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 0.f, 0.f, 4.f))
        [
            SNew(STextBlock).Font(PanelFont(9)).ColorAndOpacity(FLinearColor(0.6f, 1.f, 0.65f))
            .Text_Lambda([this]() { return FText::FromString(FString::Printf(TEXT("FPV drone: %s"), *Settings.FpvDroneName)); })
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Rates and throttle")), [this]() { EnterRates(); }) ]
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
        [ BigButton(Txt(TEXT("Drone builder")), [this]() { EnterBuilder(); }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(ButtonPad)
        [ BigButton(Txt(TEXT("Rates and throttle")), [this]() { EnterRates(); }) ]
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

// --- Drone builder page ----------------------------------------------------------------------

namespace
{
FString DronesDir()
{
    return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Drones"));
}

TArray<FString> ListUserDrones()
{
    TArray<FString> Files;
    IFileManager::Get().FindFiles(Files, *(DronesDir() / TEXT("*.json")), true, false);
    TArray<FString> Names;
    for (const FString& FoundFile : Files)
    {
        Names.Add(FPaths::GetBaseFilename(FoundFile));
    }
    Names.Sort();
    return Names;
}

int32 PresetIndexByName(const FString& Name)
{
    for (int32 I = 0; I < FpvBuild::CountOf(FpvBuild::kPresets); ++I)
    {
        if (Name == UTF8_TO_TCHAR(FpvBuild::kPresets[I].Name))
        {
            return I;
        }
    }
    return -1;
}

bool SameParts(const FDroneSimSettings& S, const FpvBuild::Preset& P)
{
    return S.FpvFrame == P.Parts.FrameIndex && S.FpvMotor == P.Parts.MotorIndex && S.FpvProp == P.Parts.PropIndex &&
           S.FpvBattery == P.Parts.BatteryIndex && S.FpvCameraSystem == P.Parts.FpvIndex &&
           FMath::IsNearlyEqual(S.FpvPidMaster, P.PidMaster, 0.001) && FMath::IsNearlyEqual(S.FpvPidDamping, P.PidDamping, 0.001);
}
} // namespace

void SDroneSettingsPanel::EnterBuilder()
{
    CommitChoices();
    SettingsBackup = Settings;
    DroneNote.Reset();
    RefreshDroneList();
    if (!Preview.IsValid() && GEngine != nullptr && GEngine->GameViewport != nullptr)
    {
        Preview = ADronePreview::SpawnIn(GEngine->GameViewport->GetWorld());
    }
    if (PreviewBrush.IsValid() && Preview.IsValid())
    {
        PreviewBrush->SetResourceObject(Preview->GetImage());
    }
    BuildKey = -1;
    RefreshBuildCache();
    ActivePage = PageBuilder;
}

SDroneSettingsPanel::~SDroneSettingsPanel()
{
    if (PreviewBrush.IsValid())
    {
        PreviewBrush->SetResourceObject(nullptr);
    }
    if (Preview.IsValid())
    {
        Preview->Destroy();
    }
}

void SDroneSettingsPanel::LeaveBuilder(bool bSave)
{
    if (PreviewBrush.IsValid())
    {
        PreviewBrush->SetResourceObject(nullptr);
    }
    if (Preview.IsValid())
    {
        Preview->Destroy();
    }
    Preview.Reset();
    if (bSave)
    {
        Settings.FpvUseBuild = 1;
        Fire(EDroneMenuAction::ApplySettings);
    }
    else
    {
        const FDroneSimSettings Edited = Settings;
        Settings = SettingsBackup;
        DroneMaps::CopyMapFields(Edited, Settings);
        SyncChoices();
    }
    ActivePage = RootPage();
}

void SDroneSettingsPanel::SelectPresetDrone(int32 Index)
{
    const FpvBuild::Preset& P = FpvBuild::kPresets[FMath::Clamp(Index, 0, FpvBuild::CountOf(FpvBuild::kPresets) - 1)];
    Settings.FpvFrame = P.Parts.FrameIndex;
    Settings.FpvMotor = P.Parts.MotorIndex;
    Settings.FpvProp = P.Parts.PropIndex;
    Settings.FpvBattery = P.Parts.BatteryIndex;
    Settings.FpvCameraSystem = P.Parts.FpvIndex;
    Settings.FpvPidMaster = P.PidMaster;
    Settings.FpvPidDamping = P.PidDamping;
    Settings.FpvDroneName = UTF8_TO_TCHAR(P.Name);
    DroneNote.Reset();
}

void SDroneSettingsPanel::SelectUserDrone(const FString& Entry)
{
    FString Text;
    TSharedPtr<FJsonObject> O;
    if (!FFileHelper::LoadFileToString(Text, *(DronesDir() / (Entry + TEXT(".json")))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), O) || !O.IsValid())
    {
        DroneNote = FString::Printf(TEXT("Could not read %s.json"), *Entry);
        return;
    }
    O->TryGetNumberField(TEXT("frame"), Settings.FpvFrame);
    O->TryGetNumberField(TEXT("motor"), Settings.FpvMotor);
    O->TryGetNumberField(TEXT("prop"), Settings.FpvProp);
    O->TryGetNumberField(TEXT("battery"), Settings.FpvBattery);
    O->TryGetNumberField(TEXT("camera_system"), Settings.FpvCameraSystem);
    O->TryGetNumberField(TEXT("pid_master"), Settings.FpvPidMaster);
    O->TryGetNumberField(TEXT("pid_damping"), Settings.FpvPidDamping);
    if (!O->TryGetStringField(TEXT("name"), Settings.FpvDroneName) || Settings.FpvDroneName.IsEmpty())
    {
        Settings.FpvDroneName = Entry;
    }
    DroneNote = FString::Printf(TEXT("Loaded \"%s\"."), *Settings.FpvDroneName);
}

void SDroneSettingsPanel::SaveUserDrone()
{
    FString Name = Settings.FpvDroneName.TrimStartAndEnd();
    if (Name.IsEmpty() || PresetIndexByName(Name) >= 0)
    {
        Name = Name.IsEmpty() ? FString(TEXT("My drone")) : Name + TEXT(" (mine)");
        Settings.FpvDroneName = Name;
    }
    TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
    O->SetStringField(TEXT("name"), Name);
    O->SetNumberField(TEXT("frame"), Settings.FpvFrame);
    O->SetNumberField(TEXT("motor"), Settings.FpvMotor);
    O->SetNumberField(TEXT("prop"), Settings.FpvProp);
    O->SetNumberField(TEXT("battery"), Settings.FpvBattery);
    O->SetNumberField(TEXT("camera_system"), Settings.FpvCameraSystem);
    O->SetNumberField(TEXT("pid_master"), Settings.FpvPidMaster);
    O->SetNumberField(TEXT("pid_damping"), Settings.FpvPidDamping);
    FString Text;
    FJsonSerializer::Serialize(O, TJsonWriterFactory<>::Create(&Text));
    IFileManager::Get().MakeDirectory(*DronesDir(), true);
    const FString Path = DronesDir() / (DroneMaps::FileNameFor(Name) + TEXT(".json"));
    DroneNote = FFileHelper::SaveStringToFile(Text, *Path) ? FString::Printf(TEXT("Saved %s"), *Path)
                                                          : FString::Printf(TEXT("Could not write %s"), *Path);
    RefreshDroneList();
}

void SDroneSettingsPanel::DeleteUserDrone()
{
    const FString Entry = DroneMaps::FileNameFor(Settings.FpvDroneName);
    const FString Path = DronesDir() / (Entry + TEXT(".json"));
    if (!IFileManager::Get().FileExists(*Path))
    {
        DroneNote = TEXT("Ready-made drones can't be deleted; pick one of your drones.");
        return;
    }
    DroneNote = IFileManager::Get().Delete(*Path, false, false, true) ? FString::Printf(TEXT("Deleted \"%s\"."), *Entry)
                                                                     : FString::Printf(TEXT("Could not delete %s"), *Path);
    RefreshDroneList();
}

void SDroneSettingsPanel::RefreshDroneList()
{
    if (!DroneListBox.IsValid())
    {
        return;
    }
    DroneListBox->ClearChildren();
    const TArray<FString> Names = ListUserDrones();
    if (Names.Num() == 0)
    {
        DroneListBox->AddSlot().AutoHeight()[ Hint(Txt(TEXT("None yet: change the parts and press \"Save as my drone\"."))) ];
    }
    for (const FString& Entry : Names)
    {
        DroneListBox->AddSlot()
            .AutoHeight()
            .Padding(FMargin(0.f, 1.f))
            [
                SNew(SButton)
                .IsFocusable(false)
                .Text(FText::FromString(Entry))
                .ButtonColorAndOpacity_Lambda([this, Entry]()
                {
                    return FSlateColor(DroneMaps::FileNameFor(Settings.FpvDroneName) == Entry ? FLinearColor(0.15f, 0.6f, 0.2f)
                                                                                                : FLinearColor(0.3f, 0.3f, 0.3f));
                })
                .OnClicked_Lambda([this, Entry]()
                {
                    SelectUserDrone(Entry);
                    return FReply::Handled();
                })
            ];
    }
}

// --- Graphs (curves like the drone / rates pages of other FPV simulators) ------------------

namespace
{
struct FGraphSeries
{
    TArray<FVector2f> Points; // data units
    FLinearColor Color;
};

struct FGraphData
{
    FString Title;
    FString TopLabel;
    FString BottomLabel;
    TArray<FGraphSeries> Series;
    double XMin = 0.0;
    double XMax = 1.0;
    double YMin = 0.0;
    double YMax = 1.0;
    bool bAxesThroughZero = false;
};

const FLinearColor GraphOrange(1.f, 0.66f, 0.05f);
const FLinearColor GraphBlue(0.35f, 0.65f, 1.f);

class SFpvGraph : public SLeafWidget
{
public:
    SLATE_BEGIN_ARGS(SFpvGraph) {}
    SLATE_ARGUMENT(TFunction<FGraphData()>, Data)
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs)
    {
        DataFn = InArgs._Data;
    }

    virtual FVector2D ComputeDesiredSize(float) const override
    {
        return FVector2D(170.0, 130.0);
    }

    virtual int32 OnPaint(const FPaintArgs& Args, const FGeometry& AllottedGeometry, const FSlateRect& MyCullingRect,
                          FSlateWindowElementList& OutDrawElements, int32 LayerId, const FWidgetStyle& InWidgetStyle,
                          bool bParentEnabled) const override
    {
        const FGraphData D = DataFn ? DataFn() : FGraphData();
        const FVector2f Size = AllottedGeometry.GetLocalSize();
        const FSlateBrush* White = FCoreStyle::Get().GetBrush("WhiteBrush");
        const TSharedRef<FSlateFontMeasure> Measure = FSlateApplication::Get().GetRenderer()->GetFontMeasureService();
        const FSlateFontInfo TitleFont = FCoreStyle::GetDefaultFontStyle("Regular", 9);
        const FSlateFontInfo ValueFont = FCoreStyle::GetDefaultFontStyle("Bold", 11);

        FSlateDrawElement::MakeBox(OutDrawElements, LayerId, AllottedGeometry.ToPaintGeometry(), White, ESlateDrawEffect::None,
                                   FLinearColor(0.02f, 0.022f, 0.028f, 0.92f));
        const float Left = 8.f, Right = Size.X - 8.f, Top = 22.f, Bottom = Size.Y - 8.f;
        const double SpanX = FMath::Max(D.XMax - D.XMin, 1e-9), SpanY = FMath::Max(D.YMax - D.YMin, 1e-9);
        auto Map = [&](double X, double Y)
        {
            return FVector2f(Left + static_cast<float>((X - D.XMin) / SpanX) * (Right - Left),
                             Bottom - static_cast<float>((Y - D.YMin) / SpanY) * (Bottom - Top));
        };
        auto Lines = [&](const TArray<FVector2f>& Points, const FLinearColor& Color, float Thickness, int32 Layer)
        {
            FSlateDrawElement::MakeLines(OutDrawElements, Layer, AllottedGeometry.ToPaintGeometry(), Points, ESlateDrawEffect::None,
                                         Color, true, Thickness);
        };
        auto Label = [&](const FString& Text, float X, float Y, const FSlateFontInfo& Font, const FLinearColor& Color, bool bRight)
        {
            const FVector2f TextSize = Measure->Measure(Text, Font);
            const float X0 = bRight ? X - TextSize.X : X;
            FSlateDrawElement::MakeText(OutDrawElements, LayerId + 3,
                                        AllottedGeometry.ToPaintGeometry(TextSize, FSlateLayoutTransform(FVector2f(X0, Y))), Text, Font,
                                        ESlateDrawEffect::None, Color);
        };

        const FLinearColor AxisColor(0.55f, 0.55f, 0.6f, 0.8f);
        if (D.bAxesThroughZero)
        {
            Lines({Map(D.XMin, 0.0), Map(D.XMax, 0.0)}, AxisColor, 1.f, LayerId + 1);
            Lines({Map(0.0, D.YMin), Map(0.0, D.YMax)}, AxisColor, 1.f, LayerId + 1);
        }
        else
        {
            Lines({FVector2f(Left, Top), FVector2f(Left, Bottom), FVector2f(Right, Bottom)}, AxisColor, 1.f, LayerId + 1);
        }
        for (const FGraphSeries& S : D.Series)
        {
            TArray<FVector2f> Mapped;
            Mapped.Reserve(S.Points.Num());
            for (const FVector2f& Point : S.Points)
            {
                Mapped.Add(Map(Point.X, Point.Y));
            }
            if (Mapped.Num() >= 2)
            {
                Lines(Mapped, S.Color, 2.f, LayerId + 2);
            }
        }
        Label(D.Title, 8.f, 3.f, TitleFont, FLinearColor::White, false);
        Label(D.TopLabel, Left + 6.f, Top + 2.f, ValueFont, GraphOrange, false);
        Label(D.BottomLabel, Right - 2.f, Bottom - 18.f, TitleFont, FLinearColor(0.75f, 0.75f, 0.8f), true);
        return LayerId + 3;
    }

private:
    TFunction<FGraphData()> DataFn;
};

FGraphData CurveGraph(const FpvBuild::Curve& C)
{
    FGraphData D;
    D.Title = UTF8_TO_TCHAR(C.Title);
    FGraphSeries S;
    S.Color = GraphOrange;
    double MaxY = 0.0;
    for (size_t I = 0; I < C.X.size(); ++I)
    {
        S.Points.Add(FVector2f(static_cast<float>(C.X[I]), static_cast<float>(C.Y[I])));
        MaxY = FMath::Max(MaxY, C.Y[I]);
    }
    D.Series.Add(S);
    if (!C.X.empty())
    {
        D.XMin = C.X.front();
        D.XMax = C.X.back();
        D.TopLabel = FString::Printf(TEXT("%.3g %s"), MaxY, UTF8_TO_TCHAR(C.YUnit));
        D.BottomLabel = FString::Printf(TEXT("%.3g %s at %.3g %s"), C.Y.back(), UTF8_TO_TCHAR(C.YUnit), C.X.back(), UTF8_TO_TCHAR(C.XUnit));
    }
    D.YMax = FMath::Max(MaxY * 1.08, 1e-6);
    return D;
}

FpvBuild::Selection SelectionOf(const FDroneSimSettings& S)
{
    FpvBuild::Selection Sel;
    Sel.FrameIndex = S.FpvFrame;
    Sel.MotorIndex = S.FpvMotor;
    Sel.PropIndex = S.FpvProp;
    Sel.BatteryIndex = S.FpvBattery;
    Sel.FpvIndex = S.FpvCameraSystem;
    return Sel;
}

// Video camera presets: what flight.mp4 / frames.csv / the DroneLocalization export will carry.
struct FCameraPreset
{
    const TCHAR* Name;
    int32 Width;
    int32 Height;
    int32 Fps;
    double SurveyHFovDeg;
    double FpvHFovDeg;
    double FpvUptiltDeg;
};

const FCameraPreset CameraPresets[] = {
    {TEXT("Survey 720p"), 1280, 720, 30, 36.87, 110.0, 30.0}, // = FlightSimulator's camera (8.8 mm sensor, 13.2 mm lens)
    {TEXT("Survey 1080p"), 1920, 1080, 30, 36.87, 110.0, 30.0},
    {TEXT("Survey 4K"), 3840, 2160, 30, 36.87, 110.0, 30.0},
    {TEXT("FPV analog"), 640, 480, 30, 36.87, 118.0, 30.0},
    {TEXT("FPV digital"), 1920, 1080, 60, 36.87, 120.0, 25.0},
    {TEXT("Action cam"), 1920, 1080, 60, 36.87, 86.0, 10.0}, // linear (de-fished) mode
};

// Rates profiles (Betaflight "Actual" rates + throttle curve).
struct FRatesPreset
{
    const TCHAR* Name;
    double RpCenter, RpMax, RpExpo;
    double YawCenter, YawMax, YawExpo;
    double ThrMid, ThrExpo, ThrLimit;
};

const FRatesPreset RatesPresets[] = {
    {TEXT("Super smooth"), 40.0, 400.0, 0.30, 40.0, 350.0, 0.30, 0.50, 0.30, 100.0},
    {TEXT("Smooth"), 60.0, 550.0, 0.45, 60.0, 450.0, 0.45, 0.50, 0.20, 100.0},
    {TEXT("Medium"), 70.0, 670.0, 0.54, 70.0, 670.0, 0.54, 0.50, 0.00, 100.0},
    {TEXT("Fast"), 100.0, 850.0, 0.55, 100.0, 700.0, 0.50, 0.50, 0.00, 100.0},
    {TEXT("Race"), 200.0, 900.0, 0.25, 150.0, 600.0, 0.20, 0.40, 0.00, 100.0},
    {TEXT("Cinematic"), 30.0, 300.0, 0.20, 30.0, 200.0, 0.20, 0.35, 0.40, 85.0},
};

int32 RatesIndexByName(const FString& Name)
{
    for (int32 I = 0; I < UE_ARRAY_COUNT(RatesPresets); ++I)
    {
        if (Name == RatesPresets[I].Name)
        {
            return I;
        }
    }
    return -1;
}

FString RatesDir()
{
    return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Rates"));
}

TArray<FString> ListJsonNames(const FString& Dir)
{
    TArray<FString> Files;
    IFileManager::Get().FindFiles(Files, *(Dir / TEXT("*.json")), true, false);
    TArray<FString> Names;
    for (const FString& FoundFile : Files)
    {
        Names.Add(FPaths::GetBaseFilename(FoundFile));
    }
    Names.Sort();
    return Names;
}

TSharedRef<SWidget> ListButton(const FText& Label, TFunction<bool()> IsActive, TFunction<void()> OnClick)
{
    return SNew(SButton)
        .IsFocusable(false)
        .Text(Label)
        .ButtonColorAndOpacity_Lambda([IsActive]()
        {
            return FSlateColor(IsActive() ? FLinearColor(0.85f, 0.55f, 0.02f) : FLinearColor(0.3f, 0.3f, 0.3f));
        })
        .OnClicked_Lambda([OnClick]()
        {
            OnClick();
            return FReply::Handled();
        });
}

// One cell of the stats strip: small grey label over a big orange value.
TSharedRef<SWidget> StatCell(const TCHAR* LabelText, TFunction<FString()> Value)
{
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()
        [ SNew(STextBlock).Text(FText::FromString(LabelText)).Font(FCoreStyle::GetDefaultFontStyle("Regular", 9))
          .ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.75f)) ]
        + SVerticalBox::Slot().AutoHeight()
        [ SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Bold", 13)).ColorAndOpacity(GraphOrange)
          .Text_Lambda([Value]() { return FText::FromString(Value()); }) ];
}
} // namespace

void SDroneSettingsPanel::RefreshBuildCache()
{
    const FpvBuild::Selection Sel = SelectionOf(Settings);
    const int64 Key = ((((static_cast<int64>(Sel.FrameIndex) * 64 + Sel.MotorIndex) * 64 + Sel.PropIndex) * 64 + Sel.BatteryIndex) * 64) +
                      Sel.FpvIndex;
    if (Key != BuildKey)
    {
        BuildKey = Key;
        const FpvBuild::Physical P = FpvBuild::Derive(Sel);
        BuildSummary = FpvBuild::Summarize(Sel, P);
        BuildCurves = FpvBuild::ComputeCurves(P);
    }
    if (ADronePreview* Model = Preview.Get())
    {
        Model->ShowBuild(Sel);
    }
}

TSharedRef<SWidget> SDroneSettingsPanel::BuilderPage()
{
    if (!PreviewBrush.IsValid())
    {
        PreviewBrush = MakeShared<FSlateBrush>();
        PreviewBrush->ImageSize = FVector2D(960.0, 540.0);
        PreviewBrush->DrawAs = ESlateBrushDrawType::Image;
    }
    TSharedRef<SVerticalBox> Ready = SNew(SVerticalBox);
    for (int32 Index = 0; Index < FpvBuild::CountOf(FpvBuild::kPresets); ++Index)
    {
        Ready->AddSlot().AutoHeight().Padding(FMargin(0.f, 1.f))
        [
            SNew(SBox)
            .ToolTipText(FText::FromString(UTF8_TO_TCHAR(FpvBuild::kPresets[Index].Description)))
            [
                ListButton(FText::FromString(UTF8_TO_TCHAR(FpvBuild::kPresets[Index].Name)),
                           [this, Index]() { return PresetIndexByName(Settings.FpvDroneName) == Index; },
                           [this, Index]() { SelectPresetDrone(Index); })
            ]
        ];
    }
    auto Graph = [this](TFunction<const FpvBuild::Curve&(const FpvBuild::Curves&)> Pick) -> TSharedRef<SWidget>
    {
        return SNew(SBox).HeightOverride(150.f)
        [
            SNew(SFpvGraph).Data([this, Pick]()
            {
                RefreshBuildCache();
                return CurveGraph(Pick(BuildCurves));
            })
        ];
    };
    auto Stat = [this](const TCHAR* LabelText, TFunction<FString(const FDroneSimSettings&, const FpvBuild::Summary&)> Fn)
    {
        return StatCell(LabelText, [this, Fn]()
        {
            RefreshBuildCache();
            return Fn(Settings, BuildSummary);
        });
    };
    using namespace FpvBuild;
    TSharedRef<SWidget> Page =
        SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("DRONE BUILDER"))) ]
        + SVerticalBox::Slot().FillHeight(1.f).Padding(FMargin(0.f, 8.f))
        [
            SNew(SHorizontalBox)
            // 1) The list of drones.
            + SHorizontalBox::Slot().FillWidth(0.2f).Padding(FMargin(0.f, 0.f, 10.f, 0.f))
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Ready-made drones"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f, 0.f, 10.f))[ Ready ]
                + SScrollBox::Slot()[ Header(Txt(TEXT("My drones"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))[ SAssignNew(DroneListBox, SVerticalBox) ]
            ]
            // 2) 3D model, stats, graphs.
            + SHorizontalBox::Slot().FillWidth(0.47f).Padding(FMargin(0.f, 0.f, 10.f, 0.f))
            [
                SNew(SVerticalBox)
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.035f, 0.037f, 0.045f))
                    .Padding(0.f)
                    .OnMouseButtonDown_Lambda([](const FGeometry&, const FPointerEvent&) { return FReply::Handled(); })
                    .OnMouseMove_Lambda([this](const FGeometry&, const FPointerEvent& Event)
                    {
                        if (Event.IsMouseButtonDown(EKeys::LeftMouseButton) && Preview.IsValid())
                        {
                            Preview->Orbit(Event.GetCursorDelta().X * 0.4f, -Event.GetCursorDelta().Y * 0.3f);
                        }
                        return FReply::Handled();
                    })
                    [
                        SNew(SBox).HeightOverride(380.f)
                        [ SNew(SImage).Image(PreviewBrush.Get()) ]
                    ]
                ]
                + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().FillWidth(1.f).VAlign(VAlign_Center)
                    [ Hint(Txt(TEXT("Drag the model to turn it."))) ]
                    + SHorizontalBox::Slot().AutoWidth().Padding(2.f)
                    [ ActionButton(Txt(TEXT("Zoom +")), [this]() { if (Preview.IsValid()) { Preview->Zoom(0.85f); } }) ]
                    + SHorizontalBox::Slot().AutoWidth().Padding(2.f)
                    [ ActionButton(Txt(TEXT("Zoom -")), [this]() { if (Preview.IsValid()) { Preview->Zoom(1.18f); } }) ]
                    + SHorizontalBox::Slot().AutoWidth().Padding(2.f)
                    [ ActionButton(Txt(TEXT("Reset view")), [this]() { if (Preview.IsValid()) { Preview->ResetView(); } }) ]
                ]
                // Stats strip.
                + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
                [
                    SNew(SBorder)
                    .BorderImage(FCoreStyle::Get().GetBrush("WhiteBrush"))
                    .BorderBackgroundColor(FLinearColor(0.9f, 0.9f, 0.92f, 0.08f))
                    .Padding(FMargin(10.f, 6.f))
                    [
                        SNew(SVerticalBox)
                        + SVerticalBox::Slot().AutoHeight()
                        [
                            SNew(SHorizontalBox)
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Props"), [](const FDroneSimSettings& S, const Summary&)
                            {
                                const Prop& Pr = kProps[Pick(S.FpvProp, CountOf(kProps))];
                                return FString::Printf(TEXT("%.1f x %.1f x%d"), Pr.DiameterIn, Pr.PitchIn, Pr.Blades);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Battery"), [](const FDroneSimSettings& S, const Summary&)
                            {
                                const Battery& B = kBatteries[Pick(S.FpvBattery, CountOf(kBatteries))];
                                return FString::Printf(TEXT("%dS %.0f mAh"), B.Cells, B.CapacityMah);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Motors, KV"), [](const FDroneSimSettings& S, const Summary&)
                            {
                                return FString::Printf(TEXT("%.0f"), kMotors[Pick(S.FpvMotor, CountOf(kMotors))].Kv);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Weight, kg"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.3f"), Sm.MassG / 1000.0);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Flight time (hover)"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.1f min"), Sm.HoverMinutes);
                            }) ]
                        ]
                        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 0.f))
                        [
                            SNew(SHorizontalBox)
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Max speed, km/h"), [this](const FDroneSimSettings&, const Summary&)
                            {
                                return BuildCurves.VelocityTime.Y.empty() ? FString(TEXT("-"))
                                                                          : FString::Printf(TEXT("%.0f"), BuildCurves.VelocityTime.Y.back());
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Max RPM"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.0f"), Sm.MaxRpm);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Thrust, kg"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.2f"), Sm.MaxThrustKg);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Thrust / weight"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.1f"), Sm.ThrustToWeight);
                            }) ]
                            + SHorizontalBox::Slot().FillWidth(1.f)[ Stat(TEXT("Hover throttle"), [](const FDroneSimSettings&, const Summary& Sm)
                            {
                                return FString::Printf(TEXT("%.0f %%"), Sm.HoverThrottle * 100.0);
                            }) ]
                        ]
                    ]
                ]
                // Graphs.
                + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.DropTime; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.ThrustRpm; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.DragVelocity; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.VelocityTime; }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(2.f)[ Graph([](const Curves& C) -> const Curve& { return C.ThrustTime; }) ]
                ]
                + SVerticalBox::Slot().AutoHeight()
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(1.f, 0.55f, 0.3f))
                    .Text_Lambda([this]()
                    {
                        RefreshBuildCache();
                        return BuildSummary.Warnings.empty() ? FText::GetEmpty()
                                                             : FText::FromString(FString(TEXT("Warning: ")) + UTF8_TO_TCHAR(BuildSummary.Warnings.c_str()));
                    })
                ]
            ]
            // 3) Name, parts, camera.
            + SHorizontalBox::Slot().FillWidth(0.33f)
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ TextRow(Txt(TEXT("Name")), &Settings.FpvDroneName) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f))
                    .Text_Lambda([this]()
                    {
                        const int32 Index = PresetIndexByName(Settings.FpvDroneName);
                        if (Index < 0)
                        {
                            return Txt(TEXT("Your own build."));
                        }
                        const FpvBuild::Preset& P = FpvBuild::kPresets[Index];
                        FString Text = UTF8_TO_TCHAR(P.Description);
                        if (!SameParts(Settings, P))
                        {
                            Text += TEXT("  (modified: save it as your drone)");
                        }
                        return FText::FromString(Text);
                    })
                ]
                + SScrollBox::Slot()[ BuildSection() ]
                + SScrollBox::Slot()[ CameraSection() ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 10.f, 0.f, 0.f))
                [
                    SNew(SHorizontalBox)
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
                    [ ActionButton(Txt(TEXT("Save as my drone")), [this]() { SaveUserDrone(); }) ]
                    + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
                    [ ActionButton(Txt(TEXT("Delete my drone")), [this]() { DeleteUserDrone(); }) ]
                ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f))
                    .Text_Lambda([this]() { return FText::FromString(DroneNote); })
                ]
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 0.f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
            [ BigButton(Txt(TEXT("Fly this drone (save)")), [this]() { LeaveBuilder(/*bSave*/ true); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
            [ BigButton(Txt(TEXT("Cancel")), [this]() { LeaveBuilder(/*bSave*/ false); }) ]
        ];
    RefreshDroneList();
    return Page;
}

// Camera for the localizer: resolution, frame rate, field of view, and the pinhole intrinsics they give.
TSharedRef<SWidget> SDroneSettingsPanel::CameraSection()
{
    const FMargin RowPad(0.f, 2.f);
    TSharedRef<SHorizontalBox> Presets = SNew(SHorizontalBox);
    for (int32 Index = 0; Index < UE_ARRAY_COUNT(CameraPresets); ++Index)
    {
        Presets->AddSlot().FillWidth(1.f).Padding(1.f)
        [
            ListButton(FText::FromString(CameraPresets[Index].Name),
                       [this, Index]()
                       {
                           const FCameraPreset& C = CameraPresets[Index];
                           return Settings.CamWidthPx == C.Width && Settings.CamHeightPx == C.Height && Settings.CamFps == C.Fps &&
                                  FMath::IsNearlyEqual(Settings.FpvCameraFovDeg, C.FpvHFovDeg, 0.01);
                       },
                       [this, Index]()
                       {
                           const FCameraPreset& C = CameraPresets[Index];
                           Settings.CamWidthPx = C.Width;
                           Settings.CamHeightPx = C.Height;
                           Settings.CamFps = C.Fps;
                           Settings.CamSurveyHFovDeg = C.SurveyHFovDeg;
                           Settings.FpvCameraFovDeg = C.FpvHFovDeg;
                           Settings.FpvCameraUptiltDeg = C.FpvUptiltDeg;
                       })
        ];
    }
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 12.f, 0.f, 2.f))[ Header(Txt(TEXT("Camera (what the localizer gets)"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ Hint(Txt(TEXT("Goes into flight.mp4, frames.csv (hfov_deg) and the DroneLocalization export. The render is a pinhole camera: ")
                   TEXT("fx = fy = W / (2 tan(HFOV/2)), cx = W/2, cy = H/2, no lens distortion. Resolution and frame rate change between flights."))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ Presets ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ IntRow(Txt(TEXT("Width, px")), &Settings.CamWidthPx, 64, 3840) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ IntRow(Txt(TEXT("Height, px")), &Settings.CamHeightPx, 64, 2160) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ IntRow(Txt(TEXT("Frame rate, fps (frame_step in DroneLocalization counts these)")), &Settings.CamFps, 1, 120) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("Survey camera HFOV, deg (36.87 = FlightSimulator)")), &Settings.CamSurveyHFovDeg, 5.0, 170.0, 0.5, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("FPV camera HFOV, deg")), &Settings.FpvCameraFovDeg, 40.0, 170.0, 1.0, 1) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("FPV camera uptilt, deg")), &Settings.FpvCameraUptiltDeg, -30.0, 80.0, 1.0, 0) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f))
        [
            SNew(STextBlock).Font(FCoreStyle::GetDefaultFontStyle("Mono", 9)).AutoWrapText(true)
            .ColorAndOpacity(FLinearColor(0.85f, 0.95f, 1.f))
            .Text_Lambda([this]()
            {
                const double W = FMath::Max(Settings.CamWidthPx, 1), H = FMath::Max(Settings.CamHeightPx, 1);
                auto Line = [W, H](const TCHAR* Name, double HFovDeg)
                {
                    const double Fx = 0.5 * W / FMath::Tan(FMath::DegreesToRadians(HFovDeg) * 0.5);
                    const double VFovDeg = FMath::RadiansToDegrees(2.0 * FMath::Atan(0.5 * H / Fx));
                    const double FocalMm = Fx * 36.0 / W; // the export's nominal 36 mm sensor
                    return FString::Printf(TEXT("%s: fx = fy = %.1f px, cx = %.1f, cy = %.1f, VFOV %.1f deg, focal %.1f mm (36 mm sensor)"),
                                           Name, Fx, 0.5 * W, 0.5 * H, VFovDeg, FocalMm);
                };
                const double SurveyFx = 0.5 * W / FMath::Tan(FMath::DegreesToRadians(Settings.CamSurveyHFovDeg) * 0.5);
                const double GsdCm = 100.0 * 100.0 / SurveyFx; // nadir, 100 m above ground
                return FText::FromString(FString::Printf(TEXT("%dx%d @ %d fps, aspect %.3f\n%s\n  ground sample distance at 100 m nadir: %.1f cm/px\n%s"),
                                                         Settings.CamWidthPx, Settings.CamHeightPx, Settings.CamFps, W / H,
                                                         *Line(TEXT("Survey"), Settings.CamSurveyHFovDeg), GsdCm,
                                                         *Line(TEXT("FPV"), Settings.FpvCameraFovDeg)));
            })
        ];
}

// --- Rates and throttle page -----------------------------------------------------------------

void SDroneSettingsPanel::EnterRates()
{
    CommitChoices();
    SettingsBackup = Settings;
    RatesNote.Reset();
    RefreshRatesList();
    ActivePage = PageRates;
}

void SDroneSettingsPanel::LeaveRates(bool bSave)
{
    if (bSave)
    {
        Fire(EDroneMenuAction::ApplySettings);
    }
    else
    {
        const FDroneSimSettings Edited = Settings;
        Settings = SettingsBackup;
        DroneMaps::CopyMapFields(Edited, Settings);
        SyncChoices();
    }
    ActivePage = RootPage();
}

void SDroneSettingsPanel::SelectPresetRates(int32 Index)
{
    const FRatesPreset& R = RatesPresets[FMath::Clamp(Index, 0, static_cast<int32>(UE_ARRAY_COUNT(RatesPresets)) - 1)];
    Settings.FpvRollPitchCenterRateDps = R.RpCenter;
    Settings.FpvRollPitchMaxRateDps = R.RpMax;
    Settings.FpvRollPitchExpo = R.RpExpo;
    Settings.FpvYawCenterRateDps = R.YawCenter;
    Settings.FpvYawMaxRateDps = R.YawMax;
    Settings.FpvYawExpo = R.YawExpo;
    Settings.FpvThrottleMid = R.ThrMid;
    Settings.FpvThrottleExpo = R.ThrExpo;
    Settings.FpvThrottleLimitPct = R.ThrLimit;
    Settings.FpvRatesName = R.Name;
    RatesNote.Reset();
}

void SDroneSettingsPanel::SelectUserRates(const FString& Entry)
{
    FString Text;
    TSharedPtr<FJsonObject> O;
    if (!FFileHelper::LoadFileToString(Text, *(RatesDir() / (Entry + TEXT(".json")))) ||
        !FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), O) || !O.IsValid())
    {
        RatesNote = FString::Printf(TEXT("Could not read %s.json"), *Entry);
        return;
    }
    O->TryGetNumberField(TEXT("roll_pitch_center_dps"), Settings.FpvRollPitchCenterRateDps);
    O->TryGetNumberField(TEXT("roll_pitch_max_dps"), Settings.FpvRollPitchMaxRateDps);
    O->TryGetNumberField(TEXT("roll_pitch_expo"), Settings.FpvRollPitchExpo);
    O->TryGetNumberField(TEXT("yaw_center_dps"), Settings.FpvYawCenterRateDps);
    O->TryGetNumberField(TEXT("yaw_max_dps"), Settings.FpvYawMaxRateDps);
    O->TryGetNumberField(TEXT("yaw_expo"), Settings.FpvYawExpo);
    O->TryGetNumberField(TEXT("throttle_mid"), Settings.FpvThrottleMid);
    O->TryGetNumberField(TEXT("throttle_expo"), Settings.FpvThrottleExpo);
    O->TryGetNumberField(TEXT("throttle_limit_pct"), Settings.FpvThrottleLimitPct);
    if (!O->TryGetStringField(TEXT("name"), Settings.FpvRatesName) || Settings.FpvRatesName.IsEmpty())
    {
        Settings.FpvRatesName = Entry;
    }
    RatesNote = FString::Printf(TEXT("Loaded \"%s\"."), *Settings.FpvRatesName);
}

void SDroneSettingsPanel::SaveUserRates()
{
    FString Name = Settings.FpvRatesName.TrimStartAndEnd();
    if (Name.IsEmpty() || RatesIndexByName(Name) >= 0)
    {
        Name = Name.IsEmpty() ? FString(TEXT("My rates")) : Name + TEXT(" (mine)");
        Settings.FpvRatesName = Name;
    }
    TSharedRef<FJsonObject> O = MakeShared<FJsonObject>();
    O->SetStringField(TEXT("name"), Name);
    O->SetNumberField(TEXT("roll_pitch_center_dps"), Settings.FpvRollPitchCenterRateDps);
    O->SetNumberField(TEXT("roll_pitch_max_dps"), Settings.FpvRollPitchMaxRateDps);
    O->SetNumberField(TEXT("roll_pitch_expo"), Settings.FpvRollPitchExpo);
    O->SetNumberField(TEXT("yaw_center_dps"), Settings.FpvYawCenterRateDps);
    O->SetNumberField(TEXT("yaw_max_dps"), Settings.FpvYawMaxRateDps);
    O->SetNumberField(TEXT("yaw_expo"), Settings.FpvYawExpo);
    O->SetNumberField(TEXT("throttle_mid"), Settings.FpvThrottleMid);
    O->SetNumberField(TEXT("throttle_expo"), Settings.FpvThrottleExpo);
    O->SetNumberField(TEXT("throttle_limit_pct"), Settings.FpvThrottleLimitPct);
    FString Text;
    FJsonSerializer::Serialize(O, TJsonWriterFactory<>::Create(&Text));
    IFileManager::Get().MakeDirectory(*RatesDir(), true);
    const FString Path = RatesDir() / (DroneMaps::FileNameFor(Name) + TEXT(".json"));
    RatesNote = FFileHelper::SaveStringToFile(Text, *Path) ? FString::Printf(TEXT("Saved %s"), *Path)
                                                          : FString::Printf(TEXT("Could not write %s"), *Path);
    RefreshRatesList();
}

void SDroneSettingsPanel::DeleteUserRates()
{
    const FString Entry = DroneMaps::FileNameFor(Settings.FpvRatesName);
    const FString Path = RatesDir() / (Entry + TEXT(".json"));
    if (!IFileManager::Get().FileExists(*Path))
    {
        RatesNote = TEXT("Ready-made profiles can't be deleted; pick one of yours.");
        return;
    }
    RatesNote = IFileManager::Get().Delete(*Path, false, false, true) ? FString::Printf(TEXT("Deleted \"%s\"."), *Entry)
                                                                      : FString::Printf(TEXT("Could not delete %s"), *Path);
    RefreshRatesList();
}

void SDroneSettingsPanel::RefreshRatesList()
{
    if (!RatesListBox.IsValid())
    {
        return;
    }
    RatesListBox->ClearChildren();
    const TArray<FString> Names = ListJsonNames(RatesDir());
    if (Names.Num() == 0)
    {
        RatesListBox->AddSlot().AutoHeight()[ Hint(Txt(TEXT("None yet: change the rates and press \"Save as my profile\"."))) ];
    }
    for (const FString& Entry : Names)
    {
        RatesListBox->AddSlot().AutoHeight().Padding(FMargin(0.f, 1.f))
        [
            ListButton(FText::FromString(Entry), [this, Entry]() { return DroneMaps::FileNameFor(Settings.FpvRatesName) == Entry; },
                       [this, Entry]() { SelectUserRates(Entry); })
        ];
    }
}

TSharedRef<SWidget> SDroneSettingsPanel::RatesPage()
{
    const FMargin RowPad(0.f, 2.f);
    TSharedRef<SVerticalBox> Ready = SNew(SVerticalBox);
    for (int32 Index = 0; Index < UE_ARRAY_COUNT(RatesPresets); ++Index)
    {
        Ready->AddSlot().AutoHeight().Padding(FMargin(0.f, 1.f))
        [
            ListButton(FText::FromString(RatesPresets[Index].Name), [this, Index]() { return RatesIndexByName(Settings.FpvRatesName) == Index; },
                       [this, Index]() { SelectPresetRates(Index); })
        ];
    }
    auto RatesOf = [](double Center, double Max, double Expo)
    {
        FpvModel::Rates R;
        R.CenterDps = Center;
        R.MaxDps = Max;
        R.Expo = Expo;
        return R;
    };
    TSharedRef<SWidget> Page =
        SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight()[ PageTitle(Txt(TEXT("RATES AND THROTTLE"))) ]
        + SVerticalBox::Slot().FillHeight(1.f).Padding(FMargin(0.f, 8.f))
        [
            SNew(SHorizontalBox)
            // Profiles.
            + SHorizontalBox::Slot().FillWidth(0.24f).Padding(FMargin(0.f, 0.f, 12.f, 0.f))
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Rates profiles"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f, 0.f, 10.f))[ Ready ]
                + SScrollBox::Slot()[ Header(Txt(TEXT("My profiles"))) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 4.f))[ SAssignNew(RatesListBox, SVerticalBox) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 10.f, 0.f, 2.f))[ TextRow(Txt(TEXT("Name")), &Settings.FpvRatesName) ]
                + SScrollBox::Slot().Padding(RowPad)[ ActionButton(Txt(TEXT("Save as my profile")), [this]() { SaveUserRates(); }) ]
                + SScrollBox::Slot().Padding(RowPad)[ ActionButton(Txt(TEXT("Delete my profile")), [this]() { DeleteUserRates(); }) ]
                + SScrollBox::Slot().Padding(RowPad)
                [
                    SNew(STextBlock).Font(PanelFont(9)).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.7f, 0.7f, 0.7f))
                    .Text_Lambda([this]() { return FText::FromString(RatesNote); })
                ]
            ]
            // Rates.
            + SHorizontalBox::Slot().FillWidth(0.42f).Padding(FMargin(0.f, 0.f, 12.f, 0.f))
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Rates (Betaflight \"Actual\")"))) ]
                + SScrollBox::Slot().Padding(RowPad)
                [ Hint(Txt(TEXT("Centre = deg/s near the stick centre (sensitivity), max = deg/s at full stick, expo = how long the curve stays soft."))) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Roll / pitch: centre, deg/s")), &Settings.FpvRollPitchCenterRateDps, 10.0, 2000.0, 5.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Roll / pitch: max, deg/s")), &Settings.FpvRollPitchMaxRateDps, 10.0, 2000.0, 10.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Roll / pitch: expo")), &Settings.FpvRollPitchExpo, 0.0, 1.0, 0.01, 2) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Yaw: centre, deg/s")), &Settings.FpvYawCenterRateDps, 10.0, 2000.0, 5.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Yaw: max, deg/s")), &Settings.FpvYawMaxRateDps, 10.0, 2000.0, 10.0, 0) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Yaw: expo")), &Settings.FpvYawExpo, 0.0, 1.0, 0.01, 2) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 8.f, 0.f, 0.f))
                [
                    SNew(SBox).HeightOverride(330.f)
                    [
                        SNew(SFpvGraph).Data([this, RatesOf]()
                        {
                            const FpvModel::Rates Rp = RatesOf(Settings.FpvRollPitchCenterRateDps, Settings.FpvRollPitchMaxRateDps, Settings.FpvRollPitchExpo);
                            const FpvModel::Rates Yw = RatesOf(Settings.FpvYawCenterRateDps, Settings.FpvYawMaxRateDps, Settings.FpvYawExpo);
                            FGraphData D;
                            D.Title = TEXT("Rate | Stick   (blue: roll / pitch, orange: yaw)");
                            FGraphSeries A, B;
                            A.Color = GraphBlue;
                            B.Color = GraphOrange;
                            for (int32 I = 0; I <= 100; ++I)
                            {
                                const double X = -1.0 + 2.0 * I / 100.0;
                                A.Points.Add(FVector2f(static_cast<float>(X), static_cast<float>(FpvModel::ActualRateDps(X, Rp))));
                                B.Points.Add(FVector2f(static_cast<float>(X), static_cast<float>(FpvModel::ActualRateDps(X, Yw))));
                            }
                            const double Peak = FMath::Max3(FpvModel::ActualRateDps(1.0, Rp), FpvModel::ActualRateDps(1.0, Yw), 1.0);
                            D.Series = {A, B};
                            D.XMin = -1.0;
                            D.XMax = 1.0;
                            D.YMin = -Peak * 1.05;
                            D.YMax = Peak * 1.05;
                            D.bAxesThroughZero = true;
                            D.TopLabel = FString::Printf(TEXT("%.0f / %.0f deg/s"), FpvModel::ActualRateDps(1.0, Rp), FpvModel::ActualRateDps(1.0, Yw));
                            D.BottomLabel = TEXT("stick -1 .. 1");
                            return D;
                        })
                    ]
                ]
            ]
            // Throttle.
            + SHorizontalBox::Slot().FillWidth(0.34f)
            [
                SNew(SScrollBox)
                + SScrollBox::Slot()[ Header(Txt(TEXT("Throttle"))) ]
                + SScrollBox::Slot().Padding(RowPad)
                [ Hint(Txt(TEXT("Like Betaflight thr_mid / thr_expo: the curve flattens around the mid point (finer hover); the limit scales the whole range."))) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Throttle mid")), &Settings.FpvThrottleMid, 0.05, 0.95, 0.01, 2) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Throttle expo")), &Settings.FpvThrottleExpo, 0.0, 1.0, 0.01, 2) ]
                + SScrollBox::Slot().Padding(RowPad)[ NumberRow(Txt(TEXT("Throttle limit, %")), &Settings.FpvThrottleLimitPct, 10.0, 100.0, 1.0, 0) ]
                + SScrollBox::Slot().Padding(FMargin(0.f, 8.f, 0.f, 0.f))
                [
                    SNew(SBox).HeightOverride(330.f)
                    [
                        SNew(SFpvGraph).Data([this]()
                        {
                            FGraphData D;
                            D.Title = TEXT("Throttle out | Stick");
                            FGraphSeries Linear, Curve;
                            Linear.Color = FLinearColor(0.4f, 0.4f, 0.45f);
                            Curve.Color = GraphOrange;
                            Linear.Points = {FVector2f(0.f, 0.f), FVector2f(1.f, 1.f)};
                            for (int32 I = 0; I <= 100; ++I)
                            {
                                const double X = I / 100.0;
                                Curve.Points.Add(FVector2f(static_cast<float>(X),
                                                           static_cast<float>(FpvBuild::ThrottleCurve(X, Settings.FpvThrottleMid, Settings.FpvThrottleExpo,
                                                                                                      Settings.FpvThrottleLimitPct))));
                            }
                            D.Series = {Linear, Curve};
                            D.YMax = 1.02;
                            D.TopLabel = FString::Printf(TEXT("max %.0f %%"), Settings.FpvThrottleLimitPct);
                            D.BottomLabel = TEXT("stick 0 .. 1");
                            return D;
                        })
                    ]
                ]
            ]
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 6.f, 0.f, 0.f))
        [
            SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(0.f, 0.f, 4.f, 0.f))
            [ BigButton(Txt(TEXT("Save")), [this]() { LeaveRates(/*bSave*/ true); }) ]
            + SHorizontalBox::Slot().FillWidth(1.f).Padding(FMargin(4.f, 0.f, 0.f, 0.f))
            [ BigButton(Txt(TEXT("Cancel")), [this]() { LeaveRates(/*bSave*/ false); }) ]
        ];
    RefreshRatesList();
    return Page;
}


// Drone builder: pick the parts, see what they add up to (FpvBuild::Summarize), fly it in FPV.
TSharedRef<SWidget> SDroneSettingsPanel::BuildSection()
{
    const FMargin RowPad(0.f, 2.f);
    // "<  name  >" stepper for one catalog; long part names don't fit in a row of buttons.
    auto PartRow = [](const TCHAR* Label, int32* Value, int32 Count, TFunction<const char*(int32)> NameOf) -> TSharedRef<SWidget>
    {
        auto Step = [Value, Count](int32 Dir)
        {
            *Value = ((FMath::Clamp(*Value, 0, Count - 1) + Dir) % Count + Count) % Count;
            return FReply::Handled();
        };
        return SNew(SHorizontalBox)
            + SHorizontalBox::Slot().FillWidth(0.35f).VAlign(VAlign_Center)
            [ SNew(STextBlock).Text(Txt(Label)).Font(PanelFont()).ColorAndOpacity(FLinearColor::White) ]
            + SHorizontalBox::Slot().AutoWidth().Padding(1.f)
            [ SNew(SButton).IsFocusable(false).Text(Txt(TEXT("<"))).OnClicked_Lambda([Step]() { return Step(-1); }) ]
            + SHorizontalBox::Slot().FillWidth(0.65f).VAlign(VAlign_Center).Padding(6.f, 0.f)
            [
                SNew(STextBlock).Font(PanelFont()).ColorAndOpacity(FLinearColor(0.6f, 1.f, 0.65f))
                .Text_Lambda([Value, Count, NameOf]() { return FText::FromString(UTF8_TO_TCHAR(NameOf(FMath::Clamp(*Value, 0, Count - 1)))); })
            ]
            + SHorizontalBox::Slot().AutoWidth().Padding(1.f)
            [ SNew(SButton).IsFocusable(false).Text(Txt(TEXT(">"))).OnClicked_Lambda([Step]() { return Step(1); }) ];
    };
    using namespace FpvBuild;
    return SNew(SVerticalBox)
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 12.f, 0.f, 2.f))[ Header(Txt(TEXT("FPV drone builder"))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ Hint(Txt(TEXT("Physical model: Betaflight-style rate PID + airmode mixer, motors from KV / winding resistance / prop inertia, battery with voltage sag. Rates above still apply; thrust/weight comes from the parts."))) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("FPV flight model")), &Settings.FpvUseBuild, {Txt(TEXT("Simple (thrust/weight)")), Txt(TEXT("Built from parts"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ PartRow(TEXT("Frame"), &Settings.FpvFrame, CountOf(kFrames), [](int32 I) { return kFrames[I].Name; }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ PartRow(TEXT("Motors"), &Settings.FpvMotor, CountOf(kMotors), [](int32 I) { return kMotors[I].Name; }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ PartRow(TEXT("Props"), &Settings.FpvProp, CountOf(kProps), [](int32 I) { return kProps[I].Name; }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ PartRow(TEXT("Battery"), &Settings.FpvBattery, CountOf(kBatteries), [](int32 I) { return kBatteries[I].Name; }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ PartRow(TEXT("Camera / VTX"), &Settings.FpvCameraSystem, CountOf(kFpvSystems), [](int32 I) { return kFpvSystems[I].Name; }) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("PID master multiplier")), &Settings.FpvPidMaster, 0.2, 3.0, 0.05, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ NumberRow(Txt(TEXT("PID damping (D) multiplier")), &Settings.FpvPidDamping, 0.2, 3.0, 0.05, 2) ]
        + SVerticalBox::Slot().AutoHeight().Padding(FMargin(0.f, 4.f, 0.f, 2.f))
        [
            SNew(STextBlock).Font(PanelFont(10)).AutoWrapText(true).ColorAndOpacity(FLinearColor(0.85f, 0.95f, 1.f))
            .Text_Lambda([this]()
            {
                Selection Sel;
                Sel.FrameIndex = Settings.FpvFrame;
                Sel.MotorIndex = Settings.FpvMotor;
                Sel.PropIndex = Settings.FpvProp;
                Sel.BatteryIndex = Settings.FpvBattery;
                Sel.FpvIndex = Settings.FpvCameraSystem;
                const Physical P = Derive(Sel);
                const Summary Sm = Summarize(Sel, P);
                return FText::FromString(FString::Printf(
                    TEXT("All-up weight %.0f g   thrust/weight %.1f   hover throttle %.0f%%\n")
                    TEXT("Hover current %.0f A (about %.1f min hovering)   full throttle %.0f A (%.0f A per motor)\n")
                    TEXT("%s%s"),
                    Sm.MassG, Sm.ThrustToWeight, Sm.HoverThrottle * 100.0, Sm.HoverCurrentA, Sm.HoverMinutes, Sm.MaxCurrentA,
                    Sm.MotorMaxCurrentA, Sm.Warnings.empty() ? TEXT("") : TEXT("\nWarning: "),
                    UTF8_TO_TCHAR(Sm.Warnings.c_str())));
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
        [ ChoiceRow(Txt(TEXT("World")), &Settings.WorldMode,
                    {Txt(TEXT("Flat: terrain + imagery")), Txt(TEXT("3D: buildings, trees (Google)"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("3D uses Google Photorealistic 3D Tiles from Cesium ion (asset 2275207; add it to your ion account ")
                     TEXT("from the Asset Depot). It replaces the terrain and the imagery above, so the imagery choice and ")
                     TEXT("Wayback dates do not apply. Telemetry heights still use the terrain (bare ground).")))
        ]
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
            Hint(Txt(TEXT("Cesium keeps the map outside the view loaded (at this detail, not drawn), so turning the drone ")
                     TEXT("or the camera shows it at once; no extra cameras. More detail costs GPU memory and loading. ")
                     TEXT("In flight, L locks the loaded map: nothing loads or unloads until L again.")))
        ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("New detail appears")), &Settings.TileLoadingStyle,
                    {Txt(TEXT("Step by step (coarse, finer, final)")), Txt(TEXT("At once (may stay coarse a long time)"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [ ChoiceRow(Txt(TEXT("Preload the map's area")), &PreloadChoice, {Txt(TEXT("Off")), Txt(TEXT("On"))}) ]
        + SVerticalBox::Slot().AutoHeight().Padding(RowPad)
        [
            Hint(Txt(TEXT("Preload: the survey area (or 1 km around the start point) plus the margin is loaded at the detail ")
                     TEXT("of the mission altitude and kept loaded while the map is selected. Progress: the main menu and the status line. ")
                     TEXT("Uses an extra camera per preloaded square (20 for 16 km2 at 500 m): each costs game-thread time every frame.")))
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
                + SVerticalBox::Slot().AutoHeight().Padding(RowPad)[ KeyRow(Txt(TEXT("R")), Txt(TEXT("after a crash: put the drone back on its legs; otherwise (video: Record mp4) start / stop a recording clip"))) ]
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
                    return (ActivePage == PageMaps || ActivePage == PageSettings || ActivePage == PageHelp || ActivePage == PageDates ||
                            ActivePage == PageBuilder || ActivePage == PageRates)
                        ? EVisibility::Visible
                        : EVisibility::Collapsed;
                })
                .WidthOverride_Lambda([this]() { return FOptionalSize(ActivePage == PageBuilder ? 1500.f : (ActivePage == PageMaps || ActivePage == PageRates ? 1180.f : 660.f)); })
                [
                    Card(SNew(SWidgetSwitcher)
                         .WidgetIndex_Lambda([this]()
                         {
                             return ActivePage == PageSettings ? 1
                                  : (ActivePage == PageHelp ? 2 : (ActivePage == PageDates ? 3 : (ActivePage == PageBuilder ? 4 : (ActivePage == PageRates ? 5 : 0))));
                         })
                         + SWidgetSwitcher::Slot()[ MapsPage() ]
                         + SWidgetSwitcher::Slot()[ SettingsPage() ]
                         + SWidgetSwitcher::Slot()[ HelpPage() ]
                         + SWidgetSwitcher::Slot()[ DatesPage() ]
                         + SWidgetSwitcher::Slot()[ BuilderPage() ]
                         + SWidgetSwitcher::Slot()[ RatesPage() ])
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
