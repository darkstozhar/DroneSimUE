#include "Sim/DroneRouteFlight.h"

#include "Camera/CameraComponent.h"
#include "Cesium3DTileset.h"
#include "CesiumGeoreference.h"
#include "CesiumGlobeAnchorComponent.h"
#include "CesiumSunSky.h"
#include "CollisionQueryParams.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Engine/TextureRenderTarget2D.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Framework/Application/SlateApplication.h"
#include "GameFramework/PlayerController.h"
#include "HAL/FileManager.h"
#include "InputCoreTypes.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "Sim/DroneAreaMarkers.h"
#include "Sim/DroneMission.h"

DEFINE_LOG_CATEGORY_STATIC(LogDroneRoute, Log, All);

namespace
{
constexpr double EarthRadiusM = 6378137.0; // WGS84 semi-major axis
constexpr double GravityMps2 = 9.80665;
constexpr double TraceTopEllipsoidM = 9000.0;
constexpr double TraceBottomEllipsoidM = -500.0;
constexpr int64 CesiumWorldTerrainAssetId = 1;
constexpr double TiltTimeConstantS = 0.3;

// (north, east) metres from point 1 to point 2, equirectangular (fine for legs of a few km).
void LocalOffsetM(double Lat1, double Lon1, double Lat2, double Lon2, double& North, double& East)
{
    North = FMath::DegreesToRadians(Lat2 - Lat1) * EarthRadiusM;
    East = FMath::DegreesToRadians(Lon2 - Lon1) * EarthRadiusM * FMath::Cos(FMath::DegreesToRadians(Lat1));
}

double CompassBearingDeg(double North, double East)
{
    const double Deg = FMath::RadiansToDegrees(FMath::Atan2(East, North));
    return Deg < 0.0 ? Deg + 360.0 : Deg;
}

double Wrap360(double Deg)
{
    const double D = FMath::Fmod(Deg, 360.0);
    return D < 0.0 ? D + 360.0 : D;
}

FDroneWaypoint MakeWaypoint(double Lat, double Lon, double AltM)
{
    FDroneWaypoint W;
    W.Latitude = Lat;
    W.Longitude = Lon;
    W.AltitudeM = AltM;
    return W;
}
} // namespace

ADroneRouteFlight::ADroneRouteFlight()
{
    PrimaryActorTick.bCanEverTick = true;
    AutoPossessPlayer = EAutoReceiveInput::Player0;

    Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
    Root->SetMobility(EComponentMobility::Movable);
    SetRootComponent(Root);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("NadirCamera"));
    Camera->SetupAttachment(Root);

    // Captures only on request (one CaptureScene per video frame).
    VideoCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("VideoCapture"));
    VideoCapture->SetupAttachment(Camera);
    VideoCapture->bCaptureEveryFrame = false;
    VideoCapture->bCaptureOnMovement = false;
    VideoCapture->bAlwaysPersistRenderingState = true; // keeps auto exposure / temporal AA history
    VideoCapture->CaptureSource = SCS_FinalColorLDR;

    GlobeAnchor = CreateDefaultSubobject<UCesiumGlobeAnchorComponent>(TEXT("GlobeAnchor"));

    // Default route: a 2 x 2 km square starting at the FlightSimulator test area, 500 m AGL.
    auto Add = [this](double Lat, double Lon)
    {
        FDroneWaypoint W;
        W.Latitude = Lat;
        W.Longitude = Lon;
        W.AltitudeM = 500.0;
        Waypoints.Add(W);
    };
    Add(48.3650, 26.0650);
    Add(48.3650, 26.0920);
    Add(48.3830, 26.0920);
    Add(48.3830, 26.0650);

    ApplyCameraSettings();
}

void ADroneRouteFlight::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);
    ApplyCameraSettings();
}

void ADroneRouteFlight::ApplyCameraSettings()
{
    if (Camera == nullptr)
    {
        return;
    }
    // Pitch -90 turns the camera's forward axis down and its up axis to the
    // nose, so the image top points in the flight heading.
    Camera->SetRelativeRotation(FRotator(CameraPitchDeg, 0.0, 0.0));
    Camera->SetFieldOfView(CameraHorizontalFovDeg);
    Camera->SetConstraintAspectRatio(true);
    Camera->SetAspectRatio(CameraAspectRatio);
    // Motion blur smears features between frames; a real survey camera would
    // use a short exposure.
    FPostProcessSettings& PP = Camera->PostProcessSettings;
    PP.bOverride_MotionBlurAmount = true;
    PP.MotionBlurAmount = 0.f;

    // The project uses the extended luminance range, so the min/max
    // brightness values are EV100. Min == Max locks the exposure.
    PP.bOverride_AutoExposureBias = true;
    PP.AutoExposureBias = ExposureCompensationEV;
    const bool bFixed = ExposureMode == EDroneExposureMode::Fixed;
    PP.bOverride_AutoExposureMinBrightness = bFixed;
    PP.bOverride_AutoExposureMaxBrightness = bFixed;
    PP.AutoExposureMinBrightness = FixedExposureEV100;
    PP.AutoExposureMaxBrightness = FixedExposureEV100;
    Camera->PostProcessBlendWeight = 1.f;

    if (VideoCapture != nullptr)
    {
        // Same view as the camera (both FOVs are horizontal).
        VideoCapture->FOVAngle = CameraHorizontalFovDeg;
        VideoCapture->PostProcessSettings = PP;
        VideoCapture->PostProcessBlendWeight = 1.f;
    }
}

#if WITH_EDITOR
void ADroneRouteFlight::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
    Super::PostEditChangeProperty(PropertyChangedEvent);
    ApplyCameraSettings();
}
#endif

// --- Lifecycle -------------------------------------------------------------------

FString ADroneRouteFlight::SettingsPath()
{
    return FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("DroneSimSettings.json"));
}

void ADroneRouteFlight::BeginPlay()
{
    Super::BeginPlay();

    Georeference = GlobeAnchor->ResolveGeoreference();
    if (Georeference == nullptr)
    {
        UE_LOG(LogDroneRoute, Error, TEXT("No CesiumGeoreference in the level; the drone will not fly."));
    }

    // The Details route is the default route and its first point the start point.
    BaseRoute = Waypoints;
    bBaseLoopRoute = bLoopRoute;
    if (Waypoints.Num() > 0)
    {
        LastSettings.StartLatitude = Waypoints[0].Latitude;
        LastSettings.StartLongitude = Waypoints[0].Longitude;
        LastSettings.AltitudeM = Waypoints[0].AltitudeM;
    }
    LastSettings.StartHeadingDeg = HeadingDeg;
    LastSettings.SpeedMps = SpeedMps;
    LastSettings.bManual = bStartInManual || FlightMode == EDroneFlightMode::Manual;
    if (bStartInManual)
    {
        FlightMode = EDroneFlightMode::Manual;
    }
    ManualAltitudeM = LastSettings.AltitudeM;

    if (UWorld* World = GetWorld())
    {
        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        SpawnParams.ObjectFlags |= RF_Transient;
        AreaMarkers = World->SpawnActor<ADroneAreaMarkers>(ADroneAreaMarkers::StaticClass(), FTransform::Identity, SpawnParams);
    }

    // Start from the values set in Details, so keys missing from the file keep them.
    FDroneSimSettings Saved = CurrentSettings();
    if (bApplySavedSettings && Saved.LoadFromFile(SettingsPath()))
    {
        UE_LOG(LogDroneRoute, Display, TEXT("Applying saved settings from %s"), *SettingsPath());
    }
    DroneMaps::EnsureDefaultMaps(Saved);
    ApplyLiveSettings(Saved);
    ApplyStartPoint(Saved);
    ClaimView();
    if (bOpenSettingsAtStart)
    {
        OpenMenu(); // the flight starts from the menu
    }
    else
    {
        BeginFlight();
    }
}

void ADroneRouteFlight::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    CloseMenu();
    RemoveStatusLine();
    StopSession();
    Super::EndPlay(EndPlayReason);
}

void ADroneRouteFlight::ResetFlightState()
{
    // Manual flight starts at the start point, a route at its first waypoint.
    if (FlightMode == EDroneFlightMode::Manual || Waypoints.Num() == 0)
    {
        Latitude = LastSettings.StartLatitude;
        Longitude = LastSettings.StartLongitude;
    }
    else
    {
        Latitude = Waypoints[0].Latitude;
        Longitude = Waypoints[0].Longitude;
    }
    LegIndex = 0;
    bRouteFinished = Waypoints.Num() < 2;
    LegLengthM = 0.0;
    if (!bRouteFinished)
    {
        double North = 0.0, East = 0.0;
        LocalOffsetM(Waypoints[0].Latitude, Waypoints[0].Longitude, Waypoints[1].Latitude, Waypoints[1].Longitude,
                     North, East);
        LegLengthM = FMath::Sqrt(North * North + East * East);
        if (FlightMode == EDroneFlightMode::Route)
        {
            HeadingDeg = bRouteFixedHeading ? RouteFixedHeadingDeg : CompassBearingDeg(North, East);
        }
    }
    else if (FlightMode == EDroneFlightMode::Route)
    {
        UE_LOG(LogDroneRoute, Warning, TEXT("Fewer than two waypoints: the drone hovers."));
    }

    ManualVelNorth = ManualVelEast = 0.0;
    ManualClimbMps = 0.0;
    PrevVelNorth = PrevVelEast = 0.0;
    PitchDeg = RollDeg = 0.0;
    bHaveGround = false;
    bHaveStartGround = false;
    SettleReferenceHeightM = 0.0;
    SettleStableS = 0.0;
    SettleWaitedS = 0.0;
    EllipsoidHeightM = InitialGroundEllipsoidHeightM + TargetAltitude();

    if (Georeference != nullptr)
    {
        GlobeAnchor->MoveToLongitudeLatitudeHeight(FVector(Longitude, Latitude, EllipsoidHeightM));
        GlobeAnchor->SetEastSouthUpRotation(FRotator(0.0, HeadingDeg - 90.0, 0.0).Quaternion());
    }
}

void ADroneRouteFlight::StartSession()
{
    SessionDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Flights") /
                                                   FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")));
    StatusNote.Reset();
    bSessionActive = true;
    SimTimeS = 0.0;
    FrameIndex = 0;
    SinceTelemetryS = 0.0;
    SinceHeartbeatS = 1.0;
    if (bSendMavlink)
    {
        Telemetry.OpenMavlink(MavlinkHost, MavlinkPort);
    }
    if (bWriteGroundTruthCsv)
    {
        Telemetry.OpenCsv(SessionDir / TEXT("telemetry.csv"));
    }
    StartVideo();
}

void ADroneRouteFlight::StopSession()
{
    StopVideo();
    Telemetry.Close();
    bSessionActive = false;
}

// --- Video -------------------------------------------------------------------------

void ADroneRouteFlight::StartVideo()
{
    if (VideoMode == EDroneVideoMode::Off || VideoCapture == nullptr)
    {
        return;
    }
    if (VideoTarget == nullptr)
    {
        VideoTarget = NewObject<UTextureRenderTarget2D>(this, TEXT("DroneVideoTarget"));
    }
    // FinalColorLDR is already display-encoded (sRGB). A linear (non-sRGB)
    // BGRA8 target stores those bytes as is; TargetGamma 2.2 makes the
    // tonemapper encode exactly as for the screen (a linear target alone
    // would report gamma 1.0 and darken the image).
    VideoTarget->TargetGamma = 2.2f;
    VideoTarget->InitCustomFormat(static_cast<uint32>(VideoWidth), static_cast<uint32>(VideoHeight), PF_B8G8R8A8,
                                 /*bInForceLinearGamma*/ true);
    VideoTarget->UpdateResourceImmediate(true);
    VideoCapture->TextureTarget = VideoTarget;
    ApplyCameraSettings();

    FDroneVideoConfig Config;
    Config.FfmpegPath = FfmpegPath;
    Config.Encoder = VideoEncoder;
    Config.Width = VideoWidth;
    Config.Height = VideoHeight;
    Config.Fps = VideoFps;
    if (VideoMode == EDroneVideoMode::Record)
    {
        Config.Sink = EDroneVideoSink::Mp4File;
        Config.OutputPath = SessionDir / TEXT("flight.mp4");
        Config.MaxQueuedFrames = 8;
    }
    else
    {
        Config.Sink = EDroneVideoSink::Rtsp;
        Config.OutputPath = RtspUrl;
        Config.MaxQueuedFrames = 4;
        if (bStartRtspServer &&
            !RtspServer.EnsureRunning(RtspUrl, RtspServerPath,
                                      FPaths::ConvertRelativePathToFull(FPaths::ProjectLogDir() / TEXT("mediamtx.log"))))
        {
            StatusNote = TEXT("RTSP server (mediamtx) is not running: check its path in F2 settings, see the log (LogDroneRtsp).");
        }
    }
    IFileManager::Get().MakeDirectory(*SessionDir, true);
    if (!Video.Start(Config))
    {
        StatusNote = TEXT("Video could not start: is ffmpeg installed / its path right (F2 settings)? See the log (LogDroneVideo).");
        return;
    }
    Telemetry.OpenFramesCsv(SessionDir / TEXT("frames.csv"));
    VideoFrameIndex = 0;
    SinceVideoFrameS = 0.0;

    if (VideoMode == EDroneVideoMode::Record && bFixedFrameRateWhenRecording && GEngine != nullptr)
    {
        // UEngine's fixed frame rate: waits when faster than VideoFps and
        // reports a delta time of exactly 1/VideoFps when slower.
        bSavedUseFixedFrameRate = GEngine->bUseFixedFrameRate != 0;
        SavedFixedFrameRate = GEngine->FixedFrameRate;
        GEngine->bUseFixedFrameRate = true;
        GEngine->FixedFrameRate = static_cast<float>(FMath::Max(VideoFps, 1));
        bFrameRateOverridden = true;
        UE_LOG(LogDroneRoute, Display, TEXT("Recording at a fixed frame rate of %d fps"), VideoFps);
    }
    else if (VideoMode == EDroneVideoMode::Live && bLimitFrameRateWhenLive && GEngine != nullptr)
    {
        SavedMaxFps = GEngine->GetMaxFPS();
        GEngine->SetMaxFPS(static_cast<float>(FMath::Max(VideoFps, 1)));
        bMaxFpsOverridden = true;
        UE_LOG(LogDroneRoute, Display, TEXT("Streaming: engine capped at %d fps, one video frame per engine frame"), VideoFps);
    }
}

void ADroneRouteFlight::StopVideo()
{
    Video.Stop();
    RtspServer.Stop(); // only if it was started here
    Telemetry.CloseFramesCsv();
    if (bFrameRateOverridden && GEngine != nullptr)
    {
        GEngine->bUseFixedFrameRate = bSavedUseFixedFrameRate;
        GEngine->FixedFrameRate = SavedFixedFrameRate;
    }
    bFrameRateOverridden = false;
    if (bMaxFpsOverridden && GEngine != nullptr)
    {
        GEngine->SetMaxFPS(SavedMaxFps);
    }
    bMaxFpsOverridden = false;
}

void ADroneRouteFlight::CaptureVideoFrame(const FDroneTelemetrySample& Sample, double Dt)
{
    if (VideoMode == EDroneVideoMode::Off || !Video.IsRunning() || VideoTarget == nullptr || VideoCapture == nullptr)
    {
        return;
    }
    const bool bFixedClock = VideoMode == EDroneVideoMode::Record && bFrameRateOverridden;
    const bool bEveryFrame = bFixedClock || (VideoMode == EDroneVideoMode::Live && bMaxFpsOverridden);
    if (!bEveryFrame)
    {
        // Real time: one frame every 1/VideoFps seconds.
        const double Period = 1.0 / FMath::Max(VideoFps, 1);
        SinceVideoFrameS += Dt;
        if (SinceVideoFrameS < Period)
        {
            return;
        }
        SinceVideoFrameS = SinceVideoFrameS >= 2.0 * Period ? 0.0 : SinceVideoFrameS - Period;
    }
    VideoCapture->CaptureScene();
    if (Video.SubmitFrame(VideoTarget))
    {
        // Fixed frame rate: the video time of frame N is exactly N / fps.
        const double VideoTimeS =
            bFixedClock ? static_cast<double>(VideoFrameIndex) / FMath::Max(VideoFps, 1) : Sample.TimeS;
        Telemetry.WriteFrameRow(Sample, VideoFrameIndex, VideoTimeS);
        ++VideoFrameIndex;
    }
}

// --- Settings panel, saved settings, status line -------------------------------------

FDroneSimSettings ADroneRouteFlight::CurrentSettings() const
{
    // Start point, default speed and mission as last applied; the rest as the drone has it now.
    FDroneSimSettings S = LastSettings;
    S.bCameraGimbal = bCameraGimbal;
    S.ExposureCompensationEV = ExposureCompensationEV;
    S.VideoMode = static_cast<int32>(VideoMode);
    S.FfmpegPath = FfmpegPath;
    S.VideoEncoder = VideoEncoder;
    S.RtspUrl = RtspUrl;
    S.bStartRtspServer = bStartRtspServer;
    S.RtspServerPath = RtspServerPath;
    S.MavlinkHost = MavlinkHost;
    S.MavlinkPort = MavlinkPort;
    S.CameraHFovDeg = CameraHorizontalFovDeg;
    S.CameraAspect = CameraAspectRatio;
    S.ImageWidthPx = VideoWidth;
    S.ManualMaxSpeedMps = ManualMaxSpeedMps;
    S.ManualAccelerationMps2 = ManualAccelerationMps2;
    S.ManualBrakingMps2 = ManualBrakingMps2;
    S.ManualFastMultiplier = ManualFastMultiplier;
    S.ManualYawRateDegPerSec = ManualYawRateDegPerSec;
    S.ManualClimbRateMps = ManualClimbRateMps;
    S.ManualVerticalAccelerationMps2 = ManualVerticalAccelerationMps2;
    S.StickDeadZone = StickDeadZone;
    S.StickExpo = StickExpo;
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<ACesiumSunSky> It(World); It; ++It)
        {
            S.TimeZone = It->TimeZone;
            S.SolarTime = It->SolarTime;
            S.Day = It->Day;
            S.Month = It->Month;
            S.Year = It->Year;
            S.bDaylightSaving = It->UseDaylightSavingTime;
            break;
        }
    }
    return S;
}

void ADroneRouteFlight::ApplyLiveSettings(const FDroneSimSettings& S)
{
    // Everything but the start point (see ApplyStartPoint) becomes the applied state.
    const FDroneSimSettings Previous = LastSettings;
    LastSettings = S;
    LastSettings.StartLatitude = Previous.StartLatitude;
    LastSettings.StartLongitude = Previous.StartLongitude;
    LastSettings.AltitudeM = Previous.AltitudeM;
    LastSettings.StartHeadingDeg = Previous.StartHeadingDeg;

    // Manual control sensitivity.
    ManualMaxSpeedMps = FMath::Max(S.ManualMaxSpeedMps, 0.5);
    ManualAccelerationMps2 = FMath::Max(S.ManualAccelerationMps2, 0.5);
    ManualBrakingMps2 = FMath::Max(S.ManualBrakingMps2, 0.5);
    ManualFastMultiplier = FMath::Max(S.ManualFastMultiplier, 1.0);
    ManualYawRateDegPerSec = FMath::Max(S.ManualYawRateDegPerSec, 5.0);
    ManualClimbRateMps = FMath::Max(S.ManualClimbRateMps, 0.5);
    ManualVerticalAccelerationMps2 = FMath::Max(S.ManualVerticalAccelerationMps2, 0.5);
    StickDeadZone = FMath::Clamp(S.StickDeadZone, 0.0, 0.5);
    StickExpo = FMath::Clamp(S.StickExpo, 0.0, 1.0);

    // Camera and sun.
    bCameraGimbal = S.bCameraGimbal;
    ExposureCompensationEV = static_cast<float>(S.ExposureCompensationEV);
    ApplyCameraSettings();
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<ACesiumSunSky> It(World); It; ++It)
        {
            It->TimeZone = S.TimeZone;
            It->SolarTime = S.SolarTime;
            It->Day = S.Day;
            It->Month = S.Month;
            It->Year = S.Year;
            It->UseDaylightSavingTime = S.bDaylightSaving;
            It->UpdateSun();
        }
    }

    // Video and telemetry are used when the next flight starts, never in the middle of one.
    if (!bSessionActive)
    {
        VideoMode = static_cast<EDroneVideoMode>(FMath::Clamp(S.VideoMode, 0, 2));
        FfmpegPath = S.FfmpegPath;
        VideoEncoder = S.VideoEncoder;
        RtspUrl = S.RtspUrl;
        bStartRtspServer = S.bStartRtspServer;
        RtspServerPath = S.RtspServerPath;
        MavlinkHost = S.MavlinkHost;
        MavlinkPort = FMath::Clamp(S.MavlinkPort, 1, 65535);
    }
    UpdateMarkers(/*bForce*/ true);
}

void ADroneRouteFlight::ApplyStartPoint(const FDroneSimSettings& S)
{
    LastSettings.StartLatitude = S.StartLatitude;
    LastSettings.StartLongitude = S.StartLongitude;
    LastSettings.AltitudeM = S.AltitudeM;
    LastSettings.StartHeadingDeg = S.StartHeadingDeg;

    // A mission ends: the Details route comes back, shifted to start at the start point.
    Waypoints = BaseRoute;
    if (Waypoints.Num() == 0)
    {
        Waypoints.AddDefaulted();
    }
    bLoopRoute = bBaseLoopRoute;
    RouteLoopStartIndex = 0;
    bRouteFixedHeading = false;
    bMissionLoaded = false;
    MissionFirstIndex = 0;
    MissionPasses = 0;
    const double DLat = S.StartLatitude - Waypoints[0].Latitude;
    const double DLon = S.StartLongitude - Waypoints[0].Longitude;
    for (FDroneWaypoint& W : Waypoints)
    {
        W.Latitude += DLat;
        W.Longitude += DLon;
        W.AltitudeM = S.AltitudeM;
    }
    SpeedMps = FMath::Max(LastSettings.SpeedMps, 0.1);
    FlightMode = (LastSettings.bManual || bStartInManual) ? EDroneFlightMode::Manual : EDroneFlightMode::Route;
    ManualAltitudeM = S.AltitudeM;

    // Put the Unreal origin at the start point: keeps precision high and makes
    // Cesium load the tiles there. The sun position depends on it.
    if (Georeference != nullptr)
    {
        Georeference->SetOriginLongitudeLatitudeHeight(
            FVector(S.StartLongitude, S.StartLatitude, InitialGroundEllipsoidHeightM));
    }
    if (UWorld* World = GetWorld())
    {
        for (TActorIterator<ACesiumSunSky> It(World); It; ++It)
        {
            It->UpdateSun();
        }
    }

    ResetFlightState();
    if (FlightMode == EDroneFlightMode::Manual)
    {
        HeadingDeg = Wrap360(S.StartHeadingDeg);
    }
    UE_LOG(LogDroneRoute, Display, TEXT("Start point: %.6f, %.6f, %.0f m, heading %.0f (map \"%s\")"), S.StartLatitude,
           S.StartLongitude, S.AltitudeM, S.StartHeadingDeg, *LastSettings.MapName);
    UpdateMarkers(/*bForce*/ true);
}

bool ADroneRouteFlight::StartPointDiffers(const FDroneSimSettings& S) const
{
    return bMissionLoaded || FMath::Abs(S.StartLatitude - LastSettings.StartLatitude) > 1e-9 ||
           FMath::Abs(S.StartLongitude - LastSettings.StartLongitude) > 1e-9 ||
           FMath::Abs(S.AltitudeM - LastSettings.AltitudeM) > 1e-6 ||
           FMath::Abs(S.StartHeadingDeg - LastSettings.StartHeadingDeg) > 1e-6;
}

void ADroneRouteFlight::SaveSettingsFile() const
{
    CurrentSettings().SaveToFile(SettingsPath());
}

void ADroneRouteFlight::BeginFlight()
{
    if (bFlying)
    {
        return;
    }
    bFlying = true;
    StartSession();
    AddStatusLine();
    UE_LOG(LogDroneRoute, Display, TEXT("Flight started: %s"), *SessionDir);
}

void ADroneRouteFlight::EndFlight()
{
    if (!bFlying)
    {
        return;
    }
    bFlying = false;
    StopSession();
    RemoveStatusLine();
    UE_LOG(LogDroneRoute, Display, TEXT("Flight ended"));
}

void ADroneRouteFlight::HandleMenuAction(EDroneMenuAction Action, const FDroneSimSettings& S)
{
    switch (Action)
    {
    case EDroneMenuAction::FreeFlight:
    case EDroneMenuAction::FlyMission:
        if (!bFlying)
        {
            // From the main menu: everything applies, the flight starts at the start point.
            ApplyLiveSettings(S);
            if (StartPointDiffers(S))
            {
                ApplyStartPoint(S);
            }
            DroneMaps::CopyMapFields(S, LastSettings);
            SaveSettingsFile();
            CloseMenu();
            BeginFlight();
        }
        if (Action == EDroneMenuAction::FlyMission)
        {
            StartMission(S);
            CloseMenu();
        }
        break;
    case EDroneMenuAction::PreviewMap:
        if (!bFlying && StartPointDiffers(S))
        {
            ApplyStartPoint(S);
        }
        DroneMaps::CopyMapFields(S, LastSettings);
        UpdateMarkers(/*bForce*/ true);
        break;
    case EDroneMenuAction::ApplySettings:
        ApplyLiveSettings(S);
        SaveSettingsFile();
        break;
    case EDroneMenuAction::Resume:
        CloseMenu();
        break;
    case EDroneMenuAction::StopMission:
        SwitchToManual();
        CloseMenu();
        break;
    case EDroneMenuAction::MainMenu:
        CloseMenu(); // keeps the map being edited
        EndFlight();
        ApplyStartPoint(LastSettings);
        OpenMenu();
        break;
    case EDroneMenuAction::Quit:
        UKismetSystemLibrary::QuitGame(this, GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr,
                                       EQuitPreference::Quit, false);
        break;
    }
}

bool ADroneRouteFlight::StartMission(const FDroneSimSettings& MissionSettings)
{
    // The panel's mission fields with the camera this drone really has.
    FDroneSimSettings Merged = CurrentSettings();
    DroneMaps::CopyMissionFields(MissionSettings, Merged);
    Merged.MapName = MissionSettings.MapName;
    const SurveyGrid::Plan MissionPlan = DroneMission::PlanMission(Merged);
    if (!MissionPlan.Error.empty() || MissionPlan.Waypoints.size() < 2)
    {
        StatusNote = FString(TEXT("Mission not started: ")) + DroneMission::FromAscii(MissionPlan.Error);
        UE_LOG(LogDroneRoute, Warning, TEXT("%s"), *StatusNote);
        return false;
    }
    LastSettings = Merged;
    Merged.SaveToFile(SettingsPath());

    const double PassAltitudeM = FMath::Max(Merged.MissionAltitudeM, 1.0);
    const bool bJump = Merged.bMissionJumpToStart;
    TArray<FDroneWaypoint> MissionRoute;
    if (!bJump)
    {
        // First leg: from here, at the current altitude, to the first pass.
        MissionRoute.Add(MakeWaypoint(Latitude, Longitude, TargetAltitude()));
    }
    const int32 FirstIndex = MissionRoute.Num();
    for (const SurveyGrid::LatLon& P : MissionPlan.Waypoints)
    {
        MissionRoute.Add(MakeWaypoint(P.Lat, P.Lon, PassAltitudeM));
    }
    if (Merged.MissionAfter == 1)
    {
        // Back to where the mission started (here, or the first pass after a jump), then hover.
        MissionRoute.Add(MakeWaypoint(MissionRoute[0].Latitude, MissionRoute[0].Longitude, PassAltitudeM));
    }

    Waypoints = MoveTemp(MissionRoute);
    bMissionLoaded = true;
    MissionFirstIndex = FirstIndex;
    MissionPasses = MissionPlan.Passes;
    bLoopRoute = Merged.MissionAfter == 2;
    RouteLoopStartIndex = FirstIndex;
    bRouteFixedHeading = Merged.bMissionFixedHeading;
    RouteFixedHeadingDeg = Wrap360(Merged.MissionHeadingDeg);
    SpeedMps = FMath::Max(Merged.MissionSpeedMps, 0.1);
    FlightMode = EDroneFlightMode::Route;
    LegIndex = 0;
    bRouteFinished = false;
    if (bJump)
    {
        // The drone appears over the start of the first pass and waits there
        // for the terrain, as at program start. The session (files, stream) goes on.
        if (Georeference != nullptr)
        {
            Georeference->SetOriginLongitudeLatitudeHeight(
                FVector(Waypoints[0].Longitude, Waypoints[0].Latitude, InitialGroundEllipsoidHeightM));
        }
        if (UWorld* World = GetWorld())
        {
            for (TActorIterator<ACesiumSunSky> It(World); It; ++It)
            {
                It->UpdateSun();
            }
        }
        ResetFlightState();
        UpdateMarkers(/*bForce*/ true); // the origin moved
    }
    else
    {
        double North = 0.0, East = 0.0;
        LocalOffsetM(Latitude, Longitude, Waypoints[1].Latitude, Waypoints[1].Longitude, North, East);
        LegLengthM = FMath::Sqrt(North * North + East * East);
    }
    if (StatusNote.StartsWith(TEXT("Mission")))
    {
        StatusNote.Reset();
    }
    UE_LOG(LogDroneRoute, Display, TEXT("Mission started: %d passes %.0f m apart at %.0f m, %d waypoints, %.1f km, %s"),
           MissionPlan.Passes, MissionPlan.SpacingM, PassAltitudeM, Waypoints.Num(), MissionPlan.LengthM / 1000.0,
           bJump ? TEXT("jumped to the first pass") : TEXT("flying there from here"));
    return true;
}

void ADroneRouteFlight::OpenMenu()
{
    UWorld* World = GetWorld();
    UGameViewportClient* Viewport = World != nullptr ? World->GetGameViewport() : nullptr;
    if (Viewport == nullptr || SettingsPanel.IsValid())
    {
        return;
    }
    TWeakObjectPtr<ADroneRouteFlight> WeakThis(this);
    TSharedRef<SDroneSettingsPanel> Panel =
        SNew(SDroneSettingsPanel)
            .Initial(CurrentSettings())
            .InFlight(bFlying)
            .DronePosition([WeakThis]()
            {
                return WeakThis.IsValid() ? FVector2D(WeakThis->Latitude, WeakThis->Longitude) : FVector2D::ZeroVector;
            })
            .Status([WeakThis]()
            {
                return WeakThis.IsValid() ? FText::FromString(WeakThis->MenuStatusText()) : FText::GetEmpty();
            })
            .OnAction_Lambda([WeakThis](EDroneMenuAction MenuAction, const FDroneSimSettings& S)
            {
                if (WeakThis.IsValid())
                {
                    WeakThis->PendingAction = MenuAction;
                    WeakThis->PendingSettings = S;
                    WeakThis->bPendingAction = true;
                }
            });
    Viewport->AddViewportWidgetContent(Panel, 100);
    SettingsPanel = Panel;
    UE_LOG(LogDroneRoute, Display, TEXT("Menu opened (%s)"), bFlying ? TEXT("in flight") : TEXT("main menu"));

    if (APlayerController* PC = World->GetFirstPlayerController())
    {
        PC->SetShowMouseCursor(true);
        // No widget to focus: the game viewport keeps the keyboard, so the
        // menu keys (Esc / F2 / Tab) still reach HandleHotkeys; a text field
        // takes the keyboard when clicked.
        FInputModeGameAndUI Mode;
        Mode.SetLockMouseToViewportBehavior(EMouseLockMode::DoNotLock);
        Mode.SetHideCursorDuringCapture(false);
        PC->SetInputMode(Mode);
    }
}

void ADroneRouteFlight::CloseMenu()
{
    if (!SettingsPanel.IsValid())
    {
        return;
    }
    // The map being edited survives closing the menu (written to the file by
    // Save map, or with the settings when a flight starts).
    DroneMaps::CopyMapFields(SettingsPanel->GetDraft(), LastSettings);
    UWorld* World = GetWorld();
    if (UGameViewportClient* Viewport = World != nullptr ? World->GetGameViewport() : nullptr)
    {
        Viewport->RemoveViewportWidgetContent(SettingsPanel.ToSharedRef());
    }
    SettingsPanel.Reset();
    UE_LOG(LogDroneRoute, Display, TEXT("Menu closed"));
    if (APlayerController* PC = World != nullptr ? World->GetFirstPlayerController() : nullptr)
    {
        PC->SetShowMouseCursor(false);
        PC->SetInputMode(FInputModeGameOnly());
    }
}

void ADroneRouteFlight::AddStatusLine()
{
    UWorld* World = GetWorld();
    UGameViewportClient* Viewport = World != nullptr ? World->GetGameViewport() : nullptr;
    if (Viewport == nullptr || StatusLine.IsValid() || !bShowDebugText)
    {
        return;
    }
    TWeakObjectPtr<ADroneRouteFlight> WeakThis(this);
    StatusLine = MakeDroneStatusLine([WeakThis]()
    {
        return WeakThis.IsValid() ? FText::FromString(WeakThis->StatusText()) : FText::GetEmpty();
    });
    Viewport->AddViewportWidgetContent(StatusLine.ToSharedRef(), 10);
}

void ADroneRouteFlight::RemoveStatusLine()
{
    if (!StatusLine.IsValid())
    {
        return;
    }
    UWorld* World = GetWorld();
    if (UGameViewportClient* Viewport = World != nullptr ? World->GetGameViewport() : nullptr)
    {
        Viewport->RemoveViewportWidgetContent(StatusLine.ToSharedRef());
    }
    StatusLine.Reset();
}

FString ADroneRouteFlight::StatusText() const
{
    FString Status;
    if (!bHaveStartGround)
    {
        Status = bHaveGround ? FString::Printf(TEXT("Waiting for the terrain to settle... %.1f / %.1f s"), SettleStableS,
                                             TerrainSettleSeconds)
                           : FString(TEXT("Waiting for Cesium terrain tiles under the drone..."));
        Status += TEXT("   [Esc / F2 / Tab: menu]");
    }
    else
    {
        const bool bManual = FlightMode == EDroneFlightMode::Manual;
        const TCHAR* ModeName = bManual ? TEXT("MANUAL") : (bMissionLoaded ? TEXT("MISSION") : TEXT("ROUTE"));
        Status = FString::Printf(TEXT("%s  lat %.6f  lon %.6f  AGL %.1f m  heading %.1f"), ModeName,
                               Latitude, Longitude, bHaveGround ? EllipsoidHeightM - GroundEllipsoidHeightM : -1.0,
                               HeadingDeg);
        if (!bCameraGimbal)
        {
            Status += FString::Printf(TEXT("  pitch %.1f  roll %.1f"), PitchDeg, RollDeg);
        }
        if (!bManual && bMissionLoaded)
        {
            Status += MissionStatus();
        }
        else if (!bManual)
        {
            Status += FString::Printf(TEXT("  leg %d/%d%s"), LegIndex + 1, Waypoints.Num(),
                                    bRouteFinished ? TEXT(" (finished)") : TEXT(""));
        }
        else if (bMissionLoaded)
        {
            Status += TEXT("  (mission paused, M resumes)");
        }
        if (VideoMode == EDroneVideoMode::Record)
        {
            Status += Video.IsRunning() ? FString::Printf(TEXT("  REC %lld"), VideoFrameIndex) : FString(TEXT("  REC failed"));
        }
        else if (VideoMode == EDroneVideoMode::Live)
        {
            Status += Video.IsRunning()
                ? FString::Printf(TEXT("  LIVE %lld (dropped %lld)"), VideoFrameIndex, Video.GetDroppedFrames())
                : FString(TEXT("  LIVE failed"));
        }
        Status += TEXT("   [Esc / F2 / Tab: menu,  M: manual <-> route / mission,  V: add area vertex]");
    }
    if (!StatusNote.IsEmpty())
    {
        Status += TEXT("\n");
        Status += StatusNote;
    }
    return Status;
}

FString ADroneRouteFlight::MenuStatusText() const
{
    FString Line;
    if (!bHaveStartGround)
    {
        Line = bHaveGround ? FString::Printf(TEXT("Terrain at the start point is still refining... %.1f / %.1f s"),
                                             SettleStableS, TerrainSettleSeconds)
                           : FString(TEXT("Loading the terrain at the start point..."));
    }
    else if (!bFlying)
    {
        Line = FString::Printf(TEXT("Ready: the drone hovers %.0f m above the ground at the start point."),
                               EllipsoidHeightM - GroundEllipsoidHeightM);
    }
    else
    {
        const bool bManual = FlightMode == EDroneFlightMode::Manual;
        Line = bManual ? FString(TEXT("Flying: manual")) : (bMissionLoaded ? TEXT("Flying: mission") + MissionStatus()
                                                                           : FString(TEXT("Flying: route")));
        Line += FString::Printf(TEXT(", %.0f m above the ground"), EllipsoidHeightM - GroundEllipsoidHeightM);
    }
    if (!StatusNote.IsEmpty())
    {
        Line += TEXT("\n");
        Line += StatusNote;
    }
    return Line;
}

void ADroneRouteFlight::UpdateMarkers(bool bForce)
{
    if (AreaMarkers == nullptr)
    {
        return;
    }
    // While the menu is open the markers follow its draft, so the area shows up while it is edited.
    const FDroneSimSettings Source = SettingsPanel.IsValid() ? SettingsPanel->GetDraft() : LastSettings;
    const FString Key = FString::Printf(TEXT("%s|%.1f|%.1f|%d|%d|%.1f|%d"), *Source.MissionPolygon, Source.MissionAltitudeM,
                                        Source.AltitudeM, Source.bShowAreaPoles ? 1 : 0, Source.bShowAreaWalls ? 1 : 0,
                                        Source.AreaWallOpacityPct, Source.bMarkersInVideo ? 1 : 0);
    if (!bForce && Key == MarkersKey)
    {
        return;
    }
    MarkersKey = Key;

    if (VideoCapture != nullptr)
    {
        if (Source.bMarkersInVideo)
        {
            VideoCapture->HiddenActors.Remove(AreaMarkers);
        }
        else
        {
            VideoCapture->HiddenActors.AddUnique(AreaMarkers);
        }
    }

    std::vector<SurveyGrid::LatLon> Parsed;
    std::string ParseError;
    TArray<FVector2D> Vertices;
    if (SurveyGrid::ParseLatLonList(DroneMission::ToAscii(Source.MissionPolygon), Parsed, ParseError))
    {
        for (const SurveyGrid::LatLon& P : Parsed)
        {
            Vertices.Add(FVector2D(P.Lat, P.Lon));
        }
    }
    // One height range for all parts: the terrain found under the vertices
    // (tiles not loaded there yet: the ground under the drone).
    bool bAnyHit = false;
    double LowM = 0.0, HighM = 0.0;
    for (const FVector2D& Vertex : Vertices)
    {
        double Hit = 0.0;
        if (TraceGroundEllipsoidHeight(Vertex.Y, Vertex.X, Hit))
        {
            LowM = bAnyHit ? FMath::Min(LowM, Hit) : Hit;
            HighM = bAnyHit ? FMath::Max(HighM, Hit) : Hit;
            bAnyHit = true;
        }
    }
    if (!bAnyHit)
    {
        LowM = HighM = bHaveGround ? GroundEllipsoidHeightM : InitialGroundEllipsoidHeightM;
    }
    const double TopM = HighM + FMath::Max(Source.MissionAltitudeM, Source.AltitudeM) + 50.0;
    AreaMarkers->Rebuild(Georeference, Vertices, LowM - 300.0, TopM, Source.bShowAreaPoles, Source.bShowAreaWalls,
                         static_cast<float>(Source.AreaWallOpacityPct / 100.0));
}

void ADroneRouteFlight::AddVertexHere()
{
    DroneMaps::AppendVertex(LastSettings.MissionPolygon, Latitude, Longitude);
    const int32 VertexCount = DroneMaps::CountVertices(LastSettings.MissionPolygon);
    StatusNote = FString::Printf(TEXT("Area vertex %d added at %.6f, %.6f (V). Save the map in Menu > Maps and missions."),
                                 VertexCount, Latitude, Longitude);
    UE_LOG(LogDroneRoute, Display, TEXT("%s"), *StatusNote);
    UpdateMarkers(/*bForce*/ false);
}

float ADroneRouteFlight::ShapeStick(float Value) const
{
    const float ZoneSize = FMath::Clamp(static_cast<float>(StickDeadZone), 0.f, 0.9f);
    const float Magnitude = FMath::Abs(Value);
    if (Magnitude <= ZoneSize)
    {
        return 0.f;
    }
    const float Linear = FMath::Min((Magnitude - ZoneSize) / (1.f - ZoneSize), 1.f);
    const float ExpoMix = FMath::Clamp(static_cast<float>(StickExpo), 0.f, 1.f);
    const float Shaped = (1.f - ExpoMix) * Linear + ExpoMix * Linear * Linear * Linear;
    return Value < 0.f ? -Shaped : Shaped;
}

FString ADroneRouteFlight::MissionStatus() const
{
    if (bRouteFinished)
    {
        return TEXT("  mission finished, hovering");
    }
    if (Waypoints.Num() < 2)
    {
        return FString();
    }
    const int32 TargetIndex = NextWaypointIndex(LegIndex);
    const int32 PlanIndex = TargetIndex - MissionFirstIndex; // pass k: plan points 2k (start), 2k + 1 (end)
    double North = 0.0, East = 0.0;
    LocalOffsetM(Latitude, Longitude, Waypoints[TargetIndex].Latitude, Waypoints[TargetIndex].Longitude, North, East);
    const double ToGoM = FMath::Sqrt(North * North + East * East);
    FString Phase;
    if (PlanIndex < 0)
    {
        Phase = TEXT("flying to the area");
    }
    else if (PlanIndex < 2 * MissionPasses)
    {
        const int32 PassNumber = PlanIndex / 2 + 1;
        Phase = PlanIndex % 2 == 0 ? FString::Printf(TEXT("to pass %d/%d"), PassNumber, MissionPasses)
                                   : FString::Printf(TEXT("pass %d/%d"), PassNumber, MissionPasses);
    }
    else
    {
        Phase = TEXT("returning to launch");
    }
    return FString::Printf(TEXT("  %s, %.0f m to go"), *Phase, ToGoM);
}

int32 ADroneRouteFlight::NextWaypointIndex(int32 Index) const
{
    if (Index + 1 < Waypoints.Num())
    {
        return Index + 1;
    }
    return Waypoints.Num() > 0 ? FMath::Clamp(RouteLoopStartIndex, 0, Waypoints.Num() - 1) : 0;
}

bool ADroneRouteFlight::IsTypingInUi() const
{
    if (!SettingsPanel.IsValid() || !FSlateApplication::IsInitialized())
    {
        return false;
    }
    // SEditableText / SMultiLineEditableText, also inside a spin box being typed into.
    const TSharedPtr<SWidget> Focused = FSlateApplication::Get().GetKeyboardFocusedWidget();
    return Focused.IsValid() && Focused->GetTypeAsString().Contains(TEXT("EditableText"));
}

void ADroneRouteFlight::SwitchToManual()
{
    if (FlightMode == EDroneFlightMode::Manual)
    {
        return;
    }
    // Hold the current altitude target (read while still in route mode).
    ManualAltitudeM = TargetAltitude();
    FlightMode = EDroneFlightMode::Manual;
    ManualVelNorth = PrevVelNorth;
    ManualVelEast = PrevVelEast;
    ManualClimbMps = 0.0;
    UE_LOG(LogDroneRoute, Display, TEXT("Flight mode: manual"));
}

void ADroneRouteFlight::SwitchToRoute()
{
    if (FlightMode == EDroneFlightMode::Route)
    {
        return;
    }
    FlightMode = EDroneFlightMode::Route;
    // Continue toward the next waypoint from where the drone is now.
    if (Waypoints.Num() >= 2)
    {
        bRouteFinished = false;
        const FDroneWaypoint& To = Waypoints[NextWaypointIndex(LegIndex)];
        double North = 0.0, East = 0.0;
        LocalOffsetM(Latitude, Longitude, To.Latitude, To.Longitude, North, East);
        LegLengthM = FMath::Sqrt(North * North + East * East);
    }
    UE_LOG(LogDroneRoute, Display, TEXT("Flight mode: %s"), bMissionLoaded ? TEXT("mission") : TEXT("route"));
}

void ADroneRouteFlight::HandleHotkeys()
{
    APlayerController* PC = GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr;
    if (PC == nullptr)
    {
        return;
    }
    // Letters typed into a menu text field reach the game too; while typing
    // only F2 / F1 / gamepad View work. Esc only outside the editor (in
    // Play-in-Editor it stops the game).
    const bool bTyping = IsTypingInUi();
    const bool bMenuKey = PC->WasInputKeyJustPressed(EKeys::F2) || PC->WasInputKeyJustPressed(EKeys::F1) ||
                          PC->WasInputKeyJustPressed(EKeys::Gamepad_Special_Left) ||
                          (!bTyping && PC->WasInputKeyJustPressed(EKeys::Tab)) ||
                          (!bTyping && !GIsEditor && PC->WasInputKeyJustPressed(EKeys::Escape));
    if (bMenuKey)
    {
        if (SettingsPanel.IsValid())
        {
            // A sub page goes back first; the main menu stays until a flight starts.
            if (!SettingsPanel->GoBack() && bFlying)
            {
                bPendingCloseMenu = true;
            }
        }
        else if (bFlying)
        {
            OpenMenu();
        }
    }
    // Flight keys only while flying with the menu closed.
    if (SettingsPanel.IsValid() || !bFlying)
    {
        return;
    }
    if (PC->WasInputKeyJustPressed(EKeys::M) || PC->WasInputKeyJustPressed(EKeys::Gamepad_Special_Right))
    {
        if (FlightMode == EDroneFlightMode::Route)
        {
            SwitchToManual();
        }
        else
        {
            SwitchToRoute();
        }
    }
    if (PC->WasInputKeyJustPressed(EKeys::V))
    {
        AddVertexHere();
    }
}

void ADroneRouteFlight::ClaimView()
{
    // The template's game mode may spawn and possess its own pawn; we only
    // need the player's view to come from this camera.
    if (APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr)
    {
        if (PC->GetViewTarget() != this)
        {
            PC->SetViewTarget(this);
        }
        bViewClaimed = true;
    }
}

// --- Flight ------------------------------------------------------------------------

bool ADroneRouteFlight::TraceGroundEllipsoidHeight(double Lon, double Lat, double& OutHeightM) const
{
    UWorld* World = GetWorld();
    if (World == nullptr || Georeference == nullptr)
    {
        return false;
    }
    const FTransform GeoToWorld = Georeference->GetActorTransform();
    const FVector Top = GeoToWorld.TransformPosition(
        Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(FVector(Lon, Lat, TraceTopEllipsoidM)));
    const FVector Bottom = GeoToWorld.TransformPosition(
        Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(FVector(Lon, Lat, TraceBottomEllipsoidM)));

    // Only the terrain counts as ground: buildings (e.g. Cesium OSM Buildings)
    // and other tilesets are ignored so AGL stays the height above terrain.
    FCollisionQueryParams Params(SCENE_QUERY_STAT(DroneGroundTrace), true, this);
    const ACesium3DTileset* Terrain = nullptr;
    for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
    {
        if (Terrain == nullptr && It->GetIonAssetID() == CesiumWorldTerrainAssetId)
        {
            Terrain = *It;
        }
    }
    if (Terrain != nullptr)
    {
        for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
        {
            if (*It != Terrain)
            {
                Params.AddIgnoredActor(*It);
            }
        }
    }

    FHitResult Hit;
    if (!World->LineTraceSingleByChannel(Hit, Top, Bottom, ECC_Visibility, Params))
    {
        return false;
    }
    const FVector Llh =
        Georeference->TransformUnrealPositionToLongitudeLatitudeHeight(GeoToWorld.InverseTransformPosition(Hit.ImpactPoint));
    OutHeightM = Llh.Z;
    return true;
}

double ADroneRouteFlight::TargetAltitude() const
{
    if (FlightMode == EDroneFlightMode::Manual || Waypoints.Num() == 0)
    {
        return ManualAltitudeM;
    }
    if (Waypoints.Num() == 1 || bRouteFinished)
    {
        return Waypoints[FMath::Clamp(LegIndex, 0, Waypoints.Num() - 1)].AltitudeM;
    }
    const FDroneWaypoint& From = Waypoints[LegIndex];
    const FDroneWaypoint& To = Waypoints[NextWaypointIndex(LegIndex)];
    double North = 0.0, East = 0.0;
    LocalOffsetM(Latitude, Longitude, To.Latitude, To.Longitude, North, East);
    const double Remaining = FMath::Sqrt(North * North + East * East);
    const double Progress = LegLengthM > 1.0 ? FMath::Clamp(1.0 - Remaining / LegLengthM, 0.0, 1.0) : 1.0;
    return FMath::Lerp(From.AltitudeM, To.AltitudeM, Progress);
}

void ADroneRouteFlight::AdvanceAlongRoute(double Dt, double& OutVelNorth, double& OutVelEast)
{
    OutVelNorth = 0.0;
    OutVelEast = 0.0;
    if (bRouteFinished || Waypoints.Num() < 2)
    {
        return;
    }
    const int32 NextIndex = NextWaypointIndex(LegIndex);
    const FDroneWaypoint& To = Waypoints[NextIndex];
    double North = 0.0, East = 0.0;
    LocalOffsetM(Latitude, Longitude, To.Latitude, To.Longitude, North, East);
    const double Distance = FMath::Sqrt(North * North + East * East);
    const double Step = SpeedMps * Dt;

    double MovedNorth = 0.0, MovedEast = 0.0;
    if (Distance <= FMath::Max(Step, 0.5))
    {
        MovedNorth = North;
        MovedEast = East;
        Latitude = To.Latitude;
        Longitude = To.Longitude;
        LegIndex = NextIndex;
        const bool bLastLeg = !bLoopRoute && LegIndex >= Waypoints.Num() - 1;
        if (bLastLeg)
        {
            bRouteFinished = true;
            UE_LOG(LogDroneRoute, Display, TEXT("Route finished; hovering at the last waypoint."));
        }
        else
        {
            const FDroneWaypoint& NewTo = Waypoints[NextWaypointIndex(LegIndex)];
            double LegNorth = 0.0, LegEast = 0.0;
            LocalOffsetM(Latitude, Longitude, NewTo.Latitude, NewTo.Longitude, LegNorth, LegEast);
            LegLengthM = FMath::Sqrt(LegNorth * LegNorth + LegEast * LegEast);
        }
    }
    else
    {
        const double Fraction = Step / Distance;
        MovedNorth = North * Fraction;
        MovedEast = East * Fraction;
        Latitude += FMath::RadiansToDegrees(MovedNorth / EarthRadiusM);
        Longitude += FMath::RadiansToDegrees(MovedEast / (EarthRadiusM * FMath::Cos(FMath::DegreesToRadians(Latitude))));
    }
    OutVelNorth = MovedNorth / Dt;
    OutVelEast = MovedEast / Dt;

    // Turn toward the current leg (or the fixed mission heading) at a limited yaw rate.
    if (!bRouteFinished && Distance > 0.5)
    {
        const double Desired = bRouteFixedHeading ? RouteFixedHeadingDeg : CompassBearingDeg(North, East);
        const double Delta = FMath::FindDeltaAngleDegrees(HeadingDeg, Desired);
        const double MaxTurn = MaxYawRateDegPerSec * Dt;
        HeadingDeg = Wrap360(HeadingDeg + FMath::Clamp(Delta, -MaxTurn, MaxTurn));
    }
}

void ADroneRouteFlight::AdvanceManual(double Dt, double& OutVelNorth, double& OutVelEast)
{
    float Forward = 0.f, Right = 0.f, Yaw = 0.f, Climb = 0.f;
    bool bFast = false;
    APlayerController* PC = GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr;
    if (PC != nullptr && !SettingsPanel.IsValid())
    {
        auto Key = [PC](const FKey& K) { return PC->IsInputKeyDown(K) ? 1.f : 0.f; };
        // Gamepad "mode 2": left stick = climb + yaw, right stick = move.
        Forward = Key(EKeys::W) - Key(EKeys::S) + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_RightY));
        Right = Key(EKeys::D) - Key(EKeys::A) + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_RightX));
        Yaw = Key(EKeys::E) - Key(EKeys::Q) + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftX));
        const float DescendKey = FMath::Max(Key(EKeys::LeftControl), Key(EKeys::RightControl));
        Climb = Key(EKeys::SpaceBar) - DescendKey + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftY));
        bFast = PC->IsInputKeyDown(EKeys::LeftShift) || PC->IsInputKeyDown(EKeys::RightShift) ||
                PC->GetInputAnalogKeyState(EKeys::Gamepad_RightTriggerAxis) > 0.5f;
    }
    Forward = FMath::Clamp(Forward, -1.f, 1.f);
    Right = FMath::Clamp(Right, -1.f, 1.f);
    Yaw = FMath::Clamp(Yaw, -1.f, 1.f);
    Climb = FMath::Clamp(Climb, -1.f, 1.f);

    HeadingDeg = Wrap360(HeadingDeg + Yaw * ManualYawRateDegPerSec * Dt);

    // Climb rate with limited vertical acceleration. The height moves with it
    // directly (feed-forward); the altitude follower in Tick then only has to
    // absorb terrain changes, so Space / Ctrl react without its 1.5 s lag.
    const double MaxClimbChange = ManualVerticalAccelerationMps2 * Dt;
    ManualClimbMps += FMath::Clamp(Climb * ManualClimbRateMps - ManualClimbMps, -MaxClimbChange, MaxClimbChange);
    const double UnclampedAltitudeM = ManualAltitudeM + ManualClimbMps * Dt;
    const double NewAltitudeM =
        FMath::Clamp(UnclampedAltitudeM, ManualMinAltitudeM, FMath::Max(ManualMaxAltitudeM, ManualMinAltitudeM));
    if (NewAltitudeM != UnclampedAltitudeM)
    {
        ManualClimbMps = 0.0; // at the altitude limit
    }
    EllipsoidHeightM += NewAltitudeM - ManualAltitudeM;
    ManualAltitudeM = NewAltitudeM;

    const double MaxSpeed = ManualMaxSpeedMps * (bFast ? ManualFastMultiplier : 1.0);
    const double H = FMath::DegreesToRadians(HeadingDeg);
    const double CosH = FMath::Cos(H);
    const double SinH = FMath::Sin(H);
    // Body forward = (cos H, sin H) and body right = (-sin H, cos H) in (north, east).
    double DesiredNorth = (Forward * CosH - Right * SinH) * MaxSpeed;
    double DesiredEast = (Forward * SinH + Right * CosH) * MaxSpeed;
    const double DesiredSpeed = FMath::Sqrt(DesiredNorth * DesiredNorth + DesiredEast * DesiredEast);
    if (DesiredSpeed > MaxSpeed && DesiredSpeed > 0.0)
    {
        DesiredNorth *= MaxSpeed / DesiredSpeed;
        DesiredEast *= MaxSpeed / DesiredSpeed;
    }
    // Limited acceleration toward the commanded velocity; a change that points
    // against the current velocity (slowing down, reversing) uses the braking limit.
    double DeltaNorth = DesiredNorth - ManualVelNorth;
    double DeltaEast = DesiredEast - ManualVelEast;
    const double DeltaLen = FMath::Sqrt(DeltaNorth * DeltaNorth + DeltaEast * DeltaEast);
    const bool bBraking = DeltaNorth * ManualVelNorth + DeltaEast * ManualVelEast < 0.0;
    const double AccelLimit = (bBraking ? ManualBrakingMps2 : ManualAccelerationMps2) * (bFast ? ManualFastMultiplier : 1.0);
    const double MaxDelta = AccelLimit * Dt;
    if (DeltaLen > MaxDelta && DeltaLen > 0.0)
    {
        DeltaNorth *= MaxDelta / DeltaLen;
        DeltaEast *= MaxDelta / DeltaLen;
    }
    ManualVelNorth += DeltaNorth;
    ManualVelEast += DeltaEast;

    Latitude += FMath::RadiansToDegrees(ManualVelNorth * Dt / EarthRadiusM);
    Longitude += FMath::RadiansToDegrees(ManualVelEast * Dt / (EarthRadiusM * FMath::Cos(FMath::DegreesToRadians(Latitude))));
    OutVelNorth = ManualVelNorth;
    OutVelEast = ManualVelEast;
}

void ADroneRouteFlight::UpdateTilt(double Dt, double VelNorth, double VelEast)
{
    const double H = FMath::DegreesToRadians(HeadingDeg);
    const double CosH = FMath::Cos(H);
    const double SinH = FMath::Sin(H);
    const double Fwd = VelNorth * CosH + VelEast * SinH;
    const double Rgt = -VelNorth * SinH + VelEast * CosH;
    const double PrevFwd = PrevVelNorth * CosH + PrevVelEast * SinH;
    const double PrevRgt = -PrevVelNorth * SinH + PrevVelEast * CosH;
    const double AccFwd = (Fwd - PrevFwd) / Dt;
    const double AccRgt = (Rgt - PrevRgt) / Dt;
    PrevVelNorth = VelNorth;
    PrevVelEast = VelEast;

    if (bCameraGimbal)
    {
        PitchDeg = RollDeg = 0.0; // reported attitude = level camera
        return;
    }
    // A multirotor tilts its thrust toward the horizontal force it needs:
    // acceleration plus air drag. Nose down = negative pitch.
    const double TargetPitch = FMath::Clamp(
        -FMath::RadiansToDegrees(FMath::Atan((AccFwd + LinearDragCoefficient * Fwd) / GravityMps2)), -MaxTiltDeg, MaxTiltDeg);
    const double TargetRoll = FMath::Clamp(
        FMath::RadiansToDegrees(FMath::Atan((AccRgt + LinearDragCoefficient * Rgt) / GravityMps2)), -MaxTiltDeg, MaxTiltDeg);
    const double Alpha = 1.0 - FMath::Exp(-Dt / TiltTimeConstantS);
    PitchDeg += (TargetPitch - PitchDeg) * Alpha;
    RollDeg += (TargetRoll - RollDeg) * Alpha;
}

void ADroneRouteFlight::Tick(float DeltaSeconds)
{
    Super::Tick(DeltaSeconds);
    const double Dt = static_cast<double>(DeltaSeconds);

    if (bPendingAction)
    {
        bPendingAction = false;
        HandleMenuAction(PendingAction, PendingSettings);
    }
    if (bPendingCloseMenu)
    {
        bPendingCloseMenu = false;
        CloseMenu();
    }
    HandleHotkeys();
    if (Georeference == nullptr || Dt <= 0.0)
    {
        return;
    }
    ClaimView();
    SinceMarkersCheckS += Dt;
    if (SinceMarkersCheckS >= 0.5)
    {
        SinceMarkersCheckS = 0.0;
        UpdateMarkers(/*bForce*/ false);
    }

    double Ground = 0.0;
    const bool bGroundHit = TraceGroundEllipsoidHeight(Longitude, Latitude, Ground);
    if (bGroundHit)
    {
        GroundEllipsoidHeightM = Ground;
        bHaveGround = true;
    }
    if (!bHaveStartGround)
    {
        // Hover at the flight altitude above the latest ground estimate so the
        // tiles load at the level of detail of the flight, and start only
        // once that estimate stops changing.
        SettleWaitedS += Dt;
        if (bGroundHit)
        {
            if (FMath::Abs(Ground - SettleReferenceHeightM) > TerrainSettleToleranceM)
            {
                SettleReferenceHeightM = Ground;
                SettleStableS = 0.0;
            }
            else
            {
                SettleStableS += Dt;
            }
            EllipsoidHeightM = Ground + TargetAltitude();
        }
        const bool bSettled = bHaveGround && SettleStableS >= TerrainSettleSeconds;
        const bool bTimedOut = bHaveGround && SettleWaitedS >= TerrainSettleTimeoutS;
        if (bSettled || bTimedOut)
        {
            StartGroundEllipsoidHeightM = GroundEllipsoidHeightM;
            bHaveStartGround = true;
            EllipsoidHeightM = GroundEllipsoidHeightM + TargetAltitude();
            UE_LOG(LogDroneRoute, Display, TEXT("Terrain %s after %.1f s: ground %.1f m (ellipsoid). Starting."),
                   bSettled ? TEXT("settled") : TEXT("still refining (timeout)"), SettleWaitedS, GroundEllipsoidHeightM);
            UpdateMarkers(/*bForce*/ true); // now with the real ground height
        }
    }

    const double PreviousHeight = EllipsoidHeightM;
    double VelNorth = 0.0, VelEast = 0.0;
    if (bHaveStartGround)
    {
        // In the main menu (not flying yet) the drone hovers at the start point.
        if (bFlying && FlightMode == EDroneFlightMode::Manual)
        {
            AdvanceManual(Dt, VelNorth, VelEast);
        }
        else if (bFlying)
        {
            AdvanceAlongRoute(Dt, VelNorth, VelEast);
        }
        const double Base = AltitudeMode == EDroneAltitudeMode::AboveTerrain ? GroundEllipsoidHeightM : StartGroundEllipsoidHeightM;
        const double Target = Base + TargetAltitude();
        const double Alpha = 1.0 - FMath::Exp(-Dt / FMath::Max(AltitudeTimeConstantS, 0.05));
        EllipsoidHeightM += (Target - EllipsoidHeightM) * Alpha;
        UpdateTilt(Dt, VelNorth, VelEast);
    }

    GlobeAnchor->MoveToLongitudeLatitudeHeight(FVector(Longitude, Latitude, EllipsoidHeightM));
    GlobeAnchor->SetEastSouthUpRotation(FRotator(0.0, HeadingDeg - 90.0, 0.0).Quaternion());
    if (Camera != nullptr)
    {
        // Camera = body tilt (zero with a gimbal) then the mount pitch.
        const FQuat Body = FRotator(PitchDeg, 0.0, RollDeg).Quaternion();
        Camera->SetRelativeRotation(Body * FRotator(CameraPitchDeg, 0.0, 0.0).Quaternion());
    }

    if (bHaveStartGround && bSessionActive)
    {
        // Samples are stamped 0, Dt, 2 Dt, ... so sample N lines up with
        // video frame N (time N / fps when recording).
        EmitTelemetry(Dt, VelNorth, VelEast, -(EllipsoidHeightM - PreviousHeight) / Dt);
        SimTimeS += Dt;
    }
}

void ADroneRouteFlight::EmitTelemetry(double Dt, double VelNorth, double VelEast, double VelDown)
{
    FDroneTelemetrySample S;
    S.Frame = FrameIndex++;
    S.TimeS = SimTimeS;
    S.UnixTimeS = (FDateTime::UtcNow() - FDateTime(1970, 1, 1)).GetTotalSeconds();
    S.Latitude = Latitude;
    S.Longitude = Longitude;
    S.EllipsoidHeightM = EllipsoidHeightM;
    S.MslHeightM = EllipsoidHeightM - GeoidUndulationM;
    S.RelativeAltM = EllipsoidHeightM - StartGroundEllipsoidHeightM;
    S.AglM = bHaveGround ? EllipsoidHeightM - GroundEllipsoidHeightM : -1.0;
    S.HeadingDeg = HeadingDeg;
    S.PitchDeg = PitchDeg; // 0 with a gimbal (see bCameraGimbal)
    S.RollDeg = RollDeg;
    S.VelNorthMps = VelNorth;
    S.VelEastMps = VelEast;
    S.VelDownMps = VelDown;
    S.CameraPitchDeg = CameraPitchDeg;
    S.CameraHFovDeg = CameraHorizontalFovDeg;

    Telemetry.WriteCsv(S);

    SinceHeartbeatS += Dt;
    SinceTelemetryS += Dt;
    const double Period = 1.0 / FMath::Max(TelemetryRateHz, 1.f);
    if (SinceTelemetryS >= Period)
    {
        const bool bHeartbeat = SinceHeartbeatS >= 1.0;
        Telemetry.SendMavlink(S, bHeartbeat);
        if (bHeartbeat)
        {
            SinceHeartbeatS = 0.0;
        }
        SinceTelemetryS = SinceTelemetryS >= 2.0 * Period ? 0.0 : SinceTelemetryS - Period;
    }

    CaptureVideoFrame(S, Dt);
}
