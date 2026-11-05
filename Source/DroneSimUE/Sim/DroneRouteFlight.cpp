#include "Sim/DroneRouteFlight.h"

#include "Camera/CameraComponent.h"
#include "Cesium3DTileset.h"
#include "CesiumCameraManager.h"
#include "CesiumRasterOverlay.h"
#include "CesiumUrlTemplateRasterOverlay.h"
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
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformProcess.h"
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
constexpr int64 GooglePhotorealistic3DTilesAssetId = 2275207;
// The same service as FlightSimulator's contextily Esri.WorldImagery (DroneLocalization's maps).
const TCHAR* const EsriWorldImageryUrl = TEXT("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}");
const TCHAR* const EsriSourceId = TEXT("esri");
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

constexpr double MaxFlyThereDistanceM = 20000.0;
constexpr double FpvGroundClearanceM = 0.15; // the frame sits this high on its legs

// Distance along the sphere (haversine), valid at any distance.
double GreatCircleM(double Lat1, double Lon1, double Lat2, double Lon2)
{
    const double P1 = FMath::DegreesToRadians(Lat1);
    const double P2 = FMath::DegreesToRadians(Lat2);
    const double DP = P2 - P1;
    const double DL = FMath::DegreesToRadians(Lon2 - Lon1);
    const double A = FMath::Square(FMath::Sin(DP * 0.5)) + FMath::Cos(P1) * FMath::Cos(P2) * FMath::Square(FMath::Sin(DL * 0.5));
    return 2.0 * EarthRadiusM * FMath::Asin(FMath::Min(1.0, FMath::Sqrt(A)));
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
    // nose, so the image top points in the flight heading. FPV: fixed, tilted up, wide.
    const bool bFpvCamera = FlightMode == EDroneFlightMode::Fpv;
    const float FovDeg = bFpvCamera ? FpvCameraFovDeg : CameraHorizontalFovDeg;
    Camera->SetRelativeRotation(FRotator(bFpvCamera ? FpvCameraUptiltDeg : CameraPitchDeg, bFpvCamera ? 0.0 : CameraYawDeg, 0.0));
    Camera->SetFieldOfView(FovDeg);
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
        VideoCapture->FOVAngle = FovDeg;
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

    FlightStartCameraPitchDeg = CameraPitchDeg;
    FlightStartCameraYawDeg = CameraYawDeg;

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
    if (ACesiumCameraManager* CameraManager = ACesiumCameraManager::GetDefaultCameraManager(this))
    {
        BaseAdditionalCameras = CameraManager->AdditionalCameras;
    }
    ApplyLiveSettings(Saved);
    ApplyStartPoint(Saved);
    if (bPerfMonitor)
    {
        PerfMonitor.Start(GetWorld(), PerfHitchMs);
    }
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
    TileStore.Stop();
    PerfMonitor.Stop();
    Super::EndPlay(EndPlayReason);
}

void ADroneRouteFlight::ResetFlightState()
{
    // Manual / FPV flight starts at the start point, a route at its first waypoint.
    if (FlightMode != EDroneFlightMode::Route || Waypoints.Num() == 0)
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
    bRecordingClip = false;
    if (VideoMode == EDroneVideoMode::Record && FlightMode != EDroneFlightMode::Route)
    {
        // Manual / FPV: nothing is written until R.
        StatusNote = TEXT("Recording is off: press R to start / stop a clip.");
        return;
    }
    if (bWriteGroundTruthCsv)
    {
        Telemetry.OpenCsv(SessionDir / TEXT("telemetry.csv"));
    }
    StartVideo();
    bRecordingClip = VideoMode == EDroneVideoMode::Record;
}

void ADroneRouteFlight::StopSession()
{
    StopVideo();
    Telemetry.Close();
    bSessionActive = false;
    bRecordingClip = false;
}

void ADroneRouteFlight::StartRecordingClip()
{
    if (VideoMode != EDroneVideoMode::Record || !bSessionActive || bRecordingClip)
    {
        return;
    }
    // Each clip gets its own folder, so the exporter sees a normal flight.
    SessionDir = FPaths::ConvertRelativePathToFull(FPaths::ProjectSavedDir() / TEXT("Flights") /
                                                   FDateTime::Now().ToString(TEXT("%Y%m%d-%H%M%S")));
    StatusNote.Reset();
    if (bWriteGroundTruthCsv)
    {
        Telemetry.OpenCsv(SessionDir / TEXT("telemetry.csv"));
    }
    StartVideo();
    bRecordingClip = true;
    UE_LOG(LogDroneRoute, Display, TEXT("Recording started: %s"), *SessionDir);
}

void ADroneRouteFlight::StopRecordingClip()
{
    if (!bRecordingClip)
    {
        return;
    }
    StopVideo();
    Telemetry.CloseCsv();
    bRecordingClip = false;
    StatusNote = FString::Printf(TEXT("Recording saved (%lld frames): %s   R: new clip"), VideoFrameIndex, *SessionDir);
    UE_LOG(LogDroneRoute, Display, TEXT("Recording stopped: %s (%lld frames)"), *SessionDir, VideoFrameIndex);
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
    if (VideoMode == EDroneVideoMode::Record)
    {
        Telemetry.OpenGroundCsv(SessionDir / TEXT("ground_points.csv"));
    }
    VideoFrameIndex = 0;
    SinceVideoFrameS = 0.0;
    VideoStartSimTimeS = SimTimeS;

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
    Telemetry.CloseGroundCsv();
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
            bFixedClock ? static_cast<double>(VideoFrameIndex) / FMath::Max(VideoFps, 1) : Sample.TimeS - VideoStartSimTimeS;
        Telemetry.WriteFrameRow(Sample, VideoFrameIndex, VideoTimeS);
        if (VideoMode == EDroneVideoMode::Record)
        {
            // Same tick as CaptureScene(): the capture transform is the one rendered.
            TraceImageGround(VideoFrameIndex);
        }
        ++VideoFrameIndex;
    }
}

void ADroneRouteFlight::TraceImageGround(int64 VideoFrame)
{
    UWorld* World = GetWorld();
    if (World == nullptr || Georeference == nullptr || VideoCapture == nullptr)
    {
        return;
    }
    const FTransform Xf = VideoCapture->GetComponentTransform();
    const FVector Origin = Xf.GetLocation();
    const FVector Fwd = Xf.GetUnitAxis(EAxis::X);
    const FVector Right = Xf.GetUnitAxis(EAxis::Y);
    const FVector Up = Xf.GetUnitAxis(EAxis::Z);
    const double W = static_cast<double>(VideoWidth);
    const double H = static_cast<double>(VideoHeight);
    // FOVAngle is horizontal; pixels are square.
    const double Fpx = 0.5 * W / FMath::Tan(FMath::DegreesToRadians(0.5 * static_cast<double>(VideoCapture->FOVAngle)));
    const double MaxRangeUu = 100000.0 * 100.0; // 100 km

    // Terrain only, as in TraceGroundEllipsoidHeight: imagery is draped on it.
    FCollisionQueryParams Params(SCENE_QUERY_STAT(DroneImageGroundTrace), true, this);
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
    const FTransform GeoToWorld = Georeference->GetActorTransform();

    // Point 0 is the image centre; 1-4 are FlightSimulator's 20/80 % corners
    // (same order); 5-8 are the edge midpoints.
    static const double Fx[9] = {0.5, 0.2, 0.8, 0.8, 0.2, 0.5, 0.8, 0.5, 0.2};
    static const double Fy[9] = {0.5, 0.2, 0.2, 0.8, 0.8, 0.2, 0.5, 0.8, 0.5};
    for (int32 P = 0; P < 9; ++P)
    {
        const double U = Fx[P] * W;
        const double V = Fy[P] * H;
        const FVector Dir =
            (Fwd + Right * ((U - 0.5 * W) / Fpx) - Up * ((V - 0.5 * H) / Fpx)).GetSafeNormal();
        FHitResult Hit;
        if (World->LineTraceSingleByChannel(Hit, Origin, Origin + Dir * MaxRangeUu, ECC_Visibility, Params))
        {
            const FVector Llh = Georeference->TransformUnrealPositionToLongitudeLatitudeHeight(
                GeoToWorld.InverseTransformPosition(Hit.ImpactPoint));
            Telemetry.WriteGroundRow(VideoFrame, P, U, V, true, Llh.Y, Llh.X, Llh.Z, Hit.Distance / 100.0);
        }
        else
        {
            Telemetry.WriteGroundRow(VideoFrame, P, U, V, false, 0.0, 0.0, 0.0, 0.0);
        }
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
    S.CameraTiltRateDegPerSec = CameraTiltRateDegPerSec;
    S.FpvThrustToWeight = FpvThrustToWeight;
    S.FpvRollPitchCenterRateDps = FpvRollPitchCenterRateDps;
    S.FpvRollPitchMaxRateDps = FpvRollPitchMaxRateDps;
    S.FpvRollPitchExpo = FpvRollPitchExpo;
    S.FpvYawCenterRateDps = FpvYawCenterRateDps;
    S.FpvYawMaxRateDps = FpvYawMaxRateDps;
    S.FpvYawExpo = FpvYawExpo;
    S.FpvCameraUptiltDeg = FpvCameraUptiltDeg;
    S.FpvCameraFovDeg = FpvCameraFovDeg;
    S.FpvKeyboardStick = FpvKeyboardStick;
    S.FpvThrottleKeyRate = FpvThrottleKeyRate;
    S.FpvCrashSpeedMps = FpvCrashSpeedMps;
    S.FpvUseBuild = FpvUseBuild;
    S.FpvFrame = FpvFrame;
    S.FpvMotor = FpvMotor;
    S.FpvProp = FpvProp;
    S.FpvBattery = FpvBattery;
    S.FpvCameraSystem = FpvCameraSystem;
    S.FpvPidMaster = FpvPidMaster;
    S.FpvPidDamping = FpvPidDamping;
    S.FpvThrottleMid = FpvThrottleMid;
    S.FpvThrottleExpo = FpvThrottleExpo;
    S.FpvThrottleLimitPct = FpvThrottleLimitPct;
    S.FpvDroneName = FpvDroneName;
    S.FpvRatesName = FpvRatesName;
    S.CamWidthPx = VideoWidth;
    S.CamHeightPx = VideoHeight;
    S.CamFps = VideoFps;
    S.CamSurveyHFovDeg = CameraHorizontalFovDeg;
    S.MapQuality = MapQuality;
    S.WorldMode = WorldMode;
    S.bForbidHoles = bForbidHoles;
    S.bPreloadArea = bPreloadArea;
    S.PreloadMarginM = PreloadMarginM;
    S.TileMemoryCacheMB = TileMemoryCacheMB;
    S.MaxSimultaneousTileLoads = MaxSimultaneousTileLoads;
    S.AroundDroneDetail = AroundDroneDetail;
    S.TileLoadingStyle = TileLoadingStyle;
    S.ImagerySource = ImagerySource;
    S.EsriMaxLevel = EsriMaxLevel;
    S.TileStoreDir = TileStoreDir;
    S.TileStorePort = TileStorePort;
    S.WaybackRelease = WaybackRelease;
    S.WaybackLabel = WaybackLabel;
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
    CameraTiltRateDegPerSec = FMath::Clamp(S.CameraTiltRateDegPerSec, 1.0, 180.0);
    FpvThrustToWeight = FMath::Clamp(S.FpvThrustToWeight, 1.5, 15.0);
    FpvRollPitchCenterRateDps = FMath::Clamp(S.FpvRollPitchCenterRateDps, 10.0, 2000.0);
    FpvRollPitchMaxRateDps = FMath::Clamp(S.FpvRollPitchMaxRateDps, 10.0, 2000.0);
    FpvRollPitchExpo = FMath::Clamp(S.FpvRollPitchExpo, 0.0, 1.0);
    FpvYawCenterRateDps = FMath::Clamp(S.FpvYawCenterRateDps, 10.0, 2000.0);
    FpvYawMaxRateDps = FMath::Clamp(S.FpvYawMaxRateDps, 10.0, 2000.0);
    FpvYawExpo = FMath::Clamp(S.FpvYawExpo, 0.0, 1.0);
    FpvCameraUptiltDeg = FMath::Clamp(S.FpvCameraUptiltDeg, -30.0, 80.0);
    FpvCameraFovDeg = static_cast<float>(FMath::Clamp(S.FpvCameraFovDeg, 40.0, 170.0));
    FpvKeyboardStick = FMath::Clamp(S.FpvKeyboardStick, 0.05, 1.0);
    FpvThrottleKeyRate = FMath::Clamp(S.FpvThrottleKeyRate, 0.05, 5.0);
    FpvCrashSpeedMps = FMath::Max(S.FpvCrashSpeedMps, 0.5);
    FpvUseBuild = FMath::Clamp(S.FpvUseBuild, 0, 1);
    FpvFrame = FMath::Clamp(S.FpvFrame, 0, FpvBuild::CountOf(FpvBuild::kFrames) - 1);
    FpvMotor = FMath::Clamp(S.FpvMotor, 0, FpvBuild::CountOf(FpvBuild::kMotors) - 1);
    FpvProp = FMath::Clamp(S.FpvProp, 0, FpvBuild::CountOf(FpvBuild::kProps) - 1);
    FpvBattery = FMath::Clamp(S.FpvBattery, 0, FpvBuild::CountOf(FpvBuild::kBatteries) - 1);
    FpvCameraSystem = FMath::Clamp(S.FpvCameraSystem, 0, FpvBuild::CountOf(FpvBuild::kFpvSystems) - 1);
    FpvPidMaster = FMath::Clamp(S.FpvPidMaster, 0.2, 3.0);
    FpvPidDamping = FMath::Clamp(S.FpvPidDamping, 0.2, 3.0);
    FpvPhysical = FpvBuild::Derive(FpvSelection()); // parts changed in the menu take effect at once
    FpvThrottleMid = FMath::Clamp(S.FpvThrottleMid, 0.05, 0.95);
    FpvThrottleExpo = FMath::Clamp(S.FpvThrottleExpo, 0.0, 1.0);
    FpvThrottleLimitPct = FMath::Clamp(S.FpvThrottleLimitPct, 10.0, 100.0);
    FpvDroneName = S.FpvDroneName;
    FpvRatesName = S.FpvRatesName;
    MapQuality = FMath::Clamp(S.MapQuality, 0, 2);
    WorldMode = FMath::Clamp(S.WorldMode, 0, 1);
    ApplyWorldMode();
    bForbidHoles = S.bForbidHoles;
    bPreloadArea = S.bPreloadArea;
    PreloadMarginM = FMath::Clamp(S.PreloadMarginM, 0.0, 20000.0);
    TileMemoryCacheMB = FMath::Clamp(S.TileMemoryCacheMB, 64, 65536);
    MaxSimultaneousTileLoads = FMath::Clamp(S.MaxSimultaneousTileLoads, 1, 512);
    AroundDroneDetail = FMath::Clamp(S.AroundDroneDetail, 0, 3);
    TileLoadingStyle = FMath::Clamp(S.TileLoadingStyle, 0, 1);
    ApplyTilesetSettings();
    ImagerySource = FMath::Clamp(S.ImagerySource, 0, 2);
    WaybackRelease = FMath::Max(S.WaybackRelease, 0);
    WaybackLabel = S.WaybackLabel;
    EsriMaxLevel = FMath::Clamp(S.EsriMaxLevel, 10, 23);
    TileStoreDir = S.TileStoreDir.TrimStartAndEnd();
    TileStorePort = FMath::Clamp(S.TileStorePort, 1024, 65535);
    ApplyImagerySource();

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
        // The video camera (the drone builder's camera section): what the localizer gets.
        VideoWidth = FMath::Clamp(S.CamWidthPx, 64, 3840);
        VideoHeight = FMath::Clamp(S.CamHeightPx, 64, 2160);
        VideoFps = FMath::Clamp(S.CamFps, 1, 120);
        CameraHorizontalFovDeg = static_cast<float>(FMath::Clamp(S.CamSurveyHFovDeg, 5.0, 170.0));
        CameraAspectRatio = static_cast<float>(VideoWidth) / static_cast<float>(VideoHeight);
        ApplyCameraSettings();
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
    bFpvCrashed = false;
    bFpvPlaceOnGround = false;
    ApplyCameraSettings(); // leaves the FPV camera if it was on

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
           S.StartLongitude, S.AltitudeM, S.StartHeadingDeg, *S.MapName);
    UpdateMarkers(/*bForce*/ true);
}

bool ADroneRouteFlight::StartPointDiffers(const FDroneSimSettings& S) const
{
    return FMath::Abs(S.StartLatitude - LastSettings.StartLatitude) > 1e-9 ||
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
    CameraPitchDeg = FlightStartCameraPitchDeg;
    CameraYawDeg = FlightStartCameraYawDeg;
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
    case EDroneMenuAction::FpvFlight:
        if (!bFlying)
        {
            // From the main menu: everything applies, the flight starts at the start point.
            ApplyLiveSettings(S);
            if (StartPointDiffers(S) || bMissionLoaded)
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
        else if (Action == EDroneMenuAction::FpvFlight)
        {
            EnterFpv(/*bOnGround*/ true);
            CloseMenu();
        }
        break;
    case EDroneMenuAction::PreviewMap:
        // Another map / start point: the drone goes there now, also during a
        // flight (a mission stops; the flight's files and stream go on).
        if (StartPointDiffers(S))
        {
            ApplyStartPoint(S);
            if (bFlying)
            {
                StatusNote = FString::Printf(TEXT("Moved to the start point of map \"%s\"."), *S.MapName);
            }
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
    LeaveFpv(); // a mission flies with the survey camera
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
    // "Fly there" only for areas nearby: the route model is flat (fine for a
    // few tens of km) and a long transit is no use in a simulator.
    const double AreaDistanceM =
        GreatCircleM(Latitude, Longitude, MissionPlan.Waypoints[0].Lat, MissionPlan.Waypoints[0].Lon);
    const bool bTooFar = AreaDistanceM > MaxFlyThereDistanceM;
    const bool bJump = Merged.bMissionJumpToStart || bTooFar;
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
    if (bTooFar && !Merged.bMissionJumpToStart)
    {
        StatusNote = FString::Printf(TEXT("Mission area %.0f km away: jumped to the first pass instead of flying there."),
                                     AreaDistanceM / 1000.0);
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
            .WaybackVersions([WeakThis]() -> const TArray<FDroneWaybackVersion>&
            {
                static const TArray<FDroneWaybackVersion> NoVersions;
                return WeakThis.IsValid() ? WeakThis->Wayback.GetVersions() : NoVersions;
            })
            .WaybackStatus([WeakThis]() { return WeakThis.IsValid() ? WeakThis->Wayback.GetStatus() : FString(); })
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
    if (Viewport == nullptr)
    {
        return;
    }
    TWeakObjectPtr<ADroneRouteFlight> WeakThis(this);
    if (bShowCompass && !Compass.IsValid())
    {
        Compass = MakeDroneCompass([WeakThis]()
        {
            return WeakThis.IsValid() ? WeakThis->CompassState() : FDroneCompassState();
        });
        Viewport->AddViewportWidgetContent(Compass.ToSharedRef(), 11);
    }
    if (bShowDebugText && !StatusLine.IsValid())
    {
        StatusLine = MakeDroneStatusLine(
            [WeakThis]()
            {
                return WeakThis.IsValid() ? FText::FromString(WeakThis->StatusText()) : FText::GetEmpty();
            },
            bShowCompass ? DroneCompassTopSpace : 8.f);
        Viewport->AddViewportWidgetContent(StatusLine.ToSharedRef(), 10);
    }
}

void ADroneRouteFlight::RemoveStatusLine()
{
    UWorld* World = GetWorld();
    UGameViewportClient* Viewport = World != nullptr ? World->GetGameViewport() : nullptr;
    if (StatusLine.IsValid())
    {
        if (Viewport != nullptr)
        {
            Viewport->RemoveViewportWidgetContent(StatusLine.ToSharedRef());
        }
        StatusLine.Reset();
    }
    if (Compass.IsValid())
    {
        if (Viewport != nullptr)
        {
            Viewport->RemoveViewportWidgetContent(Compass.ToSharedRef());
        }
        Compass.Reset();
    }
}

FDroneCompassState ADroneRouteFlight::CompassState() const
{
    FDroneCompassState C;
    C.NoseDeg = HeadingDeg;
    const bool bFpvCamera = FlightMode == EDroneFlightMode::Fpv;
    C.ViewDeg = Wrap360(HeadingDeg + (bFpvCamera ? 0.0 : CameraYawDeg));
    C.CameraTiltDeg = bFpvCamera ? FpvCameraUptiltDeg : CameraPitchDeg;
    // PrevVel* hold the velocity of the last step (set in UpdateTilt).
    C.SpeedMps = FMath::Sqrt(PrevVelNorth * PrevVelNorth + PrevVelEast * PrevVelEast);
    C.bMoving = C.SpeedMps > 0.5;
    if (C.bMoving)
    {
        C.MoveDeg = CompassBearingDeg(PrevVelNorth, PrevVelEast);
    }
    if (FlightMode == EDroneFlightMode::Route && !bRouteFinished && Waypoints.Num() >= 2)
    {
        const FDroneWaypoint& To = Waypoints[NextWaypointIndex(LegIndex)];
        double North = 0.0, East = 0.0;
        LocalOffsetM(Latitude, Longitude, To.Latitude, To.Longitude, North, East);
        C.bTarget = true;
        C.TargetDeg = CompassBearingDeg(North, East);
        C.TargetDistanceM = FMath::Sqrt(North * North + East * East);
    }
    return C;
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
        const bool bRoute = FlightMode == EDroneFlightMode::Route;
        const bool bFpv = FlightMode == EDroneFlightMode::Fpv;
        const TCHAR* ModeName = bFpv ? TEXT("FPV") : (!bRoute ? TEXT("MANUAL") : (bMissionLoaded ? TEXT("MISSION") : TEXT("ROUTE")));
        Status = FString::Printf(TEXT("%s  lat %.6f  lon %.6f  AGL %.1f m  heading %.1f"), ModeName,
                               Latitude, Longitude, bHaveGround ? EllipsoidHeightM - GroundEllipsoidHeightM : -1.0,
                               HeadingDeg);
        if (bMapLocked)
        {
            Status += TEXT("  MAP LOCKED (L)");
        }
        if (bFpv)
        {
            Status += FString::Printf(TEXT("  throttle %.0f%%%s  speed %.1f m/s  pitch %.0f  roll %.0f%s"), FpvThrottle * 100.0,
                                    bFpvGamepadThrottle ? TEXT(" (gamepad)") : TEXT(""), FpvModel::Length(FpvState.Vel),
                                    PitchDeg, RollDeg, bFpvCrashed ? TEXT("  CRASHED") : TEXT(""));
            if (FpvUseBuild != 0)
            {
                const double CellV = FpvPhys.PackVolts / FMath::Max(FpvPhysical.Cells, 1);
                Status += FString::Printf(TEXT("  battery %.1f V (%.2f V/cell) %.0f A  %.0f mAh used%s"), FpvPhys.PackVolts,
                                        CellV, FpvPhys.PackCurrentA, FpvPhys.UsedAh * 1000.0,
                                        CellV < 3.3 ? TEXT("  LOW BATTERY") : TEXT(""));
            }
        }
        else if (!bCameraGimbal)
        {
            Status += FString::Printf(TEXT("  pitch %.1f  roll %.1f"), PitchDeg, RollDeg);
        }
        if (!bFpv && (CameraPitchDeg > -89.5 || FMath::Abs(CameraYawDeg) > 0.5))
        {
            Status += FString::Printf(TEXT("  camera tilt %.0f pan %.0f"), CameraPitchDeg, CameraYawDeg);
        }
        if (bRoute && bMissionLoaded)
        {
            Status += MissionStatus();
        }
        else if (bRoute)
        {
            Status += FString::Printf(TEXT("  leg %d/%d%s"), LegIndex + 1, Waypoints.Num(),
                                    bRouteFinished ? TEXT(" (finished)") : TEXT(""));
        }
        else if (bMissionLoaded)
        {
            Status += bFpv ? TEXT("  (mission paused: F, then M resumes)") : TEXT("  (mission paused, M resumes)");
        }
        if (VideoMode == EDroneVideoMode::Record)
        {
            Status += Video.IsRunning() ? FString::Printf(TEXT("  REC %lld  (R: stop)"), VideoFrameIndex)
                                        : (bRecordingClip ? FString(TEXT("  REC failed")) : FString(TEXT("  REC off (R: start)")));
        }
        else if (VideoMode == EDroneVideoMode::Live)
        {
            Status += Video.IsRunning()
                ? FString::Printf(TEXT("  LIVE %lld (dropped %lld)"), VideoFrameIndex, Video.GetDroppedFrames())
                : FString(TEXT("  LIVE failed"));
        }
        const float LoadPercent = MapLoadProgress();
        if (LoadPercent >= 0.f && LoadPercent < 99.5f)
        {
            Status += FString::Printf(TEXT("  map loading %.0f%%"), LoadPercent);
        }
        if (bStoredImageryActive)
        {
            Status += TEXT("  imagery: Esri, Maxar, Earthstar Geographics");
        }
        Status += bFpv ? TEXT("   [Esc / F2 / Tab: menu,  L: lock map,  F: leave FPV,  Space / Ctrl: throttle,  W/S A/D: pitch / roll,  Left / Right: yaw,  R: pick up after a crash]")
                       : TEXT("   [Esc / F2 / Tab: menu,  L: lock map,  M: manual <-> route / mission,  F: FPV,  Left / Right: turn,  Up / Down, Q / E: camera (C: reset),  V: add area vertex,  R: record on / off]");
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
        Line = FlightMode == EDroneFlightMode::Fpv
            ? FString(TEXT("Flying: FPV (acro); the flight is paused while this menu is open"))
            : (bManual ? FString(TEXT("Flying: manual")) : (bMissionLoaded ? TEXT("Flying: mission") + MissionStatus()
                                                                           : FString(TEXT("Flying: route"))));
        Line += FString::Printf(TEXT(", %.0f m above the ground"), EllipsoidHeightM - GroundEllipsoidHeightM);
    }
    const float LoadPercent = MapLoadProgress();
    if (LoadPercent >= 0.f)
    {
        Line += FString::Printf(TEXT("\nMap: %s%s"),
                                LoadPercent < 99.5f ? *FString::Printf(TEXT("loading %.0f%%"), LoadPercent) : TEXT("loaded"),
                                PreloadViewCount > 0 ? *FString::Printf(TEXT(", area + margin %.1f km2 kept loaded (%d views)"),
                                                                        PreloadAreaKm2, PreloadViewCount)
                                                     : TEXT(""));
    }
    Line += TEXT("\n");
    Line += ImageryStatus();
    if (!StatusNote.IsEmpty())
    {
        Line += TEXT("\n");
        Line += StatusNote;
    }
    return Line;
}

FString ADroneRouteFlight::ImageryStatus() const
{
    if (!bStoredImageryActive)
    {
        return ImageryProblem.IsEmpty() ? FString(TEXT("Imagery: Bing Maps Aerial (Cesium ion), downloaded again every run"))
                                        : ImageryProblem;
    }
    const FDroneTileStore::FStats Stats = TileStore.GetStats();
    FString Line = FString::Printf(TEXT("Imagery: %s, kept on disk; this run %lld tiles from disk, %lld downloaded (%.1f MB)"),
                                   *StoredSourceName, Stats.FromDisk, Stats.Downloaded,
                                   static_cast<double>(Stats.DownloadedBytes) / (1024.0 * 1024.0));
    if (Stats.NoImagery > 0)
    {
        Line += FString::Printf(TEXT(", %lld with no Esri imagery at that level (coarser shown)"), Stats.NoImagery);
    }
    if (Stats.Failed > 0)
    {
        Line += FString::Printf(TEXT(", %lld failed"), Stats.Failed);
    }
    if (Stats.Pending > 0)
    {
        Line += FString::Printf(TEXT(", %d loading"), Stats.Pending);
    }
    Line += TEXT("\nImagery: Esri, Maxar, Earthstar Geographics, and the GIS User Community");
    return Line;
}

FString ADroneRouteFlight::ResolvedTileStoreDir() const
{
    if (!TileStoreDir.IsEmpty())
    {
        return FPaths::ConvertRelativePathToFull(TileStoreDir);
    }
    return FPaths::Combine(FPlatformProcess::UserSettingsDir(), TEXT("DroneSimUE"), TEXT("TileStore"));
}

void ADroneRouteFlight::ApplyImagerySource()
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    if (!bLevelOverlaysFound)
    {
        // The level's own imagery (Bing from Cesium ion), found once before ours exist.
        bLevelOverlaysFound = true;
        for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
        {
            TArray<UCesiumRasterOverlay*> Overlays;
            It->GetComponents<UCesiumRasterOverlay>(Overlays);
            for (UCesiumRasterOverlay* Overlay : Overlays)
            {
                if (Overlay != nullptr)
                {
                    LevelOverlays.Add(Overlay);
                }
            }
        }
    }

    ImageryProblem.Reset();
    bool bStored = ImagerySource == 1 || ImagerySource == 2;
    // Which stored imagery: current Esri, or one Wayback release (its own folder in the store).
    FString SourceId = EsriSourceId;
    FString Upstream = EsriWorldImageryUrl;
    FString SourceName = TEXT("Esri World Imagery");
    if (ImagerySource == 2 && WaybackRelease > 0)
    {
        SourceId = FString::Printf(TEXT("wb%d"), WaybackRelease);
        Upstream = FDroneWayback::TileUrlTemplate(WaybackRelease);
        SourceName = WaybackLabel.IsEmpty() ? FString::Printf(TEXT("Esri Wayback release %d"), WaybackRelease)
                                            : FString(TEXT("Esri Wayback, ")) + WaybackLabel;
    }
    else if (ImagerySource == 2)
    {
        SourceName = TEXT("Esri by date: the current Esri World Imagery");
    }
    if (ImagerySource == 2)
    {
        RequestWaybackVersions(LastSettings);
    }
    if (bStored && LevelOverlays.Num() == 0)
    {
        bStored = false;
        ImageryProblem = TEXT("Imagery: no raster overlay on the tilesets, nothing to replace");
    }
    if (bStored)
    {
        TileStore.SetSource(SourceId, Upstream);
        if (!TileStore.Start(TileStorePort, ResolvedTileStoreDir()))
        {
            bStored = false;
            ImageryProblem = FString::Printf(TEXT("Imagery: the tile store cannot use port %d (in use?), so Bing (online) is used"),
                                             TileStorePort);
        }
    }

    if (bStored)
    {
        const FString TemplateUrl = TileStore.OverlayTemplateUrl(SourceId);
        // One stored-imagery overlay per tileset that had imagery, in the same material layer.
        for (const TWeakObjectPtr<UCesiumRasterOverlay>& WeakLevelOverlay : LevelOverlays)
        {
            UCesiumRasterOverlay* LevelOverlay = WeakLevelOverlay.Get();
            ACesium3DTileset* Tileset = LevelOverlay != nullptr ? Cast<ACesium3DTileset>(LevelOverlay->GetOwner()) : nullptr;
            if (Tileset == nullptr)
            {
                continue;
            }
            const bool bHaveOne = StoredImageryOverlays.ContainsByPredicate(
                [Tileset](const TObjectPtr<UCesiumUrlTemplateRasterOverlay>& Existing)
                { return Existing != nullptr && Existing->GetOwner() == Tileset; });
            if (bHaveOne)
            {
                continue;
            }
            const FName OverlayName =
                MakeUniqueObjectName(Tileset, UCesiumUrlTemplateRasterOverlay::StaticClass(), TEXT("DroneStoredImagery"));
            UCesiumUrlTemplateRasterOverlay* Overlay = NewObject<UCesiumUrlTemplateRasterOverlay>(Tileset, OverlayName, RF_Transient);
            Overlay->bAutoActivate = false; // activated below, once the level's overlay is off
            Overlay->MaterialLayerKey = LevelOverlay->MaterialLayerKey;
            Overlay->Projection = ECesiumUrlTemplateRasterOverlayProjection::WebMercator;
            Overlay->TemplateUrl = TemplateUrl;
            Overlay->MinimumLevel = 0;
            Overlay->MaximumLevel = EsriMaxLevel;
            Overlay->RegisterComponent();
            StoredImageryOverlays.Add(Overlay);
        }
        for (const TWeakObjectPtr<UCesiumRasterOverlay>& WeakLevelOverlay : LevelOverlays)
        {
            if (UCesiumRasterOverlay* LevelOverlay = WeakLevelOverlay.Get())
            {
                if (LevelOverlay->IsActive())
                {
                    LevelOverlay->Deactivate();
                }
            }
        }
        for (UCesiumUrlTemplateRasterOverlay* Overlay : StoredImageryOverlays)
        {
            if (Overlay == nullptr)
            {
                continue;
            }
            const bool bChanged = Overlay->TemplateUrl != TemplateUrl || Overlay->MaximumLevel != EsriMaxLevel;
            Overlay->TemplateUrl = TemplateUrl;
            Overlay->MaximumLevel = EsriMaxLevel;
            if (!Overlay->IsActive())
            {
                Overlay->Activate(true);
            }
            else if (bChanged)
            {
                Overlay->Refresh();
            }
        }
    }
    else
    {
        for (UCesiumUrlTemplateRasterOverlay* Overlay : StoredImageryOverlays)
        {
            if (Overlay != nullptr && Overlay->IsActive())
            {
                Overlay->Deactivate();
            }
        }
        for (const TWeakObjectPtr<UCesiumRasterOverlay>& WeakLevelOverlay : LevelOverlays)
        {
            UCesiumRasterOverlay* LevelOverlay = WeakLevelOverlay.Get();
            if (LevelOverlay != nullptr && !LevelOverlay->IsActive())
            {
                LevelOverlay->Activate(true);
            }
        }
    }
    const FString ShownName = bStored ? SourceName : FString();
    if (bStored != bStoredImageryActive || ShownName != StoredSourceName || !ImageryProblem.IsEmpty())
    {
        UE_LOG(LogDroneRoute, Display, TEXT("Imagery: %s%s"),
               bStored ? *FString::Printf(TEXT("%s from the tile store (%s), up to level %d"), *SourceName,
                                          *TileStore.GetRootDir(), EsriMaxLevel)
                       : TEXT("the level's overlays (Bing, Cesium ion)"),
               ImageryProblem.IsEmpty() ? TEXT("") : *(FString(TEXT(" - ")) + ImageryProblem));
    }
    bStoredImageryActive = bStored;
    StoredSourceName = ShownName;
}

void ADroneRouteFlight::MapCentreOf(const FDroneSimSettings& S, double& OutLat, double& OutLon)
{
    OutLat = S.StartLatitude;
    OutLon = S.StartLongitude;
    std::vector<SurveyGrid::LatLon> Parsed;
    std::string ParseError;
    if (!SurveyGrid::ParseLatLonList(DroneMission::ToAscii(S.MissionPolygon), Parsed, ParseError) || Parsed.size() < 3)
    {
        return;
    }
    double LatMin = 1e9, LatMax = -1e9, LonMin = 1e9, LonMax = -1e9;
    for (const SurveyGrid::LatLon& P : Parsed)
    {
        LatMin = FMath::Min(LatMin, P.Lat);
        LatMax = FMath::Max(LatMax, P.Lat);
        LonMin = FMath::Min(LonMin, P.Lon);
        LonMax = FMath::Max(LonMax, P.Lon);
    }
    OutLat = 0.5 * (LatMin + LatMax);
    OutLon = 0.5 * (LonMin + LonMax);
}

void ADroneRouteFlight::RequestWaybackVersions(const FDroneSimSettings& Source)
{
    double CentreLat = 0.0;
    double CentreLon = 0.0;
    MapCentreOf(Source, CentreLat, CentreLon);
    Wayback.RequestVersions(CentreLat, CentreLon, ResolvedTileStoreDir());
}

void ADroneRouteFlight::ApplyWorldMode()
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    const bool b3D = WorldMode == 1;
    if (b3D && !Google3DTileset.IsValid())
    {
        for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
        {
            if (It->GetIonAssetID() == GooglePhotorealistic3DTilesAssetId)
            {
                Google3DTileset = *It;
                break;
            }
        }
        if (!Google3DTileset.IsValid())
        {
            FActorSpawnParameters Params;
            Params.Name = MakeUniqueObjectName(World->PersistentLevel, ACesium3DTileset::StaticClass(), TEXT("Google3DTiles"));
            Params.ObjectFlags |= RF_Transient;
            ACesium3DTileset* Tileset = World->SpawnActor<ACesium3DTileset>(Params);
            if (Tileset != nullptr)
            {
                Tileset->SetTilesetSource(ETilesetSource::FromCesiumIon);
                Tileset->SetIonAssetID(GooglePhotorealistic3DTilesAssetId);
#if WITH_EDITOR
                Tileset->SetActorLabel(TEXT("Google Photorealistic 3D Tiles"), false);
#endif
                Google3DTileset = Tileset;
                UE_LOG(LogDroneRoute, Display, TEXT("World: spawned Google Photorealistic 3D Tiles (ion asset %lld)"),
                       GooglePhotorealistic3DTilesAssetId);
            }
            else
            {
                UE_LOG(LogDroneRoute, Warning, TEXT("World: could not spawn the Google 3D Tiles tileset"));
            }
        }
    }
    // Hidden actors keep their collision, so terrain traces (telemetry heights) keep working.
    for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
    {
        const bool bIsGoogle = It->GetIonAssetID() == GooglePhotorealistic3DTilesAssetId;
        const bool bShow = bIsGoogle ? b3D : !b3D;
        It->SetActorHiddenInGame(!bShow);
#if WITH_EDITOR
        It->SetIsTemporarilyHiddenInEditor(!bShow);
#endif
    }
    UE_LOG(LogDroneRoute, Display, TEXT("World: %s"), b3D ? TEXT("3D (Google Photorealistic 3D Tiles)") : TEXT("terrain + imagery"));
}

void ADroneRouteFlight::ApplyTilesetSettings()
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return;
    }
    static const double ScreenSpaceErrors[] = {16.0, 8.0, 4.0};
    const double Sse = ScreenSpaceErrors[FMath::Clamp(MapQuality, 0, 2)];
    const int32 DescendantLimit = TileLoadingStyle == 1 ? 1000 : 20;
    EnsureHttpConnectionReuse();
    for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
    {
        // Cesium copies these into the tileset options every Tick.
        It->SetMaximumScreenSpaceError(Sse);
        It->ForbidHoles = bForbidHoles;
        It->MaximumSimultaneousTileLoads = bMapLocked ? 0 : MaxSimultaneousTileLoads;
        It->MaximumCachedBytes = static_cast<int64>(TileMemoryCacheMB) * 1024 * 1024;
        // Frustum culling stays on: with it off every tile up to the fog distance
        // (hundreds of km) gets refined; the 2026-10-21 test with that (and descendant
        // limit 1000) stayed at the coarsest tiles for 10 minutes. What turning would
        // show is kept instead by refining the culled tiles to a coarser error (they are
        // loaded, not drawn), in the same tile selection as the view: no extra cameras.
        It->EnableFrustumCulling = true;
        const int32 Around = FMath::Clamp(AroundDroneDetail, 0, 3);
        static const double CulledSseFactors[] = {1.0, 1.0, 2.0, 4.0};
        It->EnforceCulledScreenSpaceError = !bAroundViaCameras && Around > 0;
        It->CulledScreenSpaceError = Sse * CulledSseFactors[Around];
        It->LoadingDescendantLimit = DescendantLimit;
    }
    static const TCHAR* TurnedViewNames[] = {TEXT("off"), TEXT("same detail"), TEXT("half detail"), TEXT("quarter detail")};
    UE_LOG(LogDroneRoute, Display,
           TEXT("Tilesets: screen-space error %.0f, turned views %s, holes %s, descendant limit %d, %d loads at once, %d MB cache"),
           Sse, TurnedViewNames[FMath::Clamp(AroundDroneDetail, 0, 3)], bForbidHoles ? TEXT("forbidden") : TEXT("allowed"), DescendantLimit, MaxSimultaneousTileLoads, TileMemoryCacheMB);
}

// UE's libcurl keeps only 5 idle connections by default (CURLMOPT_MAXCONNECTS), so with
// dozens of tile loads at once almost every request opened a new connection. Against the
// local tile store that used up the ephemeral ports within minutes (2026-10-21 log: "connect
// to 127.0.0.1 port 18765 ... failed: Address already in use", ~7200 failed imagery loads),
// and against Cesium ion it means a TLS handshake per tile.
void ADroneRouteFlight::EnsureHttpConnectionReuse()
{
    const int32 Wanted = FMath::Max(128, MaxSimultaneousTileLoads + 64);
    IConsoleVariable* Idle = IConsoleManager::Get().FindConsoleVariable(TEXT("HTTP.Curl.MaxIdleConnections"));
    if (Idle != nullptr && Idle->GetInt() < Wanted)
    {
        UE_LOG(LogDroneRoute, Display, TEXT("HTTP: idle connections kept %d -> %d"), Idle->GetInt(), Wanted);
        Idle->Set(Wanted, ECVF_SetByCode);
    }
}

float ADroneRouteFlight::MapLoadProgress() const
{
    UWorld* World = GetWorld();
    if (World == nullptr)
    {
        return -1.f;
    }
    float Sum = 0.f;
    int32 Count = 0;
    for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
    {
        if (It->IsHidden())
        {
            continue;
        }
        if (It->GetIonAssetID() == (WorldMode == 1 ? GooglePhotorealistic3DTilesAssetId : CesiumWorldTerrainAssetId))
        {
            return It->GetLoadProgress();
        }
        Sum += It->GetLoadProgress();
        ++Count;
    }
    return Count > 0 ? Sum / static_cast<float>(Count) : -1.f;
}

void ADroneRouteFlight::UpdatePreload(const FDroneSimSettings& Source, bool bForce)
{
    ACesiumCameraManager* CameraManager = ACesiumCameraManager::GetDefaultCameraManager(this);
    UWorld* World = GetWorld();
    if (CameraManager == nullptr || World == nullptr || Georeference == nullptr)
    {
        return;
    }
    const FString Key = FString::Printf(TEXT("%d|%s|%.6f|%.6f|%.1f|%.1f|%.0f"), bPreloadArea ? 1 : 0, *Source.MissionPolygon,
                                        Source.StartLatitude, Source.StartLongitude, Source.MissionAltitudeM, Source.AltitudeM,
                                        PreloadMarginM);
    if (!bForce && Key == PreloadKey)
    {
        return;
    }
    PreloadKey = Key;
    PreloadCameras.Reset();
    PreloadViewCount = 0;
    PreloadAreaKm2 = 0.0;
    if (!bPreloadArea)
    {
        PushCesiumCameras();
        return;
    }

    // The area: the survey polygon, or 1 km around the start point without one.
    std::vector<SurveyGrid::LatLon> Parsed;
    std::string ParseError;
    const bool bHaveArea = SurveyGrid::ParseLatLonList(DroneMission::ToAscii(Source.MissionPolygon), Parsed, ParseError) &&
                           Parsed.size() >= 3;
    double LatMin = Source.StartLatitude, LatMax = Source.StartLatitude;
    double LonMin = Source.StartLongitude, LonMax = Source.StartLongitude;
    double ExtraM = 1000.0;
    if (bHaveArea)
    {
        LatMin = LonMin = 1e9;
        LatMax = LonMax = -1e9;
        for (const SurveyGrid::LatLon& P : Parsed)
        {
            LatMin = FMath::Min(LatMin, P.Lat);
            LatMax = FMath::Max(LatMax, P.Lat);
            LonMin = FMath::Min(LonMin, P.Lon);
            LonMax = FMath::Max(LonMax, P.Lon);
        }
        ExtraM = 0.0;
    }
    const double MarginM = PreloadMarginM + ExtraM;
    const double LatC = 0.5 * (LatMin + LatMax);
    const double LonC = 0.5 * (LonMin + LonMax);
    const double MetresPerDegLat = FMath::DegreesToRadians(1.0) * EarthRadiusM;
    const double MetresPerDegLon = MetresPerDegLat * FMath::Cos(FMath::DegreesToRadians(LatC));
    const double HalfNorthM = 0.5 * (LatMax - LatMin) * MetresPerDegLat + MarginM;
    const double HalfEastM = 0.5 * (LonMax - LonMin) * MetresPerDegLon + MarginM;

    // Views straight down from the flight height, with the same pixels per
    // radian as the player's view of the survey camera, so they ask for the
    // detail the flight will need. 90 deg views from height h each cover 2h x 2h.
    const double HeightAglM = FMath::Max(bHaveArea ? Source.MissionAltitudeM : Source.AltitudeM, 20.0);
    FVector2D ViewportPx(1920.0, 1080.0);
    if (UGameViewportClient* GameViewport = World->GetGameViewport())
    {
        GameViewport->GetViewportSize(ViewportPx);
    }
    const double PixelsPerTan = FMath::Max(ViewportPx.X, 320.0) / (2.0 * FMath::Tan(FMath::DegreesToRadians(CameraHorizontalFovDeg) * 0.5));
    const double ViewPx = 2.0 * PixelsPerTan; // square viewport, tan(45 deg) = 1
    constexpr int32 MaxViews = 256;
    double SpacingM = 2.0 * HeightAglM;
    int32 CountEast = FMath::Max(1, FMath::CeilToInt(2.0 * HalfEastM / SpacingM));
    int32 CountNorth = FMath::Max(1, FMath::CeilToInt(2.0 * HalfNorthM / SpacingM));
    if (CountEast * CountNorth > MaxViews)
    {
        // Too many views: spread them out (the detail between them is then a bit lower).
        const double Grow = FMath::Sqrt(static_cast<double>(CountEast * CountNorth) / MaxViews);
        SpacingM *= Grow;
        CountEast = FMath::Max(1, FMath::CeilToInt(2.0 * HalfEastM / SpacingM));
        CountNorth = FMath::Max(1, FMath::CeilToInt(2.0 * HalfNorthM / SpacingM));
    }
    const double GroundM = bHaveGround ? GroundEllipsoidHeightM : InitialGroundEllipsoidHeightM;
    const FTransform GeoToWorld = Georeference->GetActorTransform();
    for (int32 IndexNorth = 0; IndexNorth < CountNorth; ++IndexNorth)
    {
        for (int32 IndexEast = 0; IndexEast < CountEast; ++IndexEast)
        {
            const double NorthM = (IndexNorth - 0.5 * (CountNorth - 1)) * SpacingM;
            const double EastM = (IndexEast - 0.5 * (CountEast - 1)) * SpacingM;
            const double Lat = LatC + NorthM / MetresPerDegLat;
            const double Lon = LonC + EastM / MetresPerDegLon;
            const FVector Eye = GeoToWorld.TransformPosition(
                Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(FVector(Lon, Lat, GroundM + HeightAglM)));
            const FVector Below = GeoToWorld.TransformPosition(
                Georeference->TransformLongitudeLatitudeHeightPositionToUnreal(FVector(Lon, Lat, GroundM)));
            PreloadCameras.Add(FCesiumCamera(FVector2D(ViewPx, ViewPx), Eye, (Below - Eye).Rotation(), 90.0));
        }
    }
    PushCesiumCameras();
    PreloadViewCount = CountEast * CountNorth;
    PreloadAreaKm2 = 4.0 * HalfNorthM * HalfEastM / 1.0e6;
    UE_LOG(LogDroneRoute, Display, TEXT("Preload: %d views %.0f m apart at %.0f m over %.1f km2 (area + %.0f m)"),
           PreloadViewCount, SpacingM, HeightAglM, PreloadAreaKm2, MarginM);
}

void ADroneRouteFlight::UpdateAroundCameras()
{
    AroundCameras.Reset();
    const int32 Around = FMath::Clamp(AroundDroneDetail, 0, 3);
    UWorld* World = GetWorld();
    if (!bAroundViaCameras || Around == 0 || Camera == nullptr || World == nullptr)
    {
        return;
    }
    FVector2D ViewportPx(1920.0, 1080.0);
    if (UGameViewportClient* GameViewport = World->GetGameViewport())
    {
        GameViewport->GetViewportSize(ViewportPx);
    }
    static const double DetailScales[] = {1.0, 1.0, 0.5, 0.25};
    const double WidthPx = FMath::Max(ViewportPx.X, 320.0) * DetailScales[Around];
    const double HeightPx = WidthPx / FMath::Max(static_cast<double>(CameraAspectRatio), 0.1);
    const double FovDeg = FMath::Clamp(static_cast<double>(Camera->FieldOfView), 5.0, 170.0);
    // Headings a bit closer than the FOV so neighbouring views overlap; the
    // camera itself (heading 0) is already one of Cesium's views.
    const int32 Count = FMath::Clamp(FMath::CeilToInt(360.0 / (0.9 * FovDeg)), 2, 16);
    const FVector Location = Camera->GetComponentLocation();
    const FQuat View = Camera->GetComponentQuat();
    for (int32 Index = 1; Index < Count; ++Index)
    {
        const FQuat Turn(FVector::UpVector, FMath::DegreesToRadians(360.0 * Index / Count));
        AroundCameras.Add(FCesiumCamera(FVector2D(WidthPx, HeightPx), Location, (Turn * View).Rotator(), FovDeg));
    }
}

void ADroneRouteFlight::SetMapLocked(bool bLocked)
{
    bMapLocked = bLocked;
    if (UWorld* World = GetWorld())
    {
        // Not SuspendUpdate: that also freezes what is drawn (only the view at the moment
        // of locking). Here Cesium still picks what to draw from the loaded tiles every
        // frame, but loads nothing new; the cache keeps the loaded ones.
        for (TActorIterator<ACesium3DTileset> It(World); It; ++It)
        {
            It->MaximumSimultaneousTileLoads = bLocked ? 0 : MaxSimultaneousTileLoads;
        }
    }
    UE_LOG(LogDroneRoute, Display, TEXT("Map %s"), bLocked ? TEXT("locked: no new tile loads") : TEXT("unlocked"));
}

void ADroneRouteFlight::PushCesiumCameras()
{
    ACesiumCameraManager* CameraManager = ACesiumCameraManager::GetDefaultCameraManager(this);
    if (CameraManager == nullptr)
    {
        return;
    }
    TArray<FCesiumCamera>& Cameras = CameraManager->AdditionalCameras;
    Cameras = BaseAdditionalCameras;
    Cameras.Append(PreloadCameras);
    Cameras.Append(AroundCameras);
}

void ADroneRouteFlight::UpdateMarkers(bool bForce)
{
    // While the menu is open the markers (and the preload) follow its draft, so
    // the area shows up while it is edited.
    const FDroneSimSettings Source = SettingsPanel.IsValid() ? SettingsPanel->GetDraft() : LastSettings;
    UpdatePreload(Source, bForce);
    if (Source.ImagerySource == 2)
    {
        RequestWaybackVersions(Source); // the date list follows the map being edited
    }
    if (AreaMarkers == nullptr)
    {
        return;
    }
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

FpvModel::Params ADroneRouteFlight::FpvParams() const
{
    FpvModel::Params P;
    P.ThrustToWeight = FpvThrustToWeight;
    P.MotorTimeConstantS = FpvMotorTimeConstantS;
    P.RateTimeConstantS = FpvRateTimeConstantS;
    P.LinearDrag = FpvLinearDrag;
    P.QuadraticDrag = FpvQuadraticDrag;
    P.Roll.CenterDps = P.Pitch.CenterDps = FpvRollPitchCenterRateDps;
    P.Roll.MaxDps = P.Pitch.MaxDps = FpvRollPitchMaxRateDps;
    P.Roll.Expo = P.Pitch.Expo = FpvRollPitchExpo;
    P.Yaw.CenterDps = FpvYawCenterRateDps;
    P.Yaw.MaxDps = FpvYawMaxRateDps;
    P.Yaw.Expo = FpvYawExpo;
    return P;
}

FpvBuild::Selection ADroneRouteFlight::FpvSelection() const
{
    FpvBuild::Selection Sel;
    Sel.FrameIndex = FpvFrame;
    Sel.MotorIndex = FpvMotor;
    Sel.PropIndex = FpvProp;
    Sel.BatteryIndex = FpvBattery;
    Sel.FpvIndex = FpvCameraSystem;
    return Sel;
}

FpvBuild::Pids ADroneRouteFlight::FpvPids() const
{
    FpvBuild::Pids Pid;
    Pid.Master = FpvPidMaster;
    Pid.DGain = FpvPidDamping;
    return Pid;
}

void ADroneRouteFlight::EnterFpv(bool bOnGround)
{
    FpvState = FpvModel::State();
    FpvState.Att = FpvModel::FromYawPitchRoll(FMath::DegreesToRadians(HeadingDeg), 0.0, 0.0);
    // A fresh, fully charged pack every time FPV starts.
    FpvPhysical = FpvBuild::Derive(FpvSelection());
    FpvPhys = FpvBuild::PhysState();
    FpvBuild::ResetForPark(FpvPhys, FpvPhysical);
    double BuildHover = 0.0;
    if (!bOnGround)
    {
        // In the air: level, with the current speed and the throttle at hover.
        FpvState.Vel = FpvModel::MakeVec(ManualVelNorth, ManualVelEast, 0.0);
        FpvState.ThrustAccel = FpvModel::kGravity;
        BuildHover = FpvBuild::PrimeHover(FpvPhys, FpvPhysical, FpvState);
    }
    if (FpvUseBuild != 0)
    {
        const FpvBuild::Summary Sm = FpvBuild::Summarize(FpvSelection(), FpvPhysical);
        UE_LOG(LogDroneRoute, Display, TEXT("FPV build: %.0f g, thrust/weight %.1f, hover %.0f%%, %s"), Sm.MassG,
               Sm.ThrustToWeight, Sm.HoverThrottle * 100.0, UTF8_TO_TCHAR(Sm.Warnings.c_str()));
    }
    // The stick position that gives hover through the throttle curve.
    const double HoverOut = FpvUseBuild != 0 ? BuildHover : FpvModel::HoverThrottle(FpvParams());
    FpvThrottle = bOnGround ? 0.0 : FpvBuild::ThrottleStickFor(HoverOut, FpvThrottleMid, FpvThrottleExpo, FpvThrottleLimitPct);
    FpvKeyStick[0] = FpvKeyStick[1] = FpvKeyStick[2] = 0.0;
    bFpvGamepadThrottle = false;
    bFpvCrashed = false;
    bFpvPlaceOnGround = bOnGround;
    PitchDeg = RollDeg = 0.0;
    FlightMode = EDroneFlightMode::Fpv;
    ApplyCameraSettings();
    StatusNote = bOnGround
        ? FString(TEXT("FPV on the ground: Space raises the throttle (it stays where you leave it). F: assisted flight."))
        : FString(TEXT("FPV: throttle at hover, no self-levelling, no altitude hold. F: assisted flight."));
    UE_LOG(LogDroneRoute, Display, TEXT("Flight mode: FPV (%s)"), bOnGround ? TEXT("on the ground") : TEXT("in the air"));
}

void ADroneRouteFlight::LeaveFpv()
{
    if (FlightMode != EDroneFlightMode::Fpv)
    {
        return;
    }
    // Assisted flight holds the height the drone has now (at least the manual minimum).
    const double Base = AltitudeMode == EDroneAltitudeMode::AboveTerrain ? GroundEllipsoidHeightM : StartGroundEllipsoidHeightM;
    ManualAltitudeM = FMath::Clamp(EllipsoidHeightM - Base, ManualMinAltitudeM, FMath::Max(ManualMaxAltitudeM, ManualMinAltitudeM));
    ManualVelNorth = bFpvCrashed ? 0.0 : FpvState.Vel.X;
    ManualVelEast = bFpvCrashed ? 0.0 : FpvState.Vel.Y;
    ManualClimbMps = 0.0;
    PitchDeg = RollDeg = 0.0;
    bFpvCrashed = false;
    bFpvPlaceOnGround = false;
    FlightMode = EDroneFlightMode::Manual;
    ApplyCameraSettings();
    if (StatusNote.StartsWith(TEXT("FPV")) || StatusNote.StartsWith(TEXT("CRASHED")) || StatusNote.StartsWith(TEXT("Picked")))
    {
        StatusNote.Reset();
    }
    UE_LOG(LogDroneRoute, Display, TEXT("Flight mode: manual (from FPV)"));
}

void ADroneRouteFlight::PickUpAfterCrash()
{
    // Like walking over and putting the quad back on its legs: upright, motors at zero.
    double YawRad = 0.0, PitchRad = 0.0, RollRad = 0.0;
    FpvModel::ToYawPitchRoll(FpvState.Att, YawRad, PitchRad, RollRad);
    FpvState = FpvModel::State();
    FpvState.Att = FpvModel::FromYawPitchRoll(YawRad, 0.0, 0.0);
    FpvBuild::ResetForPark(FpvPhys, FpvPhysical); // same pack, motors stopped
    FpvThrottle = 0.0;
    bFpvCrashed = false;
    bFpvPlaceOnGround = true;
    StatusNote = TEXT("Picked up: throttle 0, Space to take off.");
}

void ADroneRouteFlight::AdvanceFpv(double Dt, double& OutVelNorth, double& OutVelEast)
{
    OutVelNorth = OutVelEast = 0.0;
    const double GroundM = (bHaveGround ? GroundEllipsoidHeightM : StartGroundEllipsoidHeightM) + FpvGroundClearanceM;
    if (bFpvPlaceOnGround)
    {
        bFpvPlaceOnGround = false;
        EllipsoidHeightM = GroundM;
    }
    // The menu pauses an FPV flight (the drone hangs where it is); a crashed drone lies still.
    if (SettingsPanel.IsValid() || bFpvCrashed)
    {
        PrevVelNorth = PrevVelEast = 0.0;
        return;
    }

    FpvModel::Input In;
    if (APlayerController* PC = GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr)
    {
        auto Key = [PC](const FKey& K) { return PC->IsInputKeyDown(K) ? 1.0 : 0.0; };
        const double KeyTarget[3] = {(Key(EKeys::D) - Key(EKeys::A)) * FpvKeyboardStick,
                                     (Key(EKeys::W) - Key(EKeys::S)) * FpvKeyboardStick,
                                     (Key(EKeys::Right) - Key(EKeys::Left)) * FpvKeyboardStick};
        // A held key moves the stick the way a thumb would: full travel in about 0.12 s.
        const double MaxMove = Dt / 0.12;
        for (int32 Axis = 0; Axis < 3; ++Axis)
        {
            FpvKeyStick[Axis] += FMath::Clamp(KeyTarget[Axis] - FpvKeyStick[Axis], -MaxMove, MaxMove);
        }
        // Gamepad mode 2: right stick pitch / roll, left stick yaw (and throttle).
        In.Roll = FMath::Clamp(FpvKeyStick[0] + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_RightX)), -1.0, 1.0);
        In.Pitch = FMath::Clamp(FpvKeyStick[1] + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_RightY)), -1.0, 1.0);
        In.Yaw = FMath::Clamp(FpvKeyStick[2] + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftX)), -1.0, 1.0);
        // Throttle: Space / Ctrl move a level that stays put, like a real
        // throttle stick; once the gamepad's left stick moves, it sets the
        // throttle directly (down = 0, up = full).
        const double PadThrottle = PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftY);
        if (FMath::Abs(PadThrottle) > 0.2)
        {
            bFpvGamepadThrottle = true;
        }
        if (bFpvGamepadThrottle)
        {
            FpvThrottle = FMath::Clamp(0.5 * (PadThrottle + 1.0), 0.0, 1.0);
        }
        else
        {
            const double ThrottleMove = Key(EKeys::SpaceBar) - FMath::Max(Key(EKeys::LeftControl), Key(EKeys::RightControl));
            FpvThrottle = FMath::Clamp(FpvThrottle + ThrottleMove * FpvThrottleKeyRate * Dt, 0.0, 1.0);
        }
    }
    In.Throttle = FpvBuild::ThrottleCurve(FpvThrottle, FpvThrottleMid, FpvThrottleExpo, FpvThrottleLimitPct);

    const FpvModel::Vec3 Moved = FpvUseBuild != 0
                                     ? FpvBuild::Step(FpvState, FpvPhys, In, FpvParams(), FpvPhysical, FpvPids(), Dt)
                                     : FpvModel::Step(FpvState, In, FpvParams(), Dt);
    Latitude += FMath::RadiansToDegrees(Moved.X / EarthRadiusM);
    Longitude += FMath::RadiansToDegrees(Moved.Y / (EarthRadiusM * FMath::Cos(FMath::DegreesToRadians(Latitude))));
    EllipsoidHeightM -= Moved.Z;

    // The ground: a soft touch lands, a hard or sideways one is a crash.
    if (EllipsoidHeightM < GroundM)
    {
        const double ImpactSpeedMps = FpvModel::Length(FpvState.Vel);
        EllipsoidHeightM = GroundM;
        const bool bHard = ImpactSpeedMps > FpvCrashSpeedMps;
        const bool bOnItsSide = FpvModel::TiltDeg(FpvState.Att) > 75.0 && ImpactSpeedMps > 1.0;
        if (bHard || bOnItsSide)
        {
            bFpvCrashed = true;
            FpvState.Vel = FpvModel::Vec3();
            FpvState.Rate = FpvModel::Vec3();
            FpvState.ThrustAccel = 0.0;
            FpvBuild::ResetForPark(FpvPhys, FpvPhysical);
            StatusNote = FString::Printf(TEXT("CRASHED at %.1f m/s (%.0f km/h). R: pick the drone up."), ImpactSpeedMps,
                                         ImpactSpeedMps * 3.6);
            UE_LOG(LogDroneRoute, Display, TEXT("FPV crash at %.1f m/s, tilt %.0f deg"), ImpactSpeedMps,
                   FpvModel::TiltDeg(FpvState.Att));
        }
        else
        {
            // On its legs: no sinking, the legs stop the sliding, and the frame
            // sits level while the thrust is below the weight.
            if (FpvState.Vel.Z > 0.0)
            {
                FpvState.Vel.Z = 0.0;
            }
            const double Friction = FMath::Exp(-8.0 * Dt);
            FpvState.Vel.X *= Friction;
            FpvState.Vel.Y *= Friction;
            if (FpvState.ThrustAccel < FpvModel::kGravity)
            {
                double SitYawRad = 0.0, SitPitchRad = 0.0, SitRollRad = 0.0;
                FpvModel::ToYawPitchRoll(FpvState.Att, SitYawRad, SitPitchRad, SitRollRad);
                FpvState.Att = FpvModel::FromYawPitchRoll(SitYawRad, 0.0, 0.0);
                FpvState.Rate = FpvModel::Vec3();
                for (double& I : FpvPhys.ITerm)
                {
                    I = 0.0; // Betaflight resets the I term on the ground too
                }
            }
        }
    }

    double YawRad = 0.0, PitchRad = 0.0, RollRad = 0.0;
    FpvModel::ToYawPitchRoll(FpvState.Att, YawRad, PitchRad, RollRad);
    HeadingDeg = Wrap360(FMath::RadiansToDegrees(YawRad));
    PitchDeg = FMath::RadiansToDegrees(PitchRad);
    RollDeg = FMath::RadiansToDegrees(RollRad);
    OutVelNorth = FpvState.Vel.X;
    OutVelEast = FpvState.Vel.Y;
    PrevVelNorth = OutVelNorth; // for the compass track (UpdateTilt does not run in FPV)
    PrevVelEast = OutVelEast;
}

void ADroneRouteFlight::UpdateCameraAim(double Dt)
{
    APlayerController* PC = GetWorld() != nullptr ? GetWorld()->GetFirstPlayerController() : nullptr;
    if (PC == nullptr || SettingsPanel.IsValid() || !bFlying || FlightMode == EDroneFlightMode::Fpv)
    {
        return;
    }
    if (PC->WasInputKeyJustPressed(EKeys::C) || PC->WasInputKeyJustPressed(EKeys::Gamepad_RightThumbstick))
    {
        CameraPitchDeg = -90.0;
        CameraYawDeg = 0.0;
        return;
    }
    auto Held = [PC](const FKey& KeyA, const FKey& KeyB) { return (PC->IsInputKeyDown(KeyA) || PC->IsInputKeyDown(KeyB)) ? 1.0 : 0.0; };
    const double Tilt = Held(EKeys::Up, EKeys::Gamepad_DPad_Up) - Held(EKeys::Down, EKeys::Gamepad_DPad_Down);
    const double Pan = Held(EKeys::E, EKeys::Gamepad_DPad_Right) - Held(EKeys::Q, EKeys::Gamepad_DPad_Left);
    const double Step = CameraTiltRateDegPerSec * Dt;
    // Applied every Tick through the camera's relative rotation; the video capture follows the camera.
    if (Tilt != 0.0)
    {
        CameraPitchDeg = FMath::Clamp(CameraPitchDeg + Tilt * Step, -90.0, 0.0);
    }
    if (Pan != 0.0)
    {
        CameraYawDeg = Wrap360(CameraYawDeg + Pan * Step + 180.0) - 180.0; // continuous, kept in [-180, 180)
    }
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
    if (FlightMode == EDroneFlightMode::Fpv)
    {
        LeaveFpv();
        return;
    }
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
    if (PC->WasInputKeyJustPressed(EKeys::L))
    {
        SetMapLocked(!bMapLocked);
    }
    if (PC->WasInputKeyJustPressed(EKeys::F))
    {
        if (FlightMode == EDroneFlightMode::Fpv)
        {
            LeaveFpv();
        }
        else
        {
            SwitchToManual(); // a mission pauses (M resumes it after leaving FPV)
            EnterFpv(/*bOnGround*/ false);
        }
    }
    else if (FlightMode != EDroneFlightMode::Fpv &&
             (PC->WasInputKeyJustPressed(EKeys::M) || PC->WasInputKeyJustPressed(EKeys::Gamepad_Special_Right)))
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
    if (PC->WasInputKeyJustPressed(EKeys::R))
    {
        if (FlightMode == EDroneFlightMode::Fpv && bFpvCrashed)
        {
            PickUpAfterCrash(); // after a crash R picks the drone up first
        }
        else if (VideoMode == EDroneVideoMode::Record)
        {
            if (bRecordingClip)
            {
                StopRecordingClip();
            }
            else
            {
                StartRecordingClip();
            }
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
    if (FlightMode != EDroneFlightMode::Route || Waypoints.Num() == 0)
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
        Yaw = Key(EKeys::Right) - Key(EKeys::Left) + ShapeStick(PC->GetInputAnalogKeyState(EKeys::Gamepad_LeftX));
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

    if (PerfMonitor.IsRunning())
    {
        FDronePerfFrameContext Ctx;
        Ctx.Mode = !bFlying ? TEXT("HOVER") : (FlightMode == EDroneFlightMode::Fpv ? TEXT("FPV")
                                             : (FlightMode == EDroneFlightMode::Manual ? TEXT("MANUAL") : TEXT("ROUTE")));
        Ctx.bMenuOpen = SettingsPanel.IsValid();
        Ctx.bCrashed = bFpvCrashed;
        Ctx.SpeedMps = PerfSpeedMps;
        Ctx.AglM = bHaveGround ? EllipsoidHeightM - GroundEllipsoidHeightM : -1.0;
        Ctx.HeadingDeg = HeadingDeg;
        Ctx.PitchDeg = PitchDeg;
        Ctx.RollDeg = RollDeg;
        Ctx.VideoMode = bSessionActive ? static_cast<int32>(VideoMode) : 0;
        Ctx.CesiumCameras = BaseAdditionalCameras.Num() + PreloadCameras.Num() + AroundCameras.Num();
        PerfMonitor.BeginFrame(GetWorld(), DeltaSeconds, Ctx);
    }

    {
        FDronePerfMonitor::FScope PerfScope(PerfMonitor, EDronePerfScope::Input);
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
    }
    if (Georeference == nullptr || Dt <= 0.0)
    {
        return;
    }
    {
        FDronePerfMonitor::FScope PerfScope(PerfMonitor, EDronePerfScope::View);
        ClaimView();
        UpdateCameraAim(Dt);
    }
    {
        FDronePerfMonitor::FScope PerfScope(PerfMonitor, EDronePerfScope::AroundCameras);
        UpdateAroundCameras();
    }
    {
        FDronePerfMonitor::FScope PerfScope(PerfMonitor, EDronePerfScope::CesiumCameras);
        PushCesiumCameras();
    }
    SinceMarkersCheckS += Dt;
    if (SinceMarkersCheckS >= 0.5)
    {
        FDronePerfMonitor::FScope PerfScope(PerfMonitor, EDronePerfScope::Markers);
        SinceMarkersCheckS = 0.0;
        UpdateMarkers(/*bForce*/ false);
    }

    double Ground = 0.0;
    bool bGroundHit = false;
    {
        FDronePerfMonitor::FScope PerfScope(PerfMonitor, EDronePerfScope::GroundTrace);
        bGroundHit = TraceGroundEllipsoidHeight(Longitude, Latitude, Ground);
    }
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
    TOptional<FDronePerfMonitor::FScope> PhysicsScope;
    PhysicsScope.Emplace(PerfMonitor, EDronePerfScope::Physics);
    if (bHaveStartGround && bFlying && FlightMode == EDroneFlightMode::Fpv)
    {
        // Physics sets the height and the attitude: no altitude hold, no gimbal.
        AdvanceFpv(Dt, VelNorth, VelEast);
    }
    else if (bHaveStartGround)
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
    PhysicsScope.Reset();
    PerfSpeedMps = FlightMode == EDroneFlightMode::Fpv ? FpvModel::Length(FpvState.Vel) : FMath::Sqrt(VelNorth * VelNorth + VelEast * VelEast);

    TOptional<FDronePerfMonitor::FScope> MoveScope;
    MoveScope.Emplace(PerfMonitor, EDronePerfScope::Move);
    GlobeAnchor->MoveToLongitudeLatitudeHeight(FVector(Longitude, Latitude, EllipsoidHeightM));
    GlobeAnchor->SetEastSouthUpRotation(FRotator(0.0, HeadingDeg - 90.0, 0.0).Quaternion());
    if (Camera != nullptr)
    {
        // Camera = body tilt (zero with a gimbal) then the mount: gimbal pan
        // about the body's up axis then tilt, or the fixed FPV uptilt.
        const FQuat Body = FRotator(PitchDeg, 0.0, RollDeg).Quaternion();
        const bool bFpvCamera = FlightMode == EDroneFlightMode::Fpv;
        const FQuat Mount =
            FRotator(bFpvCamera ? FpvCameraUptiltDeg : CameraPitchDeg, bFpvCamera ? 0.0 : CameraYawDeg, 0.0).Quaternion();
        Camera->SetRelativeRotation(Body * Mount);
    }

    MoveScope.Reset();

    if (bHaveStartGround && bSessionActive)
    {
        FDronePerfMonitor::FScope PerfScope(PerfMonitor, EDronePerfScope::Telemetry);
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
    const bool bFpvCamera = FlightMode == EDroneFlightMode::Fpv;
    S.CameraPitchDeg = bFpvCamera ? FpvCameraUptiltDeg : CameraPitchDeg;
    S.CameraYawDeg = bFpvCamera ? 0.0 : CameraYawDeg;
    S.CameraHFovDeg = bFpvCamera ? FpvCameraFovDeg : CameraHorizontalFovDeg;

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
