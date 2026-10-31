// The simulated drone: flies a lat/lon waypoint route or is flown by hand
// (keyboard / gamepad) over Cesium terrain with a down-looking camera, and
// reports its true state as MAVLink telemetry, a per-frame ground-truth CSV
// and (optionally) the camera video: an mp4 recording or a live RTSP stream.
// Kinematic model: the position follows the commanded velocity. With a
// gimballed camera the reported body stays level; with a fixed camera the body
// and the camera tilt with acceleration and air drag, and ATTITUDE reports it.
// Like a game: a main menu first (maps with ready survey missions, settings),
// then the flight (manual, or the map's survey-grid mission like Mission
// Planner's Survey (Grid)); Esc / F2 / Tab open the menu during the flight.
// The map's area is shown in the world by poles and see-through walls.
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "Sim/DroneRtspServer.h"
#include "Sim/DroneSettingsPanel.h"
#include "Sim/DroneTelemetryOutput.h"
#include "Sim/DroneVideoOutput.h"
#include "Widgets/SWidget.h"
#include "DroneRouteFlight.generated.h"

class ACesiumGeoreference;
class ADroneAreaMarkers;
class UCameraComponent;
class UCesiumGlobeAnchorComponent;
class USceneCaptureComponent2D;
class UTextureRenderTarget2D;

UENUM(BlueprintType)
enum class EDroneAltitudeMode : uint8
{
    // Keep the waypoint altitude above the terrain under the drone (constant ground scale).
    AboveTerrain UMETA(DisplayName = "Above terrain (AGL)"),
    // Keep the waypoint altitude above the start point's ground (like a barometric altitude hold).
    AboveStart UMETA(DisplayName = "Above start point"),
};

UENUM(BlueprintType)
enum class EDroneExposureMode : uint8
{
    // Auto exposure (like a real camera) shifted by ExposureCompensationEV.
    Auto UMETA(DisplayName = "Auto + compensation"),
    // Exposure locked at FixedExposureEV100: brightness does not change between frames.
    Fixed UMETA(DisplayName = "Fixed EV100"),
};

UENUM(BlueprintType)
enum class EDroneFlightMode : uint8
{
    Route UMETA(DisplayName = "Route (autopilot)"),
    Manual UMETA(DisplayName = "Manual (keyboard / gamepad)"),
};

UENUM(BlueprintType)
enum class EDroneVideoMode : uint8
{
    Off,
    // Fixed frame rate at VideoFps: flight.mp4 + frames.csv in the flight folder.
    Record UMETA(DisplayName = "Record flight.mp4"),
    // Real time: H.264 to RtspUrl (start an RTSP server such as mediamtx first).
    Live UMETA(DisplayName = "Live RTSP stream"),
};

USTRUCT(BlueprintType)
struct FDroneWaypoint
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route", meta = (ClampMin = "-90", ClampMax = "90"))
    double Latitude = 48.365;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route", meta = (ClampMin = "-180", ClampMax = "180"))
    double Longitude = 26.065;

    // Metres, interpreted according to AltitudeMode.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Route", meta = (ClampMin = "1"))
    double AltitudeM = 500.0;
};

UCLASS()
class DRONESIMUE_API ADroneRouteFlight : public APawn
{
    GENERATED_BODY()

public:
    ADroneRouteFlight();

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void Tick(float DeltaSeconds) override;
#if WITH_EDITOR
    // Camera settings edited in Details (also during Play) apply immediately.
    virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

    // --- Flight --------------------------------------------------------------
    // M (gamepad Start) switches during the flight.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight")
    EDroneFlightMode FlightMode = EDroneFlightMode::Route;

    // Start (and restart after Apply) in manual mode at the start point, so
    // the keyboard can be checked first; M or a mission switches to the autopilot.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Flight")
    bool bStartInManual = true;

    // --- Route ---------------------------------------------------------------
    // The default route (M in manual mode flies it). Apply in the settings
    // panel shifts it to start at the start point; a mission replaces it
    // until the next Apply.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route")
    TArray<FDroneWaypoint> Waypoints;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route")
    EDroneAltitudeMode AltitudeMode = EDroneAltitudeMode::AboveTerrain;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route", meta = (ClampMin = "0.1", Units = "MetersPerSecond"))
    double SpeedMps = 15.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route", meta = (ClampMin = "1"))
    double MaxYawRateDegPerSec = 30.0;

    // Time constant of the altitude follower (terrain-following and climbs).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route", meta = (ClampMin = "0.05", Units = "Seconds"))
    double AltitudeTimeConstantS = 1.5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route")
    bool bLoopRoute = true;

    // Ellipsoid height of the ground used until the first terrain hit (the
    // drone starts at this + the first waypoint altitude, then corrects).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route", meta = (Units = "Meters"))
    double InitialGroundEllipsoidHeightM = 300.0;

    // Before starting, the ground height under the drone must stay within
    // TerrainSettleToleranceM for this long (coarse tiles get replaced by
    // finer ones in the first seconds and the height jumps by tens of metres).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route", meta = (ClampMin = "0", Units = "Seconds"))
    double TerrainSettleSeconds = 2.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route", meta = (ClampMin = "0.1", Units = "Meters"))
    double TerrainSettleToleranceM = 1.0;

    // Start anyway after this long, even if the terrain is still refining.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Route", meta = (ClampMin = "1", Units = "Seconds"))
    double TerrainSettleTimeoutS = 15.0;

    // --- Manual control --------------------------------------------------------
    // Assisted control like a camera drone: the sticks set the horizontal
    // velocity, the yaw rate and the climb rate; centred sticks brake to a hover.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "0.5", Units = "MetersPerSecond"))
    double ManualMaxSpeedMps = 15.0;

    // Horizontal speed and acceleration multiplier while Shift (or the right
    // gamepad trigger) is held; the climb rate is not affected.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "1"))
    double ManualFastMultiplier = 2.0;

    // Horizontal acceleration toward the commanded velocity (0 -> 15 m/s in about 1.9 s).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "0.5"))
    double ManualAccelerationMps2 = 8.0;

    // Deceleration when the sticks are released or reversed: like a camera
    // drone, it brakes harder than it accelerates (15 m/s -> 0 in about 1.25 s).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "0.5"))
    double ManualBrakingMps2 = 12.0;

    // Space / Ctrl (left stick up / down): climb / descent rate.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "0.5", Units = "MetersPerSecond"))
    double ManualClimbRateMps = 10.0;

    // Vertical acceleration toward the commanded climb rate (0 -> 10 m/s in 1 s).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "0.5"))
    double ManualVerticalAccelerationMps2 = 10.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "5"))
    double ManualYawRateDegPerSec = 60.0;

    // Altitude limits (meaning follows AltitudeMode).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "1", Units = "Meters"))
    double ManualMinAltitudeM = 10.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "10", Units = "Meters"))
    double ManualMaxAltitudeM = 3000.0;

    // Gamepad / RC sticks: dead zone around the centre and expo
    // (0 = linear, 1 = cubic: finer control near the centre). Keys are not affected.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "0", ClampMax = "0.5"))
    double StickDeadZone = 0.12;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Manual", meta = (ClampMin = "0", ClampMax = "1"))
    double StickExpo = 0.0;

    // --- Camera --------------------------------------------------------------
    // Camera pitch relative to the body: -90 looks straight down, the image top points to the nose.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-90", ClampMax = "0", Units = "Degrees"))
    double CameraPitchDeg = -90.0;

    // On: a stabilised gimbal keeps the camera level, and the reported body
    // attitude stays level too (pitch/roll 0), so telemetry always describes
    // the camera. Off: camera fixed to the body; the body tilts toward the
    // force it needs (acceleration + air drag) and ATTITUDE reports that tilt.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    bool bCameraGimbal = true;

    // Linear air-drag coefficient (1/s) for the body tilt with a fixed camera:
    // at a steady speed v the drone tilts atan(k * v / g); 0.25 gives about 21 deg at 15 m/s.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0", EditCondition = "!bCameraGimbal"))
    double LinearDragCoefficient = 0.25;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0", ClampMax = "60", Units = "Degrees", EditCondition = "!bCameraGimbal"))
    double MaxTiltDeg = 30.0;

    // Horizontal field of view. 36.87 deg = FlightSimulator's camera (8.8 mm sensor, 13.2 mm lens).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "5", ClampMax = "170", Units = "Degrees"))
    float CameraHorizontalFovDeg = 36.87f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "0.1"))
    float CameraAspectRatio = 1280.f / 720.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera")
    EDroneExposureMode ExposureMode = EDroneExposureMode::Auto;

    // Negative = darker. Used in both modes.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (UIMin = "-5", UIMax = "5"))
    float ExposureCompensationEV = -0.5f;

    // Only for the Fixed mode. Higher = darker; roughly 14-15 for a sunny scene.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (UIMin = "8", UIMax = "18"))
    float FixedExposureEV100 = 14.f;

    // --- Video ---------------------------------------------------------------
    // Read when Play starts.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video")
    EDroneVideoMode VideoMode = EDroneVideoMode::Off;

    // "ffmpeg" if it is in PATH, otherwise the full path, e.g. C:/Tools/ffmpeg/bin/ffmpeg.exe
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video")
    FString FfmpegPath = TEXT("ffmpeg");

    // h264_nvenc = NVIDIA GPU encoder; libx264 = CPU encoder (works everywhere, slower).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video")
    FString VideoEncoder = TEXT("h264_nvenc");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video")
    FString RtspUrl = TEXT("rtsp://127.0.0.1:8554/drone");

    // Live: if nothing listens on the RtspUrl port (local host only), start
    // RtspServerPath (mediamtx, with mediamtx.yml next to it) and stop it
    // when Play ends. Its log: Saved/Logs/mediamtx.log.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video")
    bool bStartRtspServer = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video", meta = (EditCondition = "bStartRtspServer"))
    FString RtspServerPath = TEXT("C:/My programs/mediamtx_v1.21.2_windows_amd64/mediamtx.exe");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video", meta = (ClampMin = "64", ClampMax = "3840"))
    int32 VideoWidth = 1280;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video", meta = (ClampMin = "64", ClampMax = "2160"))
    int32 VideoHeight = 720;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video", meta = (ClampMin = "1", ClampMax = "120"))
    int32 VideoFps = 30;

    // Record: every engine frame advances the simulation by exactly 1/VideoFps
    // (real time is capped at VideoFps), so each frame is one video frame and
    // none is lost when rendering is slow; the flight then runs slower than real time.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video")
    bool bFixedFrameRateWhenRecording = true;

    // Live: cap the engine at VideoFps (real time, t.MaxFPS) and send one frame
    // per engine frame. Every frame then has the same cost (view + capture),
    // so frames come at an even rate; capturing every Nth frame of a faster
    // engine gives uneven gaps because the capturing frames are slower.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Video")
    bool bLimitFrameRateWhenLive = true;

    // --- Telemetry -----------------------------------------------------------
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Telemetry")
    bool bSendMavlink = true;

    // 14550 is where QGroundControl and most ground stations listen.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Telemetry")
    FString MavlinkHost = TEXT("127.0.0.1");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Telemetry", meta = (ClampMin = "1", ClampMax = "65535"))
    int32 MavlinkPort = 14550;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Telemetry", meta = (ClampMin = "1", ClampMax = "200"))
    float TelemetryRateHz = 30.f;

    // Writes Saved/Flights/<date-time>/telemetry.csv, one row per simulation step.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Telemetry")
    bool bWriteGroundTruthCsv = true;

    // Geoid height N (metres) at the flight area: MSL = ellipsoid height - N.
    // Cesium heights are WGS84 ellipsoid heights. 0 = report ellipsoid as MSL.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Telemetry", meta = (Units = "Meters"))
    double GeoidUndulationM = 0.0;

    // --- Settings panel ----------------------------------------------------------
    // On: the settings saved by the in-game panel (F2 -> Apply) in
    // Saved/DroneSimSettings.json are applied at start and override the
    // values above that the panel shows.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Settings")
    bool bApplySavedSettings = true;

    // Show the main menu when the program starts; the flight (video, telemetry)
    // begins with "Free flight" / "Fly the mission". Off: fly at once.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Settings")
    bool bOpenSettingsAtStart = true;

    // Status line in the top-left corner of the viewport (not in the video).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
    bool bShowDebugText = true;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
    TObjectPtr<USceneComponent> Root;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
    TObjectPtr<UCameraComponent> Camera;

    // Renders the video frames (same pose, FOV and post-process as Camera).
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
    TObjectPtr<USceneCaptureComponent2D> VideoCapture;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
    TObjectPtr<UCesiumGlobeAnchorComponent> GlobeAnchor;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

private:
    void ApplyCameraSettings();
    void ClaimView();
    // Ellipsoid height of the terrain under (Lon, Lat), by a line trace
    // against the loaded Cesium tiles. False if no tile is hit yet.
    bool TraceGroundEllipsoidHeight(double Lon, double Lat, double& OutHeightM) const;
    void AdvanceAlongRoute(double Dt, double& OutVelNorth, double& OutVelEast);
    void AdvanceManual(double Dt, double& OutVelNorth, double& OutVelEast);
    void UpdateTilt(double Dt, double VelNorth, double VelEast);
    void HandleHotkeys();
    void SwitchToManual();
    void SwitchToRoute();
    // Waypoint after Index: the next one, or RouteLoopStartIndex after the last.
    int32 NextWaypointIndex(int32 Index) const;
    // A text field of the settings panel has the keyboard: the letters typed
    // there also reach the game, so the flight keys are ignored.
    bool IsTypingInUi() const;
    // Altitude target (meaning follows AltitudeMode): the route leg's, or the manual one.
    double TargetAltitude() const;
    void EmitTelemetry(double Dt, double VelNorth, double VelEast, double VelDown);

    // Flight session: output folder, telemetry, CSV files, video.
    void ResetFlightState();
    void StartSession();
    void StopSession();
    void StartVideo();
    void StopVideo();
    void CaptureVideoFrame(const FDroneTelemetrySample& Sample, double Dt);

    // Menu, saved settings and the status line.
    FDroneSimSettings CurrentSettings() const;
    // Controls, camera, sun, markers, map / mission fields; video and MAVLink
    // only between flights. Not the start point.
    void ApplyLiveSettings(const FDroneSimSettings& NewSettings);
    // Moves the drone to the start point (origin, terrain settle) and ends a mission.
    void ApplyStartPoint(const FDroneSimSettings& NewSettings);
    bool StartPointDiffers(const FDroneSimSettings& NewSettings) const;
    void HandleMenuAction(EDroneMenuAction Action, const FDroneSimSettings& NewSettings);
    void BeginFlight();
    void EndFlight();
    void SaveSettingsFile() const;
    // Plans the survey grid and flies it (replaces Waypoints). False: no plan (see StatusNote).
    bool StartMission(const FDroneSimSettings& MissionSettings);
    FString MissionStatus() const;
    void OpenMenu();
    void CloseMenu();
    void AddStatusLine();
    void RemoveStatusLine();
    FString StatusText() const;
    FString MenuStatusText() const;
    static FString SettingsPath();
    // Area markers follow the menu's draft (or the applied map); rebuilt when it
    // changes, or always with bForce (origin moved, terrain known).
    void UpdateMarkers(bool bForce);
    // V: the drone position becomes the next vertex of the map's area.
    void AddVertexHere();
    // Dead zone + expo for an analog stick axis.
    float ShapeStick(float Value) const;

    UPROPERTY(Transient)
    TObjectPtr<ACesiumGeoreference> Georeference;

    UPROPERTY(Transient)
    TObjectPtr<UTextureRenderTarget2D> VideoTarget;

    UPROPERTY(Transient)
    TObjectPtr<ADroneAreaMarkers> AreaMarkers;
    FString MarkersKey;
    double SinceMarkersCheckS = 0.0;

    FDroneTelemetryOutput Telemetry;
    FDroneVideoOutput Video;
    FDroneRtspServer RtspServer;
    FString SessionDir; // Saved/Flights/<date-time>
    FString StatusNote; // last problem, shown under the status line

    TSharedPtr<SDroneSettingsPanel> SettingsPanel;
    TSharedPtr<SWidget> StatusLine;
    // Menu buttons only set these; the work happens in the next Tick,
    // outside the Slate click handler.
    FDroneSimSettings PendingSettings;
    EDroneMenuAction PendingAction = EDroneMenuAction::Resume;
    bool bPendingAction = false;
    bool bPendingCloseMenu = false;
    // false: the main menu before a flight (the drone hovers at the start
    // point, no video, no telemetry); true: flying.
    bool bFlying = false;
    bool bSessionActive = false;

    // Last applied start point, altitude, heading and default speed, and the
    // mission fields (the panel opens with them). Initialised from Details.
    FDroneSimSettings LastSettings;
    // The route from Details, restored by Apply (shifted to the start point).
    TArray<FDroneWaypoint> BaseRoute;
    bool bBaseLoopRoute = true;
    int32 RouteLoopStartIndex = 0; // a looping route continues here after its last waypoint
    bool bRouteFixedHeading = false;
    double RouteFixedHeadingDeg = 0.0;
    bool bMissionLoaded = false;   // Waypoints hold a mission
    int32 MissionFirstIndex = 0;   // Waypoints index of the first pass's start
    int32 MissionPasses = 0;

    double Latitude = 0.0;
    double Longitude = 0.0;
    double EllipsoidHeightM = 0.0;
    double HeadingDeg = 0.0;
    int32 LegIndex = 0; // flying from Waypoints[LegIndex] to Waypoints[LegIndex + 1]
    double LegLengthM = 0.0;
    bool bRouteFinished = false;

    double ManualVelNorth = 0.0;
    double ManualVelEast = 0.0;
    double ManualAltitudeM = 500.0;
    double ManualClimbMps = 0.0; // current commanded climb rate (up positive)

    double PrevVelNorth = 0.0;
    double PrevVelEast = 0.0;
    double PitchDeg = 0.0; // body, nose up positive
    double RollDeg = 0.0;  // body, right side down positive

    bool bHaveGround = false;
    double GroundEllipsoidHeightM = 0.0;
    double SettleReferenceHeightM = 0.0;
    double SettleStableS = 0.0;
    double SettleWaitedS = 0.0;
    double StartGroundEllipsoidHeightM = 0.0;
    bool bHaveStartGround = false;

    double SimTimeS = 0.0; // sum of the frame times since the route started
    double SinceTelemetryS = 0.0;
    double SinceHeartbeatS = 1.0;
    double SinceVideoFrameS = 0.0;
    int64 FrameIndex = 0;
    int64 VideoFrameIndex = 0;
    bool bViewClaimed = false;

    // Engine frame-rate settings replaced while recording (fixed frame rate)
    // or streaming (max FPS).
    bool bFrameRateOverridden = false;
    bool bSavedUseFixedFrameRate = false;
    float SavedFixedFrameRate = 30.f;
    bool bMaxFpsOverridden = false;
    float SavedMaxFps = 0.f;
};
