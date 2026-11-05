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
#include "CesiumCamera.h"
#include "Sim/DroneCompass.h"
#include "Sim/DronePerfMonitor.h"
#include "Sim/FpvModel.h"
#include "Sim/FpvBuild.h"
#include "Sim/DroneRtspServer.h"
#include "Sim/DroneSettingsPanel.h"
#include "Sim/DroneTelemetryOutput.h"
#include "Sim/DroneTileStore.h"
#include "Sim/DroneVideoOutput.h"
#include "Widgets/SWidget.h"
#include "DroneRouteFlight.generated.h"

class ACesiumGeoreference;
class ADroneAreaMarkers;
class UCameraComponent;
class UCesiumGlobeAnchorComponent;
class UCesiumRasterOverlay;
class UCesiumUrlTemplateRasterOverlay;
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
    // Acro like a real FPV drone: no self-levelling, no altitude hold (F in flight).
    Fpv UMETA(DisplayName = "FPV (acro, no assistance)"),
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

    // --- FPV (acro) ----------------------------------------------------------
    // Like a real FPV drone with no assistance: the sticks set rotation rates
    // (Betaflight "Actual" rates), the attitude stays where it is left, the
    // throttle sets the thrust (no altitude hold), gravity, momentum and drag.
    // F switches in flight; "FPV flight" in the main menu starts on the ground.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "1.5", ClampMax = "15"))
    double FpvThrustToWeight = 6.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "10", ClampMax = "2000"))
    double FpvRollPitchCenterRateDps = 70.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "10", ClampMax = "2000"))
    double FpvRollPitchMaxRateDps = 670.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0", ClampMax = "1"))
    double FpvRollPitchExpo = 0.54;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "10", ClampMax = "2000"))
    double FpvYawCenterRateDps = 70.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "10", ClampMax = "2000"))
    double FpvYawMaxRateDps = 670.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0", ClampMax = "1"))
    double FpvYawExpo = 0.54;

    // Camera fixed to the frame, tilted up from the horizon (FPV pilots use about 20..40 deg).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "-30", ClampMax = "80", Units = "Degrees"))
    double FpvCameraUptiltDeg = 30.0;

    // Horizontal field of view of the FPV camera.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "40", ClampMax = "170", Units = "Degrees"))
    float FpvCameraFovDeg = 110.f;

    // Keyboard: how far a held key moves the stick (0..1).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0.05", ClampMax = "1"))
    double FpvKeyboardStick = 0.5;

    // Keyboard: throttle change per second while Space / Ctrl is held.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0.05", ClampMax = "5"))
    double FpvThrottleKeyRate = 0.6;

    // Touching the ground faster than this is a crash (R picks the drone up).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0.5", Units = "MetersPerSecond"))
    double FpvCrashSpeedMps = 5.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0"))
    double FpvLinearDrag = 0.05;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0"))
    double FpvQuadraticDrag = 0.035;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0.005", Units = "Seconds"))
    double FpvMotorTimeConstantS = 0.03;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0.005", Units = "Seconds"))
    double FpvRateTimeConstantS = 0.03;

    // Drone builder (FpvBuild.h): 1 = fly the physical model built from the parts below
    // (Betaflight-style rate PID, airmode mixer, KV/resistance motor model, battery sag);
    // 0 = the simple model above (thrust/weight + lags).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build", meta = (ClampMin = "0", ClampMax = "1"))
    int32 FpvUseBuild = 1;

    // Indices into FpvBuild::kFrames / kMotors / kProps / kBatteries / kFpvSystems.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build")
    int32 FpvFrame = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build")
    int32 FpvMotor = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build")
    int32 FpvProp = 2;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build")
    int32 FpvBattery = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build")
    int32 FpvCameraSystem = 1;

    // Betaflight configurator sliders: master multiplier (P, I, D, FF) and damping (D).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build", meta = (ClampMin = "0.2", ClampMax = "3"))
    double FpvPidMaster = 1.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build", meta = (ClampMin = "0.2", ClampMax = "3"))
    double FpvPidDamping = 1.0;

    // Throttle curve, like Betaflight thr_mid / thr_expo / throttle limit (scale).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0.05", ClampMax = "0.95"))
    double FpvThrottleMid = 0.5;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "0", ClampMax = "1"))
    double FpvThrottleExpo = 0.0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV", meta = (ClampMin = "10", ClampMax = "100"))
    double FpvThrottleLimitPct = 100.0;

    // Names shown in the menu (the drone builder and the rates page).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV|Build")
    FString FpvDroneName = TEXT("5\" freestyle 4S");

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|FPV")
    FString FpvRatesName = TEXT("Medium");

    // --- Camera --------------------------------------------------------------
    // Camera pitch relative to the body: -90 looks straight down, the image top points to the nose.
    // Up / Down arrows (gamepad D-pad) change it in flight; every flight starts with this value.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-90", ClampMax = "0", Units = "Degrees"))
    double CameraPitchDeg = -90.0;

    // Camera pan relative to the nose (+ = to the right): Q / E (gamepad D-pad left / right).
    // With the camera straight down it turns the image. Every flight starts with this value.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "-180", ClampMax = "180", Units = "Degrees"))
    double CameraYawDeg = 0.0;

    // Tilt / pan speed of the arrow keys.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Camera", meta = (ClampMin = "1", ClampMax = "180"))
    double CameraTiltRateDegPerSec = 30.0;

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

    // Performance monitor: every run writes Saved/Perf/<date-time>/ (frames.csv,
    // system.csv with CPU / GPU / RAM / disk load, hitches.csv, summary.txt);
    // Tools/perf_report.py turns it into a report. Costs well under 0.1 ms a frame.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
    bool bPerfMonitor = true;

    // A frame at least this long is logged as a hitch (as is a spike > 2.5x the recent average).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug", meta = (EditCondition = "bPerfMonitor", ClampMin = "5", Units = "Milliseconds"))
    double PerfHitchMs = 50.0;

    // Compass tape at the top of the viewport during a flight (not in the video):
    // camera view direction, nose, ground track, next waypoint.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Debug")
    bool bShowCompass = true;

    // --- Map loading (applied to every Cesium tileset in the level) ------------
    // 0 = normal (Cesium's screen-space error 16), 1 = high (8), 2 = very high (4):
    // each step doubles the detail and roughly quadruples the tiles to load.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "0", ClampMax = "2"))
    int32 MapQuality = 1;

    // 0 terrain + draped imagery, 1 Google Photorealistic 3D Tiles (buildings, trees).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "0", ClampMax = "1"))
    int32 WorldMode = 0;

    // Keep showing a coarser tile until all its finer children are loaded, so no holes appear.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading")
    bool bForbidHoles = true;

    // Load the map's survey area plus PreloadMarginM around it at the detail of the
    // flight altitude and keep it loaded, using extra (invisible) Cesium cameras.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading")
    bool bPreloadArea = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "0", Units = "Meters"))
    double PreloadMarginM = 1000.0;

    // Tiles kept in memory after they are no longer needed (Cesium's MaximumCachedBytes).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "64"))
    int32 TileMemoryCacheMB = 4096;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "1", ClampMax = "512"))
    int32 MaxSimultaneousTileLoads = 64;

    // What turning would show: invisible copies of the camera turned around the vertical to the
    // other headings (360 / FOV of them) keep those tiles loaded, so turning the drone or the camera
    // shows the map at once. 0 = off, 1 = at the camera's detail, 2 = half, 3 = quarter of it.
    // (Not Cesium's "frustum culling off": that loads everything up to the fog distance, hundreds of km.)
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "0", ClampMax = "3"))
    int32 AroundDroneDetail = 1;

    // How AroundDroneDetail keeps the turned-away map. Off (default): Cesium keeps the tiles
    // outside the view (EnforceCulledScreenSpaceError, culled SSE = view SSE x 1 / 2 / 4) in
    // the one view's tile selection. On: the old invisible turned cameras; each one costs a
    // full tile selection on the game thread every frame (2026-11-03 perf runs: ~30 extra
    // views = ~265 ms game thread, none = ~26 ms).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading")
    bool bAroundViaCameras = false;

    // 0 = step by step (Cesium's LoadingDescendantLimit 20: coarse, finer, final);
    // 1 = straight to the final detail (limit 1000: fewer tiles, but with ForbidHoles the coarse
    // tile can stay for a long time).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "0", ClampMax = "1"))
    int32 TileLoadingStyle = 0;

    // Imagery: 0 = the level's overlay (Bing Maps Aerial from Cesium ion; downloaded again
    // every run, its tile addresses carry a per-run key), 1 = Esri World Imagery through
    // the tile store: every tile is downloaded once and kept on disk (TileStoreDir),
    // 2 = Esri by date: the Wayback release WaybackRelease (0 = current), kept on disk too.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "0", ClampMax = "2"))
    int32 ImagerySource = 0;

    // Esri World Imagery Wayback release for ImagerySource 2 (0 = the current imagery).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "0"))
    int32 WaybackRelease = 0;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading")
    FString WaybackLabel;

    // Deepest Esri level requested (19 is about 0.2 m per pixel at 48 deg latitude);
    // deeper levels exist only in some places.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "10", ClampMax = "23"))
    int32 EsriMaxLevel = 19;

    // Where the stored tiles go; empty = <local app data>/DroneSimUE/TileStore
    // (C:/Users/<you>/AppData/Local/...), shared by the editor and the packaged game.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading")
    FString TileStoreDir;

    // Loopback port of the tile store (127.0.0.1 only).
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Drone|Map loading", meta = (ClampMin = "1024", ClampMax = "65535"))
    int32 TileStorePort = 18765;

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
    // Record mode, R key: starts a new clip (own folder: telemetry.csv,
    // flight.mp4, frames.csv, ground_points.csv) or stops the current one.
    // Manual / FPV flights start with recording off; a route / mission
    // flight starts recording at once.
    void StartRecordingClip();
    void StopRecordingClip();
    void CaptureVideoFrame(const FDroneTelemetrySample& Sample, double Dt);
    // Record mode: casts rays from the video capture through a 3x3 pixel grid
    // (20/50/80 % of width and height) onto the Cesium terrain and logs the hits.
    void TraceImageGround(int64 VideoFrame);

    // Menu, saved settings and the status line.
    FDroneSimSettings CurrentSettings() const;
    // Controls, camera, sun, markers, map / mission fields; video and MAVLink
    // only between flights. Not the start point.
    void ApplyLiveSettings(const FDroneSimSettings& NewSettings);
    // Moves the drone to the start point (origin, terrain settle) and ends a mission.
    void ApplyStartPoint(const FDroneSimSettings& NewSettings);
    // The start coordinates, altitude or heading differ from the applied ones.
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
    FDroneCompassState CompassState() const;
    static FString SettingsPath();
    // Area markers follow the menu's draft (or the applied map); rebuilt when it
    // changes, or always with bForce (origin moved, terrain known).
    void UpdateMarkers(bool bForce);
    // Quality / cache / loading settings to every Cesium tileset.
    void ApplyTilesetSettings();
    // WorldMode: shows the terrain tileset (2.5D) or Google Photorealistic 3D Tiles (spawned on first use).
    void ApplyWorldMode();
    TWeakObjectPtr<class ACesium3DTileset> Google3DTileset;
    // ImagerySource: the level's overlays or the stored Esri imagery (in the same material layer).
    void ApplyImagerySource();
    void EnsureHttpConnectionReuse();
    FString ResolvedTileStoreDir() const;
    // The different Wayback images at the map's area (asynchronous, once per place).
    void RequestWaybackVersions(const FDroneSimSettings& Source);
    static void MapCentreOf(const FDroneSimSettings& S, double& OutLat, double& OutLon);
    FString ImageryStatus() const;
    // Invisible Cesium cameras over the map's area (+ margin) so its tiles load and stay loaded.
    void UpdatePreload(const FDroneSimSettings& Source, bool bForce);
    // Every Tick: the turned copies of the camera, then the camera manager's list.
    void UpdateAroundCameras();
    // L in flight: Cesium loads no new tiles (MaximumSimultaneousTileLoads 0) and draws from
    // what is loaded; L again resumes loading.
    void SetMapLocked(bool bLocked);
    bool bMapLocked = false;
    void PushCesiumCameras();
    // Load progress of the terrain tileset, 0..100 (-1 if there is none).
    float MapLoadProgress() const;
    // V: the drone position becomes the next vertex of the map's area.
    void AddVertexHere();
    // Dead zone + expo for an analog stick axis.
    float ShapeStick(float Value) const;
    // Up / Down arrows (gamepad D-pad): tilt the camera between -90 (down) and 0
    // (horizon), Q / E pan it; C (gamepad right stick click) resets it.
    void UpdateCameraAim(double Dt);
    // FPV: sticks / keys -> FpvModel; ground contact, landing and crashes.
    void AdvanceFpv(double Dt, double& OutVelNorth, double& OutVelEast);
    // Into FPV mode from the current state (in the air: level, hover throttle) or on the ground.
    void EnterFpv(bool bOnGround);
    // Back to assisted manual flight, holding the current height.
    void LeaveFpv();
    void PickUpAfterCrash();
    FpvModel::Params FpvParams() const;
    FpvBuild::Selection FpvSelection() const;
    FpvBuild::Pids FpvPids() const;

    UPROPERTY(Transient)
    TObjectPtr<ACesiumGeoreference> Georeference;

    UPROPERTY(Transient)
    TObjectPtr<UTextureRenderTarget2D> VideoTarget;

    UPROPERTY(Transient)
    TObjectPtr<ADroneAreaMarkers> AreaMarkers;
    FString MarkersKey;
    double SinceMarkersCheckS = 0.0;
    FString PreloadKey;
    int32 PreloadViewCount = 0;
    double PreloadAreaKm2 = 0.0;
    TArray<FCesiumCamera> BaseAdditionalCameras; // the camera manager's own cameras (kept)
    TArray<FCesiumCamera> PreloadCameras;        // UpdatePreload: nadir views over the area
    TArray<FCesiumCamera> AroundCameras;         // UpdateAroundCameras: the camera turned to the other headings
    double FlightStartCameraPitchDeg = -90.0; // the Details values, restored at every flight start
    double FlightStartCameraYawDeg = 0.0;

    FDroneTelemetryOutput Telemetry;
    FDroneVideoOutput Video;
    FDroneRtspServer RtspServer;
    FDroneTileStore TileStore;
    FDroneWayback Wayback;
    FDronePerfMonitor PerfMonitor;
    double PerfSpeedMps = 0.0; // horizontal / FPV speed of the last tick, for the monitor
    FString StoredSourceName; // what the stored-imagery overlays show now

    // Overlays found on the tilesets at the first ApplyImagerySource (Bing), switched
    // off while the stored imagery is used, and the stored-imagery overlays made for them.
    TArray<TWeakObjectPtr<UCesiumRasterOverlay>> LevelOverlays;
    bool bLevelOverlaysFound = false;
    UPROPERTY(Transient)
    TArray<TObjectPtr<UCesiumUrlTemplateRasterOverlay>> StoredImageryOverlays;
    bool bStoredImageryActive = false;
    FString ImageryProblem;
    FString SessionDir; // Saved/Flights/<date-time>
    FString StatusNote; // last problem, shown under the status line

    TSharedPtr<SDroneSettingsPanel> SettingsPanel;
    TSharedPtr<SWidget> StatusLine;
    TSharedPtr<SWidget> Compass;
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

    // FPV mode
    FpvModel::State FpvState;
    FpvBuild::PhysState FpvPhys;         // motors, battery, PID state of the built drone
    FpvBuild::Physical FpvPhysical;      // derived from the parts in EnterFpv
    double FpvThrottle = 0.0;            // 0..1
    double FpvKeyStick[3];               // keyboard roll, pitch, yaw stick positions (set in EnterFpv)
    bool bFpvGamepadThrottle = false;    // the gamepad left stick sets the throttle once it was moved
    bool bFpvCrashed = false;
    bool bFpvPlaceOnGround = false;      // put the drone on the ground in the next step

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
    // SimTimeS when the current video started (video time starts at 0).
    double VideoStartSimTimeS = 0.0;
    // Record mode: false while waiting for R (no clip being written).
    bool bRecordingClip = false;
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
