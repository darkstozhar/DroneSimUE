// Game-style menu (a main menu before the flight, a menu during it), the map
// library and the status line, built with Slate so they need no assets and
// work the same in the editor and in a packaged build. Both are drawn on the
// viewport only, never in the video.
#pragma once

#include "CoreMinimal.h"
#include "Templates/Function.h"
#include "Widgets/DeclarativeSyntaxSupport.h"
#include "Widgets/SCompoundWidget.h"
#include "Sim/DroneWayback.h"
#include "Sim/FpvBuild.h"
#include "UObject/WeakObjectPtrTemplates.h"

class ADronePreview;
struct FSlateBrush;

class SVerticalBox;

// Everything the menu edits. Saved to Saved/DroneSimSettings.json.
struct FDroneSimSettings
{
    // --- Current map (also saved as Saved/Maps/<MapName>.json) ---------------
    FString MapName;
    // Start point: the flight starts here in manual mode.
    double StartLatitude = 48.365;
    double StartLongitude = 26.065;
    double AltitudeM = 500.0;     // meaning follows the drone's AltitudeMode
    double StartHeadingDeg = 0.0;
    // Mission: survey grid over a polygon, like Mission Planner's Survey (Grid).
    FString MissionPolygon;              // "lat, lon; lat, lon; ..."
    double MissionAltitudeM = 500.0;     // meaning follows the drone's AltitudeMode
    double MissionSpeedMps = 15.0;
    bool bMissionAutoAngle = true;       // legs along the longest edge
    double MissionAngleDeg = 0.0;        // compass direction of the legs otherwise
    bool bMissionUsePassCount = false;   // false: spacing from the side overlap
    int32 MissionPassCount = 6;
    double MissionSideOverlapPct = 30.0;
    double MissionOvershootM = 0.0;
    bool bMissionFixedHeading = false;   // false: camera heading along the legs
    double MissionHeadingDeg = 0.0;
    int32 MissionAfter = 0;              // 0 = hover at the end, 1 = return to launch, 2 = repeat
    bool bMissionJumpToStart = false;    // false: fly from the current position
    double MissionSquareSizeM = 2000.0;  // "square around the start point" helper

    double SpeedMps = 15.0; // default route (M without a mission)
    bool bManual = true;

    // --- Manual control sensitivity -------------------------------------------
    double ManualMaxSpeedMps = 15.0;
    double ManualAccelerationMps2 = 8.0;
    double ManualBrakingMps2 = 12.0;
    double ManualFastMultiplier = 2.0;
    double ManualYawRateDegPerSec = 60.0;
    double ManualClimbRateMps = 10.0;
    double ManualVerticalAccelerationMps2 = 10.0;
    double StickDeadZone = 0.12; // gamepad / RC sticks
    double StickExpo = 0.0;      // 0 = linear, 1 = cubic (finer near the centre)
    double CameraTiltRateDegPerSec = 30.0; // Up / Down tilt, Q / E pan (gamepad D-pad)

    // --- FPV (acro): see ADroneRouteFlight's Drone|FPV properties ----------------
    double FpvThrustToWeight = 6.0;
    double FpvRollPitchCenterRateDps = 70.0;
    double FpvRollPitchMaxRateDps = 670.0;
    double FpvRollPitchExpo = 0.54;
    double FpvYawCenterRateDps = 70.0;
    double FpvYawMaxRateDps = 670.0;
    double FpvYawExpo = 0.54;
    double FpvCameraUptiltDeg = 30.0;
    double FpvCameraFovDeg = 110.0;
    double FpvKeyboardStick = 0.5;
    double FpvThrottleKeyRate = 0.6;
    double FpvCrashSpeedMps = 5.0;
    // Drone builder: 1 = physical model from the parts (FpvBuild.h), 0 = simple model.
    int32 FpvUseBuild = 1;
    FString FpvDroneName = TEXT("5\" freestyle 4S"); // a ready-made drone (FpvBuild::kPresets) or one in Saved/Drones
    int32 FpvFrame = 2;
    int32 FpvMotor = 2;
    int32 FpvProp = 2;
    int32 FpvBattery = 1;
    int32 FpvCameraSystem = 1;
    double FpvPidMaster = 1.0;
    double FpvPidDamping = 1.0;
    // Throttle curve (Betaflight thr_mid / thr_expo / throttle limit) and the rates profile name.
    double FpvThrottleMid = 0.5;
    double FpvThrottleExpo = 0.0;
    double FpvThrottleLimitPct = 100.0;
    FString FpvRatesName = TEXT("Medium");
    // Video camera as the localizer gets it (flight.mp4, frames.csv hfov_deg, the DroneLocalization export).
    int32 CamWidthPx = 1280;
    int32 CamHeightPx = 720;
    int32 CamFps = 30;
    double CamSurveyHFovDeg = 36.87;

    // --- Camera --------------------------------------------------------------
    bool bCameraGimbal = true; // false = camera fixed to the body (tilts with it)
    double ExposureCompensationEV = -0.5;

    // --- Video -----------------------------------------------------------------
    int32 VideoMode = 0; // 0 = off, 1 = record flight.mp4, 2 = live RTSP
    FString FfmpegPath = TEXT("ffmpeg");
    FString VideoEncoder = TEXT("h264_nvenc");
    FString RtspUrl = TEXT("rtsp://127.0.0.1:8554/drone");
    bool bStartRtspServer = true;
    FString RtspServerPath;

    // --- Telemetry ---------------------------------------------------------------
    FString MavlinkHost = TEXT("127.0.0.1");
    int32 MavlinkPort = 14550;

    // --- Sun (Cesium SunSky) -------------------------------------------------------
    double TimeZone = 2.0;
    double SolarTime = 12.0;
    int32 Day = 21;
    int32 Month = 7;
    int32 Year = 2024;
    bool bDaylightSaving = true;

    // --- Area markers (poles at the vertices, see-through walls) --------------------
    bool bShowAreaPoles = true;
    bool bShowAreaWalls = true;
    double AreaWallOpacityPct = 20.0;
    bool bMarkersInVideo = false; // the localizer would see them too

    // --- Map loading (Cesium tilesets) -----------------------------------------------
    int32 MapQuality = 1;            // 0 normal, 1 high, 2 very high
    // World: 0 terrain + draped imagery (2.5D), 1 Google Photorealistic 3D Tiles (buildings, trees).
    int32 WorldMode = 0;
    bool bForbidHoles = true;        // keep a coarser tile until the finer ones are loaded
    bool bPreloadArea = true;        // load the map's area (+ margin) and keep it loaded
    double PreloadMarginM = 1000.0;
    int32 TileMemoryCacheMB = 4096;
    int32 MaxSimultaneousTileLoads = 64;
    // What turning would show (turned copies of the camera): 0 off, 1 the camera's detail, 2 half, 3 quarter.
    int32 AroundDroneDetail = 1;
    // 0 step by step (Cesium default: coarse, finer, final), 1 straight to the final detail.
    int32 TileLoadingStyle = 0;
    // Imagery: 0 Bing (Cesium ion, downloaded every run; the default), 1 Esri World Imagery kept on disk,
    // 2 Esri by date (Wayback release WaybackRelease, 0 = the current Esri imagery), kept on disk too.
    int32 ImagerySource = 0;
    int32 WaybackRelease = 0;
    FString WaybackLabel; // "taken 2015-10-25, ... (release 2019-04-03)" as picked
    int32 EsriMaxLevel = 19;
    FString TileStoreDir; // empty: <local app data>/DroneSimUE/TileStore
    int32 TileStorePort = 18765;

    // Runtime only (not saved): camera used for the mission footprint.
    double CameraHFovDeg = 36.87;
    double CameraAspect = 16.0 / 9.0;
    int32 ImageWidthPx = 1280;

    // Keys missing from the file keep the values already in the struct.
    bool LoadFromFile(const FString& Path);
    bool SaveToFile(const FString& Path) const;
};

// Map library: Saved/Maps/<name>.json, one file per map (start point + area +
// grid settings), easy to copy or share.
namespace DroneMaps
{
FString MapsDir();
FString FileNameFor(const FString& Name);
TArray<FString> ListMaps();
bool SaveMap(const FDroneSimSettings& S); // as S.MapName
// Reads only the map fields; MapName becomes the file name.
bool LoadMap(const FString& Name, FDroneSimSettings& InOut);
bool DeleteMap(const FString& Name);
void CopyMissionFields(const FDroneSimSettings& From, FDroneSimSettings& To);
// Map = name + start point + mission fields.
void CopyMapFields(const FDroneSimSettings& From, FDroneSimSettings& To);
void CopyControlFields(const FDroneSimSettings& From, FDroneSimSettings& To);
// "lat, lon" appended as a new line of the vertex list.
void AppendVertex(FString& VertexText, double Lat, double Lon);
int32 CountVertices(const FString& VertexText);
FString SquareAround(double Lat, double Lon, double SizeM);
// First run (no Maps folder yet): a few ready maps.
void EnsureDefaultMaps(const FDroneSimSettings& Current);
} // namespace DroneMaps

enum class EDroneMenuAction : uint8
{
    FreeFlight,    // main menu: start flying (manual) at the map's start point
    FpvFlight,     // main menu: FPV (acro) from the ground at the map's start point
    FlyMission,    // fly the map's mission (from the main menu this also starts the flight)
    PreviewMap,    // main menu: another map / start point, move the drone there
    ApplySettings, // settings saved: controls, camera, sun, markers (+ video, MAVLink in the main menu)
    Resume,
    StopMission,   // switch to manual (M resumes)
    MainMenu,      // end the flight
    Quit,
};

DECLARE_DELEGATE_TwoParams(FOnDroneMenuAction, EDroneMenuAction, const FDroneSimSettings&);

class SDroneSettingsPanel : public SCompoundWidget
{
public:
    virtual ~SDroneSettingsPanel() override;
    SLATE_BEGIN_ARGS(SDroneSettingsPanel) : _InFlight(false) {}
    SLATE_ARGUMENT(FDroneSimSettings, Initial)
    // false: the main menu before a flight; true: the menu during a flight.
    SLATE_ARGUMENT(bool, InFlight)
    // Current drone position (X = latitude, Y = longitude).
    SLATE_ARGUMENT(TFunction<FVector2D()>, DronePosition)
    // One or two lines about the drone (terrain loading, mode), polled.
    SLATE_ARGUMENT(TFunction<FText()>, Status)
    // Wayback versions at the map's area (newest first) and the search status, polled.
    SLATE_ARGUMENT(TFunction<const TArray<FDroneWaybackVersion>&()>, WaybackVersions)
    SLATE_ARGUMENT(TFunction<FString()>, WaybackStatus)
    SLATE_EVENT(FOnDroneMenuAction, OnAction)
    SLATE_END_ARGS()

    void Construct(const FArguments& InArgs);

    virtual bool SupportsKeyboardFocus() const override { return true; }

    // What the menu shows now, applied or not.
    FDroneSimSettings GetDraft();
    // Esc / F2 / Tab: from a sub page back to the first page (false if already there).
    bool GoBack();

private:
    enum EPage : int32
    {
        PageMain = 0,
        PagePause = 1,
        PageMaps = 2,
        PageSettings = 3,
        PageHelp = 4,
        PageDates = 5,
        PageBuilder = 6,
        PageRates = 7,
    };

    // Building blocks
    TSharedRef<SWidget> NumberRow(const FText& Label, double* Value, double Min, double Max, double Delta, int32 Digits);
    TSharedRef<SWidget> IntRow(const FText& Label, int32* Value, int32 Min, int32 Max);
    // OnChanged (optional) runs after a click changed the value.
    TSharedRef<SWidget> ChoiceRow(const FText& Label, int32* Value, const TArray<FText>& Options,
                                  TFunction<void()> OnChanged = nullptr);
    TSharedRef<SWidget> ImageryModeRow();
    TSharedRef<SWidget> WaybackRow();
    TSharedRef<SWidget> DatesPage();
    const TArray<FDroneWaybackVersion>& WaybackVersionList() const;
    int32 WaybackIndex() const; // in WaybackVersionList, -1 = the current imagery or not in the list
    FText WaybackChoiceText() const;
    void StepWayback(int32 Direction); // +1 older, -1 newer
    void PickWayback(int32 Release, const FString& Label);
    TSharedRef<SWidget> TextRow(const FText& Label, FString* Value);
    TSharedRef<SWidget> Header(const FText& Title);
    TSharedRef<SWidget> PageTitle(const FText& Title);
    TSharedRef<SWidget> Hint(const FText& Message);
    TSharedRef<SWidget> ActionButton(const FText& Label, TFunction<void()> OnClick);
    TSharedRef<SWidget> BigButton(const FText& Label, TFunction<void()> OnClick, TFunction<bool()> IsEnabledFn = nullptr);
    TSharedRef<SWidget> Card(const TSharedRef<SWidget>& Inner);
    TSharedRef<SWidget> MapInfo();

    // Pages and tabs
    TSharedRef<SWidget> MainPage();
    TSharedRef<SWidget> PausePage();
    TSharedRef<SWidget> MapsPage();
    TSharedRef<SWidget> MapEditor();
    TSharedRef<SWidget> SettingsPage();
    TSharedRef<SWidget> ControlsTab();
    TSharedRef<SWidget> BuildSection();
    // Drone builder page (main / pause menu): ready-made drones, my drones (Saved/Drones/*.json), the parts.
    TSharedRef<SWidget> BuilderPage();
    void EnterBuilder();
    void LeaveBuilder(bool bSave);
    void RefreshDroneList();
    void SelectPresetDrone(int32 Index);
    void SelectUserDrone(const FString& Entry);
    void SaveUserDrone();
    void DeleteUserDrone();
    TSharedPtr<SVerticalBox> DroneListBox;
    FString DroneNote;
    // 3D preview, stats and graphs of the drone being built.
    TSharedRef<SWidget> CameraSection();
    void RefreshBuildCache();
    TWeakObjectPtr<ADronePreview> Preview;
    TSharedPtr<FSlateBrush> PreviewBrush;
    FpvBuild::Curves BuildCurves;
    FpvBuild::Summary BuildSummary;
    int64 BuildKey = -1;
    // Rates and throttle page: ready-made profiles, my profiles (Saved/Rates/*.json), curves.
    TSharedRef<SWidget> RatesPage();
    void EnterRates();
    void LeaveRates(bool bSave);
    void RefreshRatesList();
    void SelectPresetRates(int32 Index);
    void SelectUserRates(const FString& Entry);
    void SaveUserRates();
    void DeleteUserRates();
    TSharedPtr<SVerticalBox> RatesListBox;
    FString RatesNote;
    TSharedRef<SWidget> VideoTab();
    TSharedRef<SWidget> SunTab();
    TSharedRef<SWidget> MarkersTab();
    TSharedRef<SWidget> MapLoadingTab();
    TSharedRef<SWidget> HelpPage();
    TSharedRef<SWidget> KeyRow(const FText& Keys, const FText& Action);

    void Fire(EDroneMenuAction Action);
    int32 RootPage() const;
    void OpenMaps();
    void LeaveMaps();
    void RefreshMapList();
    void SelectMap(const FString& MapEntry);
    void NewMap();
    void SaveCurrentMap();
    void DeleteCurrentMap();
    void EnterSettings();
    void LeaveSettings(bool bSave);
    void SyncChoices();
    void CommitChoices();
    FText MissionSummary();
    FText MapLine() const;
    void AddDronePositionVertex();
    void StartAtDronePosition();

    FDroneSimSettings Settings;
    FDroneSimSettings SettingsBackup; // restored by Cancel on the settings page
    bool bInFlight = false;
    TFunction<FVector2D()> DronePosition;
    TFunction<FText()> StatusFn;
    TFunction<const TArray<FDroneWaybackVersion>&()> WaybackVersionsFn;
    TFunction<FString()> WaybackStatusFn;
    FOnDroneMenuAction OnAction;
    int32 ActivePage = PageMain;
    int32 SettingsTab = 0;
    TSharedPtr<SVerticalBox> MapListBox;
    FString MapNote; // result of the last map save / delete / load

    // ChoiceRow works on int32; these mirror the bools / strings of Settings.
    int32 GimbalChoice = 0;
    int32 EncoderChoice = 0;
    int32 RtspServerChoice = 1;
    int32 DstChoice = 1;
    int32 MissionAngleChoice = 0;
    int32 MissionSpacingChoice = 0;
    int32 MissionHeadingChoice = 0;
    int32 MissionStartChoice = 0;
    int32 PolesChoice = 1;
    int32 WallsChoice = 1;
    int32 MarkersVideoChoice = 0;
    int32 ForbidHolesChoice = 1;
    int32 PreloadChoice = 1;
    bool bMissionPlanOk = false;
    bool bMissionAreaEmpty = true;
};

// Status line in the top-left corner (TopPadding leaves room for the compass); GetText is polled every frame.
TSharedRef<SWidget> MakeDroneStatusLine(TFunction<FText()> GetText, float TopPadding = 8.f);
