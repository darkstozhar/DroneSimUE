# DroneSimUE

Drone flight simulator on **Unreal Engine 5.8 + Cesium for Unreal**. It feeds
[DroneLocalization](../DroneLocalization) the way a real flight would:

- **Record** mode — fixed-step flight video `flight.mp4` + per-frame poses (`frames.csv`) and
  ground ray hits (`ground_points.csv`) for an exact ground-truth benchmark;
- **Live** mode — H.264 over RTSP + MAVLink telemetry over UDP, at real timing.

Plan and status: [`Docs/SIMULATOR_IMPROVEMENT_PLAN.md`](Docs/SIMULATOR_IMPROVEMENT_PLAN.md).

## Requirements

| What | Version / note |
|---|---|
| Unreal Engine | **5.8** (installed at `D:\Epic Games\UE_5.8`) |
| Cesium for Unreal | engine plugin from Fab / Epic Launcher (`CesiumForUnreal`) |
| Visual Studio 2022 | "Game development with C++" workload; `.vsconfig` lists the components (VS Installer → *Import configuration*) |
| Windows 10/11 x64 | the perf monitor links `pdh.lib` (Windows only) |
| mediamtx | only for Live RTSP; default path `C:\My programs\mediamtx_v1.21.2_windows_amd64\mediamtx.exe` (change it in F2 settings) |
| Python | DroneLocalization's `.venv` for `Tools/` scripts |

Engine plugins used: `CesiumForUnreal`, `GeoReferencing`, `SunPosition`, `ModelingToolsEditorMode` (editor only).
Engine modules: Sockets/Networking (MAVLink), RenderCore/RHI (video readback), Slate (UI), HTTP/HTTPServer (tile store).

Maps: editor startup `/Game/SimBlank/Levels/SimBlank`, game default `/Game/Main`.

## Project layout

```
Source/DroneSimUE/Sim/   all simulator code (ADroneRouteFlight is the main actor)
Config/                  Default*.ini
Content/                 maps and assets
Tools/                   export_dronelocalization.py, mavlink_listen.py, perf_report.py, fpv_build_bench.cpp
Docs/                    plans
WIP/                     backups of files before larger edits (not built)
Packaged/Windows/        packaged game output
```

Generated, not in git: `Binaries/`, `Intermediate/`, `DerivedDataCache/`, `Saved/`, `Packaged/`,
`cesium-request-cache.sqlite` (Cesium HTTP cache, can be deleted — it is re-downloaded).

## Build (editor)

**Option A — from the IDE.**
1. Right-click `DroneSimUE.uproject` → *Generate Visual Studio project files* (only after adding/removing source files).
2. Open `DroneSimUE.sln`, configuration **Development Editor | Win64**, build the `DroneSimUE` project.
3. Run (F5) or open `DroneSimUE.uproject`.

**Option B — command line** (close the editor first, or use Live Coding `Ctrl+Alt+F11` inside it):

```bat
"D:\Epic Games\UE_5.8\Engine\Build\BatchFiles\Build.bat" DroneSimUEEditor Win64 Development ^
  -Project="D:\My Projects\DroneSimUE\DroneSimUE.uproject" -WaitMutex
```

Regenerate project files from the command line:

```bat
"D:\Epic Games\UE_5.8\Engine\Build\BatchFiles\Build.bat" -projectfiles ^
  -project="D:\My Projects\DroneSimUE\DroneSimUE.uproject" -game -engine
```

If the editor says modules are out of date and won't rebuild: delete `Binaries/` and `Intermediate/`, regenerate, build again.

## Package (standalone .exe)

**From the editor:** *Platforms → Windows → Package Project*, choose `Packaged\` as the folder
(the result is `Packaged\Windows\DroneSimUE.exe`). *Platforms → Windows* also sets the build config
(Development for testing, Shipping for release).

**Command line** (same result, no editor needed):

```bat
"D:\Epic Games\UE_5.8\Engine\Build\BatchFiles\RunUAT.bat" BuildCookRun ^
  -project="D:\My Projects\DroneSimUE\DroneSimUE.uproject" ^
  -platform=Win64 -clientconfig=Development ^
  -build -cook -stage -pak -archive ^
  -archivedirectory="D:\My Projects\DroneSimUE\Packaged" ^
  -utf8output
```

- `-clientconfig=Shipping` for a release build (no console, no stats).
- Add `-clean` for a full rebuild + recook after engine/plugin updates.
- Make sure every map the game loads is in *Project Settings → Packaging → List of maps to include*
  (or cooked by `GameDefaultMap`), otherwise it is missing in the package.

The packaged build writes its own data to `Packaged\Windows\DroneSimUE\Saved\`
(settings, flights, perf runs, logs) — separate from the editor's `Saved\`.

## Running

Keys (flying, menu closed):

| Key | Action |
|---|---|
| F1 / F2 / Tab (Esc in package) | menu / settings panel |
| W A S D, arrows | move (Manual), sticks (FPV) |
| Space / Ctrl | up / down (throttle) |
| M | resume the route mission (after Manual/FPV) |
| F | toggle FPV (acro) |
| R | start/stop a Record clip; in FPV after a crash — pick the drone up |
| V | camera view |
| Gamepad | supported in Manual and FPV |

Flight modes: **Route** (autopilot), **Manual**, **FPV** (acro). Video modes: Off / Record / Live.

Outputs (relative to `Saved/`):

| Path | Content |
|---|---|
| `DroneSimSettings.json` | settings from the F2 panel; applied at start, override the actor's defaults |
| `Maps/*.json` | saved map/area presets |
| `Flights/<date-time>/` | `flight.mp4`, `frames.csv`, `telemetry.csv`, `ground_points.csv` |
| `Perf/<date-time>/` | performance run (`frames.csv` …) |
| `Logs/` | `DroneSimUE.log`, `mediamtx.log` |

Network defaults: MAVLink UDP **14550**, RTSP `rtsp://127.0.0.1:8554/drone`, local tile store HTTP **18765**.

## Tools

Run with DroneLocalization's Python:

```bat
set PY="D:\My Projects\DroneLocalization\.venv\Scripts\python.exe"

:: Flight folder -> DroneLocalization benchmark contract
:: (flight.frames.jsonl, flight.keyframes.json, calibration.json v2.4, ground_truth.json gt-2.0, flight.manifest.json)
%PY% Tools\export_dronelocalization.py Saved\Flights\<date-time>

:: Check the MAVLink link (needs pymavlink; not together with QGroundControl on the same port)
%PY% Tools\mavlink_listen.py --port 14550

:: Perf report (newest run; packaged build first, then editor); writes report.html
%PY% Tools\perf_report.py
```

The exporter imports FlightSimulator's `CalibrationLogger` and DroneLocalization's keyframe selector,
so both projects must sit next to this one (`D:\My Projects\`).
