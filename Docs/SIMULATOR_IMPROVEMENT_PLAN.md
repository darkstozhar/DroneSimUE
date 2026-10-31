# DroneSimUE — plan for maximum benchmark accuracy & performance

Date: 2026-10-25. Basis: read-only audit of `Source/DroneSimUE/Sim/` (UE 5.8 + Cesium for Unreal) on the same day.
Companion plan: `DroneLocalization/docs/LOCALIZER_IMPROVEMENT_PLAN.md`.

## Purpose

DroneSimUE has two jobs for DroneLocalization:
- (1) an honest ground-truth benchmark: exact poses, a controllable domain gap, no "map == query" optimism;
- (2) a live real-flight stand-in: RTSP video + MAVLink telemetry at real timing.

Its accuracy goal is that its own ground-truth error stays well below 1 m, i.e. negligible against the localizer's 10 m target. Its performance goal is Record mode faster than real time with zero dropped frames.

The user's rule applies: **the simulator adapts to the localizer, not the reverse.** DroneSimUE must emit the existing DroneLocalization contract. Keyframes are selected with DroneLocalization's own algorithm, as FlightSimulator does.

## Status (2026-10-29)

- S0.1 done in code, not yet compiled: `ADroneRouteFlight::TraceImageGround` writes `ground_points.csv` (Record mode, every video frame, 3×3 grid at 20/50/80 %, terrain-only trace like the AGL trace). Originals of the 4 edited files: `WIP/backup-2026-10-29-s0/`.
- S0.2 + S0.3 done: `Tools/export_dronelocalization.py` → `flight.frames.jsonl`, `flight.keyframes.json`, `calibration.json` v2.4, `ground_truth.json` gt-2.0, `flight.manifest.json`; intrinsics in the jsonl and manifest. Tested on a synthetic nadir flight (affine LSQ rmse 0.001 m, scale and heading as expected); not yet on a real recording.
- S0.4 pending: needs a real Record-mode flight.

## S0 — Benchmark contract (blocking; nothing in DroneLocalization can score DroneSimUE without it)

Today DroneSimUE writes `flight.mp4`, `frames.csv` (pose per video frame) and `telemetry.csv` (pose per engine tick). It writes no `calibration.json` v2.4, no `ground_truth.json` gt-2.0, no `video.frames.jsonl`, no manifest and no version field, and it has no `frame_step`.

- **S0.1 Ground-projection logging in C++ (record time).** On every frame with `video_frame % frame_step == 0` (`frame_step` = 30, a setting that must equal DroneLocalization's `database.frame_step`), cast rays from the camera through a fixed pixel grid against the rendered Cesium terrain and log the hit lat/lon/height for each pixel. Use at least the 5 points FlightSimulator uses (centre + 20/80% corners); a 3×3 or 5×5 grid is better.
  - This gives anchors from the exact geometry that was rendered. It avoids the DEM mismatch you would get by re-projecting through AWS Terrarium in Python.
- **S0.2 Python post-processor `Tools/export_dronelocalization.py`.** Reads a `Flights/<ts>/` folder and writes:
  - `ground_truth.json` (gt-2.0, one record per DB slot, alt_msl = world_z + base_elevation_m contract);
  - `video.frames.jsonl`;
  - `calibration.json` v2.4: anchors as an LSQ 2×3 affine from the S0.1 points, plus `qa_data.rmse_m`, `keyframe_selection` and `video_sha256`;
  - `manifest.json` v2 (checksums, git rev, `frame_step` check).
  
  Run keyframe selection by invoking DroneLocalization's own selector from its `.venv`, the same way FlightSimulator does. Do not reimplement it.
- **S0.3 Explicit intrinsics.** Write K to the outputs: fx = fy = (W/2)/tan(HFOV/2) ≈ 1920 px at 1280 px and HFOV 36.87°, cx = 640, cy = 360. Also write an equivalent `focal_mm`/`sensor_width_mm` pair, because `GSDCalculator` uses those.
- **S0.4 Contract test.** Run one short nadir flight with imagery=1 through `validate_vs_ground_truth.py`. p95 should be in the same range as a FlightSimulator run over the same area. If it is not, the exporter is wrong, not the localizer.

Format rule: if a C++ writer later replaces the Python exporter, any change to the format updates both projects and bumps the version field in both together.

## S1 — Ground-truth correctness

- **S1.1 Geoid.** `GeoidUndulationM` defaults to 0, so `alt_msl_m` equals ellipsoidal height (763.133 m in the sample). Add an EGM2008 (or EGM96) undulation lookup at the mission origin, stored in settings and written to the outputs.
  - The magnitude near Chernivtsi (estimated ~30+ m) is unverified; take the value from the model, not from memory.
  - AGL is unaffected because it comes from a line trace.
- **S1.2 Capture/pose timing.** Verify that the pose in `frames.csv` belongs to the same engine tick as the captured image, given the async GPU readback. Test: fly a fast yaw or translation over a high-contrast marker and compare the projected marker position with the pose.
  - Stamp each readback with its source frame number.
  - In Record mode (`FixedFrameRate`=30) one tick equals one frame, so any offset must be exactly 0.
- **S1.3 Zero dropped frames in Record.** Treat `dropped > 0` as a failed recording and mark it in the manifest. Use a readback ring buffer deep enough for NVENC stalls, and block the game thread rather than drop frames in Record (blocking is fine because time is fixed-step).
- **S1.4 Tiles fully loaded before each frame.** In Record mode, wait until the Cesium tilesets report loaded for the current view before capturing. `forbid_holes` and preloading exist; add an explicit per-frame check, or the frame contains low-LOD tiles that never appear in reality.
- **S1.5 Determinism.** Seed every random source (wind, noise, jitter) from the scenario file, so the same scenario reproduces the same video bit-for-bit (or same hash of decoded frames).

## S2 — Realism and domain-gap controls (what makes the localizer's numbers honest)

Each control is a scenario parameter, so the localizer's benchmark matrix (plan item 0.1) can sweep it.

- **S2.1 Imagery epoch gap.**
  - The reference DB is built with one source and epoch; queries use a different Wayback release or Bing.
  - Default benchmark: query epoch ≠ reference epoch.
  - The same-epoch run is kept only as an upper bound.
- **S2.2 3D content (optional mode).** Google Photorealistic 3D Tiles (buildings, trees, real occlusion and parallax). Check licence terms and API cost before use.
  - Keep the 2.5D draped mode as the default for farmland.
  - Use 3D tiles as the stress test for towns.
- **S2.3 Camera realism.**
  - Brown–Conrady lens distortion (as a post-process, or by rendering a wider FOV and remapping), with the coefficients written into the outputs.
  - Sensor noise, motion blur from the shutter time.
  - Rolling shutter, optional.
  - Auto-exposure is already in place (EV −0.5).
  - Configurable NVENC bitrate and GOP to emulate a weak FPV/OpenIPC link.
- **S2.4 Atmosphere and lighting.** Sun time and date are already in place. Add:
  - haze and exponential height fog;
  - cloud shadows (volumetric clouds);
  - low sun angles.
  
  These are the conditions that break VPR.
- **S2.5 Gimbal and attitude.**
  - Sweep gimbal pitch −90 to −60° and add gimbal jitter.
  - Add wind and turbulence to the kinematic modes, so attitude is not perfectly smooth. Only a simple stochastic disturbance on velocity and attitude is needed; a full rotor model is not required for vision.
- **S2.6 Sensor telemetry realism.** Add noise, bias and rate to telemetry: GPS (or none), barometric altitude drift, heading error, plus MAVLink message rates as on the user's real quad (OpenIPC + MAVLink build). Write the clean values and the noisy values separately: clean is ground truth, noisy is the localizer input.

## S3 — Scenario automation and data generation

- **S3.1 Headless batch runner.** Command line `-Scenario=<json> -RenderOffscreen` (or an equivalent console command). It flies the mission, records, runs the S0.2 exporter and exits with a status code.
  - Scenario JSON fields: route/polygon, altitude, speed, gimbal, imagery source/epoch, sun, weather, noise seeds.
- **S3.2 Benchmark matrix generator.** A script that expands one base mission into the domain-gap sweep (S2.1 × S2.4 × S2.5 × altitude ladder 500/1000/2000 m × AGL vs constant-baro) and queues the runs.
- **S3.3 Reference (DB) flights.** A lawnmower mode at fixed layer altitudes with the overlap and sidelap the DB builder expects, so DroneSimUE can also produce the multi-layer reference video.
  - This is a candidate answer to the localizer plan's phase 4 question: how to cover a 1000 km corridor.
  - Long corridors need the georeference origin re-centred per segment (already done on teleport) and streaming that keeps up.

## S4 — Simulator performance

- **S4.1 Record faster than real time.** With fixed-step time the engine can run as fast as the GPU allows. Make the capture/readback/NVENC chain the only limit:
  - viewport small or off-screen;
  - no editor UI;
  - preload the whole route corridor (+1 km) before the first frame.
  
  Measure frames per wall-clock second and report it in the manifest.
- **S4.2 Tile cache.**
  - Persistent disk cache for Esri, Wayback and Bing tiles (the tile store on :18765 and the sqlite cache exist).
  - A "prefetch corridor" tool, so batch runs never hit the network mid-flight.
  - Report cache hit rate per run.
- **S4.3 Live mode latency.** For the RTSP stand-in:
  - NVENC low-latency preset, short GOP, no B-frames;
  - measure glass-to-glass latency (frame tick → decoded at the localizer) and stamp it into MAVLink time sync, so the localizer's telemetry/frame alignment matches a real link.
- **S4.4 GPU budget on Stozhar.** Lumen + Cesium + NVENC on the same RTX 5070 Ti as the localizer. In live closed loop, measure the localizer's latency with the simulator running, because GPU contention distorts the localizer's performance numbers. If needed, cap simulator quality in live mode or run the localizer on the other machine.

## S5 — Closed loop (final integration)

Run DroneLocalization live on RTSP + MAVLink from DroneSimUE, write its estimates back into the sim (UDP), and draw the estimated vs true trajectory in-engine. Score with the same scorecard as offline runs. This is also the demo for the internship ("Професійне стажування", period ending 09.11.2026).

## Order and dependencies

S0.1–S0.4 + S1.1–S1.3 first. These are what the localizer's phase 0 depends on. Then S1.4–S1.5 → S3.1 → S2.1 / S2.5 / S2.3 (highest value for honest numbers) → S3.2 → S4.x → S2.2 / S2.4 / S2.6 → S3.3 → S5.

## Risks

- **Synthetic ground truth can still be self-consistent but biased.** If S0.1 anchors and the rendered image share a terrain error, the benchmark cannot see it. Keep one real external dataset in the localizer's final gate.
- **Unchecked engine details.** The Cesium for Unreal plugin version and the contents of `Main.umap` were not inspected. The S1.4 tile-loaded check depends on which API that plugin version exposes.
- **Readback latency is unmeasured (S1.2).** At 15 m/s, a one-frame offset is 0.5 m, which is negligible. Under FPV yaw rates, the angular error matters more.
