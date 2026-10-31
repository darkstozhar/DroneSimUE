"""Export a DroneSimUE flight folder in DroneLocalization's benchmark contract.

Input: Saved/Flights/<ts>/ with flight.mp4, frames.csv and ground_points.csv
(ground_points.csv is written by ADroneRouteFlight::TraceImageGround in Record mode).

Output, next to flight.mp4:
  flight.frames.jsonl     frame-gt-1.0, one record per video frame (replay GT)
  flight.keyframes.json   DroneLocalization's own image keyframe selection
  calibration.json        v2.4 anchors on selected keyframes
  ground_truth.json       gt-2.0, one record per DB slot
  flight.manifest.json    checksums, counts, validation result

Anchor placement and the JSON layouts are FlightSimulator's CalibrationLogger,
imported, not reimplemented; keyframes come from DroneLocalization's selector
(FlightSimulator's localizer_keyframe_cli). Ground points are the simulator's
own ray hits on the rendered terrain, so no DEM is re-projected here.

Run with DroneLocalization's Python:
  D:\\My Projects\\DroneLocalization\\.venv\\Scripts\\python.exe Tools\\export_dronelocalization.py <flight dir>
"""

from __future__ import annotations

import argparse
import csv
import datetime
import hashlib
import json
import math
import os
import subprocess
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace

import numpy as np

EXPORTER_VERSION = "dronesimue-export-1"
R_EARTH = 6378137.0  # EPSG:3857
N_POINTS = 9


def mercator(lat: float, lon: float) -> tuple[float, float]:
    x = R_EARTH * math.radians(lon)
    y = R_EARTH * math.log(math.tan(math.pi / 4.0 + math.radians(lat) / 2.0))
    return x, y


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def write_atomic(path: Path, text: str) -> None:
    fd, tmp = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.", suffix=".part")
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def read_frames(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    out = []
    for i, r in enumerate(rows):
        if int(r["video_frame"]) != i:
            raise ValueError(f"frames.csv is not sequential at row {i}: video_frame={r['video_frame']}")
        out.append(r)
    return out


def read_ground(path: Path) -> dict[int, list[dict | None]]:
    """video_frame -> 9 entries (None where the ray missed)."""
    pts: dict[int, list[dict | None]] = {}
    with path.open(encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            vf, p = int(r["video_frame"]), int(r["point"])
            slot = pts.setdefault(vf, [None] * N_POINTS)
            if r["hit"] == "1":
                lat, lon = float(r["lat"]), float(r["lon"])
                slot[p] = {
                    "u": float(r["u_px"]), "v": float(r["v_px"]),
                    "lat": lat, "lon": lon, "h": float(r["h_ellipsoid_m"]),
                    "merc": mercator(lat, lon),
                }
    return pts


def video_frame_count(path: Path) -> tuple[int, int, int]:
    import cv2

    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {path}")
    n = 0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    while cap.grab():  # decoded count, not the container's estimate
        n += 1
    cap.release()
    return n, w, h


def fit_affine(px: np.ndarray, dst: np.ndarray) -> np.ndarray | None:
    n = len(px)
    A = np.zeros((2 * n, 6))
    b = np.zeros(2 * n)
    A[0::2, 0], A[0::2, 1], A[0::2, 2] = px[:, 0], px[:, 1], 1.0
    A[1::2, 3], A[1::2, 4], A[1::2, 5] = px[:, 0], px[:, 1], 1.0
    b[0::2], b[1::2] = dst[:, 0], dst[:, 1]
    sol, _, rank, _ = np.linalg.lstsq(A, b, rcond=None)
    return None if rank < 6 else sol.reshape(2, 3)


def run_selector(loc_root: Path, loc_python: Path, fs_root: Path, *args: str) -> str:
    env = os.environ.copy()
    env["PYTHONUTF8"] = "1"
    env["DRONE_LOCALIZATION_ROOT"] = str(loc_root)
    env["PYTHONPATH"] = os.pathsep.join(filter(None, (str(fs_root), env.get("PYTHONPATH", ""))))
    proc = subprocess.run(
        [str(loc_python), "-m", "simulator.localizer_keyframe_cli", *args],
        cwd=loc_root, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        text=True, encoding="utf-8", errors="replace",
    )
    if proc.returncode:
        raise RuntimeError(f"keyframe selector failed ({proc.returncode}): {proc.stderr[-3000:]}")
    return proc.stdout


def main(argv: list[str] | None = None) -> int:
    here = Path(__file__).resolve()
    projects = here.parents[2]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("flight_dir", type=Path)
    ap.add_argument("--localizer-root", type=Path, default=projects / "DroneLocalization")
    ap.add_argument("--localizer-python", type=Path, default=None)
    ap.add_argument("--flightsim-root", type=Path, default=projects / "FlightSimulator")
    ap.add_argument("--geometric-keyframes", action="store_true",
                    help="skip the image selector and use the geometric prediction (debug only)")
    args = ap.parse_args(argv)

    d = args.flight_dir.resolve()
    video, frames_csv, ground_csv = d / "flight.mp4", d / "frames.csv", d / "ground_points.csv"
    for p in (video, frames_csv, ground_csv):
        if not p.is_file():
            raise SystemExit(f"missing {p}")
    loc_root = args.localizer_root.resolve()
    loc_python = (args.localizer_python or loc_root / (
        ".venv/Scripts/python.exe" if os.name == "nt" else ".venv/bin/python")).resolve()
    fs_root = args.flightsim_root.resolve()
    sys.path.insert(0, str(fs_root))
    from simulator.physics.calibration_logger import CalibrationLogger  # noqa: E402

    errors: list[str] = []
    frames = read_frames(frames_csv)
    ground = read_ground(ground_csv)
    n_video, width, height = video_frame_count(video)
    print(f"video: {n_video} frames {width}x{height}; frames.csv: {len(frames)}; ground rows for {len(ground)} frames")
    if n_video != len(frames):
        errors.append(f"video has {n_video} frames, frames.csv {len(frames)}")
    n = min(n_video, len(frames))

    # Selection settings come from DroneLocalization's effective config.
    sel_cfg = json.loads(run_selector(loc_root, loc_python, fs_root, "--print-config").strip().splitlines()[-1])
    frame_step = int(sel_cfg["frame_step"])
    fps = 1.0 / float(frames[1]["time_s"]) if n > 1 and float(frames[1]["time_s"]) > 0 else 30.0
    fps = round(fps, 3)

    # ── per-frame GT (frame-gt-1.0) ───────────────────────────────────────
    hfov = float(frames[0]["hfov_deg"])
    fx = 0.5 * width / math.tan(math.radians(hfov) / 2.0)
    sensor_w_mm = 36.0  # nominal; focal_mm chosen so focal_mm*W/sensor = fx
    focal_mm = fx * sensor_w_mm / width
    intr = {"fx_px": fx, "fy_px": fx, "cx_px": width / 2.0, "cy_px": height / 2.0,
            "focal_length_mm": focal_mm, "sensor_width_mm": sensor_w_mm, "hfov_deg": hfov}
    lines = []
    n_valid = 0
    for i in range(n):
        r = frames[i]
        g = ground.get(i, [None] * N_POINTS)
        cam_xy = mercator(float(r["lat"]), float(r["lon"]))
        rec = {
            "version": "frame-gt-1.0",
            "frame_index": i,
            "timestamp_s": float(r["time_s"]),
            "frame_size": [width, height],
            "camera_position_world": [cam_xy[0], cam_xy[1], float(r["alt_ellipsoid_m"])],
            "camera_gps": [float(r["lat"]), float(r["lon"])],
            "camera_altitude_msl_m": float(r["alt_msl_m"]),
            "camera_altitude_ellipsoid_m": float(r["alt_ellipsoid_m"]),
            "heading_deg": float(r["heading_deg"]),
            "camera_pitch_deg": float(r["camera_pitch_deg"]),
            "camera_yaw_deg": float(r["camera_yaw_deg"]),
            "pitch_deg": float(r["pitch_deg"]),
            "roll_deg": float(r["roll_deg"]),
            "surface_valid_fraction": sum(p is not None for p in g) / N_POINTS,
            "renderer": "DroneSimUE/Cesium",
            "coordinate_system": {
                "world_xy": "EPSG:3857 metres",
                "world_z": "WGS84 ellipsoid height, metres",
                "vertical_datum": "alt_msl = ellipsoid - configured geoid undulation (DroneSimUE setting)",
            },
            "camera_intrinsics": intr,
            "ground_center_valid": False,
        }
        if r.get("alt_agl_m"):
            rec["camera_agl_m"] = float(r["alt_agl_m"])
        c = g[0]
        if c is not None:
            rec.update(
                ground_center_valid=True,
                ground_center_mercator_m=list(c["merc"]),
                ground_center_gps=[c["lat"], c["lon"]],
                ground_center_height_ellipsoid_m=c["h"],
            )
            n_valid += 1
        else:
            rec["invalid_reason"] = "centre ray missed the terrain" if i in ground else "no ground_points row"
        lines.append(json.dumps(rec, ensure_ascii=False, separators=(",", ":")))
    write_atomic(d / "flight.frames.jsonl", "\n".join(lines) + "\n")
    print(f"flight.frames.jsonl: {n} records, {n_valid} with a valid ground centre")
    if n_valid < n:
        errors.append(f"{n - n_valid} frames without a ground centre")

    # ── slot candidates for CalibrationLogger ─────────────────────────────
    stub_map = SimpleNamespace(elevation=None, base_elevation=0.0, _center_x=0.0, _center_y=0.0)
    logger = CalibrationLogger(
        output_file=str(d / "calibration.json"),
        ortho_map=stub_map,
        frame_step=frame_step,
        keyframe_criterion=str(sel_cfg["criterion"]),
        keyframe_max_overlap=float(sel_cfg["max_overlap"]),
        keyframe_max_gap_frames=int(sel_cfg["max_gap_frames"]),
        generator_metadata={
            "name": "DroneSimUE",
            "exporter": EXPORTER_VERSION,
            "flight_dir": d.name,
            "ground_points": "image-ray hits on Cesium terrain, 3x3 grid at 20/50/80 %",
        },
    )
    logger._frame_size = (width, height)
    skipped = 0
    centers = []
    for i in range(0, n, frame_step):
        g = ground.get(i)
        if g is None or any(p is None for p in g):
            skipped += 1
            continue
        r = frames[i]
        px = np.array([[p["u"], p["v"]] for p in g])
        merc = np.array([p["merc"] for p in g])
        M = fit_affine(px, merc)
        if M is None or M[0, 0] * M[1, 1] - M[0, 1] * M[1, 0] >= 0:
            skipped += 1
            continue
        errs = np.linalg.norm((M[:, :2] @ px.T).T + M[:, 2] - merc, axis=1)
        vn, ve = float(r.get("vn_mps") or 0.0), float(r.get("ve_mps") or 0.0)
        compass = float(r["heading_deg"]) + float(r["camera_yaw_deg"])
        yaw_math = math.radians(90.0 - compass)
        heading = math.atan2(vn, ve) if math.hypot(vn, ve) >= logger.min_speed else yaw_math
        cam_xy = mercator(float(r["lat"]), float(r["lon"]))
        centers.append(merc[0])
        logger._candidates.append({
            "slot": i // frame_step,
            "video_frame": i,
            "heading": heading,
            "yaw_deg": math.degrees(yaw_math),
            "alt": float(r["alt_rel_m"]),
            "alt_msl": float(r["alt_msl_m"]),
            "camera_agl": float(r["alt_agl_m"]) if r.get("alt_agl_m") else float(r["alt_ellipsoid_m"]) - g[0]["h"],
            "camera_position": [cam_xy[0], cam_xy[1], float(r["alt_ellipsoid_m"])],
            "camera_orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "timestamp": float(r["time_s"]),
            "ground_center_local": [float(merc[0, 0]), float(merc[0, 1])],
            "ground_center_mercator": [float(merc[0, 0]), float(merc[0, 1])],
            "ground_center_gps": [g[0]["lat"], g[0]["lon"]],
            "surface_valid_fraction": 1.0,
            "M": M,
            "rmse": float(np.sqrt(np.mean(errs ** 2))),
            "median": float(np.median(errs)),
            "max": float(np.max(errs)),
            "pts_px": px.tolist(),
            "pts_gps": [[p["lat"], p["lon"]] for p in g],
            "pts_mercator": merc.tolist(),
        })
    if not logger._candidates:
        raise SystemExit("no slot has all 9 ground hits — cannot calibrate")
    c = np.mean(centers, axis=0)
    stub_map._center_x, stub_map._center_y = float(c[0]), float(c[1])
    if skipped:
        errors.append(f"{skipped} DB slots without full ground geometry")
    rmses = [cand["rmse"] for cand in logger._candidates]
    print(f"slot candidates: {len(logger._candidates)} (skipped {skipped}); "
          f"affine LSQ rmse median {np.median(rmses):.3f} m, max {np.max(rmses):.3f} m")

    # ── keyframes from DroneLocalization's selector ───────────────────────
    kf_path = d / "flight.keyframes.json"
    video_sha = sha256_file(video)
    if not args.geometric_keyframes:
        run_selector(loc_root, loc_python, fs_root, "--video", str(video), "--output", str(kf_path))
        kf = json.loads(kf_path.read_text(encoding="utf-8"))
        if kf["frame_step"] != frame_step or kf["source_total_frames"] != n_video or kf["video_sha256"] != video_sha:
            raise SystemExit("keyframe sidecar does not match this video/config")
        usable = sorted(set(kf["selected_slots"]) - set(kf.get("featureless_selected_slots", [])))
        logger.set_exact_keyframe_slots(usable, provenance={"sidecar": kf_path.name, "video_sha256": video_sha})
        print(f"image selector kept {len(kf['selected_slots'])}/{kf['total_slots']} slots ({len(usable)} with features)")
    else:
        errors.append("geometric keyframe prediction used (not the DB selector)")

    logger.close()
    gt_path = d / "ground_truth.json"
    logger.dump_ground_truth(str(gt_path), fps=fps)

    # The logger's coordinate text describes FlightSimulator's local world.
    gt = json.loads(gt_path.read_text(encoding="utf-8"))
    gt["coordinate_contract"] = {
        "world_xy": "EPSG:3857 metres (camera_position_world)",
        "world_z": "WGS84 ellipsoid height, metres; alt_msl = ellipsoid - geoid undulation set in DroneSimUE",
        "ground_center": "direct centre-ray hit on the rendered Cesium terrain",
        "affine": "nine-point least-squares compatibility approximation",
    }
    write_atomic(gt_path, json.dumps(gt, indent=2, ensure_ascii=False))
    cal = json.loads((d / "calibration.json").read_text(encoding="utf-8"))
    cal["video_sha256"] = video_sha
    write_atomic(d / "calibration.json", json.dumps(cal, indent=2, ensure_ascii=False))

    # ── manifest ──────────────────────────────────────────────────────────
    files = {}
    for name in ("flight.mp4", "frames.csv", "ground_points.csv", "telemetry.csv", "flight.frames.jsonl",
                 "flight.keyframes.json", "calibration.json", "ground_truth.json"):
        p = d / name
        if p.is_file():
            files[name] = {"sha256": video_sha if name == "flight.mp4" else sha256_file(p), "bytes": p.stat().st_size}
    manifest = {
        "version": 2,
        "generator": "DroneSimUE",
        "exporter": EXPORTER_VERSION,
        "created_at": datetime.datetime.now().isoformat(timespec="seconds"),
        "status": "complete" if not errors else "complete_with_warnings",
        "validation_errors": errors,
        "frame_count": n_video,
        "frame_size": [width, height],
        "fps": fps,
        "frame_step": frame_step,
        "keyframe_selection": sel_cfg,
        "anchors": len(cal.get("anchors", [])),
        "slot_candidates": len(logger._candidates),
        "camera_intrinsics": intr,
        "files": files,
    }
    write_atomic(d / "flight.manifest.json", json.dumps(manifest, indent=2, ensure_ascii=False))
    print(f"manifest: {manifest['status']}" + (f" — {'; '.join(errors)}" if errors else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
