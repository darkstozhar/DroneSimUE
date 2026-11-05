"""Report for a DroneSimUE performance run (Saved/Perf/<date-time>/).

    python Tools/perf_report.py            # newest run (Packaged build or editor)
    python Tools/perf_report.py <run dir>

Prints a diagnosis per flight mode and writes report.html (timeline charts +
hitch table) into the run folder. Standard library only.
"""
from __future__ import annotations

import bisect
import csv
import html
import statistics
import sys
from pathlib import Path

PROJECT = Path(__file__).resolve().parent.parent
# Packaged build first (Packaged/Windows/DroneSimUE/Saved/Perf), then the editor's Saved/Perf.
PERF_ROOTS = sorted(PROJECT.glob("Packaged/*/*/Saved/Perf")) + [PROJECT / "Saved" / "Perf"]
TICK_COLS = ["tick_input", "tick_view", "tick_around_cams", "tick_cesium_cams", "tick_markers",
             "tick_ground_trace", "tick_physics", "tick_move", "tick_telemetry_video"]


def load(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        rows = []
        for r in csv.DictReader(f):
            out = {}
            for k, v in r.items():
                try:
                    out[k] = float(v)
                except (TypeError, ValueError):
                    out[k] = v
            rows.append(out)
        return rows


def pct(values, p):
    if not values:
        return 0.0
    s = sorted(values)
    return s[min(len(s) - 1, max(0, round(p * (len(s) - 1))))]


def mean(values):
    vals = [v for v in values if isinstance(v, float) and v >= 0]
    return statistics.fmean(vals) if vals else -1.0


def nearest_sys(system, times, t):
    if not system:
        return {}
    i = bisect.bisect_left(times, t)
    cands = [j for j in (i - 1, i) if 0 <= j < len(system)]
    return system[min(cands, key=lambda j: abs(times[j] - t))]


def diagnose(mode, frames, hitches, system, sys_times):
    """Plain-language causes, strongest evidence first."""
    out = []
    if not frames:
        return out
    ft = [f["frame_ms"] for f in frames]
    p50 = pct(ft, 0.5)
    def avg(col):
        return mean([f[col] for f in frames])
    game, render, gpu = avg("game_ms"), avg("render_ms"), avg("gpu_ms")
    bound = max((game, "game thread (CPU, gameplay/Cesium on the game thread)"),
                (render, "render thread (CPU, draw-call/scene submission)"),
                (gpu, "GPU"))
    out.append(f"steady frame {p50:.1f} ms (~{1000 / max(p50, 1e-3):.0f} FPS), mostly limited by {bound[1]} "
               f"(avg game {game:.1f} / render {render:.1f} / GPU {gpu:.1f} ms)")
    if not hitches:
        out.append("no hitches")
        return out
    minutes = max(sum(ft) / 60000.0, 1e-3)
    out.append(f"{len(hitches)} hitches ({len(hitches) / minutes:.1f}/min), worst {max(h['frame_ms'] for h in hitches):.0f} ms")
    counts = {}
    for h in hitches:
        counts[h["bound"]] = counts.get(h["bound"], 0) + 1
    for b, n in sorted(counts.items(), key=lambda kv: -kv[1]):
        out.append(f"  {n} hitch(es) limited by {b}")

    # Evidence around the hitches vs. the mode's normal state.
    hs = [nearest_sys(system, sys_times, h["t_s"]) for h in hitches]
    ms = [nearest_sys(system, sys_times, f["t_s"]) for f in frames[:: max(1, len(frames) // 400)]]
    def cmp(col):
        return mean([s.get(col, -1.0) for s in hs]), mean([s.get(col, -1.0) for s in ms])
    gc = [h for h in hitches if h["gc_ms"] > 0.4 * h["frame_ms"]]
    if gc:
        out.append(f"  GC: {len(gc)} hitch(es) are mostly garbage collection (up to {max(h['gc_ms'] for h in gc):.0f} ms)")
    cams = mean([f.get("cesium_cameras", -1.0) for f in frames])
    if cams > 8 and game > 30:
        out.append(f"  Cesium cameras: ~{cams:.0f} extra views (preload + around the drone); Cesium picks tiles for each of "
                   f"them on the game thread every frame -> try preload_area off / around_drone_detail 0")
    tiles = [h for h in hitches if h["tiles_load_pct"] < 99.5]
    if len(tiles) >= 0.5 * len(hitches):
        out.append(f"  Cesium: {len(tiles)}/{len(hitches)} hitches happen while tiles are loading "
                   f"(new tiles are created on the game thread)")
    d_h, d_n = cmp("disk_busy_pct")
    r_h, r_n = cmp("process_io_read_mbps")
    if d_h >= 0 and (d_h > 70 or (d_h > d_n + 25)):
        out.append(f"  SSD: disk busy {d_h:.0f}% at hitches vs {d_n:.0f}% normally, this process reads {r_h:.1f} MB/s "
                   f"(tile cache / shader cache reads)")
    f_h, f_n = cmp("hard_faults_per_s")
    a_h, _ = cmp("ram_available_mb")
    if 0 <= a_h < 2000:
        out.append(f"  RAM: only {a_h:.0f} MB free at hitches ({f_h:.0f} hard page faults/s) -> memory pressure / paging")
    c_h, c_n = cmp("cpu_max_core_pct")
    if c_h > 95:
        out.append(f"  CPU: busiest core {c_h:.0f}% at hitches (vs {c_n:.0f}%) -> a single saturated thread")
    g_h, _ = cmp("gpu_3d_pct")
    if g_h > 95:
        out.append(f"  GPU: 3D engine {g_h:.0f}% busy at hitches -> GPU saturated")
    v_h, _ = cmp("vram_dedicated_mb")
    if v_h > 0:
        out.append(f"  VRAM in use at hitches: {v_h:.0f} MB (compare with the card's size; near full -> texture thrash)")
    shader_like = [h for h in hitches if h["bound"].startswith(("render", "RHI")) and h["tiles_load_pct"] >= 99.5]
    if len(shader_like) >= 3:
        out.append(f"  {len(shader_like)} render/RHI-bound hitches with no tile loading: typical of first-time "
                   f"shader/PSO compilation (should disappear on a second flight over the same place)")
    secs = {}
    for h in hitches:
        secs[h["slowest_tick_section"]] = secs.get(h["slowest_tick_section"], 0) + 1
    top = max(hitches, key=lambda h: h["slowest_tick_ms"])
    if top["slowest_tick_ms"] > 5:
        out.append(f"  drone Tick: '{top['slowest_tick_section']}' took up to {top['slowest_tick_ms']:.1f} ms in a hitch frame")
    return out


def svg_series(series, t0, t1, height=140, width=1100, ymax=None, marks=()):
    """series: [(label, colour, [(t, v), ...])] -> inline SVG line chart."""
    pts_all = [v for _, _, pts in series for _, v in pts if v >= 0]
    if not pts_all:
        return ""
    top = ymax or max(pts_all) * 1.05 or 1.0
    sx = lambda t: 40 + (t - t0) / max(t1 - t0, 1e-6) * (width - 50)
    sy = lambda v: height - 18 - min(v, top) / top * (height - 28)
    parts = [f'<svg viewBox="0 0 {width} {height}" width="100%">',
             f'<text x="2" y="12" font-size="10">{top:.0f}</text><text x="2" y="{height - 18}" font-size="10">0</text>']
    for t in marks:
        x = sx(t)
        parts.append(f'<line x1="{x:.1f}" x2="{x:.1f}" y1="8" y2="{height - 18}" stroke="#e33" stroke-opacity=".25"/>')
    for i, (label, colour, pts) in enumerate(series):
        step = max(1, len(pts) // 3000)
        d = " ".join(f"{sx(t):.1f},{sy(v):.1f}" for t, v in pts[::step] if v >= 0)
        parts.append(f'<polyline fill="none" stroke="{colour}" stroke-width="1" points="{d}"/>')
        parts.append(f'<text x="{60 + i * 170}" y="{height - 3}" font-size="11" fill="{colour}">{html.escape(label)}</text>')
    parts.append("</svg>")
    return "".join(parts)


def main():
    run = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    if run is None:
        runs = [p for root in PERF_ROOTS if root.exists() for p in root.iterdir()
                if p.is_dir() and (p / "frames.csv").exists()]
        if not runs:
            sys.exit("No runs in: " + ", ".join(str(r) for r in PERF_ROOTS))
        run = max(runs, key=lambda p: p.name)  # folder name = start date-time
    frames = load(run / "frames.csv")
    system = load(run / "system.csv")
    hitches = load(run / "hitches.csv")
    if not frames:
        sys.exit(f"No frames.csv in {run}")
    sys_times = [s["t_s"] for s in system]

    report = [f"Run: {run}", f"{len(frames)} frames, {frames[-1]['t_s']:.0f} s, {len(hitches)} hitches", ""]
    modes = sorted({f["mode"] for f in frames}, key=lambda m: (m != "FPV", m))
    for mode in modes:
        mf = [f for f in frames if f["mode"] == mode and f["t_s"] >= 3]
        mh = [h for h in hitches if h["mode"] == mode]
        if not mf:
            continue
        ft = [f["frame_ms"] for f in mf]
        report.append(f"== {mode}: {len(mf)} frames  p50 {pct(ft, .5):.1f}  p95 {pct(ft, .95):.1f}  "
                      f"p99 {pct(ft, .99):.1f}  max {max(ft):.1f} ms")
        report += ["   " + line for line in diagnose(mode, mf, mh, system, sys_times)]
        report.append("")
    text = "\n".join(report)
    print(text)

    # HTML
    t0, t1 = frames[0]["t_s"], frames[-1]["t_s"]
    marks = [h["t_s"] for h in hitches][:2000]
    fr = lambda col: [(f["t_s"], f[col]) for f in frames]
    sy = lambda col: [(s["t_s"], s[col]) for s in system]
    # Mode bands
    bands, colours = [], {"FPV": "#fde2e2", "MANUAL": "#e2f0fd", "ROUTE": "#e5fbe2", "MENU": "#eee", "HOVER": "#fff6d8"}
    start = frames[0]
    for prev, cur in zip(frames, frames[1:] + [None]):
        if cur is None or cur["mode"] != start["mode"]:
            bands.append((start["t_s"], prev["t_s"], start["mode"]))
            if cur is not None:
                start = cur
    band_txt = " · ".join(f'<span style="background:{colours.get(m, "#eee")}">{m} {a:.0f}–{b:.0f}s</span>'
                          for a, b, m in bands if b - a > 1)
    charts = [
        ("Frame time, ms (red lines = hitches)", [("frame", "#000", fr("frame_ms")), ("game", "#1f77b4", fr("game_ms")),
                                                  ("render", "#ff7f0e", fr("render_ms")), ("GPU", "#2ca02c", fr("gpu_ms"))], 100),
        ("Drone Tick sections, ms", [(c.replace("tick_", ""), col, fr(c)) for c, col in zip(
            TICK_COLS, ["#999", "#1f77b4", "#d62728", "#9467bd", "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#ff7f0e"])], None),
        ("Cesium tiles loaded, % / speed m/s", [("tiles %", "#9467bd", fr("tiles_load_pct")), ("speed", "#555", fr("speed_mps"))], None),
        ("CPU %", [("total", "#1f77b4", sy("cpu_total_pct")), ("busiest core", "#d62728", sy("cpu_max_core_pct")),
                   ("this process", "#2ca02c", sy("process_cpu_pct"))], 100),
        ("GPU %", [("3D", "#2ca02c", sy("gpu_3d_pct")), ("copy", "#9467bd", sy("gpu_copy_pct")),
                   ("video encode", "#ff7f0e", sy("gpu_video_encode_pct"))], 100),
        ("Memory MB", [("VRAM", "#2ca02c", sy("vram_dedicated_mb")), ("RAM free", "#1f77b4", sy("ram_available_mb")),
                       ("process RAM", "#d62728", sy("process_working_set_mb"))], None),
        ("Disk", [("busy %", "#d62728", sy("disk_busy_pct")), ("read MB/s", "#1f77b4", sy("disk_read_mbps")),
                  ("process read MB/s", "#2ca02c", sy("process_io_read_mbps")), ("hard faults/s ÷10", "#555",
                  [(s["t_s"], s["hard_faults_per_s"] / 10) for s in system])], None),
    ]
    body = [f"<h1>DroneSimUE performance</h1><pre>{html.escape(text)}</pre><p>{band_txt}</p>"]
    for title, series, ymax in charts:
        if series and any(pts for _, _, pts in series):
            body.append(f"<h3>{html.escape(title)}</h3>" + svg_series(series, t0, t1, ymax=ymax, marks=marks))
    if hitches:
        cols = ["t_s", "mode", "frame_ms", "bound", "game_ms", "render_ms", "gpu_ms", "gc_ms", "slowest_tick_section",
                "slowest_tick_ms", "tiles_load_pct", "speed_mps", "agl_m", "cpu_max_core_pct", "gpu_3d_pct",
                "disk_busy_pct", "hard_faults_per_s"]
        rows = sorted(hitches, key=lambda h: -h["frame_ms"])[:200]
        body.append("<h3>Worst hitches</h3><table><tr>" + "".join(f"<th>{c}</th>" for c in cols) + "</tr>")
        for h in rows:
            body.append("<tr>" + "".join(f"<td>{h.get(c, '') if not isinstance(h.get(c), float) else f'{h[c]:.1f}'}</td>"
                                         for c in cols) + "</tr>")
        body.append("</table>")
    page = ("<!doctype html><meta charset='utf-8'><title>DroneSimUE perf</title><style>body{font:13px sans-serif;margin:16px}"
            "table{border-collapse:collapse}td,th{border:1px solid #ccc;padding:2px 5px;text-align:right}"
            "pre{background:#f6f6f6;padding:8px}</style>" + "".join(body))
    out = run / "report.html"
    out.write_text(page, encoding="utf-8")
    print(f"Report: {out}")


if __name__ == "__main__":
    main()
