"""Print the MAVLink telemetry the simulator sends (checks the UDP link).

Usage (any Python with pymavlink, e.g. DroneLocalization's venv):
    python -m pip install pymavlink
    python Tools/mavlink_listen.py            # listens on udp 0.0.0.0:14550
    python Tools/mavlink_listen.py --port 14551

Do not run it at the same time as QGroundControl on the same port.
"""

import argparse
import time

from pymavlink import mavutil


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=14550)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(f"udpin:0.0.0.0:{args.port}", dialect="common")
    print(f"Listening on udp 0.0.0.0:{args.port} ... (Ctrl+C to stop)")
    counts: dict[str, int] = {}
    last_print = time.monotonic()
    while True:
        msg = link.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue
        kind = msg.get_type()
        if kind == "BAD_DATA":
            print("bad frame:", msg)
            continue
        counts[kind] = counts.get(kind, 0) + 1
        now = time.monotonic()
        if kind == "GLOBAL_POSITION_INT" and now - last_print >= 1.0:
            last_print = now
            print(
                f"t={msg.time_boot_ms / 1000:8.2f}s  lat={msg.lat / 1e7:.6f} lon={msg.lon / 1e7:.6f} "
                f"rel_alt={msg.relative_alt / 1000:7.1f} m  hdg={msg.hdg / 100:6.1f}  "
                f"v=({msg.vx / 100:.1f}, {msg.vy / 100:.1f}, {msg.vz / 100:.1f}) m/s  counts={counts}"
            )


if __name__ == "__main__":
    main()
