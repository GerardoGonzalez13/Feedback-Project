import csv
import json
import math
import socket
import time
from pathlib import Path


LISTEN_IP = "0.0.0.0"
PORT = 49005
RUNS_DIR = Path("data/runs")

STABLE_COLUMNS = [
    "t",
    "phi_deg",
    "p_deg_s",
    "beta_deg",
    "r_deg_s",
    "theta_deg",
    "q_deg_s",
    "psi_deg",
    "alt_m",
    "alt_ft",
    "ias_kts",
    "vv_m_s",
    "cmd_enable",
    "cmd_roll",
    "cmd_pitch",
    "cmd_yaw",
    "yoke_roll",
    "yoke_pitch",
    "yoke_yaw",
]


def m2ft(x):
    return x * 3.28084


def ms2fpm(x):
    return x * 196.850394


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def ordered_fieldnames(rows):
    seen = {key for row in rows for key in row.keys()}
    ordered = list(STABLE_COLUMNS)
    extras = sorted(seen.difference(ordered))
    return ordered + extras


def normalize_row(msg):
    row = dict(msg)
    alt_m = as_float(row.get("alt_m"))
    if alt_m is not None and "alt_ft" not in row and "altitude_ft" not in row:
        row["alt_ft"] = m2ft(alt_m)
    return row


def print_status(msg, packets, addr):
    phi = as_float(msg.get("phi_deg"))
    p = as_float(msg.get("p_deg_s"))
    beta = as_float(msg.get("beta_deg"))
    r = as_float(msg.get("r_deg_s"))
    alt_m = as_float(msg.get("alt_m"))
    ias = as_float(msg.get("ias_kts"))
    vv = as_float(msg.get("vv_m_s"))
    cmd_enable = msg.get("cmd_enable")
    cmd_roll = as_float(msg.get("cmd_roll"))
    cmd_pitch = as_float(msg.get("cmd_pitch"))
    cmd_yaw = as_float(msg.get("cmd_yaw"))

    if phi is None:
        print(f"[logger] packets={packets} keys={list(msg.keys())} last_from={addr}")
        return

    alt_ft = m2ft(alt_m) if alt_m is not None else None
    vv_fpm = ms2fpm(vv) if vv is not None else None
    beta_text = f"{beta:7.2f}" if beta is not None and math.isfinite(beta) else "   n/a "
    p_text = f"{p:7.2f}" if p is not None and math.isfinite(p) else "   n/a "
    r_text = f"{r:7.2f}" if r is not None and math.isfinite(r) else "   n/a "
    alt_text = f"{alt_ft:8.1f}" if alt_ft is not None else "    n/a "
    ias_text = f"{ias:6.1f}" if ias is not None else "  n/a "
    vv_text = f"{vv_fpm:7.0f}" if vv_fpm is not None else "   n/a "
    roll_text = f"{cmd_roll:+.3f}" if cmd_roll is not None else "n/a"
    pitch_text = f"{cmd_pitch:+.3f}" if cmd_pitch is not None else "n/a"
    yaw_text = f"{cmd_yaw:+.3f}" if cmd_yaw is not None else "n/a"

    print(
        f"[logger] packets={packets} "
        f"phi={phi:7.2f} deg "
        f"p={p_text} deg/s "
        f"beta={beta_text} deg "
        f"r={r_text} deg/s "
        f"alt={alt_text} ft "
        f"ias={ias_text} kt "
        f"vv={vv_text} fpm "
        f"cmd_en={cmd_enable} "
        f"roll_cmd={roll_text} pitch_cmd={pitch_text} yaw_cmd={yaw_text} "
        f"from={addr}"
    )


def main():
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = RUNS_DIR / f"run_{int(time.time())}.csv"

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((LISTEN_IP, PORT))
    sock.settimeout(1.0)

    print(f"[logger] Listening on UDP {LISTEN_IP}:{PORT} ...")
    print("[logger] Ctrl+C to stop and write CSV")

    rows = []
    last_print = 0.0

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue

            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            row = normalize_row(msg)
            rows.append(row)

            now = time.time()
            if now - last_print >= 1.0:
                last_print = now
                print_status(row, len(rows), addr)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if not rows:
        print("[logger] No data captured; nothing to write.")
        return

    fieldnames = ordered_fieldnames(rows)
    with out_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"[logger] wrote {len(rows)} rows -> {out_path}")


if __name__ == "__main__":
    main()
