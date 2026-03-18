import csv
import json
import socket
import time
from pathlib import Path

# --- CONFIG ---
LISTEN_IP = "0.0.0.0"
PORT = 49005
RUNS_DIR = Path("data/runs")
# -------------

def m2ft(x): return x * 3.28084
def ms2fpm(x): return x * 196.850394

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
    t0 = time.time()

    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                # if we got nothing for a while, just keep waiting
                continue

            try:
                msg = json.loads(data.decode("utf-8"))
            except Exception:
                continue

            rows.append(msg)

            now = time.time()
            if now - last_print >= 1.0:
                last_print = now

                # pull values safely
                phi = msg.get("phi_deg", None)
                alt_m = msg.get("alt_m", None)
                ias = msg.get("ias_kts", None)
                vv = msg.get("vv_m_s", None)

                # command state (optional)
                en = msg.get("cmd_enable", None)
                cr = msg.get("cmd_roll", None)
                cy = msg.get("cmd_yaw", None)

                if phi is None:
                    print(f"[logger] packets={len(rows)} keys={list(msg.keys())} last_from={addr}")
                else:
                    alt_ft = m2ft(alt_m) if alt_m is not None else None
                    vv_fpm = ms2fpm(vv) if vv is not None else None

                    print(
                        f"[logger] packets={len(rows)} "
                        f"phi={phi:7.2f} deg "
                        f"alt={alt_ft:8.1f} ft "
                        f"ias={ias:6.1f} kt "
                        f"vv={vv_fpm:7.0f} fpm "
                        f"cmd_en={en} roll_cmd={cr} yaw_cmd={cy} "
                        f"from={addr}"
                    )

    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if not rows:
        print("[logger] No data captured; nothing to write.")
        return

    # write CSV with stable columns
    keys = sorted({k for row in rows for k in row.keys()})
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)

    print(f"[logger] wrote {len(rows)} rows -> {out_path}")

    # Auto-plot latest run
try:
    import matplotlib.pyplot as plt
    import math

    def m2ft(x): return x * 3.28084

    t = [row["t"] for row in rows]
    t0 = float(t[0])
    t = [float(x) - t0 for x in t]

    phi = [float(row.get("phi_deg", 0.0)) for row in rows]
    alt_ft = [m2ft(float(row.get("alt_m", 0.0))) for row in rows]

    plt.figure()
    plt.plot(t, phi)
    plt.xlabel("time (s)")
    plt.ylabel("bank angle phi (deg)")
    plt.grid(True)
    plt.title("Bank angle")

    plt.figure()
    plt.plot(t, alt_ft)
    plt.xlabel("time (s)")
    plt.ylabel("altitude (ft)")
    plt.grid(True)
    plt.title("Altitude")

    plt.savefig("data/figures/bank_angle.png", dpi=200)
    plt.close()
except Exception as e:
    print(f"[logger] plot skipped: {e}")

if __name__ == "__main__":
    main()