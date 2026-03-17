import csv
import math

def rad2deg(x): return x * 180.0 / math.pi
def m2ft(x): return x * 3.28084

def load(path):
    with open(path, "r") as f:
        r = csv.DictReader(f)
        rows = list(r)
    t = [float(x["t"]) for x in rows]
    phi = [float(x["phi_deg"]) for x in rows]
    alt = [m2ft(float(x["alt_m"])) for x in rows]
    return t, phi, alt

def time_to_wings_level(t, phi, tol_deg=5.0, hold_s=2.0):
    # first time |phi| stays below tol for hold_s
    t0 = t[0]
    t_rel = [x - t0 for x in t]

    for i in range(len(phi)):
        if abs(phi[i]) <= tol_deg:
            t_start = t_rel[i]
            # check forward window
            j = i
            while j < len(phi) and (t_rel[j] - t_start) < hold_s:
                if abs(phi[j]) > tol_deg:
                    break
                j += 1
            else:
                return t_start
    return None

def rms(vals):
    return math.sqrt(sum(v*v for v in vals) / max(1, len(vals)))

def main(path):
    t, phi, alt = load(path)
    t0 = t[0]
    t_rel = [x - t0 for x in t]

    t_wl = time_to_wings_level(t, phi, tol_deg=5.0, hold_s=2.0)

    # RMS phi over 5-20s
    window = [phi[i] for i in range(len(phi)) if 5.0 <= t_rel[i] <= 20.0]
    phi_rms = rms(window) if window else None

    alt_loss = alt[0] - alt[-1]

    print(f"file: {path}")
    print(f"time_to_|phi|<5deg (hold 2s): {t_wl}")
    print(f"phi_rms_5to20s (deg): {phi_rms}")
    print(f"alt_loss (ft, start-end): {alt_loss:.1f}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("usage: python external/metrics.py data/runs/run_XXXX.csv")
        raise SystemExit(2)
    main(sys.argv[1])