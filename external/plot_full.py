import csv
import matplotlib.pyplot as plt

def m2ft(x): return x * 3.28084

def load(path):
    with open(path, "r") as f:
        r = csv.DictReader(f)
        rows = list(r)

    t = [float(row["t"]) for row in rows]
    t0 = t[0]
    t = [x - t0 for x in t]

    phi = [float(row.get("phi_deg", 0.0)) for row in rows]
    p   = [float(row.get("p_deg_s", 0.0)) for row in rows]

    # Prefer actual applied yoke ratios if present, else commanded
    roll_cmd = [float(row.get("yoke_roll", row.get("cmd_roll", 0.0))) for row in rows]
    yaw_cmd  = [float(row.get("yoke_yaw",  row.get("cmd_yaw",  0.0))) for row in rows]

    alt = [m2ft(float(row.get("alt_m", 0.0))) for row in rows]

    return t, phi, p, roll_cmd, yaw_cmd, alt

def plot_all(path):
    t, phi, p, roll_cmd, yaw_cmd, alt = load(path)

    plt.figure()
    plt.plot(t, phi)
    plt.xlabel("Time (s)")
    plt.ylabel("Bank angle ϕ (deg)")
    plt.title("Bank Angle Response")
    plt.grid(True)

    plt.figure()
    plt.plot(t, p)
    plt.xlabel("Time (s)")
    plt.ylabel("Roll rate p (deg/s)")
    plt.title("Roll Rate")
    plt.grid(True)

    plt.figure()
    plt.plot(t, roll_cmd, label="Roll input (yoke/command)")
    plt.plot(t, yaw_cmd, label="Yaw input (yoke/command)")
    plt.xlabel("Time (s)")
    plt.ylabel("Control Input (-1 to 1)")
    plt.title("Control Inputs")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(t, alt)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (ft)")
    plt.title("Altitude Response")
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("usage: python external/plot_full.py data/runs/run_XXXX.csv")
        raise SystemExit(1)

    plot_all(sys.argv[1])