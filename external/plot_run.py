import csv
import math
from pathlib import Path
import matplotlib.pyplot as plt

def rad2deg(x): return x * 180.0 / math.pi

def load_csv(path):
    with open(path, "r") as f:
        r = csv.DictReader(f)
        rows = list(r)
    t = [float(row["t"]) for row in rows]
    phi = [rad2deg(float(row["phi_rad"])) for row in rows]
    return t, phi

def main(run_path: str):
    run_path = Path(run_path)
    t, phi = load_csv(run_path)
    t0 = t[0]
    t = [x - t0 for x in t]

    plt.figure()
    plt.plot(t, phi)
    plt.xlabel("time (s)")
    plt.ylabel("bank angle phi (deg)")
    plt.title(run_path.name)
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("usage: python external/plot_run.py data/runs/run_XXXX.csv")
        raise SystemExit(2)
    main(sys.argv[1])