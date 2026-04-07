import csv
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

CSV_PATH = "data/runs/controlled_1774470694.csv"

# If your CSV has no controlled_phase column, set this manually:
MANUAL_T0 = 9.0   # e.g. 9.0

PHI_REF = 0.0


def load_csv(path):
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise ValueError(f"{path} is empty")

    data = {}
    for key in rows[0].keys():
        vals = []
        all_numeric = True
        for r in rows:
            v = r[key]
            try:
                vals.append(float(v))
            except (ValueError, TypeError):
                all_numeric = False
                break
        if all_numeric:
            data[key] = np.array(vals, dtype=float)
        else:
            data[key] = np.array([r[key] for r in rows], dtype=object)

    data["t_rel"] = data["t"] - data["t"][0]
    return data


def detect_controller_start(data):
    if MANUAL_T0 is not None:
        return MANUAL_T0

    if "controlled_phase" in data:
        phase = data["controlled_phase"]
        idx = np.where(phase == "controller")[0]
        if len(idx) > 0:
            return float(data["t_rel"][idx[0]])

    # Fallback: detect first significant controller action
    if "cmd_roll" in data and "phi_deg" in data:
        t = data["t_rel"]
        u = data["cmd_roll"]
        phi = data["phi_deg"]

        # Example heuristic: once bank is large and roll command becomes strongly corrective
        mask = (np.abs(phi) > 40.0) & (np.abs(u) > 0.2)
        idx = np.where(mask)[0]
        if len(idx) > 0:
            return float(t[idx[0]])

    raise RuntimeError("Could not detect controller start. Set MANUAL_T0 explicitly.")


def settling_time(t, y, y_ref, t0, band):
    mask_after = t >= t0
    ta = t[mask_after]
    ya = y[mask_after]

    inside = np.abs(ya - y_ref) <= band
    if not np.any(inside):
        return None

    # first time that enters and stays in band forever after
    for i in range(len(ta)):
        if np.all(inside[i:]):
            return float(ta[i] - t0)

    return None


def main():
    path = CSV_PATH
    if len(sys.argv) > 1:
        path = sys.argv[1]

    data = load_csv(path)

    required = ["t_rel", "phi_deg", "p_deg_s"]
    for c in required:
        if c not in data:
            raise ValueError(f"Missing required column: {c}")

    t = data["t_rel"]
    phi = data["phi_deg"]
    p = data["p_deg_s"]

    t0 = detect_controller_start(data)

    # state at controller start
    i0 = np.argmin(np.abs(t - t0))
    phi0 = phi[i0]
    e0 = phi0 - PHI_REF

    if abs(e0) < 1e-9:
        raise RuntimeError("Initial error at controller start is too small to define overshoot.")

    # Percent overshoot for regulator-to-zero problem
    # If starting positive, overshoot is how far negative it goes past zero.
    # If starting negative, overshoot is how far positive it goes past zero.
    phi_after = phi[t >= t0]

    if e0 > 0:
        opposite_excursion = max(0.0, -np.min(phi_after - PHI_REF))
    else:
        opposite_excursion = max(0.0, np.max(phi_after - PHI_REF))

    percent_overshoot = 100.0 * opposite_excursion / abs(e0)

    # Settling bands based on initial error
    band_2pct = 0.02 * abs(e0)
    band_5pct = 0.05 * abs(e0)

    ts_2pct = settling_time(t, phi, PHI_REF, t0, band_2pct)
    ts_5pct = settling_time(t, phi, PHI_REF, t0, band_5pct)

    # Print summary
    print(f"file: {path}")
    print(f"controller_start: {t0:.3f} s")
    print(f"phi(t0): {phi0:.3f} deg")
    print(f"initial_error: {e0:.3f} deg")
    print(f"percent_overshoot: {percent_overshoot:.3f} %")
    print(f"2% band: +/- {band_2pct:.3f} deg")
    print(f"5% band: +/- {band_5pct:.3f} deg")
    print(f"settling_time_2pct: {ts_2pct if ts_2pct is not None else 'NOT_SETTLED'}")
    print(f"settling_time_5pct: {ts_5pct if ts_5pct is not None else 'NOT_SETTLED'}")

    # Find overshoot point for plotting
    overshoot_t = None
    overshoot_phi = None
    if percent_overshoot > 0:
        mask = t >= t0
        ta = t[mask]
        phia = phi[mask]
        if e0 > 0:
            j = np.argmin(phia)
        else:
            j = np.argmax(phia)
        overshoot_t = ta[j]
        overshoot_phi = phia[j]

    # Plot
    fig, axs = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    # Bank angle plot
    axs[0].plot(t, phi, label="Bank angle φ (deg)")
    axs[0].axvline(t0, linestyle="--", label="Controller start")
    axs[0].axhline(PHI_REF, linestyle=":")
    axs[0].axhline(PHI_REF + band_2pct, linestyle=":", linewidth=1)
    axs[0].axhline(PHI_REF - band_2pct, linestyle=":", linewidth=1)
    axs[0].axhline(PHI_REF + band_5pct, linestyle="--", linewidth=1)
    axs[0].axhline(PHI_REF - band_5pct, linestyle="--", linewidth=1)

    if overshoot_t is not None:
        axs[0].plot(overshoot_t, overshoot_phi, "o", label="Overshoot point")

    axs[0].set_ylabel("Bank angle φ (deg)")
    axs[0].set_title("Controlled Response")
    axs[0].legend()
    axs[0].grid(True)

    # Roll rate plot
    axs[1].plot(t, p, label="Roll rate p (deg/s)")
    axs[1].axvline(t0, linestyle="--", label="Controller start")
    axs[1].axhline(0.0, linestyle=":")
    axs[1].set_ylabel("Roll rate p (deg/s)")
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    out = path.replace(".csv", "_response_check.png")
    plt.savefig(out, dpi=150)
    print(f"saved plot: {out}")
    plt.show()


if __name__ == "__main__":
    main()