from pathlib import Path

import matplotlib.pyplot as plt

from analysis_utils import detect_events, load_run


EVENT_SPECS = [
    ("time_to_60deg_bank", "first ~60 deg bank", "tab:purple"),
    ("yaw_pulse_start", "yaw pulse start", "tab:red"),
    ("yaw_pulse_end", "yaw pulse end", "tab:pink"),
    ("controller_start", "controller start", "tab:green"),
]


def add_event_markers(ax, events):
    ymax = ax.get_ylim()[1]
    used = set()
    for key, label, color in EVENT_SPECS:
        t = events.get(key)
        if t is None:
            continue
        ax.axvline(t, color=color, linestyle="--", linewidth=1.2, alpha=0.9)
        text_x = t
        if text_x in used:
            text_x += 0.15
        used.add(text_x)
        ax.text(
            text_x,
            ymax,
            label,
            rotation=90,
            va="top",
            ha="right",
            fontsize=8,
            color=color,
            backgroundcolor="white",
        )


def plot_all(path):
    run = load_run(path)
    events = detect_events(run)

    t = run["t_rel"]
    phi = run["phi"]
    p = run["p"]
    alt = run["alt_ft"]
    roll_cmd = run["yoke_roll"] if any(abs(v) > 1e-6 for v in run["yoke_roll"]) else run["cmd_roll"]
    yaw_cmd = run["yoke_yaw"] if any(abs(v) > 1e-6 for v in run["yoke_yaw"]) else run["cmd_yaw"]

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(f"Full Run Analysis: {Path(path).name}")

    axes[0].plot(t, phi, color="tab:blue")
    axes[0].set_ylabel("Bank angle phi (deg)")
    axes[0].grid(True)
    axes[0].set_title("Bank Angle")

    axes[1].plot(t, p, color="tab:orange")
    axes[1].set_ylabel("Roll rate p (deg/s)")
    axes[1].grid(True)
    axes[1].set_title("Roll Rate")

    axes[2].plot(t, roll_cmd, label="roll input", color="tab:green")
    axes[2].plot(t, yaw_cmd, label="yaw input", color="tab:red")
    axes[2].set_ylabel("Control input")
    axes[2].grid(True)
    axes[2].legend()
    axes[2].set_title("Control Inputs")

    axes[3].plot(t, alt, color="tab:brown")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_ylabel("Altitude (ft)")
    axes[3].grid(True)
    axes[3].set_title("Altitude")

    for ax in axes:
        add_event_markers(ax, events)

    fig.tight_layout()

    out_dir = Path("data/figures")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{Path(path).stem}_full.png"
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    print(f"saved_plot: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("usage: python external/plot_full.py data/runs/run_XXXX.csv")
        raise SystemExit(1)

    plot_all(sys.argv[1])
