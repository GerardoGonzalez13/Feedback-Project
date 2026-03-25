import argparse
import time
from pathlib import Path

import matplotlib.pyplot as plt

from analysis_utils import detect_events, load_run


ALIGN_CHOICES = ("start", "controller_start", "bank60")
RUNS_DIR = Path("data/runs")


def aligned_time(run, align_mode):
    events = detect_events(run)
    t_rel = run["t_rel"]

    if align_mode == "start":
        return t_rel, events, None

    if align_mode == "controller_start":
        event_key = "controller_start"
    elif align_mode == "bank60":
        event_key = "time_to_60deg_bank"
    else:
        raise ValueError(f"Unsupported align mode: {align_mode}")

    event_time = events.get(event_key)
    if event_time is None:
        return None, events, f"missing alignment event '{event_key}'"

    return [t - event_time for t in t_rel], events, None


def event_marker_time(events, align_mode, event_key):
    t = events.get(event_key)
    if t is None:
        return None

    if align_mode == "start":
        return t

    if align_mode == "controller_start":
        anchor = events.get("controller_start")
    elif align_mode == "bank60":
        anchor = events.get("time_to_60deg_bank")
    else:
        anchor = None

    if anchor is None:
        return None
    return t - anchor


def add_event_lines(ax, events, align_mode):
    event_specs = [
        ("time_to_60deg_bank", "60 deg bank", "tab:purple"),
        ("controller_start", "controller start", "tab:green"),
    ]
    ymax = ax.get_ylim()[1]
    for event_key, label, color in event_specs:
        t = event_marker_time(events, align_mode, event_key)
        if t is None:
            continue
        ax.axvline(t, color=color, linestyle="--", linewidth=1.0, alpha=0.8)
        ax.text(
            t,
            ymax,
            label,
            rotation=90,
            va="top",
            ha="right",
            fontsize=8,
            color=color,
            backgroundcolor="white",
        )


def plot_overlay(baseline_csv, controlled_csv, align_mode):
    baseline_run = load_run(baseline_csv)
    controlled_run = load_run(controlled_csv)

    baseline_t, baseline_events, baseline_error = aligned_time(baseline_run, align_mode)
    controlled_t, controlled_events, controlled_error = aligned_time(controlled_run, align_mode)

    warnings = []
    if align_mode != "start":
        if baseline_error or controlled_error:
            if align_mode == "controller_start":
                warnings.append(
                    "controller_start alignment unavailable for one or both runs; "
                    "falling back to start alignment."
                )
                align_mode = "start"
                baseline_t, baseline_events, _ = aligned_time(baseline_run, align_mode)
                controlled_t, controlled_events, _ = aligned_time(controlled_run, align_mode)
            else:
                missing = []
                if baseline_error:
                    missing.append(f"baseline: {baseline_error}")
                if controlled_error:
                    missing.append(f"controlled: {controlled_error}")
                raise RuntimeError(
                    "Cannot align runs on bank60 because required event is missing: "
                    + "; ".join(missing)
                )

    fig, axes = plt.subplots(3, 1, figsize=(12, 11), sharex=True)
    fig.suptitle(
        f"Overlay Comparison ({align_mode} alignment)\n"
        f"baseline={Path(baseline_csv).name} | controlled={Path(controlled_csv).name}"
    )

    # Bank angle overlay.
    axes[0].plot(baseline_t, baseline_run["phi"], label="baseline", color="tab:blue")
    axes[0].plot(controlled_t, controlled_run["phi"], label="controlled", color="tab:orange")
    axes[0].set_ylabel("Bank angle phi (deg)")
    axes[0].set_title("Bank Angle")
    axes[0].grid(True)
    axes[0].legend()

    # Roll-rate overlay.
    axes[1].plot(baseline_t, baseline_run["p"], label="baseline", color="tab:blue")
    axes[1].plot(controlled_t, controlled_run["p"], label="controlled", color="tab:orange")
    axes[1].set_ylabel("Roll rate p (deg/s)")
    axes[1].set_title("Roll Rate")
    axes[1].grid(True)
    axes[1].legend()

    # Altitude overlay.
    axes[2].plot(baseline_t, baseline_run["alt_ft"], label="baseline", color="tab:blue")
    axes[2].plot(controlled_t, controlled_run["alt_ft"], label="controlled", color="tab:orange")
    axes[2].set_ylabel("Altitude (ft)")
    axes[2].set_xlabel("Aligned time (s)")
    axes[2].set_title("Altitude")
    axes[2].grid(True)
    axes[2].legend()

    for ax in axes:
        add_event_lines(ax, baseline_events, align_mode)
        add_event_lines(ax, controlled_events, align_mode)

    fig.tight_layout()

    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = RUNS_DIR / f"overlay_{int(time.time())}.png"
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)

    for warning in warnings:
        print(f"warning: {warning}")
    print(f"saved_overlay: {out_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Overlay one baseline CSV and one controlled CSV on shared bank-angle, roll-rate, and altitude plots."
    )
    parser.add_argument("baseline_csv", help="Path to baseline raw telemetry CSV")
    parser.add_argument("controlled_csv", help="Path to controlled raw telemetry CSV")
    parser.add_argument(
        "--align",
        choices=ALIGN_CHOICES,
        default="start",
        help="Alignment event for both runs",
    )
    args = parser.parse_args()

    plot_overlay(args.baseline_csv, args.controlled_csv, args.align)


if __name__ == "__main__":
    main()
