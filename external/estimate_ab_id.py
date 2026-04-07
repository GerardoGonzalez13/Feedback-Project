import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np


# Smoothing for p before differentiating.
SMOOTH_N = 7

# Command-based window detection.
NEUTRAL_EPS = 0.02
PULSE_MIN_ABS_CMD = 0.05
COMMAND_CHANGE_THRESHOLD = 0.03
COMMAND_STD_MAX = 0.01

# Ignore a short transient immediately after each command change.
IGNORE_AFTER_CHANGE_S = 0.20

# Reject windows that are too short to be useful.
MIN_WINDOW_S = 0.40
MIN_WINDOW_SAMPLES = 8
MIN_POINTS_A = 20
MIN_POINTS_B = 20
MIN_POINTS_AB = 30


@dataclass
class Window:
    kind: str
    start: int
    end: int
    start_used: int
    end_used: int
    cmd_mean: float
    cmd_std: float
    duration_s: float
    used_duration_s: float


def moving_average(x, n):
    if n <= 1:
        return x.copy()
    kernel = np.ones(n, dtype=float) / float(n)
    xpad = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xpad, kernel, mode="valid")


def load_csv_numeric(path):
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise ValueError(f"{path} is empty.")

    required = {"t", "p_deg_s", "cmd_roll"}
    missing = required.difference(rows[0].keys())
    if missing:
        missing_text = ", ".join(sorted(missing))
        raise ValueError(f"{path} is missing required columns: {missing_text}")

    data = {}
    for key in rows[0].keys():
        values = []
        for row in rows:
            try:
                values.append(float(row[key]))
            except (TypeError, ValueError):
                values.append(np.nan)
        data[key] = np.array(values, dtype=float)

    if len(data["t"]) < 3:
        raise ValueError("Need at least 3 samples for differentiation.")

    data["t_rel"] = data["t"] - data["t"][0]
    data["p_smooth"] = moving_average(data["p_deg_s"], SMOOTH_N)
    data["dpdt"] = np.gradient(data["p_smooth"], data["t_rel"])
    return data


def finite_mask(*arrays):
    mask = np.ones(len(arrays[0]), dtype=bool)
    for array in arrays:
        mask &= np.isfinite(array)
    return mask


def detect_constant_command_segments(t_rel, cmd_roll):
    valid = np.isfinite(t_rel) & np.isfinite(cmd_roll)
    idx = np.where(valid)[0]
    if len(idx) == 0:
        return []

    segments = []
    start = idx[0]
    ref_cmd = cmd_roll[start]
    prev = start

    for i in idx[1:]:
        is_gap = i != prev + 1
        is_change = abs(cmd_roll[i] - ref_cmd) > COMMAND_CHANGE_THRESHOLD
        if is_gap or is_change:
            segments.append((start, prev))
            start = i
            ref_cmd = cmd_roll[i]
        prev = i

    segments.append((start, prev))
    return segments


def trim_window_start(t_rel, start, end, ignore_after_change_s):
    first_keep_time = t_rel[start] + ignore_after_change_s
    used_start = start
    while used_start <= end and t_rel[used_start] < first_keep_time:
        used_start += 1
    return used_start


def build_windows(data):
    t_rel = data["t_rel"]
    cmd_roll = data["cmd_roll"]
    segments = detect_constant_command_segments(t_rel, cmd_roll)

    neutral_windows = []
    pulse_windows = []

    for start, end in segments:
        duration_s = t_rel[end] - t_rel[start]
        if duration_s < MIN_WINDOW_S:
            continue

        segment_cmd = cmd_roll[start:end + 1]
        cmd_mean = float(np.nanmean(segment_cmd))
        cmd_std = float(np.nanstd(segment_cmd))

        if cmd_std > COMMAND_STD_MAX:
            continue

        used_start = trim_window_start(t_rel, start, end, IGNORE_AFTER_CHANGE_S)
        if used_start > end:
            continue

        used_duration_s = t_rel[end] - t_rel[used_start]
        used_count = end - used_start + 1
        if used_duration_s < MIN_WINDOW_S or used_count < MIN_WINDOW_SAMPLES:
            continue

        if abs(cmd_mean) < NEUTRAL_EPS:
            kind = "neutral"
        elif abs(cmd_mean) >= PULSE_MIN_ABS_CMD:
            kind = "pulse"
        else:
            continue

        window = Window(
            kind=kind,
            start=start,
            end=end,
            start_used=used_start,
            end_used=end,
            cmd_mean=cmd_mean,
            cmd_std=cmd_std,
            duration_s=duration_s,
            used_duration_s=used_duration_s,
        )

        if kind == "neutral":
            neutral_windows.append(window)
        else:
            pulse_windows.append(window)

    return neutral_windows, pulse_windows


def windows_to_mask(n_samples, windows):
    mask = np.zeros(n_samples, dtype=bool)
    for window in windows:
        mask[window.start_used:window.end_used + 1] = True
    return mask


def print_windows(label, data, windows):
    print(f"\n{label}:")
    if not windows:
        print("  none")
        return

    for i, window in enumerate(windows, start=1):
        raw_points = window.end - window.start + 1
        used_points = window.end_used - window.start_used + 1
        print(
            f"  {i:02d}: "
            f"t={data['t_rel'][window.start]:7.3f}..{data['t_rel'][window.end]:7.3f} s | "
            f"used={data['t_rel'][window.start_used]:7.3f}..{data['t_rel'][window.end_used]:7.3f} s | "
            f"cmd={window.cmd_mean:+.3f} +/- {window.cmd_std:.4f} | "
            f"raw_pts={raw_points:3d} used_pts={used_points:3d}"
        )


def estimate_a(data, mask):
    p = data["p_smooth"]
    dpdt = data["dpdt"]
    mask &= finite_mask(p, dpdt)

    n = int(mask.sum())
    if n < MIN_POINTS_A:
        raise ValueError(f"Not enough neutral samples for estimating a ({n} found).")

    denom = float(np.dot(p[mask], p[mask]))
    if denom <= 0.0:
        raise ValueError("Neutral-window p signal is too small to estimate a.")

    a_est = float(np.dot(p[mask], dpdt[mask]) / denom)
    y = dpdt[mask]
    yhat = a_est * p[mask]
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = np.nan if ss_tot == 0.0 else 1.0 - ss_res / ss_tot
    return a_est, r2, n


def estimate_b_given_a(data, a_est, mask):
    p = data["p_smooth"]
    u = data["cmd_roll"]
    dpdt = data["dpdt"]
    mask &= finite_mask(p, u, dpdt)

    n = int(mask.sum())
    if n < MIN_POINTS_B:
        raise ValueError(f"Not enough pulse samples for estimating b ({n} found).")

    denom = float(np.dot(u[mask], u[mask]))
    if denom <= 0.0:
        raise ValueError("Pulse-window command signal is too small to estimate b.")

    residual_target = dpdt[mask] - a_est * p[mask]
    b_est = float(np.dot(u[mask], residual_target) / denom)
    yhat = b_est * u[mask]
    ss_res = float(np.sum((residual_target - yhat) ** 2))
    ss_tot = float(np.sum((residual_target - np.mean(residual_target)) ** 2))
    r2 = np.nan if ss_tot == 0.0 else 1.0 - ss_res / ss_tot
    return b_est, r2, n


def estimate_ab_joint(data, mask):
    p = data["p_smooth"]
    u = data["cmd_roll"]
    dpdt = data["dpdt"]
    mask &= finite_mask(p, u, dpdt)

    n = int(mask.sum())
    if n < MIN_POINTS_AB:
        raise ValueError(f"Not enough samples for joint estimate ({n} found).")

    X = np.column_stack([p[mask], u[mask]])
    y = dpdt[mask]
    theta, *_ = np.linalg.lstsq(X, y, rcond=None)
    a_est = float(theta[0])
    b_est = float(theta[1])
    yhat = X @ theta
    ss_res = float(np.sum((y - yhat) ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = np.nan if ss_tot == 0.0 else 1.0 - ss_res / ss_tot
    return a_est, b_est, r2, n


def parse_args():
    parser = argparse.ArgumentParser(
        description="Estimate roll-axis model parameters a and b from a raw ID-test CSV."
    )
    parser.add_argument("csv_path", type=Path, help="Path to the raw logger CSV from run_id_test.py")
    return parser.parse_args()


def main():
    args = parse_args()
    data = load_csv_numeric(args.csv_path)

    neutral_windows, pulse_windows = build_windows(data)
    print_windows("Detected neutral windows", data, neutral_windows)
    print_windows("Detected pulse windows", data, pulse_windows)

    neutral_mask = windows_to_mask(len(data["t_rel"]), neutral_windows)
    pulse_mask = windows_to_mask(len(data["t_rel"]), pulse_windows)
    joint_mask = neutral_mask | pulse_mask

    print("\nEstimates:")

    a_est, a_r2, a_n = estimate_a(data, neutral_mask)
    print(f"  a from neutral windows: {a_est:.6f} 1/s  (R^2={a_r2:.4f}, n={a_n})")

    b_est, b_r2, b_n = estimate_b_given_a(data, a_est, pulse_mask)
    print(f"  b from pulse windows:   {b_est:.6f} (deg/s^2)/command  (R^2={b_r2:.4f}, n={b_n})")

    joint_a, joint_b, joint_r2, joint_n = estimate_ab_joint(data, joint_mask)
    print(
        f"  joint least squares:    a={joint_a:.6f} 1/s, "
        f"b={joint_b:.6f} (deg/s^2)/command  (R^2={joint_r2:.4f}, n={joint_n})"
    )


if __name__ == "__main__":
    main()
