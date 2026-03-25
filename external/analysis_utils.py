import csv
import json
import math
from pathlib import Path

TARGET_BANK_DEG = 60.0
BANK_EVENT_TOL_DEG = 3.0
YAW_CMD_EVENT_THRESH = 0.05
CONTROLLER_ROLL_CMD_THRESH = 0.03

NOT_REACHED = "NOT_REACHED"
INSUFFICIENT_DATA = "INSUFFICIENT_DATA"
NO_CONTROLLER_PHASE_FOUND = "NO_CONTROLLER_PHASE_FOUND"


def m2ft(x):
    return x * 3.28084


def rms(vals):
    if not vals:
        return None
    return math.sqrt(sum(v * v for v in vals) / len(vals))


def to_float(row, *keys, default=0.0):
    for key in keys:
        value = row.get(key)
        if value not in (None, ""):
            try:
                return float(value)
            except ValueError:
                continue
    return default


def load_run(path):
    path = Path(path)
    with path.open("r") as f:
        rows = list(csv.DictReader(f))

    if not rows:
        raise ValueError(f"No rows found in {path}")

    t_abs = [to_float(row, "t") for row in rows]
    t0 = t_abs[0]
    t_rel = [t - t0 for t in t_abs]

    phi = [to_float(row, "phi_deg") for row in rows]
    p = [to_float(row, "p_deg_s") for row in rows]
    alt_ft = [
        to_float(row, "alt_ft", "altitude_ft", default=float("nan"))
        if row.get("alt_ft") or row.get("altitude_ft")
        else m2ft(to_float(row, "alt_m"))
        for row in rows
    ]

    cmd_enable = [to_float(row, "cmd_enable") for row in rows]
    cmd_roll = [to_float(row, "cmd_roll") for row in rows]
    cmd_pitch = [to_float(row, "cmd_pitch") for row in rows]
    cmd_yaw = [to_float(row, "cmd_yaw") for row in rows]
    yoke_roll = [to_float(row, "yoke_roll") for row in rows]
    yoke_yaw = [to_float(row, "yoke_yaw") for row in rows]
    baseline_phase = [row.get("baseline_phase", "") for row in rows]

    return {
        "path": path,
        "rows": rows,
        "t_abs": t_abs,
        "t_rel": t_rel,
        "phi": phi,
        "p": p,
        "alt_ft": alt_ft,
        "cmd_enable": cmd_enable,
        "cmd_roll": cmd_roll,
        "cmd_pitch": cmd_pitch,
        "cmd_yaw": cmd_yaw,
        "yoke_roll": yoke_roll,
        "yoke_yaw": yoke_yaw,
        "baseline_phase": baseline_phase,
    }


def first_index(seq, predicate, start=0):
    for i in range(start, len(seq)):
        if predicate(i):
            return i
    return None


def first_time_hold(t_rel, values, predicate, hold_s, start_idx=0):
    stable_start_idx = None
    for i in range(start_idx, len(values)):
        if predicate(i):
            if stable_start_idx is None:
                stable_start_idx = i
            elif t_rel[i] - t_rel[stable_start_idx] >= hold_s:
                return stable_start_idx
        else:
            stable_start_idx = None
    return None


def window_values(t_rel, values, start_t, duration_s):
    end_t = start_t + duration_s
    out = [values[i] for i, t in enumerate(t_rel) if start_t <= t <= end_t]
    if len(out) < 2:
        return None
    return out


def detect_events(run):
    t_rel = run["t_rel"]
    phi = run["phi"]
    cmd_roll = run["cmd_roll"]
    cmd_yaw = run["cmd_yaw"]
    phases = run["baseline_phase"]

    peak_idx = max(range(len(phi)), key=lambda i: abs(phi[i]))
    target_bank = TARGET_BANK_DEG if phi[peak_idx] >= 0.0 else -TARGET_BANK_DEG

    reach_60_idx = first_index(
        phi,
        lambda i: abs(phi[i] - target_bank) <= BANK_EVENT_TOL_DEG,
    )

    yaw_active = [abs(v) >= YAW_CMD_EVENT_THRESH for v in cmd_yaw]
    yaw_pulse_start_idx = first_index(yaw_active, lambda i: yaw_active[i])
    yaw_pulse_end_idx = None
    if yaw_pulse_start_idx is not None:
        yaw_pulse_end_idx = first_index(
            yaw_active,
            lambda i: not yaw_active[i],
            start=yaw_pulse_start_idx + 1,
        )

    controller_start_idx = None
    explicit_phases_present = any(bool(phase) for phase in phases)

    if any("controller" in phase for phase in phases):
        controller_start_idx = first_index(
            phases,
            lambda i: "controller" in phases[i],
        )
    elif not explicit_phases_present and yaw_pulse_start_idx is None and reach_60_idx is not None:
        target_sign = 1.0 if target_bank >= 0.0 else -1.0

        def controller_takeover(i):
            if i + 2 >= len(cmd_roll):
                return False
            sustained = 0
            for j in range(i, min(i + 5, len(cmd_roll))):
                if target_sign * cmd_roll[j] <= -CONTROLLER_ROLL_CMD_THRESH:
                    sustained += 1
            return sustained >= 3

        controller_start_idx = first_index(cmd_roll, controller_takeover, start=reach_60_idx + 1)

    return {
        "target_bank_deg": target_bank,
        "time_to_60deg_bank": t_rel[reach_60_idx] if reach_60_idx is not None else None,
        "reach_60_idx": reach_60_idx,
        "yaw_pulse_start_idx": yaw_pulse_start_idx,
        "yaw_pulse_end_idx": yaw_pulse_end_idx,
        "controller_start_idx": controller_start_idx,
        "yaw_pulse_start": t_rel[yaw_pulse_start_idx] if yaw_pulse_start_idx is not None else None,
        "yaw_pulse_end": t_rel[yaw_pulse_end_idx] if yaw_pulse_end_idx is not None else None,
        "controller_start": t_rel[controller_start_idx] if controller_start_idx is not None else None,
    }


def status_or_value(value):
    if value is None:
        return INSUFFICIENT_DATA
    return value


def time_from_index(t_rel, start_idx, predicate):
    idx = first_index(t_rel, predicate, start=start_idx)
    if idx is None:
        return NOT_REACHED
    return t_rel[idx] - t_rel[start_idx]


def compute_metrics(run, events):
    t_rel = run["t_rel"]
    phi = run["phi"]
    p = run["p"]
    alt_ft = run["alt_ft"]

    peak_idx = max(range(len(phi)), key=lambda i: abs(phi[i]))
    peak_bank_deg = abs(phi[peak_idx])
    overshoot = max(0.0, peak_bank_deg - TARGET_BANK_DEG)

    metrics = {
        "file": str(run["path"]),
        "time_to_60deg_bank": events["time_to_60deg_bank"] if events["time_to_60deg_bank"] is not None else NOT_REACHED,
        "peak_bank_deg": peak_bank_deg,
        "bank_overshoot_deg": overshoot,
        "roll_rate_at_60deg": p[events["reach_60_idx"]] if events["reach_60_idx"] is not None else NOT_REACHED,
        "roll_rate_at_controller_start": NO_CONTROLLER_PHASE_FOUND,
        "time_from_controller_start_to_|phi|<10deg": NO_CONTROLLER_PHASE_FOUND,
        "time_from_controller_start_to_|phi|<5deg": NO_CONTROLLER_PHASE_FOUND,
        "time_from_controller_start_to_|p|<1deg_s": NO_CONTROLLER_PHASE_FOUND,
        "settling_time_phi_to_5deg_hold_2s": NO_CONTROLLER_PHASE_FOUND,
        "phi_rms_10s_after_controller_start": NO_CONTROLLER_PHASE_FOUND,
        "p_rms_10s_after_controller_start": NO_CONTROLLER_PHASE_FOUND,
        "max_abs_roll_rate_after_controller_start": NO_CONTROLLER_PHASE_FOUND,
        "min_altitude_after_controller_start_ft": NO_CONTROLLER_PHASE_FOUND,
        "max_altitude_deviation_from_controller_start_ft": NO_CONTROLLER_PHASE_FOUND,
        "net_altitude_change_from_controller_start_to_end_ft": NO_CONTROLLER_PHASE_FOUND,
    }

    cidx = events["controller_start_idx"]
    if cidx is None:
        return metrics

    metrics["roll_rate_at_controller_start"] = p[cidx]
    metrics["time_from_controller_start_to_|phi|<10deg"] = time_from_index(
        t_rel, cidx, lambda i: abs(phi[i]) < 10.0
    )
    metrics["time_from_controller_start_to_|phi|<5deg"] = time_from_index(
        t_rel, cidx, lambda i: abs(phi[i]) < 5.0
    )
    metrics["time_from_controller_start_to_|p|<1deg_s"] = time_from_index(
        t_rel, cidx, lambda i: abs(p[i]) < 1.0
    )

    settle_idx = first_time_hold(
        t_rel,
        phi,
        lambda i: abs(phi[i]) < 5.0,
        hold_s=2.0,
        start_idx=cidx,
    )
    metrics["settling_time_phi_to_5deg_hold_2s"] = (
        t_rel[settle_idx] - t_rel[cidx] if settle_idx is not None else NOT_REACHED
    )

    phi_window = window_values(t_rel, phi, t_rel[cidx], 10.0)
    p_window = window_values(t_rel, p, t_rel[cidx], 10.0)
    metrics["phi_rms_10s_after_controller_start"] = (
        rms(phi_window) if phi_window is not None else INSUFFICIENT_DATA
    )
    metrics["p_rms_10s_after_controller_start"] = (
        rms(p_window) if p_window is not None else INSUFFICIENT_DATA
    )

    p_after = p[cidx:]
    alt_after = alt_ft[cidx:]
    alt0 = alt_ft[cidx]
    metrics["max_abs_roll_rate_after_controller_start"] = max(abs(v) for v in p_after) if p_after else INSUFFICIENT_DATA
    metrics["min_altitude_after_controller_start_ft"] = min(alt_after) if alt_after else INSUFFICIENT_DATA
    metrics["max_altitude_deviation_from_controller_start_ft"] = (
        max(abs(v - alt0) for v in alt_after) if alt_after else INSUFFICIENT_DATA
    )
    metrics["net_altitude_change_from_controller_start_to_end_ft"] = (
        alt_after[-1] - alt0 if alt_after else INSUFFICIENT_DATA
    )

    return metrics


def write_summary_outputs(metrics):
    path = Path(metrics["file"])
    json_path = path.with_name(f"{path.stem}_summary.json")
    csv_path = path.with_name(f"{path.stem}_summary.csv")

    with json_path.open("w") as f:
        json.dump(metrics, f, indent=2)

    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(metrics.keys()))
        writer.writeheader()
        writer.writerow(metrics)

    return json_path, csv_path
