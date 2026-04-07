import csv
import json
import math
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from estimate_ab_id import (
    build_windows,
    estimate_a,
    estimate_ab_joint,
    estimate_b_given_a,
    load_csv_numeric,
    windows_to_mask,
)
from estimate_lateral_mimo import compute_closed_loop
from run_controlled import LQR_TUNING_CANDIDATES, compute_lqr_mimo_gain, get_lqr_tuning


ROOT = Path(__file__).resolve().parent.parent
RUNS_DIR = ROOT / "data" / "runs"
OUT_DIR = ROOT / "data" / "recovery"

UNRECOVERABLE = "[UNRECOVERABLE FROM CURRENT FILES]"
NOT_REACHED = "NOT_REACHED"


TARGETS = [
    {
        "controller_type": "baseline",
        "run_path": RUNS_DIR / "baseline_1775260512.csv",
    },
    {
        "controller_type": "PD",
        "run_path": RUNS_DIR / "controlled_1774837914.csv",
    },
    {
        "controller_type": "2-state LQR",
        "run_path": RUNS_DIR / "controlled_1774849067.csv",
    },
    {
        "controller_type": "2-state LQI",
        "run_path": RUNS_DIR / "controlled_1774927980.csv",
    },
    {
        "controller_type": "4-state LQR",
        "run_path": RUNS_DIR / "controlled_1774975570.csv",
    },
]


def safe_float(value):
    if value in (None, ""):
        return math.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def finite_list(values):
    return [v for v in values if math.isfinite(v)]


def rms(values):
    values = finite_list(values)
    if not values:
        return None
    return math.sqrt(sum(v * v for v in values) / len(values))


def first_index(seq, predicate):
    for i, item in enumerate(seq):
        if predicate(item):
            return i
    return None


def first_time_hold(times, values, threshold, hold_s, start_idx):
    stable_idx = None
    for i in range(start_idx, len(values)):
        if math.isfinite(values[i]) and abs(values[i]) < threshold:
            if stable_idx is None:
                stable_idx = i
            elif times[i] - times[stable_idx] >= hold_s:
                return stable_idx
        else:
            stable_idx = None
    return None


@dataclass
class RunData:
    path: Path
    rows: list
    fieldnames: list
    times: np.ndarray
    t_rel: np.ndarray
    phi: np.ndarray
    p: np.ndarray
    alt_ft: np.ndarray


def load_run(path):
    with path.open("r", newline="") as f:
        rows = list(csv.DictReader(f))
        fieldnames = list(rows[0].keys()) if rows else []
    if not rows:
        raise ValueError(f"{path} is empty")

    times = np.array([safe_float(row.get("t")) for row in rows], dtype=float)
    t_rel = times - times[0]
    phi = np.array([safe_float(row.get("phi_deg")) for row in rows], dtype=float)
    p = np.array([safe_float(row.get("p_deg_s")) for row in rows], dtype=float)
    alt_ft = np.array(
        [
            safe_float(row.get("alt_ft"))
            if row.get("alt_ft") not in (None, "")
            else safe_float(row.get("alt_m")) * 3.28084
            for row in rows
        ],
        dtype=float,
    )
    return RunData(path=path, rows=rows, fieldnames=fieldnames, times=times, t_rel=t_rel, phi=phi, p=p, alt_ft=alt_ft)


def target_bank_and_time(run):
    peak_idx = int(np.nanargmax(np.abs(run.phi)))
    target_bank = 60.0 if run.phi[peak_idx] >= 0.0 else -60.0
    hit_idx = first_index(run.phi, lambda value: math.isfinite(value) and abs(value - target_bank) <= 3.0)
    return target_bank, hit_idx


def detect_handoff(run, controller_type):
    if "controller_handoff_elapsed_s" in run.fieldnames:
        values = [row.get("controller_handoff_elapsed_s", "") for row in run.rows]
        idx = first_index(values, lambda value: value not in ("", None))
        if idx is not None:
            return idx, "first logged controller_handoff_elapsed_s"

    if "controlled_phase" in run.fieldnames:
        target_bank, reach_60_idx = target_bank_and_time(run)
        if reach_60_idx is not None:
            target_sign = 1.0 if target_bank >= 0.0 else -1.0
            cmd_roll = [safe_float(row.get("cmd_roll")) for row in run.rows]

            def corrective_takeover(i):
                if i + 2 >= len(cmd_roll):
                    return False
                sustained = 0
                for j in range(i, min(i + 5, len(cmd_roll))):
                    if math.isfinite(cmd_roll[j]) and target_sign * cmd_roll[j] <= -0.03:
                        sustained += 1
                return sustained >= 3

            for i in range(reach_60_idx + 1, len(cmd_roll)):
                if corrective_takeover(i):
                    return i, "first sustained corrective cmd_roll after reaching target bank"

        phase = [row.get("controlled_phase", "") for row in run.rows]
        idx = first_index(phase, lambda value: value == "controller")
        if idx is not None:
            return idx, "fallback: first controlled_phase == controller"

    if "baseline_phase" in run.fieldnames:
        phase = [row.get("baseline_phase", "") for row in run.rows]
        idx = first_index(phase, lambda value: value == "neutral_observation")
        if idx is not None:
            return idx, "first baseline_phase == neutral_observation"

    _, hit_idx = target_bank_and_time(run)
    if hit_idx is not None:
        return hit_idx, "fallback: first sample within 3 deg of target bank"

    return None, "handoff could not be detected"


def max_command_info(rows, start_idx):
    command_columns = [
        "cmd_roll",
        "cmd_pitch",
        "cmd_yaw",
        "roll_cmd_raw_lqr",
        "roll_cmd_raw_lqi",
        "roll_cmd_raw_lqr_mimo",
        "yaw_cmd_raw_lqr_mimo",
        "roll_cmd_clamped",
        "yaw_cmd_clamped",
    ]
    maxima = {}
    overall = None
    for column in command_columns:
        values = [safe_float(row.get(column)) for row in rows[start_idx:]]
        finite = finite_list(values)
        if not finite:
            continue
        max_abs = max(abs(v) for v in finite)
        maxima[column] = max_abs
        overall = max_abs if overall is None else max(overall, max_abs)
    return overall, maxima


def compute_run_metrics(run, controller_type):
    handoff_idx, handoff_reason = detect_handoff(run, controller_type)
    target_bank, bank60_idx = target_bank_and_time(run)

    metrics = {
        "target_bank_deg": target_bank,
        "handoff_detection": handoff_reason,
        "time_to_60deg_bank_s": run.t_rel[bank60_idx] if bank60_idx is not None else NOT_REACHED,
        "roll_rate_at_60deg_deg_s": run.p[bank60_idx] if bank60_idx is not None else UNRECOVERABLE,
    }
    if handoff_idx is None:
        metrics["controller_handoff_elapsed_s"] = UNRECOVERABLE
        metrics["bank_angle_at_handoff_deg"] = UNRECOVERABLE
        metrics["roll_rate_at_handoff_deg_s"] = UNRECOVERABLE
        metrics["peak_bank_after_handoff_deg"] = UNRECOVERABLE
        metrics["rms_bank_after_handoff_deg"] = UNRECOVERABLE
        metrics["time_to_abs_phi_lt_10_deg_s"] = UNRECOVERABLE
        metrics["time_to_abs_phi_lt_5_deg_s"] = UNRECOVERABLE
        metrics["settling_time_abs_phi_lt_5_deg_hold_2s_s"] = UNRECOVERABLE
        metrics["altitude_loss_after_handoff_ft"] = UNRECOVERABLE
        metrics["maximum_roll_rate_after_handoff_deg_s"] = UNRECOVERABLE
        metrics["minimum_negative_bank_after_handoff_deg"] = UNRECOVERABLE
        metrics["maximum_logged_command_magnitude"] = UNRECOVERABLE
        metrics["maximum_logged_command_by_column"] = {}
        return metrics

    phi_after = run.phi[handoff_idx:]
    p_after = run.p[handoff_idx:]
    alt_after = run.alt_ft[handoff_idx:]
    times_after = run.t_rel[handoff_idx:]

    def time_to_threshold(threshold):
        for i in range(handoff_idx, len(run.phi)):
            if math.isfinite(run.phi[i]) and abs(run.phi[i]) < threshold:
                return run.t_rel[i] - run.t_rel[handoff_idx]
        return NOT_REACHED

    settle_idx = first_time_hold(run.t_rel, run.phi, threshold=5.0, hold_s=2.0, start_idx=handoff_idx)
    max_cmd, max_cmd_by_col = max_command_info(run.rows, handoff_idx)

    alt0 = run.alt_ft[handoff_idx]
    min_alt = float(np.nanmin(alt_after)) if len(alt_after) else math.nan
    altitude_loss = alt0 - min_alt if math.isfinite(alt0) and math.isfinite(min_alt) else None

    metrics.update(
        {
            "controller_handoff_elapsed_s": float(run.t_rel[handoff_idx]),
            "bank_angle_at_handoff_deg": float(run.phi[handoff_idx]),
            "roll_rate_at_handoff_deg_s": float(run.p[handoff_idx]),
            "peak_bank_after_handoff_deg": float(np.nanmax(np.abs(phi_after))),
            "rms_bank_after_handoff_deg": rms(phi_after.tolist()),
            "time_to_abs_phi_lt_10_deg_s": time_to_threshold(10.0),
            "time_to_abs_phi_lt_5_deg_s": time_to_threshold(5.0),
            "settling_time_abs_phi_lt_5_deg_hold_2s_s": (
                float(run.t_rel[settle_idx] - run.t_rel[handoff_idx]) if settle_idx is not None else NOT_REACHED
            ),
            "altitude_loss_after_handoff_ft": altitude_loss,
            "maximum_roll_rate_after_handoff_deg_s": float(np.nanmax(np.abs(p_after))),
            "minimum_negative_bank_after_handoff_deg": float(np.nanmin(phi_after)),
            "maximum_logged_command_magnitude": max_cmd if max_cmd is not None else UNRECOVERABLE,
            "maximum_logged_command_by_column": max_cmd_by_col,
            "post_handoff_duration_s": float(times_after[-1] - times_after[0]) if len(times_after) > 1 else 0.0,
        }
    )
    return metrics


def git_show_text(revision, path):
    result = subprocess.run(
        ["git", "show", f"{revision}:{path}"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    )
    return result.stdout


def recover_pd_gains():
    text = git_show_text("aedd842", "external/controller_roll.py")
    values = {}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("Kp ="):
            values["Kp"] = float(line.split("=")[1].strip())
        elif line.startswith("Kd ="):
            values["Kd"] = float(line.split("=")[1].strip())
        elif line.startswith("Ky ="):
            values["Ky"] = float(line.split("=")[1].strip())
    return {
        "files_used": [
            "git:aedd842:external/controller_roll.py",
            "data/runs/controlled_1774837914.csv",
        ],
        "gains": values,
        "notes": [
            "Recovered from the historical controller file in local git history.",
            "The March 29 PD run log does not store gains directly.",
        ],
    }


def recover_2state_model():
    candidates = [
        RUNS_DIR / "run_1774645989.csv",
        RUNS_DIR / "run_1774842310.csv",
    ]
    candidate_summaries = []
    chosen = None
    for path in candidates:
        data = load_csv_numeric(path)
        neutral_windows, pulse_windows = build_windows(data)
        neutral_mask = windows_to_mask(len(data["t_rel"]), neutral_windows)
        pulse_mask = windows_to_mask(len(data["t_rel"]), pulse_windows)
        joint_mask = neutral_mask | pulse_mask

        a_neutral, a_r2, a_n = estimate_a(data, neutral_mask)
        b_given_a, b_r2, b_n = estimate_b_given_a(data, a_neutral, pulse_mask)
        joint_a, joint_b, joint_r2, joint_n = estimate_ab_joint(data, joint_mask)
        summary = {
            "path": str(path.relative_to(ROOT)),
            "a_neutral_1_s": a_neutral,
            "a_neutral_r2": a_r2,
            "a_neutral_n": a_n,
            "b_given_a_deg_s2_per_cmd": b_given_a,
            "b_given_a_r2": b_r2,
            "b_given_a_n": b_n,
            "joint_a_1_s": joint_a,
            "joint_b_deg_s2_per_cmd": joint_b,
            "joint_r2": joint_r2,
            "joint_n": joint_n,
        }
        candidate_summaries.append(summary)
        if path.name == "run_1774842310.csv":
            chosen = summary

    a = chosen["joint_a_1_s"]
    b = chosen["joint_b_deg_s2_per_cmd"]
    a_matrix = np.array([[0.0, 1.0], [0.0, a]], dtype=float)
    b_matrix = np.array([[0.0], [b]], dtype=float)
    return {
        "chosen_id_run": chosen["path"],
        "candidate_id_runs": candidate_summaries,
        "selection_reason": (
            "Selected the latest open-loop pulse-identification run immediately preceding the 2-state LQR run. "
            "The earlier run_1774645989.csv is also preserved as an ambiguity candidate."
        ),
        "regression_method": "least-squares on smoothed roll-rate dynamics from external/estimate_ab_id.py",
        "A": a_matrix.tolist(),
        "B": b_matrix.tolist(),
    }


def recover_2state_lqr_gain(run_path):
    run = load_run(run_path)
    handoff_idx, _ = detect_handoff(run, "2-state LQR")
    phi = run.phi[handoff_idx:]
    p = run.p[handoff_idx:]
    rows = run.rows[handoff_idx:]
    u = np.array([safe_float(row.get("roll_cmd_raw_lqr")) for row in rows], dtype=float)
    x = np.column_stack([phi, p])
    theta, *_ = np.linalg.lstsq(x, -u, rcond=None)
    residual = u + x @ theta
    return {
        "K": theta.tolist(),
        "fit_rmse": float(np.sqrt(np.mean(residual ** 2))),
        "fit_max_abs_error": float(np.max(np.abs(residual))),
        "recovery_method": "least-squares fit of logged roll_cmd_raw_lqr = -K [phi, p]^T during controlled_phase == controller",
    }


def recover_2state_lqi_gain(run_path):
    run = load_run(run_path)
    handoff_idx, _ = detect_handoff(run, "2-state LQI")
    t = run.times[handoff_idx:]
    phi = run.phi[handoff_idx:]
    p = run.p[handoff_idx:]
    rows = run.rows[handoff_idx:]
    u = np.array([safe_float(row.get("roll_cmd_raw_lqi")) for row in rows], dtype=float)

    z = np.zeros_like(phi)
    for i in range(1, len(phi)):
        dt = t[i] - t[i - 1]
        z[i] = z[i - 1] + 0.5 * (phi[i] + phi[i - 1]) * dt

    x_aug = np.column_stack([phi, p, z])
    theta, *_ = np.linalg.lstsq(x_aug, -u, rcond=None)
    residual = u + x_aug @ theta
    return {
        "K_aug": theta.tolist(),
        "K_state": theta[:2].tolist(),
        "Ki": float(theta[2]),
        "integrator_definition": "z_dot = phi_deg, initialized to zero at controller handoff and integrated with trapezoidal rule",
        "fit_rmse": float(np.sqrt(np.mean(residual ** 2))),
        "fit_max_abs_error": float(np.max(np.abs(residual))),
        "recovery_method": "least-squares fit of logged roll_cmd_raw_lqi = -[K_phi, K_p, K_i] [phi, p, z]^T during controlled_phase == controller",
    }


def controllability_rank(a_matrix, b_matrix):
    n = a_matrix.shape[0]
    mats = [b_matrix]
    current = b_matrix
    for _ in range(1, n):
        current = a_matrix @ current
        mats.append(current)
    ctrb = np.hstack(mats)
    return int(np.linalg.matrix_rank(ctrb))


def complex_list(values):
    out = []
    for value in values:
        out.append({"real": float(np.real(value)), "imag": float(np.imag(value))})
    return out


def recover_2state_lqr_design(run_path):
    model = recover_2state_model()
    gain = recover_2state_lqr_gain(run_path)
    a = np.array(model["A"], dtype=float)
    b = np.array(model["B"], dtype=float)
    k = np.array([gain["K"]], dtype=float)
    a_cl = a - b @ k
    return {
        "files_used": [
            model["chosen_id_run"],
            str(run_path.relative_to(ROOT)),
            "external/estimate_ab_id.py",
        ],
        "model": model,
        "Q": UNRECOVERABLE,
        "R": UNRECOVERABLE,
        "K": gain,
        "A_cl": a_cl.tolist(),
        "open_loop_eigenvalues": complex_list(np.linalg.eigvals(a)),
        "closed_loop_eigenvalues": complex_list(np.linalg.eigvals(a_cl)),
        "controllability_rank": controllability_rank(a, b),
    }


def recover_2state_lqi_design(run_path):
    model = recover_2state_model()
    gain = recover_2state_lqi_gain(run_path)
    a = np.array(model["A"], dtype=float)
    b = np.array(model["B"], dtype=float)

    a_aug = np.array(
        [
            [a[0, 0], a[0, 1], 0.0],
            [a[1, 0], a[1, 1], 0.0],
            [1.0, 0.0, 0.0],
        ],
        dtype=float,
    )
    b_aug = np.array([[b[0, 0]], [b[1, 0]], [0.0]], dtype=float)
    k_aug = np.array([gain["K_aug"]], dtype=float)
    a_cl = a_aug - b_aug @ k_aug
    return {
        "files_used": [
            model["chosen_id_run"],
            str(run_path.relative_to(ROOT)),
            "external/estimate_ab_id.py",
        ],
        "model_base": model,
        "A_aug": a_aug.tolist(),
        "B_aug": b_aug.tolist(),
        "Q": UNRECOVERABLE,
        "R": UNRECOVERABLE,
        "K_aug": gain,
        "open_loop_eigenvalues": complex_list(np.linalg.eigvals(a_aug)),
        "closed_loop_eigenvalues": complex_list(np.linalg.eigvals(a_cl)),
        "A_cl": a_cl.tolist(),
        "controllability_rank": controllability_rank(a_aug, b_aug),
    }


def recover_4state_lqr_design():
    summary_path = RUNS_DIR / "run_1774933421_lateral_mimo_summary.json"
    with summary_path.open("r", encoding="utf-8") as f:
        model_summary = json.load(f)

    a = np.array(model_summary["A"], dtype=float)
    b = np.array(model_summary["B"], dtype=float)

    run = load_run(RUNS_DIR / "controlled_1774975570.csv")
    handoff_idx, _ = detect_handoff(run, "4-state LQR")
    rows = run.rows[handoff_idx:]
    x = np.column_stack(
        [
            [safe_float(row.get("phi_deg")) for row in rows],
            [safe_float(row.get("p_deg_s")) for row in rows],
            [safe_float(row.get("beta_deg")) for row in rows],
            [safe_float(row.get("r_deg_s")) for row in rows],
        ]
    )
    u_roll = np.array([safe_float(row.get("roll_cmd_raw_lqr_mimo")) for row in rows], dtype=float)
    u_yaw = np.array([safe_float(row.get("yaw_cmd_raw_lqr_mimo")) for row in rows], dtype=float)

    candidate_errors = []
    for name in sorted(LQR_TUNING_CANDIDATES):
        tuning = get_lqr_tuning(name)
        design = compute_lqr_mimo_gain(tuning)
        prediction = -(x @ design["K_physical"].T)
        roll_rmse = float(np.sqrt(np.mean((u_roll - prediction[:, 0]) ** 2)))
        yaw_rmse = float(np.sqrt(np.mean((u_yaw - prediction[:, 1]) ** 2)))
        candidate_errors.append(
            {
                "candidate": name,
                "q_diag": tuning["q_diag"],
                "r_diag": tuning["r_diag"],
                "K_physical": design["K_physical"].tolist(),
                "roll_rmse": roll_rmse,
                "yaw_rmse": yaw_rmse,
                "score": roll_rmse + yaw_rmse,
            }
        )

    candidate_errors.sort(key=lambda item: item["score"])
    best = candidate_errors[0]
    k = np.array(best["K_physical"], dtype=float)
    a_cl = compute_closed_loop(a, b, k)
    return {
        "files_used": [
            "data/runs/run_1774933421_lateral_mimo_summary.json",
            "external/run_controlled.py",
            "data/runs/controlled_1774975570.csv",
        ],
        "model_estimation_method": "multivariable least-squares on smoothed state derivatives from external/estimate_lateral_mimo.py",
        "A": model_summary["A"],
        "B": model_summary["B"],
        "fit": model_summary["fit"],
        "phi_dot_minus_p_mean_deg_s": model_summary["phi_dot_minus_p_mean_deg_s"],
        "open_loop_eigenvalues": model_summary["open_loop_eigenvalues"],
        "controllability_rank": controllability_rank(a, b),
        "best_matching_current_candidate": best,
        "candidate_match_table": candidate_errors,
        "selected_Q": best["q_diag"],
        "selected_R": best["r_diag"],
        "selected_K_physical": best["K_physical"],
        "A_cl": a_cl.tolist(),
        "closed_loop_eigenvalues": complex_list(np.linalg.eigvals(a_cl)),
        "ambiguity_note": (
            "The exact candidate name was not logged in the March 31 run CSV. "
            "The values above are the closest match among the candidates currently present in external/run_controlled.py."
        ),
    }


def recover_4state_single_input_model():
    summary_path = RUNS_DIR / "run_1774842591_lateral4_summary.json"
    with summary_path.open("r", encoding="utf-8") as f:
        return json.load(f)


def metric_value_for_table(value):
    if isinstance(value, float):
        return f"{value:.3f}"
    if value is None:
        return ""
    return str(value)


def build_markdown_table(results):
    headers = [
        "Controller",
        "Run",
        "Handoff phi (deg)",
        "Handoff p (deg/s)",
        "Peak |phi| (deg)",
        "RMS phi (deg)",
        "t to |phi|<10 (s)",
        "t to |phi|<5 (s)",
        "Settling (s)",
        "Alt loss (ft)",
        "Max |p| (deg/s)",
    ]
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(["---"] * len(headers)) + " |"]
    for result in results:
        metrics = result["performance_metrics"]
        lines.append(
            "| "
            + " | ".join(
                [
                    result["controller_type"],
                    result["run_identification"]["selected_run"],
                    metric_value_for_table(metrics["bank_angle_at_handoff_deg"]),
                    metric_value_for_table(metrics["roll_rate_at_handoff_deg_s"]),
                    metric_value_for_table(metrics["peak_bank_after_handoff_deg"]),
                    metric_value_for_table(metrics["rms_bank_after_handoff_deg"]),
                    metric_value_for_table(metrics["time_to_abs_phi_lt_10_deg_s"]),
                    metric_value_for_table(metrics["time_to_abs_phi_lt_5_deg_s"]),
                    metric_value_for_table(metrics["settling_time_abs_phi_lt_5_deg_hold_2s_s"]),
                    metric_value_for_table(metrics["altitude_loss_after_handoff_ft"]),
                    metric_value_for_table(metrics["maximum_roll_rate_after_handoff_deg_s"]),
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def write_csv_table(results, out_path):
    fieldnames = [
        "controller_type",
        "run",
        "handoff_phi_deg",
        "handoff_p_deg_s",
        "peak_abs_phi_deg",
        "rms_phi_deg",
        "time_to_abs_phi_lt_10_deg_s",
        "time_to_abs_phi_lt_5_deg_s",
        "settling_time_s",
        "altitude_loss_ft",
        "max_abs_roll_rate_deg_s",
    ]
    with out_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for result in results:
            metrics = result["performance_metrics"]
            writer.writerow(
                {
                    "controller_type": result["controller_type"],
                    "run": result["run_identification"]["selected_run"],
                    "handoff_phi_deg": metrics["bank_angle_at_handoff_deg"],
                    "handoff_p_deg_s": metrics["roll_rate_at_handoff_deg_s"],
                    "peak_abs_phi_deg": metrics["peak_bank_after_handoff_deg"],
                    "rms_phi_deg": metrics["rms_bank_after_handoff_deg"],
                    "time_to_abs_phi_lt_10_deg_s": metrics["time_to_abs_phi_lt_10_deg_s"],
                    "time_to_abs_phi_lt_5_deg_s": metrics["time_to_abs_phi_lt_5_deg_s"],
                    "settling_time_s": metrics["settling_time_abs_phi_lt_5_deg_hold_2s_s"],
                    "altitude_loss_ft": metrics["altitude_loss_after_handoff_ft"],
                    "max_abs_roll_rate_deg_s": metrics["maximum_roll_rate_after_handoff_deg_s"],
                }
            )


def make_result(target):
    controller_type = target["controller_type"]
    run_path = target["run_path"]
    run = load_run(run_path)

    run_identification = {
        "selected_run": str(run_path.relative_to(ROOT)),
        "selection_reason": "Exact run file provided in the recovery request.",
        "ambiguity": "None for the primary run file itself.",
    }
    performance_metrics = compute_run_metrics(run, controller_type)

    controller_design = {}
    model_estimation_evidence = {}
    ambiguities = []
    still_missing = []

    if controller_type == "baseline":
        controller_design = {
            "baseline_capture_gains_from_external_run_baseline_py": {
                "K_TO_TARGET": 0.018,
                "P_DAMP": 0.025,
                "CAPTURE_K_TO_TARGET": 0.010,
                "CAPTURE_P_DAMP": 0.040,
            },
            "note": "These are the baseline entry/capture gains, not a closed-loop recovery controller.",
        }
        still_missing.extend(
            [
                "state-space model for baseline [UNRECOVERABLE FROM CURRENT FILES]",
                "closed-loop matrices for baseline [UNRECOVERABLE FROM CURRENT FILES]",
                "eigenvalues for baseline [UNRECOVERABLE FROM CURRENT FILES]",
            ]
        )
    elif controller_type == "PD":
        controller_design = recover_pd_gains()
        still_missing.extend(
            [
                "PD closed-loop matrix [UNRECOVERABLE FROM CURRENT FILES]",
                "PD weighting matrices [UNRECOVERABLE FROM CURRENT FILES]",
                "PD eigenvalues [UNRECOVERABLE FROM CURRENT FILES]",
                "PD controllability rank [UNRECOVERABLE FROM CURRENT FILES]",
            ]
        )
    elif controller_type == "2-state LQR":
        controller_design = recover_2state_lqr_design(run_path)
        model_estimation_evidence = controller_design["model"]
        still_missing.extend(
            [
                "2-state LQR Q matrix [UNRECOVERABLE FROM CURRENT FILES]",
                "2-state LQR R matrix [UNRECOVERABLE FROM CURRENT FILES]",
            ]
        )
        ambiguities.append(model_estimation_evidence["selection_reason"])
    elif controller_type == "2-state LQI":
        controller_design = recover_2state_lqi_design(run_path)
        model_estimation_evidence = controller_design["model_base"]
        still_missing.extend(
            [
                "2-state LQI Q matrix [UNRECOVERABLE FROM CURRENT FILES]",
                "2-state LQI R matrix [UNRECOVERABLE FROM CURRENT FILES]",
            ]
        )
        ambiguities.append(model_estimation_evidence["selection_reason"])
    elif controller_type == "4-state LQR":
        controller_design = recover_4state_lqr_design()
        model_estimation_evidence = {
            "mimo_model_used_for_controller": {
                "A": controller_design["A"],
                "B": controller_design["B"],
                "fit": controller_design["fit"],
                "phi_dot_minus_p_mean_deg_s": controller_design["phi_dot_minus_p_mean_deg_s"],
                "open_loop_eigenvalues": controller_design["open_loop_eigenvalues"],
                "regression_method": controller_design["model_estimation_method"],
            },
            "earlier_single_input_4state_estimate": recover_4state_single_input_model(),
        }
        ambiguities.append(controller_design["ambiguity_note"])

    return {
        "controller_type": controller_type,
        "Recovered values": {
            "performance_metrics": performance_metrics,
            "controller_design": controller_design,
            "model_estimation_evidence": model_estimation_evidence,
        },
        "Files used": sorted(
            set(
                [str(run_path.relative_to(ROOT))]
                + controller_design.get("files_used", [])
                + ([str(RUNS_DIR / "run_1774842591_lateral4_summary.json").replace(str(ROOT) + "/", "")] if controller_type == "4-state LQR" else [])
            )
        ),
        "How each value was obtained": {
            "run_identification": run_identification,
            "performance_metrics": performance_metrics["handoff_detection"],
            "controller_design": controller_design.get("recovery_method", controller_design.get("note", "See nested controller_design fields.")),
        },
        "Ambiguities or assumptions": ambiguities or ["None beyond file-format-based handoff detection."],
        "Still missing": still_missing,
        "Suggested placeholders for unrecoverable values": still_missing,
        "run_identification": run_identification,
        "performance_metrics": performance_metrics,
        "controller_design": controller_design,
        "model_estimation_evidence": model_estimation_evidence,
    }


def write_markdown(results, table_md, out_path):
    lines = ["# Technical Report Recovery", ""]
    lines.append("## Recovered values")
    for result in results:
        lines.append(f"### {result['controller_type']}")
        lines.append(f"- Run: `{result['run_identification']['selected_run']}`")
        lines.append(f"- Handoff metric source: {result['performance_metrics']['handoff_detection']}")
        lines.append("")
    lines.append("## Files used")
    for result in results:
        lines.append(f"### {result['controller_type']}")
        for item in result["Files used"]:
            lines.append(f"- `{item}`")
        lines.append("")
    lines.append("## How each value was obtained")
    for result in results:
        lines.append(f"### {result['controller_type']}")
        lines.append(f"- Run identification: {result['run_identification']['selection_reason']}")
        lines.append(f"- Performance metrics: {result['performance_metrics']['handoff_detection']}")
        lines.append("")
    lines.append("## Ambiguities or assumptions")
    for result in results:
        lines.append(f"### {result['controller_type']}")
        for item in result["Ambiguities or assumptions"]:
            lines.append(f"- {item}")
        lines.append("")
    lines.append("## Still missing")
    for result in results:
        lines.append(f"### {result['controller_type']}")
        for item in result["Still missing"]:
            lines.append(f"- {item}")
        lines.append("")
    lines.append("## Suggested placeholders for unrecoverable values")
    lines.append(f"`{UNRECOVERABLE}`")
    lines.append("")
    lines.append("## Paste-ready table")
    lines.append(table_md)
    out_path.write_text("\n".join(lines), encoding="utf-8")


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    results = [make_result(target) for target in TARGETS]

    table_md = build_markdown_table(results)
    json_path = OUT_DIR / "technical_report_recovery.json"
    csv_path = OUT_DIR / "technical_report_recovery_table.csv"
    md_path = OUT_DIR / "technical_report_recovery.md"

    with json_path.open("w", encoding="utf-8") as f:
        json.dump({"results": results, "paste_ready_table_markdown": table_md}, f, indent=2)
    write_csv_table(results, csv_path)
    write_markdown(results, table_md, md_path)

    print(f"json: {json_path}")
    print(f"csv: {csv_path}")
    print(f"md: {md_path}")
    print()
    print(table_md)


if __name__ == "__main__":
    main()
