import argparse
import csv
import json
from pathlib import Path

import numpy as np


SMOOTH_N = 7
MIN_SAMPLES = 12
STATE_COLUMNS = ["phi_deg", "p_deg_s", "beta_deg", "r_deg_s"]
INPUT_COLUMNS = ["cmd_roll", "cmd_yaw"]
REQUIRED_COLUMNS = ["t", *STATE_COLUMNS, *INPUT_COLUMNS]


def moving_average(x, n):
    if n <= 1:
        return x.copy()
    kernel = np.ones(n, dtype=float) / float(n)
    xpad = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(xpad, kernel, mode="valid")


def nan_interp(x):
    y = x.astype(float).copy()
    finite = np.isfinite(y)
    if finite.all():
        return y
    idx = np.arange(len(y))
    if not finite.any():
        raise ValueError("Signal contains no finite samples.")
    y[~finite] = np.interp(idx[~finite], idx[finite], y[finite])
    return y


def parse_float_list(text, expected_len):
    values = [value.strip() for value in text.split(",")]
    if len(values) != expected_len:
        raise argparse.ArgumentTypeError(
            f"Expected {expected_len} comma-separated values, got {len(values)}: {text!r}"
        )
    try:
        return np.asarray([float(value) for value in values], dtype=float)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Could not parse gain list {text!r}: {exc}") from exc


def load_csv(path):
    with path.open("r", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    if not rows:
        raise ValueError(f"{path} is empty.")

    missing = [column for column in REQUIRED_COLUMNS if column not in reader.fieldnames]
    if missing:
        found = ", ".join(reader.fieldnames or [])
        raise ValueError(
            f"{path} is missing required columns: {', '.join(missing)}. "
            f"Found columns: {found}."
        )

    data = {}
    for key in reader.fieldnames:
        values = []
        for row in rows:
            try:
                values.append(float(row[key]))
            except (TypeError, ValueError):
                values.append(np.nan)
        data[key] = np.asarray(values, dtype=float)

    return data, len(rows)


def prepare_data(data):
    finite_time = np.isfinite(data["t"])
    if "cmd_enable" in data:
        finite_time &= np.nan_to_num(data["cmd_enable"], nan=0.0) > 0.5

    if int(finite_time.sum()) < MIN_SAMPLES:
        raise ValueError("Not enough finite enabled samples for estimation.")

    trimmed = {key: value[finite_time] for key, value in data.items()}
    t = trimmed["t"]
    order = np.argsort(t)
    trimmed = {key: value[order] for key, value in trimmed.items()}

    t_rel = trimmed["t"] - trimmed["t"][0]
    dt = np.diff(t_rel)
    if np.any(dt <= 0.0):
        raise ValueError("Timestamps must be strictly increasing after filtering.")

    prepared = {
        "t_rel": t_rel,
        "raw_sample_count": len(t_rel),
        "nan_counts": {},
    }

    for key in [*STATE_COLUMNS, *INPUT_COLUMNS]:
        prepared["nan_counts"][key] = int(np.count_nonzero(~np.isfinite(trimmed[key])))
        prepared[key] = nan_interp(trimmed[key])

    finite_counts = {
        key: int(np.isfinite(trimmed[key]).sum())
        for key in [*STATE_COLUMNS, *INPUT_COLUMNS]
    }
    for key, count in finite_counts.items():
        if count < MIN_SAMPLES:
            raise ValueError(f"{key} does not contain enough finite samples for estimation.")

    prepared["phi_smooth"] = moving_average(prepared["phi_deg"], SMOOTH_N)
    prepared["p_smooth"] = moving_average(prepared["p_deg_s"], SMOOTH_N)
    prepared["beta_smooth"] = moving_average(prepared["beta_deg"], SMOOTH_N)
    prepared["r_smooth"] = moving_average(prepared["r_deg_s"], SMOOTH_N)
    prepared["u_a_smooth"] = moving_average(prepared["cmd_roll"], SMOOTH_N)
    prepared["u_r_smooth"] = moving_average(prepared["cmd_yaw"], SMOOTH_N)

    prepared["pdot"] = np.gradient(prepared["p_smooth"], t_rel)
    prepared["betadot"] = np.gradient(prepared["beta_smooth"], t_rel)
    prepared["rdot"] = np.gradient(prepared["r_smooth"], t_rel)
    prepared["phidot_from_data"] = np.gradient(prepared["phi_smooth"], t_rel)
    return prepared


def fit_row(y, regressors):
    theta, *_ = np.linalg.lstsq(regressors, y, rcond=None)
    yhat = regressors @ theta
    residual = y - yhat
    ss_res = float(np.sum(residual ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = np.nan if ss_tot == 0.0 else 1.0 - ss_res / ss_tot
    return theta, yhat, r2


def estimate_structured_model(prepared):
    phi = prepared["phi_smooth"]
    p = prepared["p_smooth"]
    beta = prepared["beta_smooth"]
    r = prepared["r_smooth"]
    u_a = prepared["u_a_smooth"]
    u_r = prepared["u_r_smooth"]

    regressors = np.column_stack([phi, p, beta, r, u_a, u_r])

    p_theta, _, p_r2 = fit_row(prepared["pdot"], regressors)
    beta_theta, _, beta_r2 = fit_row(prepared["betadot"], regressors)
    r_theta, _, r_r2 = fit_row(prepared["rdot"], regressors)

    a_matrix = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            p_theta[:4],
            beta_theta[:4],
            r_theta[:4],
        ],
        dtype=float,
    )
    b_matrix = np.array(
        [
            [0.0, 0.0],
            [p_theta[4], p_theta[5]],
            [beta_theta[4], beta_theta[5]],
            [r_theta[4], r_theta[5]],
        ],
        dtype=float,
    )

    return {
        "A": a_matrix,
        "B": b_matrix,
        "rows": {
            "p_dot": {"theta": p_theta, "r2": p_r2},
            "beta_dot": {"theta": beta_theta, "r2": beta_r2},
            "r_dot": {"theta": r_theta, "r2": r_r2},
        },
        "sample_count": len(phi),
    }


def format_vector(x):
    parts = []
    for value in x:
        if abs(value.imag) < 1e-12:
            parts.append(f"{value.real:+.6f}")
        else:
            parts.append(f"{value.real:+.6f}{value.imag:+.6f}j")
    return "[" + ", ".join(parts) + "]"


def format_matrix(matrix):
    return np.array2string(matrix, formatter={"float_kind": lambda x: f"{x:+.6f}"})


def compute_closed_loop(a_matrix, b_matrix, gain_matrix):
    return a_matrix - (b_matrix @ gain_matrix)


def print_results(result, prepared, gain_matrix=None):
    a_matrix = result["A"]
    b_matrix = result["B"]

    print("Structured MIMO model:")
    print("  phi_dot  = p")
    print("  p_dot    = a_p_phi*phi + a_p_p*p + a_p_beta*beta + a_p_r*r + b_p_a*u_a + b_p_rud*u_r")
    print("  beta_dot = a_b_phi*phi + a_b_p*p + a_b_beta*beta + a_b_r*r + b_b_a*u_a + b_b_rud*u_r")
    print("  r_dot    = a_r_phi*phi + a_r_p*p + a_r_beta*beta + a_r_r*r + b_r_a*u_a + b_r_rud*u_r")
    print()

    print(f"Samples used: {result['sample_count']}")
    print(f"Smoothing window: {SMOOTH_N} samples")
    print("NaN samples filled by interpolation before smoothing/differentiation:")
    for key in [*STATE_COLUMNS, *INPUT_COLUMNS]:
        print(f"  {key}: {prepared['nan_counts'][key]}")
    print(
        "phi consistency check: "
        f"mean(phi_dot_from_data - p) = {np.mean(prepared['phidot_from_data'] - prepared['p_smooth']):+.6f} deg/s"
    )
    print()

    labels = ["phi", "p", "beta", "r", "u_a", "u_r"]
    for row_name in ["p_dot", "beta_dot", "r_dot"]:
        row = result["rows"][row_name]
        print(f"{row_name}:")
        for label, coeff in zip(labels, row["theta"]):
            print(f"  {label:>4s}: {coeff:+.6f}")
        print(f"  R^2: {row['r2']:.4f}")
        print()

    print("A =")
    print(format_matrix(a_matrix))
    print()
    print("B =")
    print(format_matrix(b_matrix))
    print()

    open_loop_eigs = np.linalg.eigvals(a_matrix)
    print("Open-loop eigenvalues of A:")
    print(format_vector(open_loop_eigs))
    print()

    if gain_matrix is not None:
        a_cl = compute_closed_loop(a_matrix, b_matrix, gain_matrix)
        closed_loop_eigs = np.linalg.eigvals(a_cl)
        print("Closed-loop gain matrix K (u = -K x) =")
        print(format_matrix(gain_matrix))
        print()
        print("A_cl = A - B K =")
        print(format_matrix(a_cl))
        print()
        print("Closed-loop eigenvalues of A_cl:")
        print(format_vector(closed_loop_eigs))
        print()


def save_summary(path, result, prepared, raw_row_count, gain_matrix=None):
    a_matrix = result["A"]
    b_matrix = result["B"]
    summary = {
        "csv_path": str(path),
        "raw_row_count": raw_row_count,
        "sample_count_used": result["sample_count"],
        "smoothing_window_samples": SMOOTH_N,
        "state_vector": ["phi_deg", "p_deg_s", "beta_deg", "r_deg_s"],
        "input_vector": ["cmd_roll", "cmd_yaw"],
        "model_structure": {
            "phi_dot": "p",
            "p_dot": "a_p_phi*phi + a_p_p*p + a_p_beta*beta + a_p_r*r + b_p_a*u_a + b_p_rud*u_r",
            "beta_dot": "a_b_phi*phi + a_b_p*p + a_b_beta*beta + a_b_r*r + b_b_a*u_a + b_b_rud*u_r",
            "r_dot": "a_r_phi*phi + a_r_p*p + a_r_beta*beta + a_r_r*r + b_r_a*u_a + b_r_rud*u_r",
        },
        "A": a_matrix.tolist(),
        "B": b_matrix.tolist(),
        "fit": {
            row_name: {
                "coefficients": result["rows"][row_name]["theta"].tolist(),
                "r2": result["rows"][row_name]["r2"],
            }
            for row_name in ["p_dot", "beta_dot", "r_dot"]
        },
        "nan_counts_filled": prepared["nan_counts"],
        "phi_dot_minus_p_mean_deg_s": float(np.mean(prepared["phidot_from_data"] - prepared["p_smooth"])),
        "open_loop_eigenvalues": [
            {"real": float(value.real), "imag": float(value.imag)}
            for value in np.linalg.eigvals(a_matrix)
        ],
    }

    if gain_matrix is not None:
        a_cl = compute_closed_loop(a_matrix, b_matrix, gain_matrix)
        summary["closed_loop"] = {
            "K": gain_matrix.tolist(),
            "A_cl": a_cl.tolist(),
            "eigenvalues": [
                {"real": float(value.real), "imag": float(value.imag)}
                for value in np.linalg.eigvals(a_cl)
            ],
        }

    out_path = path.with_name(f"{path.stem}_lateral_mimo_summary.json")
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(f"Summary JSON: {out_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Estimate a structured 4-state, 2-input lateral-directional model from one raw logger CSV."
    )
    parser.add_argument("csv_path", type=Path, help="Path to the raw logger CSV.")
    parser.add_argument(
        "--k-row1",
        type=lambda text: parse_float_list(text, 4),
        default=None,
        help="Optional first input feedback row as four comma-separated gains for u_a = -K_row1 x.",
    )
    parser.add_argument(
        "--k-row2",
        type=lambda text: parse_float_list(text, 4),
        default=None,
        help="Optional second input feedback row as four comma-separated gains for u_r = -K_row2 x.",
    )
    parser.add_argument(
        "--K",
        dest="gain_matrix",
        nargs=2,
        metavar=("ROW1", "ROW2"),
        default=None,
        help="Optional full 2x4 feedback matrix as two quoted comma-separated rows.",
    )
    return parser.parse_args()


def resolve_gain_matrix(args):
    if args.gain_matrix is not None and (args.k_row1 is not None or args.k_row2 is not None):
        raise ValueError("Use either --K ROW1 ROW2 or the pair --k-row1/--k-row2, not both.")

    if args.gain_matrix is not None:
        row1 = parse_float_list(args.gain_matrix[0], 4)
        row2 = parse_float_list(args.gain_matrix[1], 4)
        return np.vstack([row1, row2])

    if args.k_row1 is None and args.k_row2 is None:
        return None
    if args.k_row1 is None or args.k_row2 is None:
        raise ValueError("Provide both --k-row1 and --k-row2 to define a full 2x4 gain matrix.")
    return np.vstack([args.k_row1, args.k_row2])


def main():
    args = parse_args()
    gain_matrix = resolve_gain_matrix(args)
    raw_data, raw_row_count = load_csv(args.csv_path)
    prepared = prepare_data(raw_data)
    result = estimate_structured_model(prepared)
    print_results(result, prepared, gain_matrix=gain_matrix)
    save_summary(args.csv_path, result, prepared, raw_row_count, gain_matrix=gain_matrix)


if __name__ == "__main__":
    main()
