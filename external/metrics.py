from analysis_utils import (
    compute_metrics,
    detect_events,
    load_run,
    write_summary_outputs,
)


EVENT_LABELS = {
    "time_to_60deg_bank": "first reaches ~60 deg bank",
    "yaw_pulse_start": "yaw pulse start",
    "yaw_pulse_end": "yaw pulse end",
    "controller_start": "controller start",
}


METRIC_LABELS = [
    ("time_to_60deg_bank", "time_to_60deg_bank (s)"),
    ("peak_bank_deg", "peak_bank_deg"),
    ("bank_overshoot_deg", "bank_overshoot_deg"),
    ("roll_rate_at_60deg", "roll_rate_at_60deg (deg/s)"),
    ("roll_rate_at_controller_start", "roll_rate_at_controller_start (deg/s)"),
    ("time_from_controller_start_to_|phi|<10deg", "time_from_controller_start_to_|phi|<10deg (s)"),
    ("time_from_controller_start_to_|phi|<5deg", "time_from_controller_start_to_|phi|<5deg (s)"),
    ("time_from_controller_start_to_|p|<1deg_s", "time_from_controller_start_to_|p|<1deg_s (s)"),
    ("settling_time_phi_to_5deg_hold_2s", "settling_time_phi_to_5deg_hold_2s (s)"),
    ("phi_rms_10s_after_controller_start", "phi_rms_10s_after_controller_start (deg)"),
    ("p_rms_10s_after_controller_start", "p_rms_10s_after_controller_start (deg/s)"),
    ("max_abs_roll_rate_after_controller_start", "max_abs_roll_rate_after_controller_start (deg/s)"),
    ("min_altitude_after_controller_start_ft", "min_altitude_after_controller_start_ft"),
    ("max_altitude_deviation_from_controller_start_ft", "max_altitude_deviation_from_controller_start_ft"),
    ("net_altitude_change_from_controller_start_to_end_ft", "net_altitude_change_from_controller_start_to_end_ft"),
]


def fmt(value):
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def main(path):
    run = load_run(path)
    events = detect_events(run)
    metrics = compute_metrics(run, events)
    json_path, csv_path = write_summary_outputs(metrics)

    print(f"file: {path}")
    print("events:")
    for key, label in EVENT_LABELS.items():
        value = events.get(key)
        print(f"  {label}: {fmt(value) if value is not None else 'NOT_FOUND'}")

    print("metrics:")
    for key, label in METRIC_LABELS:
        print(f"  {label}: {fmt(metrics[key])}")

    print(f"summary_json: {json_path}")
    print(f"summary_csv: {csv_path}")


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("usage: python external/metrics.py data/runs/run_XXXX.csv")
        raise SystemExit(2)
    main(sys.argv[1])
