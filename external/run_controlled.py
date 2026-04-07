import csv
import json
import socket
import sys
import time
import argparse
from pathlib import Path

import numpy as np

THIS_DIR = Path(__file__).resolve().parent
if str(THIS_DIR) not in sys.path:
    sys.path.insert(0, str(THIS_DIR))

from lqr_utils import continuous_lqr

# =========================
# NETWORK
# =========================
TELEM_PORT = 49005
CMD_IP = "172.26.96.1"
CMD_PORT = 49006
RUNS_DIR = Path("data/runs")

# =========================
# LOOP
# =========================
RATE_HZ = 25
DT = 1.0 / RATE_HZ
STATE_TIMEOUT_S = 0.4

# =========================
# STAGE 1: CAPTURE ~60 DEG BANK
# =========================
TARGET_BANK_DEG = 60.0
BANK_CAPTURE_MIN_DEG = 58.0
BANK_CAPTURE_MAX_DEG = 62.0
BANK_CAPTURE_MAX_P_DEG_S = 2.0
BANK_CAPTURE_DWELL_S = 0.2
BANK_CAPTURE_TIMEOUT_S = 15.0

# Entry / capture gains
K_TO_TARGET = 0.018
P_DAMP = 0.025
UMAX_TO_TARGET = 0.18

# Stronger damping near capture
CAPTURE_MODE_START_DEG = 55.0
CAPTURE_K_TO_TARGET = 0.010
CAPTURE_P_DAMP = 0.040
CAPTURE_UMAX = 0.12

# Controller takeover configuration. The default mode now uses a first-valid
# capture event near 60 deg bank so we hand off at the first clean opportunity
# instead of waiting for a late settled hold.
TAKEOVER_MODE_DEFAULT = "settled_60"
EARLY_TAKEOVER_PHI_DEG = 50.0
EARLY_TAKEOVER_MIN_TIME_S = 1.0
MANUAL_TAKEOVER_TIME_S = 5.0

# Roll-command smoothing during capture to avoid ugly spikes
MAX_ROLL_CMD_STEP_CAPTURE = 0.03  # max change per cycle

# =========================
# STAGE 2: CONTROLLER
# =========================
CONTROL_DURATION_S = 20.0

# Continuous-time 4-state, 2-input lateral-directional model identified from
# the MIMO open-loop test.
A_LATERAL = np.array([
    [0.0, 1.0, 0.0, 0.0],
    [-0.025546, -0.742954, 3.947808, 1.173654],
    [-0.005251, -0.215633, 0.605060, 0.274297],
    [0.049386, -0.166693, -3.610712, -0.769096],
], dtype=float)

B_LATERAL = np.array([
    [0.0, 0.0],
    [50.081445, 5.571537],
    [5.048179, 6.845589],
    [3.411679, 30.816660],
], dtype=float)

# Design on normalized states so the LQR weights stay interpretable even though
# the runtime state vector mixes angle and rate units.
PHI_SCALE_DEG = 60.0
P_SCALE_DEG_S = 10.0
BETA_SCALE_DEG = 5.0
R_SCALE_DEG_S = 10.0

STATE_SCALE_VECTOR = np.array([
    PHI_SCALE_DEG,
    P_SCALE_DEG_S,
    BETA_SCALE_DEG,
    R_SCALE_DEG_S,
], dtype=float)
STATE_SCALE_MATRIX = np.diag(STATE_SCALE_VECTOR)
STATE_SCALE_INV = np.diag(1.0 / STATE_SCALE_VECTOR)

# Small, structured MIMO LQR retuning set. These candidates primarily push on
# bank-angle regulation while leaving the yaw/rudder channel priorities intact.
LQR_TUNING_CANDIDATES = {
    "current": {
        "q_diag": [20.0, 8.0, 14.0, 18.0],
        "r_diag": [18.0, 12.0],
        "description": "Current smooth baseline.",
    },
    "phi_plus_50": {
        "q_diag": [30.0, 8.0, 14.0, 18.0],
        "r_diag": [18.0, 12.0],
        "description": "Increase phi weight only for faster bank-angle recovery with minimal structural change.",
    },
    "phi_x2": {
        "q_diag": [40.0, 8.0, 14.0, 18.0],
        "r_diag": [18.0, 12.0],
        "description": "Increase phi weight more aggressively while keeping both input penalties unchanged.",
    },
    "phi_plus_50_roll_r_down": {
        "q_diag": [30.0, 8.0, 14.0, 18.0],
        "r_diag": [15.0, 12.0],
        "description": "Increase phi weight and slightly reduce aileron effort penalty to make roll authority less conservative.",
    },
    "phi_x3_roll_r_down": {
        "q_diag": [60.0, 8.0, 14.0, 18.0],
        "r_diag": [12.0, 12.0],
        "description": "Current manually strengthened reference: higher phi urgency and cheaper aileron usage than the original sweep.",
    },
    "phi_x4_roll_r_down": {
        "q_diag": [80.0, 8.0, 14.0, 18.0],
        "r_diag": [12.0, 12.0],
        "description": "Less conservative than phi_x3_roll_r_down by increasing phi urgency again while keeping command penalties unchanged.",
    },
    "phi_x5_roll_r_down": {
        "q_diag": [100.0, 8.0, 14.0, 18.0],
        "r_diag": [12.0, 12.0],
        "description": "Push phi urgency further without changing yaw or aileron penalties from the current candidate.",
    },
    "phi_x4_roll_r_more_down": {
        "q_diag": [80.0, 8.0, 14.0, 18.0],
        "r_diag": [10.0, 12.0],
        "description": "Increase phi urgency and make the aileron channel slightly less penalized than phi_x3_roll_r_down.",
    },
    "phi_x5_roll_r_more_down": {
        "q_diag": [100.0, 8.0, 14.0, 18.0],
        "r_diag": [10.0, 12.0],
        "description": "Strongest practical next step: highest phi urgency in this sweep plus a slightly cheaper aileron channel.",
    },
}
DEFAULT_LQR_CANDIDATE = "phi_x4_roll_r_more_down"

ROLL_MAX_CMD_STEADY = 0.50
ROLL_MAX_CMD_INITIAL = 0.65
UMAX_YAW = 0.30
ROLL_RATE_LIMIT_STEADY = 0.02  # max allowed steady roll-command change per controller cycle
MAX_YAW_CMD_STEP = 0.02  # max allowed yaw-command change per controller cycle
ROLL_RATE_LIMIT_INITIAL = 0.08  # looser roll-command limit right after takeover
INITIAL_MAX_YAW_CMD_STEP = 0.04  # optional looser limit right after takeover
INITIAL_LIMIT_WINDOW_S = 2.0
POST_TAKEOVER_DIAG_WINDOW_S = 2.0

# Pitch PD
USE_PITCH = True
THETA_REF = 0.0
KTHETA = 0.06
KQ = 0.05
UMAX_PITCH = 0.25
IAS_MIN_KTS = 90.0


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def rate_limit(new_cmd, prev_cmd, max_step):
    delta = new_cmd - prev_cmd
    delta = clamp(delta, -max_step, max_step)
    return prev_cmd + delta


def send(sock, enable, roll=0.0, pitch=0.0, yaw=0.0):
    msg = {
        "enable": int(enable),
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
    }
    sock.sendto(json.dumps(msg).encode("utf-8"), (CMD_IP, CMD_PORT))
    return msg


def make_telem_socket():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", TELEM_PORT))
    rx.setblocking(False)
    return rx


def altitude_ft_from_state(state):
    if "alt_ft" in state:
        return float(state["alt_ft"])
    if "altitude_ft" in state:
        return float(state["altitude_ft"])
    if "alt_m" in state:
        return float(state["alt_m"]) * 3.28084
    return 0.0


def make_run_logger():
    RUNS_DIR.mkdir(parents=True, exist_ok=True)
    return {
        "out_path": RUNS_DIR / f"controlled_{int(time.time())}.csv",
        "rows": [],
        "last_cmd": {"enable": 0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
        "last_roll_cmd_raw_lqr_mimo": 0.0,
        "last_yaw_cmd_raw_lqr_mimo": 0.0,
        "last_roll_cmd_clamped": 0.0,
        "last_yaw_cmd_clamped": 0.0,
        "max_abs_roll_cmd_raw_lqr_mimo": 0.0,
        "max_abs_yaw_cmd_raw_lqr_mimo": 0.0,
        "max_abs_roll_cmd_clamped": 0.0,
        "max_abs_yaw_cmd_clamped": 0.0,
        "roll_cmd_saturation_count": 0,
        "yaw_cmd_saturation_count": 0,
        "roll_cmd_sample_count": 0,
        "yaw_cmd_sample_count": 0,
        "max_roll_cmd_clamp_gap": 0.0,
        "max_yaw_cmd_clamp_gap": 0.0,
        "roll_cmd_rate_limited_count": 0,
        "yaw_cmd_rate_limited_count": 0,
        "max_roll_cmd_rate_limit_gap": 0.0,
        "max_yaw_cmd_rate_limit_gap": 0.0,
        "controller_start_time": None,
        "controller_handoff_status": "not_started",
        "controller_handoff_phi_deg": None,
        "controller_handoff_p_deg_s": None,
        "controller_handoff_elapsed_s": None,
        "post_takeover_window_sample_count": 0,
        "post_takeover_roll_rate_limited_count": 0,
        "post_takeover_yaw_rate_limited_count": 0,
        "post_takeover_roll_clamped_count": 0,
        "post_takeover_yaw_clamped_count": 0,
        "post_takeover_max_abs_roll_raw_cmd": 0.0,
        "post_takeover_max_abs_yaw_raw_cmd": 0.0,
        "post_takeover_max_abs_roll_clamped_cmd": 0.0,
        "post_takeover_max_abs_yaw_clamped_cmd": 0.0,
        "post_takeover_max_abs_roll_sent_cmd": 0.0,
        "post_takeover_max_abs_yaw_sent_cmd": 0.0,
        "phase": "init",
    }


def record_state(logger, state):
    row = dict(state)
    row["alt_ft"] = altitude_ft_from_state(state)
    row.setdefault("cmd_enable", logger["last_cmd"]["enable"])
    row.setdefault("cmd_roll", logger["last_cmd"]["roll"])
    row.setdefault("cmd_pitch", logger["last_cmd"]["pitch"])
    row.setdefault("cmd_yaw", logger["last_cmd"]["yaw"])
    row["roll_cmd_raw_lqr_mimo"] = logger["last_roll_cmd_raw_lqr_mimo"]
    row["yaw_cmd_raw_lqr_mimo"] = logger["last_yaw_cmd_raw_lqr_mimo"]
    row["roll_cmd_clamped"] = logger["last_roll_cmd_clamped"]
    row["yaw_cmd_clamped"] = logger["last_yaw_cmd_clamped"]
    row["controller_handoff_status"] = logger["controller_handoff_status"]
    row["controller_handoff_phi_deg"] = logger["controller_handoff_phi_deg"]
    row["controller_handoff_p_deg_s"] = logger["controller_handoff_p_deg_s"]
    row["controller_handoff_elapsed_s"] = logger["controller_handoff_elapsed_s"]
    row["controlled_phase"] = logger["phase"]
    logger["rows"].append(row)


def send_logged(logger, sock, enable, roll=0.0, pitch=0.0, yaw=0.0):
    logger["last_cmd"] = send(sock, enable=enable, roll=roll, pitch=pitch, yaw=yaw)


def flush_log(logger):
    if not logger["rows"]:
        return

    keys = sorted({k for row in logger["rows"] for k in row.keys()})
    with logger["out_path"].open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(logger["rows"])
    print(f"[seq] Wrote {len(logger['rows'])} telemetry rows -> {logger['out_path']}")


def drain_latest(rx, last_state, logger=None):
    """Drain UDP socket to the most recent telemetry packet."""
    st_t = time.time()
    while True:
        try:
            data, _ = rx.recvfrom(65535)
        except BlockingIOError:
            break
        try:
            last_state = json.loads(data.decode("utf-8"))
            st_t = time.time()
            if logger is not None:
                record_state(logger, last_state)
        except Exception:
            pass
    return last_state, st_t


def get_state_fields(state):
    phi = float(state.get("phi_deg", 0.0))
    p = float(state.get("p_deg_s", 0.0))
    r = float(state.get("r_deg_s", 0.0))
    theta = float(state.get("theta_deg", 0.0))
    q = float(state.get("q_deg_s", 0.0))
    ias = float(state.get("ias_kts", 0.0))
    return phi, p, r, theta, q, ias


def require_state_value(state, key):
    if key not in state:
        raise RuntimeError(
            f"Controlled run requires telemetry field '{key}', but it was missing. "
            "Make sure the updated UDP bridge/plugin is running and streaming the 4-state telemetry."
        )

    try:
        value = float(state[key])
    except (TypeError, ValueError):
        raise RuntimeError(
            f"Controlled run requires telemetry field '{key}', but received an invalid value: {state[key]!r}"
        ) from None

    return value


def format_matrix(name, matrix):
    return f"{name}=\n{np.array2string(matrix, precision=6, suppress_small=False)}"


def get_lqr_tuning(candidate_name):
    if candidate_name not in LQR_TUNING_CANDIDATES:
        available = ", ".join(sorted(LQR_TUNING_CANDIDATES))
        raise ValueError(f"Unknown LQR tuning candidate '{candidate_name}'. Available: {available}")

    candidate = dict(LQR_TUNING_CANDIDATES[candidate_name])
    candidate["name"] = candidate_name
    candidate["Q_normalized"] = np.diag(candidate["q_diag"])
    candidate["R"] = np.diag(candidate["r_diag"])
    return candidate


def compute_lqr_mimo_gain(tuning):
    a_normalized = STATE_SCALE_INV @ A_LATERAL @ STATE_SCALE_MATRIX
    b_normalized = STATE_SCALE_INV @ B_LATERAL
    k_normalized, _p_matrix, closed_loop_eigs_normalized = continuous_lqr(
        a_normalized,
        b_normalized,
        tuning["Q_normalized"],
        tuning["R"],
    )
    k_physical = k_normalized @ STATE_SCALE_INV
    closed_loop_eigs_physical = np.linalg.eigvals(A_LATERAL - B_LATERAL @ k_physical)
    return {
        "A_normalized": a_normalized,
        "B_normalized": b_normalized,
        "Q_normalized": tuning["Q_normalized"],
        "R": tuning["R"],
        "K_normalized": k_normalized,
        "K_physical": k_physical,
        "closed_loop_eigs_normalized": closed_loop_eigs_normalized,
        "closed_loop_eigs_physical": closed_loop_eigs_physical,
        "tuning": tuning,
    }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run the controlled X-Plane recovery sequence with a normalized 2-input LQR controller."
    )
    parser.add_argument(
        "--candidate",
        default=DEFAULT_LQR_CANDIDATE,
        choices=sorted(LQR_TUNING_CANDIDATES),
        help="Named 2-input LQR tuning candidate to use.",
    )
    parser.add_argument(
        "--list-candidates",
        action="store_true",
        help="Print the available 2-input LQR tuning candidates and exit.",
    )
    parser.add_argument(
        "--takeover-mode",
        default=TAKEOVER_MODE_DEFAULT,
        choices=["settled_60", "earlier_phi", "manual_time"],
        help="Controller takeover trigger mode.",
    )
    return parser.parse_args()


def update_axis_command_diagnostics(logger, axis_name, raw_cmd, clamped_cmd, sent_cmd, command_limit):
    logger[f"last_{axis_name}_cmd_raw_lqr_mimo"] = raw_cmd
    logger[f"last_{axis_name}_cmd_clamped"] = clamped_cmd
    logger[f"{axis_name}_cmd_sample_count"] += 1
    logger[f"max_abs_{axis_name}_cmd_raw_lqr_mimo"] = max(
        logger[f"max_abs_{axis_name}_cmd_raw_lqr_mimo"], abs(raw_cmd)
    )
    logger[f"max_abs_{axis_name}_cmd_clamped"] = max(
        logger[f"max_abs_{axis_name}_cmd_clamped"], abs(clamped_cmd)
    )
    clamp_gap = abs(raw_cmd - clamped_cmd)
    logger[f"max_{axis_name}_cmd_clamp_gap"] = max(logger[f"max_{axis_name}_cmd_clamp_gap"], clamp_gap)
    if abs(raw_cmd) > command_limit:
        logger[f"{axis_name}_cmd_saturation_count"] += 1
    rate_limit_gap = abs(clamped_cmd - sent_cmd)
    logger[f"max_{axis_name}_cmd_rate_limit_gap"] = max(
        logger[f"max_{axis_name}_cmd_rate_limit_gap"], rate_limit_gap
    )
    if rate_limit_gap > 1e-9:
        logger[f"{axis_name}_cmd_rate_limited_count"] += 1


def print_axis_command_diagnostics(logger, axis_name):
    n = logger[f"{axis_name}_cmd_sample_count"]
    if n <= 0:
        return
    sat_fraction = logger[f"{axis_name}_cmd_saturation_count"] / float(n)
    rate_limit_fraction = logger[f"{axis_name}_cmd_rate_limited_count"] / float(n)
    print(
        f"[seq] {axis_name.capitalize()}-command diagnostics: "
        f"max|raw|={logger[f'max_abs_{axis_name}_cmd_raw_lqr_mimo']:.3f}, "
        f"max|clamped|={logger[f'max_abs_{axis_name}_cmd_clamped']:.3f}, "
        f"saturation_fraction={sat_fraction:.3f}, "
        f"rate_limited_fraction={rate_limit_fraction:.3f}, "
        f"max|raw-clamped|={logger[f'max_{axis_name}_cmd_clamp_gap']:.3f}, "
        f"max|clamped-sent|={logger[f'max_{axis_name}_cmd_rate_limit_gap']:.3f}"
    )


def print_post_takeover_diagnostics(logger):
    n0 = logger["post_takeover_window_sample_count"]
    if n0 <= 0:
        return
    roll_rate_fraction = logger["post_takeover_roll_rate_limited_count"] / float(n0)
    yaw_rate_fraction = logger["post_takeover_yaw_rate_limited_count"] / float(n0)
    roll_clamp_fraction = logger["post_takeover_roll_clamped_count"] / float(n0)
    yaw_clamp_fraction = logger["post_takeover_yaw_clamped_count"] / float(n0)
    print(
        f"[seq] First {POST_TAKEOVER_DIAG_WINDOW_S:.1f}s after takeover: "
        f"roll max|raw|={logger['post_takeover_max_abs_roll_raw_cmd']:.3f}, "
        f"roll max|clamped|={logger['post_takeover_max_abs_roll_clamped_cmd']:.3f}, "
        f"roll max|sent|={logger['post_takeover_max_abs_roll_sent_cmd']:.3f}, "
        f"roll rate_limited_fraction={roll_rate_fraction:.3f}, "
        f"roll clamped_fraction={roll_clamp_fraction:.3f}, "
        f"yaw max|raw|={logger['post_takeover_max_abs_yaw_raw_cmd']:.3f}, "
        f"yaw max|clamped|={logger['post_takeover_max_abs_yaw_clamped_cmd']:.3f}, "
        f"yaw max|sent|={logger['post_takeover_max_abs_yaw_sent_cmd']:.3f}, "
        f"yaw rate_limited_fraction={yaw_rate_fraction:.3f}, "
        f"yaw clamped_fraction={yaw_clamp_fraction:.3f}"
    )


def print_controller_diagnostics(logger):
    print_axis_command_diagnostics(logger, "roll")
    print_axis_command_diagnostics(logger, "yaw")
    print_post_takeover_diagnostics(logger)


def capture_condition_met(phi, p):
    in_bank_window = BANK_CAPTURE_MIN_DEG <= phi <= BANK_CAPTURE_MAX_DEG
    in_roll_rate_window = abs(p) <= BANK_CAPTURE_MAX_P_DEG_S
    return in_bank_window and in_roll_rate_window


def register_handoff(logger, status, elapsed_s, phi, p, message_prefix):
    logger["controller_handoff_status"] = status
    logger["controller_handoff_phi_deg"] = float(phi)
    logger["controller_handoff_p_deg_s"] = float(p)
    logger["controller_handoff_elapsed_s"] = float(elapsed_s)
    logger["controller_start_time"] = time.time()
    print(
        f"[seq] {message_prefix}: status={status} "
        f"t={elapsed_s:.2f}s phi={phi:.2f} deg p={p:.2f} deg/s"
    )


def should_takeover(takeover_mode, elapsed_s, phi, p, capture_candidate_start, now):
    if takeover_mode == "manual_time":
        return elapsed_s >= MANUAL_TAKEOVER_TIME_S

    if takeover_mode == "earlier_phi":
        return elapsed_s >= EARLY_TAKEOVER_MIN_TIME_S and phi >= EARLY_TAKEOVER_PHI_DEG

    if takeover_mode != "settled_60":
        raise ValueError(f"Unsupported takeover mode: {takeover_mode}")

    if capture_candidate_start is None:
        return False
    return (now - capture_candidate_start) >= BANK_CAPTURE_DWELL_S


def capture_60_bank(sock_tx, rx, logger, takeover_mode):
    """
    Capture approximately +60 deg bank using telemetry-based logic,
    then hand off on the first valid clean-capture event.
    """
    print("[seq] Stage 1: capturing ~60 deg bank")
    logger["phase"] = "capture_60_bank"

    last = None
    last_t = 0.0
    t0 = time.time()
    prev_roll_cmd = 0.0
    capture_candidate_start = None

    send_logged(logger, sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
    time.sleep(0.5)

    while time.time() - t0 < BANK_CAPTURE_TIMEOUT_S:
        now = time.time()
        elapsed_s = now - t0
        last, last_t = drain_latest(rx, last, logger=logger)

        if last is None or (now - last_t) > STATE_TIMEOUT_S:
            send_logged(logger, sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(DT)
            continue

        phi, p, _, _, _, _ = get_state_fields(last)
        e = TARGET_BANK_DEG - phi

        if phi < CAPTURE_MODE_START_DEG:
            raw_roll_cmd = (K_TO_TARGET * e) - (P_DAMP * p)
            raw_roll_cmd = clamp(raw_roll_cmd, -UMAX_TO_TARGET, UMAX_TO_TARGET)
        else:
            raw_roll_cmd = (CAPTURE_K_TO_TARGET * e) - (CAPTURE_P_DAMP * p)
            raw_roll_cmd = clamp(raw_roll_cmd, -CAPTURE_UMAX, CAPTURE_UMAX)

        roll_cmd = rate_limit(raw_roll_cmd, prev_roll_cmd, MAX_ROLL_CMD_STEP_CAPTURE)
        prev_roll_cmd = roll_cmd

        send_logged(logger, sock_tx, enable=1, roll=roll_cmd, pitch=0.0, yaw=0.0)
        time.sleep(DT)

        if capture_condition_met(phi, p):
            if capture_candidate_start is None:
                capture_candidate_start = now
        else:
            capture_candidate_start = None

        if should_takeover(takeover_mode, elapsed_s, phi, p, capture_candidate_start, now):
            register_handoff(
                logger,
                status="clean_capture",
                elapsed_s=elapsed_s,
                phi=phi,
                p=p,
                message_prefix=f"Controller takeover triggered ({takeover_mode})",
            )
            return

    if last is not None:
        phi, p, _, _, _, _ = get_state_fields(last)
        elapsed_s = time.time() - t0
        register_handoff(
            logger,
            status="capture_failed",
            elapsed_s=elapsed_s,
            phi=phi,
            p=p,
            message_prefix="Stage 1 capture failed",
        )
        raise RuntimeError(
            "Stage 1 failed to achieve clean capture before timeout. "
            f"Required phi in [{BANK_CAPTURE_MIN_DEG:.1f}, {BANK_CAPTURE_MAX_DEG:.1f}] deg, "
            f"|p| <= {BANK_CAPTURE_MAX_P_DEG_S:.1f} deg/s, held for {BANK_CAPTURE_DWELL_S:.1f} s "
            f"within {BANK_CAPTURE_TIMEOUT_S:.1f} s. "
            f"Final state at timeout: phi={phi:.2f} deg, p={p:.2f} deg/s."
        )

    register_handoff(
        logger,
        status="capture_failed",
        elapsed_s=BANK_CAPTURE_TIMEOUT_S,
        phi=float("nan"),
        p=float("nan"),
        message_prefix="Stage 1 capture failed",
    )
    raise RuntimeError("Stage 1 timed out without valid telemetry; refusing to start Stage 2.")


def run_controller(sock_tx, rx, logger, k_lqr_physical):
    """Run 2-input lateral-directional state feedback plus the existing pitch PD."""
    print("[seq] Stage 2: running controller")
    logger["phase"] = "controller"
    print(
        f"[seq] Stage 2 actual takeover instant: "
        f"t={logger['controller_handoff_elapsed_s']:.2f}s "
        f"phi={logger['controller_handoff_phi_deg']:.2f} deg "
        f"p={logger['controller_handoff_p_deg_s']:.2f} deg/s"
    )
    prev_roll_cmd = 0.0
    prev_yaw_cmd = 0.0

    last = None
    last_t = 0.0
    end = time.time() + CONTROL_DURATION_S

    while time.time() < end:
        now = time.time()
        last, last_t = drain_latest(rx, last, logger=logger)

        if last is None or (now - last_t) > STATE_TIMEOUT_S:
            send_logged(logger, sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(DT)
            continue

        phi = require_state_value(last, "phi_deg")
        p = require_state_value(last, "p_deg_s")
        beta = require_state_value(last, "beta_deg")
        r = require_state_value(last, "r_deg_s")
        theta = require_state_value(last, "theta_deg")
        q = require_state_value(last, "q_deg_s")
        ias = require_state_value(last, "ias_kts")

        x_state = np.array([phi, p, beta, r], dtype=float)
        u_raw = -(k_lqr_physical @ x_state)

        roll_cmd_raw = float(u_raw[0])
        yaw_cmd_raw = float(u_raw[1])

        controller_elapsed_s = 0.0
        if logger["controller_start_time"] is not None:
            controller_elapsed_s = max(0.0, time.time() - logger["controller_start_time"])

        active_roll_cmd_limit = ROLL_MAX_CMD_STEADY
        active_roll_cmd_step = ROLL_RATE_LIMIT_STEADY
        active_yaw_cmd_step = MAX_YAW_CMD_STEP
        if controller_elapsed_s <= INITIAL_LIMIT_WINDOW_S:
            active_roll_cmd_limit = ROLL_MAX_CMD_INITIAL
            active_roll_cmd_step = ROLL_RATE_LIMIT_INITIAL
            active_yaw_cmd_step = INITIAL_MAX_YAW_CMD_STEP

        roll_cmd_clamped = clamp(roll_cmd_raw, -active_roll_cmd_limit, active_roll_cmd_limit)
        yaw_cmd_clamped = clamp(yaw_cmd_raw, -UMAX_YAW, UMAX_YAW)

        roll_cmd = rate_limit(roll_cmd_clamped, prev_roll_cmd, active_roll_cmd_step)
        yaw_cmd = rate_limit(yaw_cmd_clamped, prev_yaw_cmd, active_yaw_cmd_step)
        prev_roll_cmd = roll_cmd
        prev_yaw_cmd = yaw_cmd

        update_axis_command_diagnostics(
            logger, "roll", roll_cmd_raw, roll_cmd_clamped, roll_cmd, active_roll_cmd_limit
        )
        update_axis_command_diagnostics(logger, "yaw", yaw_cmd_raw, yaw_cmd_clamped, yaw_cmd, UMAX_YAW)

        if controller_elapsed_s <= POST_TAKEOVER_DIAG_WINDOW_S:
            logger["post_takeover_window_sample_count"] += 1
            logger["post_takeover_max_abs_roll_raw_cmd"] = max(
                logger["post_takeover_max_abs_roll_raw_cmd"], abs(roll_cmd_raw)
            )
            logger["post_takeover_max_abs_yaw_raw_cmd"] = max(
                logger["post_takeover_max_abs_yaw_raw_cmd"], abs(yaw_cmd_raw)
            )
            logger["post_takeover_max_abs_roll_clamped_cmd"] = max(
                logger["post_takeover_max_abs_roll_clamped_cmd"], abs(roll_cmd_clamped)
            )
            logger["post_takeover_max_abs_yaw_clamped_cmd"] = max(
                logger["post_takeover_max_abs_yaw_clamped_cmd"], abs(yaw_cmd_clamped)
            )
            logger["post_takeover_max_abs_roll_sent_cmd"] = max(
                logger["post_takeover_max_abs_roll_sent_cmd"], abs(roll_cmd)
            )
            logger["post_takeover_max_abs_yaw_sent_cmd"] = max(
                logger["post_takeover_max_abs_yaw_sent_cmd"], abs(yaw_cmd)
            )
            if abs(roll_cmd_clamped - roll_cmd) > 1e-9:
                logger["post_takeover_roll_rate_limited_count"] += 1
            if abs(yaw_cmd_clamped - yaw_cmd) > 1e-9:
                logger["post_takeover_yaw_rate_limited_count"] += 1
            if abs(roll_cmd_raw - roll_cmd_clamped) > 1e-9:
                logger["post_takeover_roll_clamped_count"] += 1
            if abs(yaw_cmd_raw - yaw_cmd_clamped) > 1e-9:
                logger["post_takeover_yaw_clamped_count"] += 1

        pitch_cmd = 0.0
        if USE_PITCH and ias >= IAS_MIN_KTS:
            e_th = THETA_REF - theta
            pitch_cmd = (KTHETA * e_th) - (KQ * q)
            pitch_cmd = clamp(pitch_cmd, -UMAX_PITCH, UMAX_PITCH)

        send_logged(logger, sock_tx, enable=1, roll=roll_cmd, pitch=pitch_cmd, yaw=yaw_cmd)
        time.sleep(DT)


def main():
    args = parse_args()

    if args.list_candidates:
        print("Available 2-input LQR tuning candidates:")
        for name in sorted(LQR_TUNING_CANDIDATES):
            tuning = get_lqr_tuning(name)
            design = compute_lqr_mimo_gain(tuning)
            print(f"  {name}: {tuning['description']}")
            print(f"    q_diag={tuning['q_diag']}, r_diag={tuning['r_diag']}")
            print(f"    K=\n{np.array2string(design['K_physical'], precision=6, suppress_small=False)}")
            print(
                "    eigs="
                f"{np.array2string(design['closed_loop_eigs_physical'], precision=6, suppress_small=False)}"
            )
        return

    print("[seq] Starting sequence.")
    print("[seq] Do NOT run logger simultaneously unless you explicitly support port sharing.")
    print(f"[seq] CMD -> {CMD_IP}:{CMD_PORT} | TELEM <- 0.0.0.0:{TELEM_PORT}")

    tuning = get_lqr_tuning(args.candidate)
    lqr_design = compute_lqr_mimo_gain(tuning)
    print("[seq] Using 2-input LQR state feedback: [roll_cmd_raw, yaw_cmd_raw]^T = -K x")
    print(f"[seq] Selected candidate: {tuning['name']} | {tuning['description']}")
    print(
        "[seq] Runtime state vector x = "
        f"[phi_deg, p_deg_s, beta_deg, r_deg_s]^T in physical units."
    )
    print(
        "[seq] Raw/clamped logging enabled: "
        "CSV will include roll_cmd_raw_lqr_mimo, yaw_cmd_raw_lqr_mimo, "
        "roll_cmd_clamped, yaw_cmd_clamped, plus the final sent cmd_roll/cmd_yaw."
    )
    print(
        "[seq] Takeover config: "
        f"mode={args.takeover_mode}, "
        f"early_phi={EARLY_TAKEOVER_PHI_DEG:.1f} deg, "
        f"early_min_time={EARLY_TAKEOVER_MIN_TIME_S:.1f} s, "
        f"manual_time={MANUAL_TAKEOVER_TIME_S:.1f} s"
    )
    print(
        "[seq] Clean Stage 1 handoff condition: "
        f"phi in [{BANK_CAPTURE_MIN_DEG:.1f}, {BANK_CAPTURE_MAX_DEG:.1f}] deg, "
        f"|p| <= {BANK_CAPTURE_MAX_P_DEG_S:.1f} deg/s, "
        f"held for {BANK_CAPTURE_DWELL_S:.1f} s"
    )
    print(
        "[seq] Clean capture timeout: "
        f"abort the run if clean capture is not achieved within {BANK_CAPTURE_TIMEOUT_S:.1f} s."
    )
    print(
        "[seq] Command protections enabled: "
        f"roll clamp steady={ROLL_MAX_CMD_STEADY:.3f}, "
        f"roll clamp initial={ROLL_MAX_CMD_INITIAL:.3f}, "
        f"roll rate steady={ROLL_RATE_LIMIT_STEADY:.3f}/cycle, "
        f"roll rate initial={ROLL_RATE_LIMIT_INITIAL:.3f}/cycle, "
        f"yaw steady={MAX_YAW_CMD_STEP:.3f}/cycle, "
        f"yaw initial={INITIAL_MAX_YAW_CMD_STEP:.3f}/cycle, "
        f"initial window={INITIAL_LIMIT_WINDOW_S:.1f}s"
    )
    print(f"[seq] Post-takeover diagnostic window: {POST_TAKEOVER_DIAG_WINDOW_S:.1f} s")
    print(f"[seq] State scales = {np.array2string(STATE_SCALE_VECTOR, precision=3, suppress_small=False)}")
    print(format_matrix("A_lateral", A_LATERAL))
    print(format_matrix("B_lateral", B_LATERAL))
    print(format_matrix("Q_lqr_n", lqr_design["Q_normalized"]))
    print(format_matrix("R_lqr", lqr_design["R"]))
    print(format_matrix("K_lqr_n", lqr_design["K_normalized"]))
    print(format_matrix("K_lqr_physical", lqr_design["K_physical"]))
    print(
        "[seq] Closed-loop eigenvalues (normalized design model) = "
        f"{np.array2string(lqr_design['closed_loop_eigs_normalized'], precision=6, suppress_small=False)}"
    )
    print(
        "[seq] Closed-loop eigenvalues (physical model) = "
        f"{np.array2string(lqr_design['closed_loop_eigs_physical'], precision=6, suppress_small=False)}"
    )

    logger = make_run_logger()
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = make_telem_socket()

    try:
        capture_60_bank(sock_tx, rx, logger, args.takeover_mode)
        run_controller(sock_tx, rx, logger, lqr_design["K_physical"])
    finally:
        logger["phase"] = "release"
        send_logged(logger, sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
        time.sleep(0.2)
        send_logged(logger, sock_tx, enable=0, roll=0.0, pitch=0.0, yaw=0.0)
        logger["phase"] = "complete"
        flush_log(logger)
        print_controller_diagnostics(logger)
        rx.close()
        sock_tx.close()
        print("[seq] Done. Released control.")


if __name__ == "__main__":
    main()
