import csv
import json
import socket
import time
from pathlib import Path

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
TARGET_PHI_DEG = 60.0
CAPTURE_PHI_TOL_DEG = 3.0
CAPTURE_P_TOL_DEG_S = 1.0
CAPTURE_DWELL_S = 0.3
MAX_TIME_TO_TARGET = 12.0

# Entry / capture gains
K_TO_TARGET = 0.018
P_DAMP = 0.025
UMAX_TO_TARGET = 0.18

# Stronger damping near capture
CAPTURE_MODE_START_DEG = 55.0
CAPTURE_K_TO_TARGET = 0.010
CAPTURE_P_DAMP = 0.040
CAPTURE_UMAX = 0.12

# Roll-command smoothing to avoid ugly spikes
MAX_ROLL_CMD_STEP = 0.03  # max change per cycle

# =========================
# STAGE 2: CONTROLLER
# =========================
CONTROL_DURATION_S = 20.0

# Roll PD (wings-leveler)
PHI_REF = 0.0
KPHI = 0.08
KP = 0.04
UMAX_ROLL = 0.50

# Pitch PD
USE_PITCH = True
THETA_REF = 0.0
KTHETA = 0.06
KQ = 0.05
UMAX_PITCH = 0.25
IAS_MIN_KTS = 90.0

# Yaw damper
KY = 0.00
UMAX_YAW = 0.30


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
        "phase": "init",
    }


def record_state(logger, state):
    row = dict(state)
    row["alt_ft"] = altitude_ft_from_state(state)
    row.setdefault("cmd_enable", logger["last_cmd"]["enable"])
    row.setdefault("cmd_roll", logger["last_cmd"]["roll"])
    row.setdefault("cmd_pitch", logger["last_cmd"]["pitch"])
    row.setdefault("cmd_yaw", logger["last_cmd"]["yaw"])
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


def capture_60_bank(sock_tx, rx, logger):
    """
    Capture approximately +60 deg bank using telemetry-based logic,
    then hold until bank and roll rate are reasonably settled.
    """
    print("[seq] Stage 1: capturing ~60 deg bank")
    logger["phase"] = "capture_60_bank"

    last = None
    last_t = 0.0
    t0 = time.time()
    prev_roll_cmd = 0.0
    settled_start = None

    # Take control and settle briefly
    send_logged(logger, sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
    time.sleep(0.5)

    while time.time() - t0 < MAX_TIME_TO_TARGET:
        now = time.time()
        last, last_t = drain_latest(rx, last, logger=logger)

        if last is None or (now - last_t) > STATE_TIMEOUT_S:
            send_logged(logger, sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(DT)
            continue

        phi, p, _, _, _, _ = get_state_fields(last)
        e = TARGET_PHI_DEG - phi

        # Entry mode vs capture mode
        if phi < CAPTURE_MODE_START_DEG:
            raw_roll_cmd = (K_TO_TARGET * e) - (P_DAMP * p)
            raw_roll_cmd = clamp(raw_roll_cmd, -UMAX_TO_TARGET, UMAX_TO_TARGET)
        else:
            # Near target: prioritize killing roll rate, not continuing to drive bank hard
            raw_roll_cmd = (CAPTURE_K_TO_TARGET * e) - (CAPTURE_P_DAMP * p)
            raw_roll_cmd = clamp(raw_roll_cmd, -CAPTURE_UMAX, CAPTURE_UMAX)

        roll_cmd = rate_limit(raw_roll_cmd, prev_roll_cmd, MAX_ROLL_CMD_STEP)
        prev_roll_cmd = roll_cmd

        send_logged(logger, sock_tx, enable=1, roll=roll_cmd, pitch=0.0, yaw=0.0)
        time.sleep(DT)

        in_phi_window = abs(phi - TARGET_PHI_DEG) <= CAPTURE_PHI_TOL_DEG
        in_p_window = abs(p) <= CAPTURE_P_TOL_DEG_S

        if in_phi_window and in_p_window:
            if settled_start is None:
                settled_start = now
            elif (now - settled_start) >= CAPTURE_DWELL_S:
                print(f"[seq] Captured ~60 deg bank: phi={phi:.2f}, p={p:.2f}")
                return
        else:
            settled_start = None

    # If we get here, we did not cleanly capture in time.
    # Continue anyway rather than hard failing, because for project use
    # a near-target state is still usable.
    if last is not None:
        phi, p, _, _, _, _ = get_state_fields(last)
        print(f"[seq] WARNING: timed out near target. Continuing anyway. phi={phi:.2f}, p={p:.2f}")
    else:
        print("[seq] WARNING: timed out without valid telemetry. Continuing anyway.")


def run_controller(sock_tx, rx, logger):
    """Run roll/pitch controller for a fixed duration."""
    print("[seq] Stage 2: running controller")
    logger["phase"] = "controller"

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

        phi, p, r, theta, q, ias = get_state_fields(last)

        # Roll PD
        e_phi = PHI_REF - phi
        roll_cmd = (KPHI * e_phi) - (KP * p)
        roll_cmd = clamp(roll_cmd, -UMAX_ROLL, UMAX_ROLL)

        # Pitch PD
        pitch_cmd = 0.0
        if USE_PITCH and ias >= IAS_MIN_KTS:
            e_th = THETA_REF - theta
            pitch_cmd = (KTHETA * e_th) - (KQ * q)
            pitch_cmd = clamp(pitch_cmd, -UMAX_PITCH, UMAX_PITCH)

        # Optional yaw damper
        yaw_cmd = clamp(-(KY * r), -UMAX_YAW, UMAX_YAW)

        send_logged(logger, sock_tx, enable=1, roll=roll_cmd, pitch=pitch_cmd, yaw=yaw_cmd)
        time.sleep(DT)


def main():
    print("[seq] Starting sequence.")
    print("[seq] Do NOT run logger simultaneously unless you explicitly support port sharing.")
    print(f"[seq] CMD -> {CMD_IP}:{CMD_PORT} | TELEM <- 0.0.0.0:{TELEM_PORT}")

    logger = make_run_logger()
    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = make_telem_socket()

    try:
        capture_60_bank(sock_tx, rx, logger)
        run_controller(sock_tx, rx, logger)
    finally:
        logger["phase"] = "release"
        send_logged(logger, sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
        time.sleep(0.2)
        send_logged(logger, sock_tx, enable=0, roll=0.0, pitch=0.0, yaw=0.0)
        logger["phase"] = "complete"
        flush_log(logger)
        rx.close()
        sock_tx.close()
        print("[seq] Done. Released control.")


if __name__ == "__main__":
    main()
