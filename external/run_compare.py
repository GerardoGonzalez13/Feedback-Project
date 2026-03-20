import json
import socket
import time

# ---- NETWORK ----
TELEM_PORT = 49005
CMD_IP     = "172.26.96.1"   # set to your working WSL->Windows gateway
CMD_PORT   = 49006
# -----------------

# ---- LOOP ----
RATE_HZ = 25
DT = 1.0 / RATE_HZ
STATE_TIMEOUT_S = 0.4
# ----------

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

def send(sock, enable, roll=0.0, pitch=0.0, yaw=0.0):
    msg = {"enable": int(enable), "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw)}
    sock.sendto(json.dumps(msg).encode("utf-8"), (CMD_IP, CMD_PORT))

def make_telem_socket():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", TELEM_PORT))
    rx.setblocking(False)
    return rx

def drain_latest(rx, last_state):
    """Drain UDP to most recent packet. Returns (state, state_time)."""
    st_t = time.time()
    while True:
        try:
            data, _ = rx.recvfrom(65535)
        except BlockingIOError:
            break
        try:
            last_state = json.loads(data.decode("utf-8"))
            st_t = time.time()
        except Exception:
            pass
    return last_state, st_t

# ----------------------------
# STAGE 1: Reach 60° bank
# ----------------------------
TARGET_PHI_DEG = 60.0
PHI_TOL_DEG = 2.0
MAX_TIME_TO_TARGET = 8.0
HOLD_AT_TARGET_S = 2.0

# This is NOT your “final controller,” it’s a guidance law to reach 60°.
# Keep it simple and bounded.
K_TO_TARGET = 0.02     # roll command per degree error
P_DAMP = 0.01          # damping per deg/s
UMAX_TO_TARGET = 0.60  # allow stronger command to reach 60°

# ----------------------------
# STAGE 2: Controller (attitude stabilization)
# ----------------------------
CONTROL_DURATION_S = 20.0

# Roll PD (wings-leveler)
PHI_REF = 0.0
KPHI = 0.08
KP   = 0.04
UMAX_ROLL = 0.50

# Pitch PD (optional) — requires theta_deg and q_deg_s in telemetry
USE_PITCH = True
THETA_REF = 0.0
KTHETA = 0.06
KQ     = 0.05
UMAX_PITCH = 0.25
IAS_MIN_KTS = 90.0   # if slow, don't command pitch

# Optional yaw damper
KY = 0.00
UMAX_YAW = 0.30

def reach_bank_60(sock_tx, rx):
    """Drive bank angle to +60° and hold briefly."""
    last = None
    last_t = 0.0
    t0 = time.time()
    t_end = t0 + MAX_TIME_TO_TARGET

    print("[seq] Stage 1: rolling to +60° bank")

    # take control
    send(sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
    time.sleep(0.5)

    reached = False
    while time.time() < t_end:
        now = time.time()
        last, last_t = drain_latest(rx, last)

        if last is None or (now - last_t) > STATE_TIMEOUT_S:
            # no telemetry -> go neutral but stay enabled
            send(sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(DT)
            continue

        phi = float(last.get("phi_deg", 0.0))
        p   = float(last.get("p_deg_s", 0.0))

        e = TARGET_PHI_DEG - phi

        # simple bounded guidance to reach target bank
        u = (K_TO_TARGET * e) - (P_DAMP * p)
        u = clamp(u, -UMAX_TO_TARGET, UMAX_TO_TARGET)

        send(sock_tx, enable=1, roll=u, pitch=0.0, yaw=0.0)
        time.sleep(DT)

        if abs(e) <= PHI_TOL_DEG:
            reached = True
            break

    if not reached:
        print("[seq] WARNING: did not reach 60° within time; continuing anyway.")
    else:
        print("[seq] Reached ~60°. Holding briefly...")

    # Hold near target using the same bounded law (or just small fixed roll)
    hold_end = time.time() + HOLD_AT_TARGET_S
    while time.time() < hold_end:
        now = time.time()
        last, last_t = drain_latest(rx, last)
        if last is None or (now - last_t) > STATE_TIMEOUT_S:
            send(sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(DT)
            continue

        phi = float(last.get("phi_deg", 0.0))
        p   = float(last.get("p_deg_s", 0.0))
        e = TARGET_PHI_DEG - phi
        u = (K_TO_TARGET * e) - (P_DAMP * p)
        u = clamp(u, -0.25, 0.25)  # gentler during hold
        send(sock_tx, enable=1, roll=u, pitch=0.0, yaw=0.0)
        time.sleep(DT)

def run_controller(sock_tx, rx):
    """Run roll (and optional pitch) controller for a fixed duration."""
    print("[seq] Stage 2: running controller")
    last = None
    last_t = 0.0
    end = time.time() + CONTROL_DURATION_S

    while time.time() < end:
        now = time.time()
        last, last_t = drain_latest(rx, last)

        if last is None or (now - last_t) > STATE_TIMEOUT_S:
            # neutral but enabled
            send(sock_tx, enable=1, roll=0.0, pitch=0.0, yaw=0.0)
            time.sleep(DT)
            continue

        phi   = float(last.get("phi_deg", 0.0))
        p     = float(last.get("p_deg_s", 0.0))
        r     = float(last.get("r_deg_s", 0.0))
        ias   = float(last.get("ias_kts", 0.0))

        # Roll PD
        e_phi = PHI_REF - phi
        roll_cmd = (KPHI * e_phi) - (KP * p)
        roll_cmd = clamp(roll_cmd, -UMAX_ROLL, UMAX_ROLL)

        # Pitch PD (optional)
        pitch_cmd = 0.0
        if USE_PITCH:
            theta = float(last.get("theta_deg", 0.0))
            q     = float(last.get("q_deg_s", 0.0))

            if ias < IAS_MIN_KTS:
                pitch_cmd = 0.0
            else:
                e_th = THETA_REF - theta
                pitch_cmd = (KTHETA * e_th) - (KQ * q)
                pitch_cmd = clamp(pitch_cmd, -UMAX_PITCH, UMAX_PITCH)

        # Yaw damper (optional)
        yaw_cmd = clamp(-(KY * r), -UMAX_YAW, UMAX_YAW)

        send(sock_tx, enable=1, roll=roll_cmd, pitch=pitch_cmd, yaw=yaw_cmd)
        time.sleep(DT)

def main():
    print("[seq] Starting. (Do NOT run logger simultaneously unless you handle port sharing.)")
    print(f"[seq] CMD -> {CMD_IP}:{CMD_PORT} | TELEM <- 0.0.0.0:{TELEM_PORT}")

    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = make_telem_socket()

    try:
        reach_bank_60(sock_tx, rx)
        run_controller(sock_tx, rx)
    finally:
        # Release back to mouse/user
        send(sock_tx, enable=0, roll=0.0, pitch=0.0, yaw=0.0)
        rx.close()
        sock_tx.close()
        print("[seq] Done. Released control.")

if __name__ == "__main__":
    main()