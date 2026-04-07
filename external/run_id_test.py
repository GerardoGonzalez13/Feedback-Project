import json
import socket
import time

# =========================
# NETWORK
# =========================
WINDOWS_HOST_IP = "172.26.96.1"
CMD_PORT = 49006

# =========================
# LOOP TIMING
# =========================
RATE_HZ = 25
DT = 1.0 / RATE_HZ

# =========================
# ID TEST DESIGN
# =========================
# Keep amplitudes modest so the aircraft stays closer to a linear region.
# You can adjust these later if the response is too weak.
SETTLE_TIME_S = 3.0
PULSE_TIME_S = 1.0
DECAY_TIME_S = 4.0
FINAL_NEUTRAL_S = 3.0

# Small roll inputs for approximate local linearization
PULSE_LEVELS = [0.10, -0.10, 0.15, -0.15]

# Keep pitch/yaw neutral for this identification test
PITCH_CMD = 0.0
YAW_CMD = 0.0


def send(sock, msg):
    sock.sendto(json.dumps(msg).encode("utf-8"), (WINDOWS_HOST_IP, CMD_PORT))


def hold(sock, duration_s, roll=0.0, pitch=0.0, yaw=0.0, enable=1, label=None):
    if label is not None:
        print(f"[id-test] {label}: {duration_s:.2f}s | roll={roll:+.3f}, pitch={pitch:+.3f}, yaw={yaw:+.3f}, enable={enable}")
    t_end = time.time() + duration_s
    while time.time() < t_end:
        send(sock, {
            "enable": int(enable),
            "roll": float(roll),
            "pitch": float(pitch),
            "yaw": float(yaw),
        })
        time.sleep(DT)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        print("[id-test] Starting roll-axis identification sequence.")
        print("[id-test] Make sure X-Plane is trimmed and near wings-level before running.")
        print("[id-test] Run your logger simultaneously so the full time history is saved.")

        # 0) Take control and settle
        hold(
            sock,
            SETTLE_TIME_S,
            roll=0.0,
            pitch=PITCH_CMD,
            yaw=YAW_CMD,
            enable=1,
            label="Initial neutral settle"
        )

        # 1) Apply a sequence of small positive/negative roll pulses with neutral decay windows
        for i, u in enumerate(PULSE_LEVELS, start=1):
            hold(
                sock,
                PULSE_TIME_S,
                roll=u,
                pitch=PITCH_CMD,
                yaw=YAW_CMD,
                enable=1,
                label=f"Pulse {i}"
            )

            hold(
                sock,
                DECAY_TIME_S,
                roll=0.0,
                pitch=PITCH_CMD,
                yaw=YAW_CMD,
                enable=1,
                label=f"Neutral decay after pulse {i}"
            )

        # 2) Final neutral window
        hold(
            sock,
            FINAL_NEUTRAL_S,
            roll=0.0,
            pitch=PITCH_CMD,
            yaw=YAW_CMD,
            enable=1,
            label="Final neutral"
        )

        # 3) Release control
        print("[id-test] Releasing control back to X-Plane/user.")
        send(sock, {"enable": 0})

        print("[id-test] Done.")
        print("[id-test] Your logger CSV should now contain clean neutral and forced segments for estimating a and b.")

    finally:
        try:
            send(sock, {"enable": 0})
        except Exception:
            pass
        sock.close()


if __name__ == "__main__":
    main()