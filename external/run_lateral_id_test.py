import json
import socket
import time


WINDOWS_HOST_IP = "172.26.96.1"
CMD_PORT = 49006

RATE_HZ = 25
DT = 1.0 / RATE_HZ

INITIAL_SETTLE_S = 3.0
PULSE_S = 1.0
DECAY_S = 4.0
FINAL_NEUTRAL_S = 3.0

ROLL_PULSES = [0.10, -0.10, 0.15, -0.15]
PITCH_CMD = 0.0
YAW_CMD = 0.0


def send_command(sock, enable, roll=0.0, pitch=0.0, yaw=0.0):
    message = {
        "enable": int(enable),
        "roll": float(roll),
        "pitch": float(pitch),
        "yaw": float(yaw),
    }
    sock.sendto(json.dumps(message).encode("utf-8"), (WINDOWS_HOST_IP, CMD_PORT))


def hold_phase(sock, label, duration_s, roll=0.0, pitch=0.0, yaw=0.0, enable=1):
    print(
        f"[lateral-id] {label}: {duration_s:.2f}s | "
        f"enable={enable} roll={roll:+.3f} pitch={pitch:+.3f} yaw={yaw:+.3f}"
    )
    t_end = time.time() + duration_s
    while time.time() < t_end:
        send_command(sock, enable=enable, roll=roll, pitch=pitch, yaw=yaw)
        time.sleep(DT)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        print("[lateral-id] Starting open-loop 4-state lateral identification test.")
        print("[lateral-id] Run the logger in parallel so telemetry is captured to CSV.")
        print("[lateral-id] This sequence commands roll only; pitch and yaw stay at zero.")

        hold_phase(
            sock,
            label="Initial neutral settle",
            duration_s=INITIAL_SETTLE_S,
            roll=0.0,
            pitch=PITCH_CMD,
            yaw=YAW_CMD,
            enable=1,
        )

        for idx, roll_cmd in enumerate(ROLL_PULSES, start=1):
            hold_phase(
                sock,
                label=f"Pulse {idx}",
                duration_s=PULSE_S,
                roll=roll_cmd,
                pitch=PITCH_CMD,
                yaw=YAW_CMD,
                enable=1,
            )
            hold_phase(
                sock,
                label=f"Neutral decay after pulse {idx}",
                duration_s=DECAY_S,
                roll=0.0,
                pitch=PITCH_CMD,
                yaw=YAW_CMD,
                enable=1,
            )

        hold_phase(
            sock,
            label="Final neutral",
            duration_s=FINAL_NEUTRAL_S,
            roll=0.0,
            pitch=PITCH_CMD,
            yaw=YAW_CMD,
            enable=1,
        )

        print("[lateral-id] Releasing control back to X-Plane/user.")
        send_command(sock, enable=0, roll=0.0, pitch=0.0, yaw=0.0)
        print("[lateral-id] Done.")
    finally:
        try:
            send_command(sock, enable=0, roll=0.0, pitch=0.0, yaw=0.0)
        except Exception:
            pass
        sock.close()


if __name__ == "__main__":
    main()
