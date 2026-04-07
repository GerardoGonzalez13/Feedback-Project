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

PITCH_CMD = 0.0

SEQUENCE = [
    ("Initial neutral settle", INITIAL_SETTLE_S, 0.0, 0.0),
    ("+0.10 roll pulse", PULSE_S, +0.10, 0.0),
    ("Neutral decay after +0.10 roll", DECAY_S, 0.0, 0.0),
    ("-0.10 roll pulse", PULSE_S, -0.10, 0.0),
    ("Neutral decay after -0.10 roll", DECAY_S, 0.0, 0.0),
    ("+0.10 yaw pulse", PULSE_S, 0.0, +0.10),
    ("Neutral decay after +0.10 yaw", DECAY_S, 0.0, 0.0),
    ("-0.10 yaw pulse", PULSE_S, 0.0, -0.10),
    ("Neutral decay after -0.10 yaw", DECAY_S, 0.0, 0.0),
    ("+0.15 roll pulse", PULSE_S, +0.15, 0.0),
    ("Neutral decay after +0.15 roll", DECAY_S, 0.0, 0.0),
    ("-0.15 roll pulse", PULSE_S, -0.15, 0.0),
    ("Neutral decay after -0.15 roll", DECAY_S, 0.0, 0.0),
    ("+0.15 yaw pulse", PULSE_S, 0.0, +0.15),
    ("Neutral decay after +0.15 yaw", DECAY_S, 0.0, 0.0),
    ("-0.15 yaw pulse", PULSE_S, 0.0, -0.15),
    ("Neutral decay after -0.15 yaw", DECAY_S, 0.0, 0.0),
    ("Final neutral", FINAL_NEUTRAL_S, 0.0, 0.0),
]


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
        f"[lateral-mimo-id] {label}: {duration_s:.2f}s | "
        f"enable={enable} roll={roll:+.3f} pitch={pitch:+.3f} yaw={yaw:+.3f}"
    )
    t_end = time.time() + duration_s
    while time.time() < t_end:
        send_command(sock, enable=enable, roll=roll, pitch=pitch, yaw=yaw)
        time.sleep(DT)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        print("[lateral-mimo-id] Starting open-loop lateral-directional MIMO identification test.")
        print("[lateral-mimo-id] Run the raw logger in parallel so telemetry is captured to CSV.")
        print("[lateral-mimo-id] Pitch stays at zero; roll and yaw are excited one at a time for clean input separation.")

        for label, duration_s, roll_cmd, yaw_cmd in SEQUENCE:
            hold_phase(
                sock,
                label=label,
                duration_s=duration_s,
                roll=roll_cmd,
                pitch=PITCH_CMD,
                yaw=yaw_cmd,
                enable=1,
            )

        print("[lateral-mimo-id] Final release: returning control to X-Plane/user.")
        send_command(sock, enable=0, roll=0.0, pitch=0.0, yaw=0.0)
        print("[lateral-mimo-id] Done.")
    finally:
        try:
            send_command(sock, enable=0, roll=0.0, pitch=0.0, yaw=0.0)
        except Exception:
            pass
        sock.close()


if __name__ == "__main__":
    main()
