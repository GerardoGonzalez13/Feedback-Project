import json
import socket
import time


STATE_PORT = 49005
CMD_PORT   = 49006
IP = "172.26.96.1"


RATE_HZ = 25
DT = 1.0 / RATE_HZ


# Roll / yaw gains
Kp = 0.08
Kd = 0.04
Ky = 0.01


# Pitch gains
Ktheta = 0.04   # pitch-angle correction
Kv = 0.01       # vertical-speed damping


phi_ref = 0.0
theta_ref = 0.0


def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


def main():
    sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_rx.bind(("0.0.0.0", STATE_PORT))
    sock_rx.setblocking(False)


    sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    last_state = None


    while True:
        try:
            data, _ = sock_rx.recvfrom(4096)
            msg = json.loads(data.decode("utf-8"))
            last_state = msg
        except BlockingIOError:
            pass


        if last_state is None:
            time.sleep(0.01)
            continue


        phi = float(last_state["phi_deg"])
        p   = float(last_state["p_deg_s"])
        r   = float(last_state["r_deg_s"])
        vv  = float(last_state["vv_m_s"])


        # You need theta_deg in telemetry for this:
        theta = float(last_state["theta_deg"])


        # Roll / yaw
        roll_error = phi - phi_ref
        roll_cmd = -Kp * roll_error - Kd * p
        yaw_cmd  = -Ky * r


        # Pitch
        pitch_error = theta - theta_ref
        pitch_cmd = -Ktheta * pitch_error - Kv * vv


        roll_cmd  = clamp(roll_cmd)
        yaw_cmd   = clamp(yaw_cmd)
        pitch_cmd = clamp(pitch_cmd)


        cmd = {
            "enable": 1,
            "roll": roll_cmd,
            "pitch": pitch_cmd,
            "yaw": yaw_cmd
        }


        sock_tx.sendto(json.dumps(cmd).encode("utf-8"), (IP, CMD_PORT))
        time.sleep(DT)


if __name__ == "__main__":
    main()

